#!/usr/bin/env python3
"""
Convert a ROS1/ROS2 bag file into folders of extracted images using the
`rosbags` Python library.

Supports sensor_msgs/msg/Image and sensor_msgs/msg/CompressedImage topics.

Example:
    python scripts/bag_to_images.py my.bag output_dir --topics /camera/color/image_raw /camera/depth/image_rect_raw \
        --image-format png --start 5.0 --end 25.0

Requirements: pip install -r requirements.txt
"""
from __future__ import annotations

import argparse
import io
import numpy as np
import sys
from PIL import Image as PILImage
from pathlib import Path
from typing import Optional, Sequence, Tuple, Dict

try:
    from rosbags.highlevel import AnyReader
    from rosbags.rosbag2 import Reader as ROS2Reader  # noqa: F401 (for type reference)
except ImportError as exc:  # pragma: no cover
    print("ERROR: rosbags is not installed. Run: pip install -r requirements.txt", file=sys.stderr)
    raise

# ROS message type names we handle
IMAGE_TYPES = {
    'sensor_msgs/msg/Image',  # ROS2 style
    'sensor_msgs/Image',  # ROS1 style
}
COMPRESSED_IMAGE_TYPES = {
    'sensor_msgs/msg/CompressedImage',
    'sensor_msgs/CompressedImage',
}


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description='Extract images from a ROS bag using rosbags.')
    p.add_argument('bag', help='Path to bag file or folder (ROS2)')
    p.add_argument('output', help='Output directory root')
    p.add_argument('--topics', nargs='*', default=None,
                   help='Specific image topics to extract (default: all image topics)')
    p.add_argument('--image-format', default='png', choices=['png', 'jpg', 'jpeg'],
                   help='Image format/extension to write.')
    p.add_argument('--limit-per-topic', type=int, default=None, help='Maximum number of images per topic.')
    p.add_argument('--flat', action='store_true',
                   help='Do not create per-topic subdirectories, put all images in output root.')
    p.add_argument('--export-images', action='store_true', help='Extract and export images from image topics.')
    p.add_argument('--export-tf', action='store_true', help='Extract and export TF tree from /tf and /tf_static topics.')
    p.add_argument('--quiet', action='store_true', help='Reduce logging output.')
    p.add_argument('--no-progress', action='store_true', help='Disable progress bar display.')
    return p.parse_args(argv)


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def normalize_topic(topic: str) -> str:
    # Remove leading slash for filesystem usage but keep original for names
    return topic.lstrip('/')


def msg_to_np(msg) -> Tuple[np.ndarray, str]:
    """Convert Image or CompressedImage ROS message to numpy array and encoding.

    Returns:
        (array(H,W,C or 1), encoding_string)
    """
    typename = msg.__msgtype__  # rosbags adds this attribute
    if typename in IMAGE_TYPES:
        # Raw Image. Assume msg has fields: height, width, encoding, data, step.
        h = msg.height
        w = msg.width
        encoding = msg.encoding.decode() if isinstance(msg.encoding, bytes) else msg.encoding
        data = msg.data
        if isinstance(data, bytes):
            buf = data
        else:  # sequence of uint8
            buf = bytes(data)
        arr = np.frombuffer(buf, dtype=np.uint8)
        if encoding in ('rgb8', 'bgr8'):  # 3 channel
            arr = arr.reshape(h, w, 3)
            if encoding == 'bgr8':
                arr = arr[:, :, ::-1]  # convert to RGB
            encoding = 'rgb8'
        elif encoding in ('mono8', '8UC1'):
            arr = arr.reshape(h, w)
            encoding = 'mono8'
        else:
            # Fallback: attempt to infer channels based on step
            channels = msg.step // w
            arr = arr.reshape(h, w, channels)
            if channels == 3:
                encoding = 'rgb8'
            elif channels == 1:
                encoding = 'mono8'
            else:
                encoding = f'channels{channels}'
        return arr, encoding
    elif typename in COMPRESSED_IMAGE_TYPES:
        # Compressed image: format like 'jpeg' or 'png'
        fmt = msg.format.decode() if isinstance(msg.format, bytes) else msg.format
        # Data is complete compressed image bytes
        if isinstance(msg.data, bytes):
            buf = msg.data
        else:
            buf = bytes(msg.data)
        # Let Pillow decode
        with PILImage.open(io.BytesIO(buf)) as im:  # type: ignore
            im = im.convert('RGB')
            arr = np.array(im)
        return arr, fmt
    else:
        raise TypeError(f'Unsupported message type: {typename}')


def save_image(arr: np.ndarray, path: Path, image_format: str):
    if arr.ndim == 2:
        im = PILImage.fromarray(arr, mode='L')
    elif arr.ndim == 3 and arr.shape[2] == 3:
        im = PILImage.fromarray(arr, mode='RGB')
    else:
        # Attempt squeeze last dim if 1
        if arr.ndim == 3 and arr.shape[2] == 1:
            im = PILImage.fromarray(arr[:, :, 0], mode='L')
        else:
            raise ValueError(f'Unsupported array shape for image: {arr.shape}')
    im.save(path, format=image_format.upper())


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    bag_path = Path(args.bag)
    out_root = Path(args.output)
    ensure_dir(out_root)

    if not bag_path.exists():
        print(f'ERROR: Bag path not found: {bag_path}', file=sys.stderr)
        return 1
    
    # print all connections in the bag for debugging
    with AnyReader([bag_path]) as reader:
        print("Connections in the bag:")
        for conn in reader.connections:
            print(f"  Topic: {conn.topic}, Type: {conn.msgtype}")

    selected_topics = set(args.topics) if args.topics else None

    if args.export_tf:
        with AnyReader([bag_path]) as reader:
            # Find TF topics
            tf_connections = [
                conn for conn in reader.connections if conn.topic in ['/tf_static']
            ]

            # Read TF messages
            i = 0
            for connection, timestamp, rawdata in reader.messages(connections=tf_connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                
                # msg.transforms is a list of geometry_msgs/TransformStamped
                for transform in msg.transforms:
                    print(f"{transform.header.frame_id} -> {transform.child_frame_id}")
                    # print(f"  Translation: {transform.transform.translation}")
                    # print(f"  Rotation: {transform.transform.rotation}")

                if i > 3:
                    break

                i += 1

    if not args.export_images:
        return 0

    # Use AnyReader to automatically detect bag type
    with AnyReader([bag_path]) as reader:
        # Map connections to types
        image_connections = []
        for connection in reader.connections:
            t = connection.msgtype
            if t in IMAGE_TYPES or t in COMPRESSED_IMAGE_TYPES:
                if selected_topics is None or connection.topic in selected_topics:
                    image_connections.append(connection)
        if not image_connections:
            print('No image topics found (after filtering).' if not args.quiet else '', file=sys.stderr)
            return 1

        counts: Dict[str, int] = {}
        written = 0
        limit = args.limit_per_topic
        total_candidate = reader.message_count

        if not args.quiet:
            print(
                f'Found {len(image_connections)} image topic(s). Extracting {total_candidate} messages (before filtering time/limits)...')

        progress_iter = reader.messages(connections=image_connections)
        pbar = None

        if not args.no_progress and not args.quiet:
            try:
                from tqdm import tqdm  # type: ignore
                pbar = tqdm(total=total_candidate, unit='msg', desc='Extract')
            except Exception:
                pbar = None

        for connection, timestamp, rawdata in progress_iter:
            # No start/end time filtering (extraction covers whole bag)
            topic = connection.topic
            if limit is not None and counts.get(topic, 0) >= limit:
                if pbar:
                    pbar.update(1)
                continue

            # Deserialize message
            msg = reader.deserialize(rawdata, connection.msgtype)
            typename = connection.msgtype
            if typename not in IMAGE_TYPES and typename not in COMPRESSED_IMAGE_TYPES:
                if pbar:
                    pbar.update(1)
                continue

            # Convert
            try:
                arr, enc = msg_to_np(msg)
            except Exception as e:  # pragma: no cover
                if not args.quiet:
                    print(f'WARN: Failed to convert message on {topic} at {timestamp}: {e}', file=sys.stderr)
                if pbar:
                    pbar.update(1)
                continue

            topic_dir = out_root if args.flat else (out_root / normalize_topic(topic))
            ensure_dir(topic_dir)

            idx = counts.get(topic, 0)
            base_name = str(timestamp)  # use raw ns timestamp from bag
            out_path = topic_dir / f'{base_name}.{args.image_format}'

            if out_path.exists():  # rare collision safeguard
                suffix = 1
                while True:
                    alt = topic_dir / f'{base_name}_{suffix}.{args.image_format}'
                    if not alt.exists():
                        out_path = alt
                        break
                    suffix += 1

            try:
                save_image(arr, out_path, args.image_format)
            except Exception as e:  # pragma: no cover
                if not args.quiet:
                    print(f'WARN: Failed to save image {out_path}: {e}', file=sys.stderr)
                continue

            counts[topic] = idx + 1
            written += 1

            if pbar:
                pbar.update(1)
            elif not args.quiet and written % 50 == 0:
                print(f'Wrote {written} images...')

        if pbar:
            pbar.close()

        if not args.quiet:
            print(f'Done. Wrote {written} images across {len(counts)} topics.')
    return 0


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
