# zenslam
ZenSLAM - A Nothing Special SLAM System

## Bag Image Extraction

This repository provides a helper script to extract images from a ROS1 or ROS2 bag
file using the pure-Python `rosbags` library (no full ROS environment required).

### Installation

Create a virtual environment (recommended) and install dependencies:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Usage

```bash
python scripts/bag_to_images.py <bag_path> <output_dir> [--topics /camera/color/image_raw ...] \
    [--image-format png|jpg] [--limit-per-topic N] [--flat] [--quiet]
```

Arguments:

* `bag_path`: Path to a ROS bag file (ROS1: `.bag`) or ROS2 bag directory.
* `output_dir`: Directory where extracted images will be stored.
* `--topics`: Optional list of topics to restrict extraction (default: all image topics).
* `--image-format`: Output format/extension (png, jpg, jpeg). Default: png.
* `--limit-per-topic`: Maximum number of images to save per topic.
* `--flat`: Save all images directly in `output_dir` instead of per-topic subdirectories.
* `--quiet`: Reduce logging output.

Images are named with a timestamp in nanoseconds:

```
1630577757546048415.png
```

### Examples

Extract all image topics:

```bash
python scripts/bag_to_images.py data/run1.bag extracted_images
```

Only RGB and Depth topics, store as JPEG:

```bash
python scripts/bag_to_images.py data/run1.bag extracted_images \
  --topics /camera/color/image_raw /camera/depth/image_rect_raw \
  --image-format jpg
```

Limit to 100 frames per topic and flatten directory structure:

```bash
python scripts/bag_to_images.py data/run1.bag extracted_images --limit-per-topic 100 --flat
```

### Notes

* Supports `sensor_msgs/Image` and `sensor_msgs/CompressedImage`.
* Color images in BGR encoding are converted to RGB.
* Unrecognized encodings attempt fallback shaping from `step` size; verify correctness.
* Timestamps in filenames are relative to the earliest message timestamp in the bag.
* For large bags, consider running with `--quiet` to reduce console output.
