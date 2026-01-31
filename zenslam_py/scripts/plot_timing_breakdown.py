#!/usr/bin/env python3
"""
Timing Breakdown Visualization for ZenSLAM

This script visualizes the timing breakdown of each processing stage
to identify bottlenecks and performance trends.
"""

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys


def load_data(csv_path):
    """Load frame statistics from CSV file."""
    try:
        df = pd.read_csv(csv_path)
        print(f"Loaded {len(df)} frames from {csv_path}")
        return df
    except FileNotFoundError:
        print(f"Error: File not found: {csv_path}", file=sys.stderr)
        sys.exit(1)


def plot_timing_over_time(df, output_dir):
    """Plot timing breakdown over time."""
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Set style
    sns.set_style("whitegrid")
    
    # 1. Stacked area plot of timing breakdown
    fig, ax = plt.subplots(figsize=(14, 6))
    
    # Convert to milliseconds
    timing_components = {
        'Wait': df['t_wait'] * 1000,
        'Preprocessing': df['t_preprocessing'] * 1000,
        'KLT Tracking': df['t_klt'] * 1000,
        'Detection (L+R)': (df['t_detection_left'] + df['t_detection_right']) * 1000,
        'Matching': df['t_matching'] * 1000,
        'Triangulation': df['t_triangulation'] * 1000,
        'Estimation': df['t_estimation'] * 1000,
    }
    
    # Create stacked area plot
    ax.stackplot(
        df['timestamp'],
        *timing_components.values(),
        labels=timing_components.keys(),
        alpha=0.8
    )
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Processing Time (ms)', fontsize=12)
    ax.set_title('Processing Time Breakdown Over Time (Stacked)', fontsize=14)
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'timing_stacked.png', dpi=150)
    print(f"Saved: {output_dir / 'timing_stacked.png'}")
    plt.close()
    
    # 2. Individual line plots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Detailed Timing Breakdown', fontsize=14)
    
    # Detection timing
    axes[0, 0].plot(df['timestamp'], df['t_detection_left'] * 1000, label='Detection Left', alpha=0.7)
    axes[0, 0].plot(df['timestamp'], df['t_detection_right'] * 1000, label='Detection Right', alpha=0.7)
    axes[0, 0].fill_between(df['timestamp'], 0, (df['t_detection_left'] + df['t_detection_right']) * 1000, alpha=0.2)
    axes[0, 0].set_ylabel('Time (ms)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_title('Feature Detection Timing')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Tracking components
    axes[0, 1].plot(df['timestamp'], df['t_klt'] * 1000, label='KLT Tracking', alpha=0.7)
    axes[0, 1].plot(df['timestamp'], df['t_matching'] * 1000, label='Stereo Matching', alpha=0.7)
    axes[0, 1].plot(df['timestamp'], df['t_triangulation'] * 1000, label='Triangulation', alpha=0.7)
    axes[0, 1].set_ylabel('Time (ms)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_title('Tracking Pipeline Timing')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Overall tracking vs estimation
    axes[1, 0].plot(df['timestamp'], df['t_tracking'] * 1000, label='Tracking', alpha=0.7)
    axes[1, 0].plot(df['timestamp'], df['t_estimation'] * 1000, label='Estimation', alpha=0.7)
    axes[1, 0].set_ylabel('Time (ms)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_title('High-Level Stage Timing')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Total time with components
    axes[1, 1].plot(df['timestamp'], df['t_total'] * 1000, label='Total', linewidth=2, alpha=0.7)
    axes[1, 1].axhline(y=df['t_total'].mean() * 1000, color='r', linestyle='--', 
                       label=f'Mean: {df["t_total"].mean()*1000:.1f}ms', alpha=0.5)
    axes[1, 1].set_ylabel('Time (ms)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_title('Total Processing Time')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'timing_detailed.png', dpi=150)
    print(f"Saved: {output_dir / 'timing_detailed.png'}")
    plt.close()
    
    # 3. Pie chart of average time distribution
    fig, ax = plt.subplots(figsize=(10, 8))
    
    avg_times = {k: v.mean() for k, v in timing_components.items() if v.mean() > 0}
    colors = sns.color_palette("husl", len(avg_times))
    
    pie_result = ax.pie(
        list(avg_times.values()),
        labels=list(avg_times.keys()),
        autopct='%1.1f%%',
        colors=colors,
        startangle=90
    )
    # ax.pie returns (patches, texts, autotexts) if autopct is set, else only (patches, texts)
    if len(pie_result) == 3:
        wedges, texts, autotexts = pie_result
        # Make percentage text more readable
        for autotext in autotexts:
            autotext.set_color('white')
            autotext.set_fontsize(10)
            autotext.set_fontweight('bold')
    else:
        wedges, texts = pie_result

    ax.set_title('Average Time Distribution Across Stages', fontsize=14)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'timing_pie.png', dpi=150)
    print(f"Saved: {output_dir / 'timing_pie.png'}")
    plt.close()


def print_timing_statistics(df):
    """Print timing statistics."""
    print("\n" + "="*80)
    print("TIMING STATISTICS (in milliseconds)")
    print("="*80)
    
    timing_cols = {
        'Wait': 't_wait',
        'Preprocessing': 't_preprocessing',
        'Detection Left': 't_detection_left',
        'Detection Right': 't_detection_right',
        'KLT Tracking': 't_klt',
        'Stereo Matching': 't_matching',
        'Triangulation': 't_triangulation',
        'Tracking (Total)': 't_tracking',
        'Estimation': 't_estimation',
        'Total': 't_total',
    }
    
    stats = []
    for name, col in timing_cols.items():
        if col in df.columns:
            values_ms = df[col] * 1000
            stats.append({
                'Stage': name,
                'Mean (ms)': values_ms.mean(),
                'Std (ms)': values_ms.std(),
                'Min (ms)': values_ms.min(),
                'Max (ms)': values_ms.max(),
                'Median (ms)': values_ms.median(),
                '% of Total': (df[col].mean() / df['t_total'].mean() * 100) if col != 't_total' else 100.0
            })
    
    stats_df = pd.DataFrame(stats)
    print(stats_df.to_string(index=False, float_format=lambda x: f'{x:.2f}'))
    
    # Check for frames exceeding real-time threshold
    fps_30 = (df['t_total'] > 1.0/30).sum()
    fps_20 = (df['t_total'] > 1.0/20).sum()
    print(f"\nFrames exceeding 30 FPS (>33.3ms): {fps_30} ({fps_30/len(df)*100:.1f}%)")
    print(f"Frames exceeding 20 FPS (>50.0ms): {fps_20} ({fps_20/len(df)*100:.1f}%)")


def plot_timing_histogram(df, output_dir):
    """Plot histogram of timing distribution."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Timing Distribution Histograms', fontsize=14)
    
    # Detection time
    detection_total = (df['t_detection_left'] + df['t_detection_right']) * 1000
    axes[0, 0].hist(detection_total, bins=50, alpha=0.7, edgecolor='black')
    axes[0, 0].axvline(detection_total.mean(), color='r', linestyle='--', 
                       label=f'Mean: {detection_total.mean():.1f}ms')
    axes[0, 0].set_xlabel('Time (ms)')
    axes[0, 0].set_ylabel('Frequency')
    axes[0, 0].set_title('Feature Detection Time (L+R)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Matching time
    matching_time = df['t_matching'] * 1000
    axes[0, 1].hist(matching_time, bins=50, alpha=0.7, edgecolor='black')
    axes[0, 1].axvline(matching_time.mean(), color='r', linestyle='--', 
                       label=f'Mean: {matching_time.mean():.1f}ms')
    axes[0, 1].set_xlabel('Time (ms)')
    axes[0, 1].set_ylabel('Frequency')
    axes[0, 1].set_title('Stereo Matching Time')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Tracking time
    tracking_time = df['t_tracking'] * 1000
    axes[1, 0].hist(tracking_time, bins=50, alpha=0.7, edgecolor='black')
    axes[1, 0].axvline(tracking_time.mean(), color='r', linestyle='--', 
                       label=f'Mean: {tracking_time.mean():.1f}ms')
    axes[1, 0].set_xlabel('Time (ms)')
    axes[1, 0].set_ylabel('Frequency')
    axes[1, 0].set_title('Tracking Time')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Total time
    total_time = df['t_total'] * 1000
    axes[1, 1].hist(total_time, bins=50, alpha=0.7, edgecolor='black')
    axes[1, 1].axvline(total_time.mean(), color='r', linestyle='--', 
                       label=f'Mean: {total_time.mean():.1f}ms')
    axes[1, 1].axvline(33.3, color='g', linestyle='--', alpha=0.5, label='30 FPS limit')
    axes[1, 1].set_xlabel('Time (ms)')
    axes[1, 1].set_ylabel('Frequency')
    axes[1, 1].set_title('Total Processing Time')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'timing_histogram.png', dpi=150)
    print(f"Saved: {output_dir / 'timing_histogram.png'}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description="Visualize timing breakdown of ZenSLAM processing stages"
    )
    parser.add_argument(
        'csv_file',
        type=Path,
        help='Path to frame_data.csv file'
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path('timing_analysis'),
        help='Output directory for plots (default: timing_analysis/)'
    )
    
    args = parser.parse_args()
    
    # Load data
    df = load_data(args.csv_file)
    
    # Print statistics
    print_timing_statistics(df)
    
    # Generate plots
    print("\nGenerating timing visualization plots...")
    plot_timing_over_time(df, args.output_dir)
    plot_timing_histogram(df, args.output_dir)
    
    print(f"\nAll plots saved to: {args.output_dir}")


if __name__ == '__main__':
    main()
