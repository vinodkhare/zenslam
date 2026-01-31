#!/usr/bin/env python3
"""
Detector Performance Comparison for ZenSLAM

This script compares performance across multiple detector configurations
by analyzing multiple frame_data.csv files.
"""

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys


def load_multiple_runs(csv_paths):
    """Load multiple frame_data.csv files."""
    runs = {}
    for path in csv_paths:
        try:
            df = pd.read_csv(path)
            # Use detector_type as run name, or filename if not available
            if 'detector_type' in df.columns and not df['detector_type'].isna().all():
                run_name = f"{df['detector_type'].iloc[0]}+{df['descriptor_type'].iloc[0]} ({path.parent.name})"
            else:
                run_name = path.stem
            runs[run_name] = df
            print(f"Loaded {len(df)} frames from {path}")
        except Exception as e:
            print(f"Error loading {path}: {e}", file=sys.stderr)
    return runs


def compute_comparison_metrics(runs):
    """Compute comparison metrics for all runs."""
    metrics = []
    
    for name, df in runs.items():
        metric = {
            'Run': name,
            'Frames': len(df),
            'Avg Keypoints (L)': df['n_keypoints_l'].mean(),
            'Avg Keypoints (R)': df['n_keypoints_r'].mean(),
            'Avg Matches': df['n_matches'].mean(),
            'Avg Triangulated': df['n_triangulated'].mean(),
            'KLT Success Rate': df['klt_success_rate'].mean() * 100,
            'Avg Response (L)': df['response_mean_l'].mean(),
            'Avg Response (R)': df['response_mean_r'].mean(),
            'Avg Detection Time (L+R)': (df['t_detection_left'] + df['t_detection_right']).mean() * 1000,  # ms
            'Avg Matching Time': df['t_matching'].mean() * 1000,  # ms
            'Avg Tracking Time': df['t_tracking'].mean() * 1000,  # ms
            'Avg Total Time': df['t_total'].mean() * 1000,  # ms
            'Frames < 20 matches': (df['n_matches'] < 20).sum(),
            'Frames < 30% KLT': (df['klt_success_rate'] < 0.3).sum(),
        }
        metrics.append(metric)
    
    return pd.DataFrame(metrics)


def plot_comparison(runs, output_dir):
    """Generate comparison plots."""
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Set style
    sns.set_style("whitegrid")
    colors = sns.color_palette("husl", len(runs))
    
    # 1. Feature counts over time
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Feature Detection and Matching Performance', fontsize=16)
    
    for (name, df), color in zip(runs.items(), colors):
        axes[0, 0].plot(df['timestamp'], df['n_keypoints_l'], label=name, alpha=0.7, color=color)
        axes[0, 1].plot(df['timestamp'], df['n_matches'], label=name, alpha=0.7, color=color)
        axes[1, 0].plot(df['timestamp'], df['n_triangulated'], label=name, alpha=0.7, color=color)
        axes[1, 1].plot(df['timestamp'], df['klt_success_rate'] * 100, label=name, alpha=0.7, color=color)
    
    axes[0, 0].set_ylabel('Keypoints (Left)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].set_ylabel('Stereo Matches')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[1, 0].set_ylabel('Triangulated Points')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].set_ylabel('KLT Success Rate (%)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'feature_comparison.png', dpi=150)
    print(f"Saved: {output_dir / 'feature_comparison.png'}")
    plt.close()
    
    # 2. Timing comparison
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Timing Performance Comparison', fontsize=16)
    
    for (name, df), color in zip(runs.items(), colors):
        axes[0, 0].plot(df['timestamp'], df['t_detection_left'] * 1000, label=name, alpha=0.7, color=color)
        axes[0, 1].plot(df['timestamp'], df['t_matching'] * 1000, label=name, alpha=0.7, color=color)
        axes[1, 0].plot(df['timestamp'], df['t_tracking'] * 1000, label=name, alpha=0.7, color=color)
        axes[1, 1].plot(df['timestamp'], df['t_total'] * 1000, label=name, alpha=0.7, color=color)
    
    axes[0, 0].set_ylabel('Detection Time (ms)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].set_ylabel('Matching Time (ms)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[1, 0].set_ylabel('Tracking Time (ms)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].set_ylabel('Total Time (ms)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'timing_comparison.png', dpi=150)
    print(f"Saved: {output_dir / 'timing_comparison.png'}")
    plt.close()
    
    # 3. Box plots for key metrics
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('Distribution Comparison (Box Plots)', fontsize=16)
    
    # Prepare data for box plots
    data_matches = []
    data_triangulated = []
    data_klt = []
    data_detection = []
    data_matching = []
    data_total = []
    labels = []
    
    for name, df in runs.items():
        labels.append(name)
        data_matches.append(df['n_matches'].values)
        data_triangulated.append(df['n_triangulated'].values)
        data_klt.append(df['klt_success_rate'].values * 100)
        data_detection.append((df['t_detection_left'] + df['t_detection_right']).values * 1000)
        data_matching.append(df['t_matching'].values * 1000)
        data_total.append(df['t_total'].values * 1000)
    
    axes[0, 0].boxplot(data_matches, labels=labels)
    axes[0, 0].set_ylabel('Matches')
    axes[0, 0].set_title('Stereo Matches')
    axes[0, 0].tick_params(axis='x', rotation=15)
    
    axes[0, 1].boxplot(data_triangulated, labels=labels)
    axes[0, 1].set_ylabel('Triangulated Points')
    axes[0, 1].set_title('Triangulated Points')
    axes[0, 1].tick_params(axis='x', rotation=15)
    
    axes[0, 2].boxplot(data_klt, labels=labels)
    axes[0, 2].set_ylabel('KLT Success Rate (%)')
    axes[0, 2].set_title('KLT Success Rate')
    axes[0, 2].tick_params(axis='x', rotation=15)
    
    axes[1, 0].boxplot(data_detection, labels=labels)
    axes[1, 0].set_ylabel('Detection Time (ms)')
    axes[1, 0].set_title('Feature Detection Time (L+R)')
    axes[1, 0].tick_params(axis='x', rotation=15)
    
    axes[1, 1].boxplot(data_matching, labels=labels)
    axes[1, 1].set_ylabel('Matching Time (ms)')
    axes[1, 1].set_title('Feature Matching Time')
    axes[1, 1].tick_params(axis='x', rotation=15)
    
    axes[1, 2].boxplot(data_total, labels=labels)
    axes[1, 2].set_ylabel('Total Time (ms)')
    axes[1, 2].set_title('Total Processing Time')
    axes[1, 2].tick_params(axis='x', rotation=15)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'distribution_comparison.png', dpi=150)
    print(f"Saved: {output_dir / 'distribution_comparison.png'}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description="Compare performance across different detector configurations"
    )
    parser.add_argument(
        'csv_files',
        type=Path,
        nargs='+',
        help='Paths to frame_data.csv files to compare'
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path('detector_comparison'),
        help='Output directory for plots (default: detector_comparison/)'
    )
    
    args = parser.parse_args()
    
    # Load all runs
    runs = load_multiple_runs(args.csv_files)
    
    if len(runs) == 0:
        print("Error: No valid data files loaded", file=sys.stderr)
        sys.exit(1)
    
    # Compute comparison metrics
    comparison = compute_comparison_metrics(runs)
    
    # Print comparison table
    print("\n" + "="*120)
    print("DETECTOR PERFORMANCE COMPARISON")
    print("="*120)
    print(comparison.to_string(index=False))
    
    # Save comparison to CSV
    args.output_dir.mkdir(parents=True, exist_ok=True)
    comparison.to_csv(args.output_dir / 'detector_comparison.csv', index=False)
    print(f"\nComparison table saved to: {args.output_dir / 'detector_comparison.csv'}")
    
    # Generate plots
    print("\nGenerating comparison plots...")
    plot_comparison(runs, args.output_dir)
    
    print(f"\nAll outputs saved to: {args.output_dir}")


if __name__ == '__main__':
    main()
