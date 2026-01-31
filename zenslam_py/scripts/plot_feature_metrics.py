#!/usr/bin/env python3
"""
Feature Metrics Visualization for ZenSLAM

This script visualizes feature detection, matching, and tracking metrics
to analyze performance trends and identify issues.
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


def plot_feature_counts(df, output_dir):
    """Plot feature counts over time."""
    output_dir.mkdir(parents=True, exist_ok=True)
    
    sns.set_style("whitegrid")
    
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle('Feature Detection and Matching Metrics', fontsize=16)
    
    # Keypoints detected
    axes[0, 0].plot(df['timestamp'], df['n_keypoints_l'], label='Left Camera', alpha=0.7)
    axes[0, 0].plot(df['timestamp'], df['n_keypoints_r'], label='Right Camera', alpha=0.7)
    axes[0, 0].axhline(y=df['n_keypoints_l'].mean(), color='b', linestyle='--', alpha=0.3)
    axes[0, 0].axhline(y=df['n_keypoints_r'].mean(), color='orange', linestyle='--', alpha=0.3)
    axes[0, 0].set_ylabel('Keypoints')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_title('Total Keypoints Detected')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Tracked vs new keypoints
    axes[0, 1].plot(df['timestamp'], df['n_tracked_l'], label='Tracked', alpha=0.7)
    axes[0, 1].plot(df['timestamp'], df['n_new_l'], label='New Detected', alpha=0.7)
    axes[0, 1].set_ylabel('Keypoints (Left)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_title('Tracked vs New Keypoints (Left Camera)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Stereo matches
    axes[1, 0].plot(df['timestamp'], df['n_matches'], alpha=0.7, color='green')
    axes[1, 0].fill_between(df['timestamp'], 0, df['n_matches'], alpha=0.2, color='green')
    axes[1, 0].axhline(y=df['n_matches'].mean(), color='green', linestyle='--', 
                       label=f'Mean: {df["n_matches"].mean():.1f}', alpha=0.5)
    axes[1, 0].axhline(y=20, color='r', linestyle='--', label='Min threshold (20)', alpha=0.5)
    axes[1, 0].set_ylabel('Matches')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_title('Stereo Matches')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Triangulated points
    axes[1, 1].plot(df['timestamp'], df['n_triangulated'], alpha=0.7, color='purple')
    axes[1, 1].fill_between(df['timestamp'], 0, df['n_triangulated'], alpha=0.2, color='purple')
    axes[1, 1].axhline(y=df['n_triangulated'].mean(), color='purple', linestyle='--', 
                       label=f'Mean: {df["n_triangulated"].mean():.1f}', alpha=0.5)
    axes[1, 1].set_ylabel('Triangulated Points')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_title('Triangulated 3D Points')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    # KLT success rate
    axes[2, 0].plot(df['timestamp'], df['klt_success_rate'] * 100, alpha=0.7, color='darkblue')
    axes[2, 0].fill_between(df['timestamp'], 0, df['klt_success_rate'] * 100, alpha=0.2, color='darkblue')
    axes[2, 0].axhline(y=df['klt_success_rate'].mean() * 100, color='darkblue', linestyle='--', 
                       label=f'Mean: {df["klt_success_rate"].mean()*100:.1f}%', alpha=0.5)
    axes[2, 0].axhline(y=30, color='r', linestyle='--', label='Min threshold (30%)', alpha=0.5)
    axes[2, 0].set_ylabel('Success Rate (%)')
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_title('KLT Tracking Success Rate')
    axes[2, 0].legend()
    axes[2, 0].grid(True, alpha=0.3)
    axes[2, 0].set_ylim([0, 105])
    
    # Pose estimation inliers
    axes[2, 1].plot(df['timestamp'], df['n_3d3d_inliers'], label='3D-3D', alpha=0.7)
    axes[2, 1].plot(df['timestamp'], df['n_3d2d_inliers'], label='3D-2D', alpha=0.7)
    axes[2, 1].plot(df['timestamp'], df['n_2d2d_inliers'], label='2D-2D', alpha=0.7)
    axes[2, 1].set_ylabel('Inliers')
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_title('Pose Estimation Inliers')
    axes[2, 1].legend()
    axes[2, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'feature_metrics.png', dpi=150)
    print(f"Saved: {output_dir / 'feature_metrics.png'}")
    plt.close()


def plot_quality_metrics(df, output_dir):
    """Plot quality metrics."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Feature Quality Metrics', fontsize=16)
    
    # Response values
    axes[0, 0].plot(df['timestamp'], df['response_mean_l'], label='Left Camera', alpha=0.7)
    axes[0, 0].plot(df['timestamp'], df['response_mean_r'], label='Right Camera', alpha=0.7)
    axes[0, 0].fill_between(df['timestamp'], 
                            df['response_mean_l'] - df['response_std_l'],
                            df['response_mean_l'] + df['response_std_l'],
                            alpha=0.2)
    axes[0, 0].set_ylabel('Response Value')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_title('Feature Response (Strength)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Grid occupancy
    axes[0, 1].plot(df['timestamp'], df['grid_occupancy_l'] * 100, label='Left Camera', alpha=0.7)
    axes[0, 1].plot(df['timestamp'], df['grid_occupancy_r'] * 100, label='Right Camera', alpha=0.7)
    axes[0, 1].set_ylabel('Grid Occupancy (%)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_title('Grid Cell Occupancy')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_ylim([0, 105])
    
    # Track ages
    if 'track_age_mean' in df.columns:
        axes[1, 0].plot(df['timestamp'], df['track_age_mean'], label='Mean Track Age', alpha=0.7)
        axes[1, 0].plot(df['timestamp'], df['track_age_max'], label='Max Track Age', alpha=0.7)
        axes[1, 0].set_ylabel('Track Age (frames)')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_title('Feature Track Ages')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
    
    # Epipolar error
    if 'epipolar_error_mean' in df.columns:
        axes[1, 1].plot(df['timestamp'], df['epipolar_error_mean'], alpha=0.7, color='red')
        axes[1, 1].axhline(y=df['epipolar_error_mean'].mean(), color='red', linestyle='--', 
                          label=f'Mean: {df["epipolar_error_mean"].mean():.2f}px', alpha=0.5)
        axes[1, 1].set_ylabel('Error (pixels)')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_title('Epipolar Error')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'quality_metrics.png', dpi=150)
    print(f"Saved: {output_dir / 'quality_metrics.png'}")
    plt.close()


def plot_correlation_analysis(df, output_dir):
    """Plot correlation between different metrics."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Metric Correlation Analysis', fontsize=16)
    
    # KLT success vs matches
    axes[0, 0].scatter(df['klt_success_rate'] * 100, df['n_matches'], alpha=0.5)
    axes[0, 0].set_xlabel('KLT Success Rate (%)')
    axes[0, 0].set_ylabel('Stereo Matches')
    axes[0, 0].set_title('KLT Success vs Stereo Matches')
    axes[0, 0].grid(True, alpha=0.3)
    
    # Matches vs triangulated
    axes[0, 1].scatter(df['n_matches'], df['n_triangulated'], alpha=0.5, color='green')
    axes[0, 1].set_xlabel('Stereo Matches')
    axes[0, 1].set_ylabel('Triangulated Points')
    axes[0, 1].set_title('Matches vs Triangulation')
    axes[0, 1].grid(True, alpha=0.3)
    
    # Response vs matches
    axes[1, 0].scatter(df['response_mean_l'], df['n_matches'], alpha=0.5, color='purple')
    axes[1, 0].set_xlabel('Feature Response (Left)')
    axes[1, 0].set_ylabel('Stereo Matches')
    axes[1, 0].set_title('Feature Strength vs Matches')
    axes[1, 0].grid(True, alpha=0.3)
    
    # Keypoints vs inliers
    axes[1, 1].scatter(df['n_keypoints_l'], df['n_3d3d_inliers'], alpha=0.5, color='orange')
    axes[1, 1].set_xlabel('Keypoints (Left)')
    axes[1, 1].set_ylabel('3D-3D Inliers')
    axes[1, 1].set_title('Keypoints vs Pose Inliers')
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'correlation_analysis.png', dpi=150)
    print(f"Saved: {output_dir / 'correlation_analysis.png'}")
    plt.close()


def print_feature_statistics(df):
    """Print feature statistics."""
    print("\n" + "="*80)
    print("FEATURE DETECTION AND MATCHING STATISTICS")
    print("="*80)
    
    stats = {
        'Keypoints (Left)': df['n_keypoints_l'],
        'Keypoints (Right)': df['n_keypoints_r'],
        'Tracked (Left)': df['n_tracked_l'],
        'New Detected (Left)': df['n_new_l'],
        'Stereo Matches': df['n_matches'],
        'Triangulated Points': df['n_triangulated'],
        'KLT Success Rate (%)': df['klt_success_rate'] * 100,
        'Response (Left)': df['response_mean_l'],
        'Response (Right)': df['response_mean_r'],
        '3D-3D Inliers': df['n_3d3d_inliers'],
        '3D-2D Inliers': df['n_3d2d_inliers'],
    }
    
    summary = []
    for name, series in stats.items():
        summary.append({
            'Metric': name,
            'Mean': series.mean(),
            'Std': series.std(),
            'Min': series.min(),
            'Max': series.max(),
            'Median': series.median()
        })
    
    summary_df = pd.DataFrame(summary)
    print(summary_df.to_string(index=False, float_format=lambda x: f'{x:.2f}'))


def main():
    parser = argparse.ArgumentParser(
        description="Visualize feature detection, matching, and tracking metrics"
    )
    parser.add_argument(
        'csv_file',
        type=Path,
        help='Path to frame_data.csv file'
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path('feature_analysis'),
        help='Output directory for plots (default: feature_analysis/)'
    )
    
    args = parser.parse_args()
    
    # Load data
    df = load_data(args.csv_file)
    
    # Print statistics
    print_feature_statistics(df)
    
    # Generate plots
    print("\nGenerating feature metric visualization plots...")
    plot_feature_counts(df, args.output_dir)
    plot_quality_metrics(df, args.output_dir)
    plot_correlation_analysis(df, args.output_dir)
    
    print(f"\nAll plots saved to: {args.output_dir}")


if __name__ == '__main__':
    main()
