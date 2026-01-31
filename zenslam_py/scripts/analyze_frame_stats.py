#!/usr/bin/env python3
"""
Frame Statistics Analyzer for ZenSLAM

This script analyzes the frame_data.csv output from ZenSLAM to identify
problematic frames where tracking fails or performance degrades.
"""

import argparse
import pandas as pd
import numpy as np
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
    except pd.errors.EmptyDataError:
        print(f"Error: CSV file is empty: {csv_path}", file=sys.stderr)
        sys.exit(1)


def identify_tracking_failures(df, threshold=0.3):
    """
    Identify frames where tracking success rate is below threshold.
    
    Args:
        df: DataFrame with frame statistics
        threshold: Minimum acceptable KLT success rate (default 0.3 = 30%)
    
    Returns:
        DataFrame with problematic frames
    """
    failures = df[df['klt_success_rate'] < threshold].copy()
    failures['issue_type'] = 'Low KLT success rate'
    return failures


def identify_low_matches(df, threshold=20):
    """
    Identify frames with very few feature matches.
    
    Args:
        df: DataFrame with frame statistics
        threshold: Minimum acceptable number of matches
    
    Returns:
        DataFrame with problematic frames
    """
    low_matches = df[df['n_matches'] < threshold].copy()
    low_matches['issue_type'] = 'Low feature matches'
    return low_matches


def identify_low_triangulation(df, threshold=10):
    """
    Identify frames with very few triangulated points.
    
    Args:
        df: DataFrame with frame statistics
        threshold: Minimum acceptable number of triangulated points
    
    Returns:
        DataFrame with problematic frames
    """
    low_tri = df[df['n_triangulated'] < threshold].copy()
    low_tri['issue_type'] = 'Low triangulated points'
    return low_tri


def identify_low_inliers(df, threshold=10):
    """
    Identify frames with very few pose estimation inliers.
    
    Args:
        df: DataFrame with frame statistics
        threshold: Minimum acceptable number of inliers
    
    Returns:
        DataFrame with problematic frames
    """
    # Check all three correspondence types
    low_inliers = df[
        (df['n_3d3d_inliers'] < threshold) &
        (df['n_3d2d_inliers'] < threshold) &
        (df['n_2d2d_inliers'] < threshold)
    ].copy()
    low_inliers['issue_type'] = 'Low pose inliers'
    return low_inliers


def identify_high_timing(df, percentile=95):
    """
    Identify frames with unusually high processing time.
    
    Args:
        df: DataFrame with frame statistics
        percentile: Percentile threshold for "high" timing
    
    Returns:
        DataFrame with slow frames
    """
    threshold = df['t_total'].quantile(percentile / 100.0)
    slow_frames = df[df['t_total'] > threshold].copy()
    slow_frames['issue_type'] = f'Slow processing (>{threshold:.3f}s)'
    return slow_frames


def compute_summary_statistics(df):
    """Compute overall summary statistics."""
    summary = {
        'Total frames': len(df),
        'Mean KLT success rate': f"{df['klt_success_rate'].mean():.2%}",
        'Mean matches': f"{df['n_matches'].mean():.1f}",
        'Mean triangulated': f"{df['n_triangulated'].mean():.1f}",
        'Mean processing time': f"{df['t_total'].mean():.3f}s",
        'Mean tracking time': f"{df['t_tracking'].mean():.3f}s",
        'Mean detection time (L+R)': f"{(df['t_detection_left'] + df['t_detection_right']).mean():.3f}s",
        'Mean matching time': f"{df['t_matching'].mean():.3f}s",
        'Mean response (L/R)': f"{df['response_mean_l'].mean():.1f} / {df['response_mean_r'].mean():.1f}",
    }
    return summary


def analyze_detector_performance(df):
    """Analyze performance by detector type if multiple detectors are used."""
    if 'detector_type' in df.columns:
        detector_stats = df.groupby('detector_type').agg({
            'n_keypoints_l': 'mean',
            'n_matches': 'mean',
            'n_triangulated': 'mean',
            'klt_success_rate': 'mean',
            't_detection_left': 'mean',
            't_detection_right': 'mean',
            't_matching': 'mean',
            'response_mean_l': 'mean',
        }).round(3)
        return detector_stats
    return None


def print_problem_summary(df, problems):
    """Print a summary of identified problems."""
    print("\n" + "="*80)
    print("PROBLEM FRAMES SUMMARY")
    print("="*80)
    
    if len(problems) == 0:
        print("No problematic frames identified!")
        return
    
    # Group by issue type
    issue_counts = problems['issue_type'].value_counts()
    print(f"\nFound {len(problems)} problem frames ({len(problems)/len(df)*100:.1f}% of total):")
    for issue, count in issue_counts.items():
        print(f"  - {issue}: {count} frames")
    
    # Show worst frames
    print("\nTop 10 worst frames by multiple criteria:")
    worst = problems.sort_values(['klt_success_rate', 'n_matches']).head(10)
    print(worst[['timestamp', 'klt_success_rate', 'n_matches', 'n_triangulated', 'issue_type']].to_string(index=False))


def save_problem_frames(problems, output_path):
    """Save problematic frames to CSV."""
    if len(problems) > 0:
        problems.to_csv(output_path, index=False)
        print(f"\nProblem frames saved to: {output_path}")
    else:
        print("\nNo problem frames to save.")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze ZenSLAM frame statistics to identify tracking failures and performance issues"
    )
    parser.add_argument(
        'csv_file',
        type=Path,
        help='Path to frame_data.csv file'
    )
    parser.add_argument(
        '--klt-threshold',
        type=float,
        default=0.3,
        help='KLT success rate threshold (default: 0.3)'
    )
    parser.add_argument(
        '--match-threshold',
        type=int,
        default=20,
        help='Minimum feature matches threshold (default: 20)'
    )
    parser.add_argument(
        '--triangulation-threshold',
        type=int,
        default=10,
        help='Minimum triangulated points threshold (default: 10)'
    )
    parser.add_argument(
        '--inlier-threshold',
        type=int,
        default=10,
        help='Minimum pose inliers threshold (default: 10)'
    )
    parser.add_argument(
        '--output',
        type=Path,
        help='Output path for problem frames CSV (default: problems_<input>.csv)'
    )
    
    args = parser.parse_args()
    
    # Load data
    df = load_data(args.csv_file)
    
    # Compute summary statistics
    print("\n" + "="*80)
    print("OVERALL STATISTICS")
    print("="*80)
    summary = compute_summary_statistics(df)
    for key, value in summary.items():
        print(f"{key:.<40} {value}")
    
    # Detector-specific analysis
    detector_stats = analyze_detector_performance(df)
    if detector_stats is not None:
        print("\n" + "="*80)
        print("DETECTOR PERFORMANCE COMPARISON")
        print("="*80)
        print(detector_stats.to_string())
    
    # Identify problems
    tracking_failures = identify_tracking_failures(df, args.klt_threshold)
    low_matches = identify_low_matches(df, args.match_threshold)
    low_tri = identify_low_triangulation(df, args.triangulation_threshold)
    low_inliers = identify_low_inliers(df, args.inlier_threshold)
    slow_frames = identify_high_timing(df, 95)
    
    # Combine all problems
    all_problems = pd.concat([
        tracking_failures,
        low_matches,
        low_tri,
        low_inliers,
        slow_frames
    ], ignore_index=True)
    
    # Remove duplicates (keep first issue type)
    all_problems = all_problems.drop_duplicates(subset=["timestamp"], keep="first") # type: ignore
    
    # Print summary
    print_problem_summary(df, all_problems)
    
    # Save to file
    output_path = args.output if args.output else args.csv_file.parent / f"problems_{args.csv_file.name}"
    save_problem_frames(all_problems, output_path)


if __name__ == '__main__':
    main()
