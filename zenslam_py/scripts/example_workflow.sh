#!/bin/bash
# Example workflow for analyzing ZenSLAM statistics and comparing detectors

set -e  # Exit on error

echo "=========================================="
echo "ZenSLAM Statistics Analysis Workflow"
echo "=========================================="
echo ""

# Configuration
DATASET_PATH=${1:-"/path/to/dataset"}
OUTPUT_BASE="analysis_output"

if [ ! -d "$DATASET_PATH" ]; then
    echo "Error: Dataset path not found: $DATASET_PATH"
    echo "Usage: $0 <dataset_path>"
    exit 1
fi

# Create output directory
mkdir -p "$OUTPUT_BASE"

echo "Step 1: Running ZenSLAM with different detector configurations..."
echo "-------------------------------------------------------------------"

# Run with FAST+ORB
echo "Running with FAST+ORB detector..."
./zenslam_app \
    --feature FAST \
    --descriptor ORB \
    --matcher BRUTE \
    --folder "$DATASET_PATH" \
    --output "$OUTPUT_BASE/fast_orb"

# Run with ORB+ORB
echo "Running with ORB+ORB detector..."
./zenslam_app \
    --feature ORB \
    --descriptor ORB \
    --matcher BRUTE \
    --folder "$DATASET_PATH" \
    --output "$OUTPUT_BASE/orb_orb"

# Run with SIFT+SIFT (if available)
echo "Running with SIFT+SIFT detector..."
./zenslam_app \
    --feature SIFT \
    --descriptor SIFT \
    --matcher FLANN \
    --folder "$DATASET_PATH" \
    --output "$OUTPUT_BASE/sift_sift"

echo ""
echo "Step 2: Analyzing individual runs..."
echo "-------------------------------------------------------------------"

cd zenslam_py/scripts

# Analyze each run
for detector_dir in "$OUTPUT_BASE"/*/; do
    detector_name=$(basename "$detector_dir")
    csv_file="${detector_dir}frame_data.csv"
    
    if [ -f "$csv_file" ]; then
        echo "Analyzing $detector_name..."
        
        # Identify problem frames
        ./analyze_frame_stats.py "$csv_file" \
            --output "${detector_dir}problems.csv"
        
        # Generate timing breakdown
        ./plot_timing_breakdown.py "$csv_file" \
            --output-dir "${detector_dir}timing_analysis"
        
        # Generate feature metrics
        ./plot_feature_metrics.py "$csv_file" \
            --output-dir "${detector_dir}feature_analysis"
        
        echo ""
    fi
done

echo "Step 3: Comparing detector configurations..."
echo "-------------------------------------------------------------------"

# Compare all detectors
./plot_detector_comparison.py \
    "$OUTPUT_BASE"/fast_orb/frame_data.csv \
    "$OUTPUT_BASE"/orb_orb/frame_data.csv \
    "$OUTPUT_BASE"/sift_sift/frame_data.csv \
    --output-dir "$OUTPUT_BASE/comparison"

echo ""
echo "=========================================="
echo "Analysis Complete!"
echo "=========================================="
echo ""
echo "Results saved to: $OUTPUT_BASE/"
echo ""
echo "Individual analyses:"
echo "  - $OUTPUT_BASE/fast_orb/timing_analysis/"
echo "  - $OUTPUT_BASE/fast_orb/feature_analysis/"
echo "  - $OUTPUT_BASE/fast_orb/problems.csv"
echo "  (and similar for other detectors)"
echo ""
echo "Comparison results:"
echo "  - $OUTPUT_BASE/comparison/detector_comparison.csv"
echo "  - $OUTPUT_BASE/comparison/*.png"
echo ""
echo "Open the PNG files to view visualizations."
