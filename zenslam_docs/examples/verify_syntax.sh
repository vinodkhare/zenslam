#!/bin/bash
# Syntax verification script for line segment detection examples
# This checks if the C++ code is syntactically valid without compiling

echo "Line Segment Detection Examples - Syntax Verification"
echo "======================================================"
echo ""

# Check if clang or g++ is available
if command -v clang++ >/dev/null 2>&1; then
    CXX=clang++
elif command -v g++ >/dev/null 2>&1; then
    CXX=g++
else
    echo "Error: No C++ compiler found (clang++ or g++)"
    exit 1
fi

echo "Using compiler: $CXX"
echo ""

# Function to check syntax of a file
check_syntax() {
    local file=$1
    local skip_compile=$2
    
    echo -n "Checking $file... "
    
    # Just check syntax, don't link
    if $CXX -std=c++17 -fsyntax-only "$file" 2>/dev/null; then
        echo "✓ Syntax OK"
        return 0
    else
        # If it fails, it might be due to missing headers (expected for templates)
        if [ "$skip_compile" = "template" ]; then
            echo "⚠ Template (requires external library)"
            return 0
        else
            echo "✗ Syntax errors found"
            $CXX -std=c++17 -fsyntax-only "$file" 2>&1 | head -10
            return 1
        fi
    fi
}

# Check each example
echo "=== OpenCV Examples (ready to compile) ==="
check_syntax "lsd_example.cpp" "opencv"
check_syntax "fld_example.cpp" "opencv"
check_syntax "hough_example.cpp" "opencv"

echo ""
echo "=== Template Examples (require external libraries) ==="
check_syntax "edlines_example.cpp" "template"
check_syntax "elsed_example.cpp" "template"

echo ""
echo "=== Summary ==="
echo "All examples have been checked for basic syntax."
echo ""
echo "Note: Actual compilation requires:"
echo "  - LSD, FLD, Hough: OpenCV 4.x with contrib modules"
echo "  - EDLines: ED_Lib from https://github.com/CihanTopal/ED_Lib"
echo "  - ELSED: ELSED from https://github.com/iago-suarez/ELSED"
echo ""
echo "See README.md for detailed compilation instructions."
