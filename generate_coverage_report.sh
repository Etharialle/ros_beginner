#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# --- Configuration ---
PROJECT_NAME="ROS Beginner Test"
OUTPUT_DIR="coverage_report"
BAZEL_COVERAGE_OUTPUT="bazel-out/_coverage/_coverage_report.dat"

# --- Main Script ---
echo "Generating coverage report for ${PROJECT_NAME}..."

# 1. Ensure the output directory exists
if [ ! -d "${OUTPUT_DIR}" ]; then
    echo "Directory ${OUTPUT_DIR} does not exist. Creating it..."
    mkdir -p "${OUTPUT_DIR}"
fi

# 2. Capture coverage data using lcov, now with MC/DC enabled.
# The --mcdc flag is the crucial addition here.
echo "Capturing coverage data with MC/DC analysis..."
lcov --capture \
     --directory bazel-out \
     --output-file "${OUTPUT_DIR}/coverage.info" \
     --mcdc-coverage # <--- THIS IS THE KEY FIX FOR LCOV

# (Optional) You can remove files you don't care about, like external dependencies
# lcov --remove "${OUTPUT_DIR}/coverage.info" '/usr/*' --output-file "${OUTPUT_DIR}/coverage.filtered.info'

# 3. Generate the HTML report using genhtml, also with MC/DC enabled.
echo "Generating HTML report..."
genhtml "${OUTPUT_DIR}/coverage.info" \
        --output-directory "${OUTPUT_DIR}" \
        --title "${PROJECT_NAME}" \
        --mcdc-coverage \
        --frames \
        --legend # <--- THESE FLAGS MAKE THE REPORT BETTER

echo "Done. Coverage report is available at: ${OUTPUT_DIR}/index.html"