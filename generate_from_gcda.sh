#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# --- Configuration ---
PROJECT_NAME="ESE Coverage"
OUTPUT_DIR="coverage_report"
GCOV_TOOL="/usr/bin/gcov-14"

# --- Main Script ---
echo "Generating coverage report for ${PROJECT_NAME}..."

# 1. Ask Bazel for the true location where build artifacts are generated.
#    This is the single most important fix.
EXEC_ROOT=$(bazel info execution_root)
echo "Using Bazel's execution root: ${EXEC_ROOT}"

# 2. PRE-FLIGHT CHECK: Verify that coverage files were actually generated.
echo "Searching for .gcda files in the execution root..."
# Use an array to handle spaces in paths and `find`'s null-delimited output
readarray -d '' gcda_files < <(find "${EXEC_ROOT}" -name '*.gcda' -print0)

if [ ${#gcda_files[@]} -eq 0 ]; then
    echo "----------------------------------------------------------------"
    echo "FATAL ERROR: No .gcda files found."
    echo "This means the 'bazel coverage' command either failed or did not generate any coverage data."
    echo "Please run the following command first to generate the necessary files:"
    echo "  bazel coverage --copt=\"-fcondition-coverage\" //..."
    echo "----------------------------------------------------------------"
    exit 1
fi
echo "Found ${#gcda_files[@]} .gcda file(s). Proceeding with report generation."

# 3. Ensure the output directory exists
if [ ! -d "${OUTPUT_DIR}" ]; then
    echo "Directory ${OUTPUT_DIR} does not exist. Creating it..."
    mkdir -p "${OUTPUT_DIR}"
fi

# 4. Capture coverage data using lcov, pointing it to the execution root.
#    --base-directory tells lcov to remove the long EXEC_ROOT prefix from file paths,
#    making the final report clean (e.g., "src/ese.cpp" instead of a huge path).
echo "Capturing coverage data with MC/DC analysis..."
lcov --capture \
     --directory "${EXEC_ROOT}" \
     --base-directory "${EXEC_ROOT}" \
     --output-file "${OUTPUT_DIR}/coverage.info" \
     --gcov-tool "${GCOV_TOOL}" \
     --mcdc

# 5. Generate the HTML report using genhtml.
echo "Generating HTML report..."
genhtml "${OUTPUT_DIR}/coverage.info" \
        --output-directory "${OUTPUT_DIR}" \
        --title "${PROJECT_NAME}" \
        --mcdc \
        --frames \
        --legend

echo "Done. Coverage report is available at: ${OUTPUT_DIR}/index.html"