#!/bin/bash
# THIS IS THE FINAL, WORKING SCRIPT

set -e

# --- Configuration ---
PROJECT_NAME="ESE Coverage"
OUTPUT_DIR="coverage_report"
GCOV_TOOL="/usr/bin/gcov-14"

# The path to our source code within the workspace
SOURCE_SUBDIR="src" 

# --- Main Script ---
echo "Generating coverage report for ${PROJECT_NAME}..."
EXEC_ROOT=$(bazel info execution_root)
WORKSPACE_ROOT=$(bazel info workspace)
echo "Using Bazel execution root: ${EXEC_ROOT}"
echo "Using project workspace: ${WORKSPACE_ROOT}"

echo "Searching for .gcda files in the execution root..."
readarray -d '' gcda_files < <(find "${EXEC_ROOT}" -name '*.gcda' -print0)

if [ ${#gcda_files[@]} -eq 0 ]; then
    echo "FATAL ERROR: No .gcda files found."
    exit 1
fi
echo "Found ${#gcda_files[@]} .gcda file(s). Proceeding..."

mkdir -p "${OUTPUT_DIR}"

# 1. Capture ALL coverage data initially
echo "Capturing all coverage data..."
lcov --capture \
     --directory "${EXEC_ROOT}" \
     --base-directory "${EXEC_ROOT}" \
     --output-file "${OUTPUT_DIR}/coverage.unfiltered.info" \
     --gcov-tool "${GCOV_TOOL}" \
     --mcdc-coverage \
     --ignore-errors path,source # Ignore errors from external libs we are about to remove

# 2. FILTER the data to keep ONLY our source files.
#    This removes all the external ROS2 library coverage and their errors.
echo "Filtering report to keep only '${SOURCE_SUBDIR}/*'..."
lcov --extract "${OUTPUT_DIR}/coverage.unfiltered.info" \
     "${WORKSPACE_ROOT}/${SOURCE_SUBDIR}/*" \
     --output-file "${OUTPUT_DIR}/coverage.info"

# 3. Generate the final HTML report from the CLEAN data.
echo "Generating final HTML report..."
genhtml "${OUTPUT_DIR}/coverage.info" \
        --output-directory "${OUTPUT_DIR}" \
        --title "${PROJECT_NAME}" \
        --mcdc-coverage --frames --legend

echo "Done. Coverage report is available at: ${OUTPUT_DIR}/index.html"