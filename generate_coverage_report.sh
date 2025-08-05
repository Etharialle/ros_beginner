#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# --- Configuration ---
PROJECT_NAME="ROS Beginner Test"
OUTPUT_DIR="coverage_report"
#BAZEL_COVERAGE_OUTPUT="bazel-out/_coverage/_coverage_report.dat"
COMBINED_INFO_FILE="${OUTPUT_DIR}/coverage.info"

# --- Main Script ---
echo "Generating coverage report for ${PROJECT_NAME}..."
EXEC_ROOT=$(bazel info execution_root)
echo "Using Bazel's execution root: ${EXEC_ROOT}"

###
#echo "Searching for .gcda files in the execution root..."
## Use an array to handle spaces in paths and `find`'s null-delimited output
#readarray -d '' gcda_files < <(find "${EXEC_ROOT}" -name '*.gcda' -print0)

echo "Searching for pre-generated 'coverage.dat' files..."
readarray -d '' dat_files < <(find "${EXEC_ROOT}" -name 'coverage.dat' -print0)

#if [ ${#gcda_files[@]} -eq 0 ]; then
#    echo "----------------------------------------------------------------"
#    echo "FATAL ERROR: No .gcda files found."
#    echo "This means the 'bazel coverage' command either failed or did not generate any coverage data."
#    echo "Please run the following command first to generate the necessary files:"
#    echo "  bazel coverage --copt=\"-fcondition-coverage\" //..."
#    echo "----------------------------------------------------------------"
#    exit 1
#fi
#echo "Found ${#gcda_files[@]} .gcda file(s). Proceeding with report generation."

if [ ${#dat_files[@]} -eq 0 ]; then
    echo "----------------------------------------------------------------"
    echo "FATAL ERROR: No 'coverage.dat' files found."
    echo "This means the 'bazel coverage' command did not produce any test-specific coverage reports."
    echo "Please ensure the test ran successfully and was not cached."
    echo "----------------------------------------------------------------"
    exit 1
fi
echo "Found ${#dat_files[@]} coverage.dat file(s). Merging them into a single report."



# 1. Ensure the output directory exists
if [ ! -d "${OUTPUT_DIR}" ]; then
    echo "Directory ${OUTPUT_DIR} does not exist. Creating it..."
    mkdir -p "${OUTPUT_DIR}"
fi

# 2. Capture coverage data using lcov, now with MC/DC enabled.
# The --mcdc flag is the crucial addition here.
#echo "Capturing coverage data with MC/DC analysis..."
#lcov --capture \
#     --directory "${EXEC_ROOT}" \
#     --base-directory "${EXEC_ROOT}" \
#     --output-file "${OUTPUT_DIR}/coverage.info" \
#     --mcdc-coverage # <--- THIS IS THE KEY FIX FOR LCOV


  lcov_merge_cmd="lcov"
for file in "${dat_files[@]}"; do
    lcov_merge_cmd+=" -a \"$file\""
done
lcov_merge_cmd+=" -o \"${COMBINED_INFO_FILE}\""

echo "Executing merge command..."
eval "$lcov_merge_cmd"

# (Optional) You can remove files you don't care about, like external dependencies
# lcov --remove "${OUTPUT_DIR}/coverage.info" '/usr/*' --output-file "${OUTPUT_DIR}/coverage.filtered.info'

# 3. Generate the HTML report using genhtml, also with MC/DC enabled.
echo "Generating HTML report..."
genhtml "${COMBINED_INFO_FILE}" \
        --output-directory "${OUTPUT_DIR}" \
        --title "${PROJECT_NAME}" \
        --mcdc-coverage \
        --frames \
        --legend # <--- THESE FLAGS MAKE THE REPORT BETTER

echo "Done. Coverage report is available at: ${OUTPUT_DIR}/index.html"