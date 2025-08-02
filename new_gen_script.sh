#!/bin/bash
set -e

PROJECT_NAME="ESE Coverage"
OUTPUT_DIR="coverage_report"
GCOV_TOOL="/usr/bin/gcov-14"

EXEC_ROOT=$(bazel info execution_root)
WORKSPACE_ROOT=$(bazel info workspace)

echo "Searching for .gcda files in ${EXEC_ROOT}..."
readarray -d '' gcda_files < <(find "${EXEC_ROOT}" -name '*.gcda' -print0)
if [ ${#gcda_files[@]} -eq 0 ]; then
    echo "FATAL ERROR: No .gcda files found."
    exit 1
fi
echo "Found ${#gcda_files[@]} .gcda file(s)."

mkdir -p "${OUTPUT_DIR}"

echo "Capturing all coverage data..."
lcov --capture \
     --directory "${EXEC_ROOT}" \
     --base-directory "${EXEC_ROOT}" \
     --output-file "${OUTPUT_DIR}/coverage.unfiltered.info" \
     --gcov-tool "${GCOV_TOOL}" \
     --mcdc-coverage \
     --ignore-errors path,source

echo "Filtering report to keep only your project's code..."
lcov --extract "${OUTPUT_DIR}/coverage.unfiltered.info" \
     "${WORKSPACE_ROOT}/src/*" \
     "${WORKSPACE_ROOT}/test/*" \
     --output-file "${OUTPUT_DIR}/coverage.info"

echo "Generating final HTML report..."
genhtml "${OUTPUT_DIR}/coverage.info" \
        --output-directory "${OUTPUT_DIR}" \
        --title "${PROJECT_NAME}" \
        --mcdc-coverage --frames --legend

echo "Done. Report is at ${OUTPUT_DIR}/index.html"