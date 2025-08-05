#!/bin/bash

COVERAGE_DIR="coverage_report"

if [ ! -d "$COVERAGE_DIR" ]; then
  echo "Directory $COVERAGE_DIR does not exist. Creating it..."
  mkdir -p "$COVERAGE_DIR"
else
  echo "Directory $COVERAGE_DIR already exists."
fi

# Generate the HTML report
genhtml bazel-out/_coverage/_coverage_report.dat --output-directory ${COVERAGE_DIR}