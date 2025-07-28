#!/bin/bash
# This script acts as a wrapper for llvm-cov gcov
# Bazel will call this script as if it were the "gcov" tool.
# We pass all arguments ("$@") to the real command.
/usr/bin/llvm-cov gcov "$@"