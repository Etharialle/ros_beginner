# Basic ROS2 Functions

## Build System

Using bazel as the build system

### Commands to Use

`bazel build //src/talker:talker`
.bazelrc includes the `--enable_workspace` flag

`bazel run //src/talker:talker`

### Code Coverage and Unit Tests

Using a docker image and gtest to test
Requires Bazel 8.3.1 to work