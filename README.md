# Basic ROS2 Functions

## Build System

Using bazel as the build system

### Commands to Use

`bazel build //src/talker:talker`
.bazelrc includes the `--enable_workspace` flag

`bazel run //src/talker:talker`


## Testing Code Coverage

Using `bazel coverage --combined_report=lcov //...`

Need a change to push again