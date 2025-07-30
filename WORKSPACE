# WORKSPACE
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "bazel_ros2_rules",
    # You will need to get the new sha256 by running bazel.
    # sha256 = "...", 
    strip_prefix = "drake-ros-0.1-fork/bazel_ros2_rules",
    urls = ["https://github.com/Etharialle/drake-ros/archive/refs/tags/0.1-fork.tar.gz"],
)

load("@bazel_ros2_rules//deps:defs.bzl", "add_bazel_ros2_rules_dependencies")

add_bazel_ros2_rules_dependencies()

load("@bazel_ros2_rules//ros2:defs.bzl", "ros2_local_repository")
ros2_local_repository(
    name = "ros2",
    workspaces = ["/opt/ros/jazzy"],
)