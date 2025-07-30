# WORKSPACE
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "drake_ros",
    # You will need to get the new sha256 by running bazel.
    # sha256 = "...", 
    strip_prefix = "drake-ros",
    urls = ["https://github.com/Etharialle/drake-ros/releases/tag/0.1-fork"],
)

load("@drake_ros//bazel_ros2_rules/deps:defs.bzl", "add_bazel_ros2_rules_dependencies")

add_bazel_ros2_rules_dependencies()

load("@bazel_ros2_rules//ros2:defs.bzl", "ros2_local_repository")
ros2_local_repository(
    name = "ros2",
    workspaces = ["/opt/ros/jazzy"],
)