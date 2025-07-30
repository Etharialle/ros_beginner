# WORKSPACE
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "drake_ros",
    # You will need to get the new sha256 by running bazel.
    # sha256 = "...", 
    strip_prefix = "drake-ros-6f5f05d1b819797f2852b789da7b716c589a82de",
    urls = ["https://github.com/RobotLocomotion/drake-ros/archive/6f5f05d1b819797f2852b789da7b716c589a82de.tar.gz"],
)

load("@drake_ros//bazel_ros2_rules/deps:defs.bzl", "add_bazel_ros2_rules_dependencies")

add_bazel_ros2_rules_dependencies()

load("@bazel_ros2_rules//ros2:defs.bzl", "ros2_local_repository")
ros2_local_repository(
    name = "ros2",
    workspaces = ["/opt/ros/jazzy"],
)