# WORKSPACE

# Define a name for your workspace
workspace(name = "ros_beginner_workspace")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# --- Dependency 1: C++ Rules ---
# A common dependency for many C++ based rulesets.
http_archive(
    name = "rules_cc",
    sha256 = "625473a393d0dd32f5838d2f5b5f6b2158869b360b38c0f592d3e4492a5a544f",
    strip_prefix = "rules_cc-0972b212f451f80168e27c8d98d248de30c4e793",
    urls = ["https://github.com/bazelbuild/rules_cc/archive/0972b212f451f80168e27c8d98d248de30c4e793.zip"],
)

# --- Dependency 2: Protobuf Rules ---
# A transitive dependency of rules_ros2 that was causing your error.
# Defining it explicitly makes the build more robust.
http_archive(
    name = "com_google_protobuf",
    sha256 = "e13ca6c2f1522924b8482f3b3a482427d0589ff8ea251088f7e39f4713236053",
    strip_prefix = "protobuf-21.7",
    urls = ["https://github.com/protocolbuffers/protobuf/archive/v21.7.zip"],
)
# This function loads Protobuf's own dependencies, fixing your error.
load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")
protobuf_deps()

# --- Dependency 3: ROS 2 Rules ---
# Using a stable, versioned release instead of the 'main' branch.
#http_archive(
#    name = "com_github_mvukov_rules_ros2",
#    sha256 = "6130e8f61c1343fab7809e933c9faf47ab1e9fd4",
#    strip_prefix = "rules_ros2-0.5.0",
#    urls = ["https://github.com/mvukov/rules_ros2/archive/6130e8f61c1343fab7809e933c9faf47ab1e9fd4.zip"],
#)

git_repository(
    name = "com_github_mvukov_rules_ros2",
    remote = "https://github.com/mvukov/rules_ros2.git",
    #branch = "main"
    commit = "6130e8f61c1343fab7809e933c9faf47ab1e9fd4"
)
# This function points Bazel to your system's ROS 2 installation.
load(
    "@com_github_mvukov_rules_ros2//ros2:repositories.bzl",
    "ros2_repositories",
)
ros2_repositories(
    distro_name = "iron", # Make sure this matches your Dockerfile and CI
    ament_prefix_path = "/opt/ros/iron",
)

# --- Dependency 4: GoogleTest ---
# Required for your `ros2_cpp_test` target.
http_archive(
    name = "googletest",
    sha256 = "b487069925d69341050341071443657366d8e80544c32986847c2d110d0d2105",
    strip_prefix = "googletest-release-1.12.1",
    urls = ["https://github.com/google/googletest/archive/refs/tags/release-1.12.1.zip"],
)