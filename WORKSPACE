# WORKSPACE

workspace(name = "ros_beginner_workspace")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# --- Dependency 1: The entire drake-ros repository via git_repository ---
# This bypasses the http cache and guarantees the patch will be applied.
git_repository(
    name = "drake_ros_rules",
    remote = "https://github.com/RobotLocomotion/drake-ros.git",
    # Pinning to a recent, known-good commit from July 2024.
    commit = "6f5f05d1b819797f2852b789da7b716c589a82de",
    # These patch commands will now run on the fresh clone.
    patch_cmds = [
        "touch bazel_ros2_rules/deps/cpython/BUILD.bazel",
    ],
)

# --- Dependency 2: Load the dependencies for the new rules ---
load("@drake_ros_rules//bazel_ros2_rules/deps:defs.bzl", "add_bazel_ros2_rules_dependencies")
add_bazel_ros2_rules_dependencies()

# --- Dependency 3: Bind your local ROS 2 installation ---
load("@drake_ros_rules//bazel_ros2_rules/ros2:defs.bzl", "ros2_local_repository")
ros2_local_repository(
    name = "ros2",
    workspaces = ["/opt/ros/iron"],
)

# --- Dependency 4: GoogleTest ---
http_archive(
    name = "com_google_googletest",
    sha256 = "b487069925d69341050341071443657366d8e80544c32986847c2d110d0d2105",
    strip_prefix = "googletest-release-1.12.1",
    urls = ["https://github.com/google/googletest/archive/refs/tags/release-1.12.1.zip"],
)