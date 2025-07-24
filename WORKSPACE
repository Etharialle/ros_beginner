workspace(name = "test_nodes")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")



http_archive(
    name = "hedron_compile_commands",

    # Replace the commit hash (0e990032f3c5a866e72615cf67e5ce22186dcb97) in both places (below) with the latest (https://github.com/hedronvision/bazel-compile-commands-extractor/commits/main), rather than using the stale one here.
    # Even better, set up Renovate and let it do the work for you (see "Suggestion: Updates" in the README).
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/refs/heads/main.tar.gz",
    strip_prefix = "bazel-compile-commands-extractor-main",
    # When you first run this tool, it'll recommend a sha256 hash to put here with a message like: "DEBUG: Rule 'hedron_compile_commands' indicated that a canonical reproducible form can be obtained by modifying arguments sha256 = ..."
)
load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")
hedron_compile_commands_setup()
load("@hedron_compile_commands//:workspace_setup_transitive.bzl", "hedron_compile_commands_setup_transitive")
hedron_compile_commands_setup_transitive()
load("@hedron_compile_commands//:workspace_setup_transitive_transitive.bzl", "hedron_compile_commands_setup_transitive_transitive")
hedron_compile_commands_setup_transitive_transitive()
load("@hedron_compile_commands//:workspace_setup_transitive_transitive_transitive.bzl", "hedron_compile_commands_setup_transitive_transitive_transitive")
hedron_compile_commands_setup_transitive_transitive_transitive()


# This import is relevant for these examples and this (rules_ros2) repository.
# local_repository(
#     name = "com_github_mvukov_rules_ros2",
#     path = "..",
# )

# In a normal workflow, you would typically import rules_ros2 into your
# (mono)repo as follows:
# http_archive(
#     name = "com_github_mvukov_rules_ros2",
#     # Here you can use e.g. sha256sum cli utility to compute the sha sum.
#     sha256 = "<sha sum of the .tar.gz archive below>",
#     strip_prefix = "rules_ros2-main",
#     url = "https://github.com/mvukov/rules_ros2/archive/refs/heads/main.zip",
# )

git_repository(
    name = "com_github_mvukov_rules_ros2",
    remote = "https://github.com/mvukov/rules_ros2.git",
    branch = "main"
)



load("@com_github_mvukov_rules_ros2//repositories:repositories.bzl", "ros2_repositories", "ros2_workspace_repositories")

ros2_workspace_repositories()

ros2_repositories()

load("@com_github_mvukov_rules_ros2//repositories:deps.bzl", "ros2_deps")

ros2_deps()

load("@rules_python//python:repositories.bzl", "py_repositories", "python_register_toolchains")

py_repositories()

python_register_toolchains(
    name = "rules_ros2_python",
    python_version = "3.10",
)

load("@rules_python//python:pip.bzl", "pip_parse")

pip_parse(
    name = "rules_ros2_pip_deps",
    python_interpreter_target = "@rules_ros2_python_host//:python",
    requirements_lock = "@com_github_mvukov_rules_ros2//:requirements_lock.txt",
)

load(
    "@rules_ros2_pip_deps//:requirements.bzl",
    install_rules_ros2_pip_deps = "install_deps",
)

install_rules_ros2_pip_deps()

# Below is an optional setup for Rust support for ROS 2.

load("@com_github_mvukov_rules_ros2//repositories:rust_setup_stage_1.bzl", "rust_setup_stage_1")

rust_setup_stage_1()

load("@com_github_mvukov_rules_ros2//repositories:rust_setup_stage_2.bzl", "rust_setup_stage_2")

rust_setup_stage_2()

load("@com_github_mvukov_rules_ros2//repositories:rust_setup_stage_3.bzl", "rust_setup_stage_3")

rust_setup_stage_3()

load("@com_github_mvukov_rules_ros2//repositories:rust_setup_stage_4.bzl", "rust_setup_stage_4")

rust_setup_stage_4()