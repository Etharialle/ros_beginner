load("@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl", "tool_path")

def _etharialle_cc_toolchain_config_impl(ctx):
    """The implementation of our toolchain configuration rule."""

    # Define the paths to the compiler/linker tools.
    # The names (gcc, ld, ar, etc.) are conventional keys that Bazel understands.
    tool_paths = [
        tool_path(name = "ar", path = "/usr/bin/ar"),
        tool_path(name = "cpp", path = "/usr/bin/clang-cpp-14"),
        tool_path(name = "gcc", path = "/usr/bin/clang++"),
        tool_path(name = "clang", path = "/usr/bin/clang++"),
        tool_path(name = "gcov", path = "llvm-gcov-wrapper.sh"),
        tool_path(name = "ld", path = "/usr/bin/lld"),
        tool_path(name = "nm", path = "/usr/bin/nm"),
        tool_path(name = "objcopy", path = "/usr/bin/objcopy"),
        tool_path(name = "objdump", path = "/usr/bin/objdump"),
        tool_path(name = "strip", path = "/usr/bin/strip"),
    ]
    builtin_include_dirs = [
        "/usr/include/x86_64-linux-gnu",
        "/usr/include",
        # This path is specific to Clang 14.
        "/usr/lib/llvm-14/lib/clang/14.0.0/include",
    ]

        # --- ACTION CONFIGURATIONS ---
    # This section tells Bazel HOW to use the tools listed above.

    # How to compile C/C++ files
    cxx_compile_action = action_config(
        action_name = "c++-compile",
        tools = [tool(path = "gcc")], # Use the tool named 'gcc' (which points to clang++)
    )

    # How to link an executable
    cxx_link_executable_action = action_config(
        action_name = "c++-link-executable",
        tools = [tool(path = "gcc")],
    )

    # How to link a dynamic library (.so) - THIS FIXES YOUR ERROR
    cxx_link_dynamic_library_action = action_config(
        action_name = "c++-link-dynamic-library",
        tools = [tool(path = "gcc")],
    )

    # How to create a static library (.a)
    cxx_archive_action = action_config(
        action_name = "c++-archive",
        tools = [tool(path = "ar")],
    )

    # --- FEATURE FOR COVERAGE ---
    # This adds the --coverage flags ONLY when you run 'bazel coverage'
    coverage_feature = feature(
        name = "coverage",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = [
                    "c++-compile",
                    "c++-link-executable",
                    "c++-link-dynamic-library",
                ],
                flag_groups = [
                    flag_group(
                        flags = [
                            "--coverage",
                            "-fprofile-arcs",
                            "-ftest-coverage",
                        ],
                    ),
                ],
            ),
        ],
    )


    return cc_common.create_cc_toolchain_config_info(
        ctx = ctx,
        tool_paths = tool_paths,
        # These are crucial for telling Bazel where the compiler and host system are.
        host_system_name = "local",
        target_system_name = "local",
        target_cpu = "k8", # A common value for x86_64
        target_libc = "local",
        compiler = "clang",
        abi_version = "local",
        abi_libc_version = "local",
        toolchain_identifier = "etharialle_coverage",
        cxx_builtin_include_directories = builtin_include_dirs,
        action_configs = [
            cxx_compile_action,
            cxx_link_executable_action,
            cxx_link_dynamic_library_action,
            cxx_archive_action,
        ]

    )

etharialle_cc_toolchain_config = rule(
    implementation = _etharialle_cc_toolchain_config_impl,
    attrs = {},
    provides = [CcToolchainConfigInfo],
)