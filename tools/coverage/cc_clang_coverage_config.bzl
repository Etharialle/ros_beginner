load("@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl", "feature", "tool_path", "action_config", "flag_group", "tool", "with_feature_set")

CcToolchainConfigInfo = provider()

def clang_cc_toolchain_config(ctx):
    return CcToolchainConfigInfo(
        toolchain_identifier = "cc-clang-coverage-toolchain",
        host_system_name = "local",
        target_system_name = "local",
        target_cpu = "k8",
        target_libc = "local",
        compiler = "clang",
        abi_version = "local",
        abi_libc_version = "local",
        tool_paths = [
            tool_path(name = "ar", path = "/usr/bin/ar"),
            tool_path(name = "cpp", path = "/usr/bin/clang-cpp"),
            tool_path(name = "gcc", path = "/usr/bin/clang"),
            tool_path(name = "gcov", path = "llvm-gcov-wrapper.sh"),
            tool_path(name = "ld", path = "/usr/bin/ld"),
            tool_path(name = "nm", path = "/usr/bin/nm"),
            tool_path(name = "objcopy", path = "/usr/bin/objcopy"),
            tool_path(name = "objdump", path = "/usr/bin/objdump"),
            tool_path(name = "strip", path = "/usr/bin/strip"),
        ],
        features = [],
    )


#
#
#def _impl(ctx):
#    tool_paths = [ # NEW
#        tool_path(name = "gcov", path = "llvm-gcov-wrapper.sh"),
#        tool_path(
#            name = "gcc",  # Compiler is referenced by the name "gcc" for historic reasons.
#            path = "/usr/bin/clang",
#        ),
#        tool_path(
#            name = "ld",
#            path = "/usr/bin/ld",
#        ),
#        tool_path(
#            name = "ar",
#            path = "/usr/bin/ar",
#        ),
#        tool_path(
#            name = "cpp",
#            path = "/bin/false",
#        ),
#        tool_path(
#            name = "gcov",
#            path = "/bin/false",
#        ),
#        tool_path(
#            name = "nm",
#            path = "/bin/false",
#        ),
#        tool_path(
#            name = "objdump",
#            path = "/bin/false",
#        ),
#        tool_path(
#            name = "strip",
#            path = "/bin/false",
#        ),
#    ]
#    
#    
#    return cc_common.create_cc_toolchain_config_info(
#        ctx = ctx,
#        toolchain_identifier = "k8-toolchain",
#        host_system_name = "local",
#        target_system_name = "local",
#        target_cpu = "k8",
#        target_libc = "unknown",
#        compiler = "clang",
#        abi_version = "unknown",
#        abi_libc_version = "unknown",
#    )
#
#cc_toolchain_config = rule(
#    implementation = _impl,
#    attrs = {},
#    provides = [CcToolchainConfigInfo],
#)