"""A custom C++ toolchain configuration that fixes the gcov/llvm-cov mismatch."""

def _impl(ctx):
    # This creates a toolchain config based on the existing one,
    # but with one crucial override for the 'gcov' tool path.
    return cc_common.create_cc_toolchain_config_info(
        ctx = ctx,
        tool_paths = [
            # THIS IS THE FIX:
            # Define a tool named "gcov" but point it to the llvm-cov binary.
            cc_common.tool_path(
                name = "gcov",
                path = "/usr/bin/llvm-cov",
            ),
        ],
    )

cc_toolchain_config = rule(
    implementation = _impl,
    attrs = {},
    provides = [CcToolchainConfigInfo],
)