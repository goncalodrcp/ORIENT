
tc = aarch64_toolchain;
if ispc
    save aarch64_toolchain_win64 tc;
else
    save aarch64_toolchain_glnxa64 tc;
end
RTW.TargetRegistry.getInstance('reset');
