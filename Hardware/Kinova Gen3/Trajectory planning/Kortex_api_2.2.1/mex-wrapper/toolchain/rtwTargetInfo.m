function rtwTargetInfo(tr)
%RTWTARGETINFO Register toolchain
tr.registerTargetInfo(@loc_createToolchain);

end

%--------------------------------------------------------------------------
function config = loc_createToolchain
rootDir = fileparts(mfilename('fullpath'));
config = coder.make.ToolchainInfoRegistry; % initialize
archName = computer('arch');

% aarch64-gnu-linux
config(1).Name               = 'Arm 64 - C/C++ Compiler';
config(1).Alias              = ['AARCH64-', upper(archName)];
config(1).TargetHWDeviceType = {'ARMCortex'};
if ispc
    config(1).FileName       = fullfile(rootDir, 'aarch64_toolchain_win64.mat');
else
    config(1).FileName       = fullfile(rootDir, 'aarch64_toolchain_glnxa64.mat');
end

config(1).Platform           = {archName};

end
% [EOF]
