function kortexPostGeneration(h)
% Post Code Generation function hook to setup custom buildInfo parameters 
%   This will be called right before deploying the package, and is used to
%   apply last-minute build info parameters needed for deployment.
%   We should put here build info settings that are needed for deployment, 
%   but not for build itself
%   For settings needed for the build itself, see kortex_ros2_build.m
   
    %% Add Kinova source and header files 
    buildInfo = h.BuildInfo;
    

    %% Link to Simplified API lib on target
    libName = 'libSimplifiedApi.a';
    libFolder = './lib';
    libPriority = '';
    libPreCompiled = true;
    libLinkOnly = true;

    buildInfo.addLinkObjects(libName, libFolder, libPriority, libPreCompiled, libLinkOnly);
    buildInfo.addIncludePaths(libFolder);
    
    %% Call standard Post Code Generation Command
    codertarget.postCodeGenHookCommand(h);  % <--- It's not clear to me where exactly this should be called.
    
end