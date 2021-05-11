classdef kortex_ros2_build < coder.ExternalDependency & handle
% Encapsulates Build settings needed to build an interface using the Kortex Simplified API
%   Classes inheriting from this will benefit from this interface
%   For settings to apply after building, see kortexPostGeneration.m
    
    methods (Static)
        function name = getDescriptiveName()
            name = 'kortex_ros2_build';
        end
        
        function supported = isSupportedContext(context)
            myTarget = {'mex','rtw'};
            if  context.isCodeGenTarget(myTarget)
                supported = true;
            else
                error('API only supported for mex, lib, exe, dll');
            end
        end
        
        function updateBuildInfo(buildInfo, context)
        % Updates the build info with Matlab Simplified API requirements
        %   This function sets the include paths, files and link objects
        %   Add here all needed configurations necessary for building the
        %   model interfacing with the Kortex Simplified API

            workingdir = fileparts(mfilename('fullpath'));
            
            buildInfo.addDefines('_OS_UNIX');
            
            apiLibIncludesPath = fullfile(workingdir, '..', 'simplified_api', 'install', 'package', 'lib', 'Release');
            apiIncludesPath = fullfile(workingdir, '..', 'simplified_api', 'install', 'package', 'include');
            wrapperIncludesPath = fullfile(workingdir, 'include');
            
            wrapperSourcesPath = fullfile(workingdir, 'src');

            kortexIncludesPath = fullfile(workingdir, '..', 'simplified_api', 'kortex_api', 'cpp', 'linux_gcc_x86-64', 'include');
            
            % Update buildInfo
            buildInfo.addSourcePaths( wrapperSourcesPath );
            buildInfo.addSourceFiles('kortexApiWrapper.cpp');
            buildInfo.addSourceFiles('proto_converter.cpp');

            buildInfo.addIncludePaths( wrapperIncludesPath );
            buildInfo.addIncludePaths( apiLibIncludesPath );
            
            buildInfo.addIncludePaths( '../include');
            
            buildInfo.addIncludePaths('./include');
            buildInfo.addIncludePaths('./include/client');
            buildInfo.addIncludePaths('./include/client_stubs');
            buildInfo.addIncludePaths('./include/common');
            buildInfo.addIncludePaths('./include/messages');
            
            buildInfo.addIncludePaths( apiIncludesPath );

            buildInfo.addIncludePaths( kortexIncludesPath );
            
            buildInfo.addIncludeFiles('proto_converter.h');
            buildInfo.addIncludeFiles('ActionNotificationManager.h');
            buildInfo.addIncludeFiles('ArmBaseApi.h');
            buildInfo.addIncludeFiles('IArmBaseApi.h');
            buildInfo.addIncludeFiles('SimplifiedApi.h');
            buildInfo.addIncludeFiles('TrajectoryNotificationManager.h');
            buildInfo.addIncludeFiles('TrajectoryState.h');

        end
    end
end


