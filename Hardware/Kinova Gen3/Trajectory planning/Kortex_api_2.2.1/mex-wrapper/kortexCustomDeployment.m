function kortexCustomDeployment(modelName)
% Deploys the generated ROS2 package onto a target and runs it on target 
    
    %% ZIP Files that will be sent to the target
    disp("Transferring files to target folder...");
    
    folderStruct = RTW.getBuildDir(modelName);
    
    TargetRootFolder = fullfile(fileparts(folderStruct.BuildDirectory),'src',modelName);
    rootFolder = folderStruct.CodeGenFolder;
    
    apiIncludeFolder = fullfile(rootFolder, '..', 'simplified_api','kortex_api','cpp','linux_gcc_x86-64','include');
    simplifiedApiLibFolder = fullfile(rootFolder, '..', 'simplified_api','install','package','lib','Release');
    
    Ros2Folder = fullfile(rootFolder, 'src','example_model');
    Ros2IncludeFolder = fullfile(Ros2Folder,'include');
    Ros2LibFolder = fullfile(Ros2Folder, 'lib');
    % Libs are copied directly in the package folder
    
    copyfile(...
        fullfile(apiIncludeFolder, 'client'), ...
        fullfile(Ros2IncludeFolder, 'client'));
    copyfile(...
        fullfile(apiIncludeFolder, 'client_stubs'), ...
        fullfile(Ros2IncludeFolder, 'client_stubs'));
    copyfile(...
        fullfile(apiIncludeFolder, 'common'), ...
        fullfile(Ros2IncludeFolder, 'common'));
    copyfile(...
        fullfile(apiIncludeFolder, 'google'), ...
        fullfile(Ros2IncludeFolder, 'google'));
    copyfile(...
        fullfile(apiIncludeFolder, 'messages'), ...
        fullfile(Ros2IncludeFolder, 'messages'));
    
    mkdir(Ros2LibFolder);
    copyfile(...
        fullfile(simplifiedApiLibFolder, 'libSimplifiedApi.a'), ...
        fullfile(Ros2LibFolder, 'libSimplifiedApi.a'));
        
     
    zipFile = tempname + ".zip";
    zip(zipFile, TargetRootFolder)
    
    %% Connect to Target

    %%% Get Target User Information %%%
    username = char(rosTargetPreferences.getUserName());
    password = char(rosTargetPreferences.getPassword());
    address  = char(rosTargetPreferences.getIpAddress());
    ros2_root_dir = char(rosTargetPreferences.getRos2RootDir());
    ros2_workspace_path = char(rosTargetPreferences.getRos2WorkspacePath());
    ros2_domain_id = uint8(rosTargetPreferences.getRos2DomainId());
    
    device = rosdevice(address, username, password); % <---- connects to specific ROS device

    %% Create local function for sending system commands to ROS device
    function r = sendCommand(c)
        r = system(device, char(c));
    end
    
    %% Send ZIP File
    onTargetFolder = convertCharsToStrings(ros2_workspace_path) + '/src';
    onTargetDestination = onTargetFolder + "/" + modelName + ".zip";
    
    putFile(device, char(zipFile), char(onTargetDestination));    
    
    %% Unpack ZIP file on target
    sendCommand("cd " + onTargetFolder + "; rm -rf " + modelName); % Remove existing folder
    sendCommand("cd " + onTargetFolder + "; unzip -o " + modelName + ".zip"); % Unzip file
    sendCommand("cd " + onTargetFolder + "; rm -f " + modelName + ".zip"); % Remove ZIP file
    
    %% Run compilation command on target
    % NOTE: Need to pull folder names from user preferences
    disp("Building generated ROS 2 node...");
    sendCommand("cd " + onTargetFolder + "/..; source " + ros2_root_dir + "/setup.bash; colcon build");
    
    %% Run the generated ROS 2 node
    % NOTE: Need to pull folder names and domain ID from user preferences
    disp("Running generated ROS 2 node...");
    sendCommand("export ROS_DOMAIN_ID=" + int2str(ros2_domain_id) + "; source " + ros2_workspace_path + "/install/setup.bash;" ...
                           + "ros2 run " + lower(modelName) + " " + modelName);
end

