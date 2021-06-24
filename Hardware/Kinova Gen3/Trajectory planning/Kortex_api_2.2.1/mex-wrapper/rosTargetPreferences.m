classdef rosTargetPreferences
% Defines the ROS2 Target preferences for deployment
%   Used to store and retrieve parameters used to deploy 
%   on a ROS2 target during post code generation step.
%   Change the default parameters to be able to deploy 
%   on a specific target 

    % Here, set the target connection parameters
    properties (Constant)
        DefaultIpAddress = "10.0.2.62";
        DefaultUserName = "nvidia";
        DefaultPassword = "nvidia";
        DefaultRos2RootDir = "/opt/ros/bouncy";
        DefaultRos2WorkspacePath = "/home/nvidia/stash/workspace/";
        DefaultRos2DomainId = 25;
    end
    
    properties (Constant, GetAccess=private)
        PreferenceGroupName = "Kortex_ROS_Target_Pref";
        UserNameProp = 'username';
        PasswordProp = 'password';
        IpAddressProp = 'ipaddress';
        Ros2RootDirProp = 'ros2rootdir';
        Ros2WorkspacePathProp = 'ros2workspace';
        Ros2DomainIdProp = 'ros2domainid';
    end
    
    methods (Static)
        function username = getUserName()
            s = rosTargetPreferences.getpref();
            username = s.(rosTargetPreferences.UserNameProp);
        end
        
        function password = getPassword()
            s = rosTargetPreferences.getpref();
            password = s.(rosTargetPreferences.PasswordProp);
        end
        
        function address = getIpAddress()
            s = rosTargetPreferences.getpref();
            address = s.(rosTargetPreferences.IpAddressProp);
        end

        function ros2rootdir = getRos2RootDir()
            s = rosTargetPreferences.getpref();
            ros2rootdir = s.(rosTargetPreferences.Ros2RootDirProp);
        end

        function ros2workspacepath = getRos2WorkspacePath()
            s = rosTargetPreferences.getpref();
            ros2workspacepath = s.(rosTargetPreferences.Ros2WorkspacePathProp);
        end

        function ros2domainid = getRos2DomainId()
            s = rosTargetPreferences.getpref();
            ros2domainid = s.(rosTargetPreferences.Ros2DomainIdProp);
        end
        
        function p = getpref()
            rosTargetPreferences.ensurePreferencesExist();
            p = getpref(rosTargetPreferences.PreferenceGroupName);
        end
        
        function setUserName(value)
            rosTargetPreferences.ensurePreferencesExist();
            setpref(rosTargetPreferences.PreferenceGroupName, ...
                       rosTargetPreferences.UserNameProp, ...
                       value);
        end
        
        function setPassword(value)
            rosTargetPreferences.ensurePreferencesExist();
            setpref(rosTargetPreferences.PreferenceGroupName, ...
                       rosTargetPreferences.PasswordProp, ...
                       value);
        end
        
        function setIpAddress(value)
            rosTargetPreferences.ensurePreferencesExist();
            setpref(rosTargetPreferences.PreferenceGroupName, ...
                       rosTargetPreferences.IpAddressProp, ...
                       value);
        end

        function setRos2RootDir(value)
            rosTargetPreferences.ensurePreferencesExist();
            setpref(rosTargetPreferences.PreferenceGroupName, ...
                       rosTargetPreferences.Ros2RootDirProp, ...
                       value);
        end

        function setRos2WorkspacePath(value)
            rosTargetPreferences.ensurePreferencesExist();
            setpref(rosTargetPreferences.PreferenceGroupName, ...
                       rosTargetPreferences.Ros2WorkspacePathProp, ...
                       value);
        end

        function setRos2DomainId(value)
            rosTargetPreferences.ensurePreferencesExist();
            setpref(rosTargetPreferences.PreferenceGroupName, ...
                       rosTargetPreferences.Ros2DomainIdProp, ...
                       value);
        end
        
        function removepref()
            if ispref(rosTargetPreferences.PreferenceGroupName)
                rmpref(rosTargetPreferences.PreferenceGroupName);
            end
        end
        
        function setpref(prefArgs)
            arguments
                prefArgs.UserName
                prefArgs.Password
                prefArgs.IpAddress
                prefArgs.Ros2RootDir
                prefArgs.Ros2WorkspacePath
                prefArgs.Ros2DomainId
            end
            
            m = containers.Map({'UserName', 'Password', 'IpAddress', 'Ros2RootDir', 'Ros2WorkspacePath', 'Ros2DomainId'}, ...
                               {rosTargetPreferences.UserNameProp, ...
                                rosTargetPreferences.PasswordProp, ...
                                rosTargetPreferences.IpAddressProp, ...
                                rosTargetPreferences.Ros2RootDirProp, ...
                                rosTargetPreferences.Ros2WorkspacePathProp, ...
                                rosTargetPreferences.Ros2DomainIdProp ...
                                });
                           
            rosTargetPreferences.ensurePreferencesExist();
            
            for n = fieldnames(prefArgs)'
                name = n{1};
                setpref(rosTargetPreferences.PreferenceGroupName, ...
                                m(name), ...
                                prefArgs.(name));
            end
                
            
        end
        
            
    end
    
    methods (Static, Access=private)
        function ensurePreferencesExist()
            if ~ispref(rosTargetPreferences.PreferenceGroupName)
                rosTargetPreferences.setDefaultPreferences;
            end
        end
        
        function setDefaultPreferences()
            setpref(rosTargetPreferences.PreferenceGroupName, ...
                {rosTargetPreferences.UserNameProp, ...
                 rosTargetPreferences.PasswordProp, ...
                 rosTargetPreferences.IpAddressProp, ...
                 rosTargetPreferences.Ros2RootDirProp, ...
                 rosTargetPreferences.Ros2WorkspacePathProp, ...
                 rosTargetPreferences.Ros2DomainIdProp, ...
                }, ...
                {rosTargetPreferences.DefaultUserName, ...
                 rosTargetPreferences.DefaultPassword, ...
                 rosTargetPreferences.DefaultIpAddress, ...
                 rosTargetPreferences.DefaultRos2RootDir, ...
                 rosTargetPreferences.DefaultRos2WorkspacePath, ...
                 rosTargetPreferences.DefaultRos2DomainId, ...
                 });
            
        end
    end
    
end