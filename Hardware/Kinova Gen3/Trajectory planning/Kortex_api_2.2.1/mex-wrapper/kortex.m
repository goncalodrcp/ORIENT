classdef kortex < matlab.System & matlab.system.mixin.Propagates ...
        & kortex_ros2_build & matlab.system.mixin.CustomIcon
% Defines a Simulink System Object to encapsulate Kinova Simplified API
%   This class defines protocols for interfacing between Simulink buses
%   and Kortex Simplified API. 


    properties (Nontunable)
        %IP address
        ip_address = '192.168.1.10';
        
        %Joint actuator count
        nbrJointActuators = 7;

        %Vision flag
        hasVision = true;
        
        %Tool actuator count
        nbrToolActuators = 1;
        
        %Username
        user = 'admin';
        
        %Password
        password = 'admin';

        %Session timeout in ms
        session_connection_timeout = 60000;
        
        %Control timeout in ms
        session_ctrl_timeout = 10000;
    end
   
    properties (Access = private)

        %Handle to the robot API. This handle is given by the function CreateRobotApisWrapper during the initialization.
        apiHandle = uint32(0);

        %Max actuator count that this system object support.
        nbrMaxJointActuators = 7;

        %Max tool count that this system object support.
        nbrMaxToolActuators = 6;
        
        %Flag that indicates if the system object is initialized or not.
        isInitialized = false;

        %Flag that indicates if an API command is currently in progress.
        isCmdInProgress = false;

        %Which command is currently processed.
        idCmdInProgress = enumCmd.undefined;
        
        %Command received as an input
        cmdToProcess;

        %A flag that tells the system object that it needs to process the given command. 
        %When this system object receives a new command, this flag needs to be true.
        processCmdSignal;
        
        %Those variables contain the data of the next pre computed trajectory. (Used in stepImpl function)
        precompute_trj_position;
        precompute_trj_velocity;
        precompute_trj_acceleration;
        precompute_trj_timestamp;
        
        precompute_count;
        
        %Those variable contains information about the next Cartesian command. (Used in stepImpl function)
        cartesian_constraint;
        cartesian_cmd;
        
        %Those variable contains information about the next joint command. (Used in stepImpl function)
        joint_constraint;
        joint_cmd;
        
        %Those variable contains information about the next tool command. (Used in stepImpl function)
        tool_constraint;
        tool_cmd;
        
        %Those variable represent C struct and are used as parameter passed to Kortex functions.
        baseFeedback = Simulink.Bus.createMATLABStruct('BaseFeedback');
        actuatorFeedback = Simulink.Bus.createMATLABStruct('ActuatorsFeedback');
        toolFeedback = Simulink.Bus.createMATLABStruct('InterconnectFeedback');
        last_Error = Simulink.Bus.createMATLABStruct('ErrorInfo');        
        intrinsic_info = Simulink.Bus.createMATLABStruct('IntrinsicParameters');
        extrinsic_info = Simulink.Bus.createMATLABStruct('ExtrinsicParameters');
        option_value = Simulink.Bus.createMATLABStruct('OptionValue');
        sensor_settings = Simulink.Bus.createMATLABStruct('SensorSettings');
        sensor_focus_action = Simulink.Bus.createMATLABStruct('SensorFocusAction');
        option_identifier = Simulink.Bus.createMATLABStruct('OptionIdentifier');

        %Name of the last error.
        last_ErrorName;

        %Flag that indicated if all the system object's properties are valid.
        property_validation = true;

        %ID of the camera.
        visionDeviceID;
    end

    methods
        
        %This function initialize the Kortex API.
        function [isOk] = CreateRobotApisWrapper(obj)
            coder.cinclude('kortexApiWrapper.h');
            if coder.target('MATLAB')
                obj.isInitialized = false;

                [errorCreate, obj.apiHandle] = kortexApiMexInterface('CreateRobotApisWrapper', obj.ip_address, obj.user, obj.password, uint32(obj.session_connection_timeout), uint32(obj.session_ctrl_timeout));
                
                if(errorCreate == KortexErrorCodes.SUB_ERROR_NONE)
                    obj.isInitialized = true;

                    [resultJointCount, actuator_count] = kortexApiMexInterface('GetJointCount', obj.apiHandle);

                    obj.isInitialized = resultJointCount == uint32(KortexErrorCodes.SUB_ERROR_NONE) && (obj.nbrJointActuators == actuator_count);

                    if(obj.isInitialized && obj.hasVision)
                        [resultVision] = kortexApiMexInterface('InitVision', obj.apiHandle);
                        
                        if(resultVision ~= KortexErrorCodes.SUB_ERROR_NONE)
                            if coder.target('MATLAB')
                                warning('Cannot initialize vision module.');
                            end
                        end
                        
                        obj.isInitialized = resultVision ==  KortexErrorCodes.SUB_ERROR_NONE;
                    else
                        if coder.target('MATLAB')
                            warning('Unexpected joint count. Expecting: %s  Detected: %s', actuator_count, obj.nbrJointActuators)
                        end
                    end

                    isOk = obj.isInitialized;
                else
                    obj.isInitialized = false;
                    isOk = obj.isInitialized;
                end
            else
                ipAddress = [obj.ip_address, 0];
                userName = [obj.user, 0];
                password = [obj.password, 0];
                sessionTimeout = uint32(obj.session_connection_timeout);
                controlTimeout = uint32(obj.session_ctrl_timeout);

                ret = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                ret =  coder.ceval('kapi_CreateRobotApisWrapper', coder.ref(obj.apiHandle), ...
                     ipAddress, coder.ref(userName), coder.ref(password), sessionTimeout, controlTimeout);
                isOk = ret == KortexErrorCodes.SUB_ERROR_NONE;
                obj.isInitialized = isOk;
            end
        end

        %This function close the connection with the Kortex API.
        function [isOk] = DestroyRobotApisWrapper(obj)
            isOk = false;

            if obj.isInitialized
                if coder.target('MATLAB')
                    [result] = kortexApiMexInterface('DestroyRobotApisWrapper', obj.apiHandle);
                    isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
                else
                    result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                    result = coder.ceval('kapi_DestroyRobotApisWrapper', obj.apiHandle);
                    isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
                end
            end
        end
        
        %This function reboot the device. WARNING: Make sure the device is in a secure position. 
        function [isOk] = SendCmd_Reboot(obj)
            if coder.target('MATLAB')
                [result] = kortexApiMexInterface('Reboot', obj.apiHandle);
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
            else 
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                result = coder.ceval('kapi_Reboot', obj.apiHandle);
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
            end
        end

        function [isOk, extrinsic] = GetExtrinsicParameters(obj)
            if coder.target('MATLAB')
                [result, extrinsic] = kortexApiMexInterface('GetExtrinsicParameters', obj.apiHandle);
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
            else
                
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                tempExtrinsic = obj.extrinsic_info;
                result = coder.ceval('kapi_GetExtrinsicParameters', obj.apiHandle, coder.ref(tempExtrinsic));

                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
                extrinsic = tempExtrinsic;
            end
        end

        function [isOk, intrinsic] = GetIntrinsicParameters(obj, sensor)
            if coder.target('MATLAB')
                [result, intrinsic] = kortexApiMexInterface('GetIntrinsicParameters', obj.apiHandle, uint32(sensor));
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
            else
            
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                tempSensor = uint32(sensor);
                tempIntrinsic = obj.intrinsic_info;

                result = coder.ceval('kapi_GetIntrinsicParameters', obj.apiHandle, tempSensor, coder.ref(tempIntrinsic));
                
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
                intrinsic = tempIntrinsic;
            end
        end

        function [isOk, status] = GetMovementStatus(obj)
            if coder.target('MATLAB')
                [result, status] = kortexApiMexInterface('GetMovementStatus', obj.apiHandle);
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
            else
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                status = int32(0);
                result = coder.ceval('kapi_GetMovementStatus', obj.apiHandle, coder.ref(status));
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
            end
        end

        function [isOk, value] = GetOptionValue(obj, sensor, option)
            if coder.target('MATLAB')
                [result, value] = kortexApiMexInterface('GetOptionValue', obj.apiHandle, uint32(sensor), uint32(option));
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
            else
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);

                tempOptionValue = obj.option_value;

                tempOptionIdentifier = obj.option_identifier;
                tempOptionIdentifier.sensor = sensor;
                tempOptionIdentifier.option = option;

                result = coder.ceval('kapi_GetOptionValue', obj.apiHandle, tempOptionIdentifier, coder.ref(tempOptionValue));
                
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
                value = tempOptionValue;
            end
        end
        
        function [isOk] = SetOptionValue(obj, sensor, option, value)
            if coder.target('MATLAB')
                [result] = kortexApiMexInterface('SetOptionValue', obj.apiHandle, uint32(sensor), uint32(option), value);
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
            else
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);

                tempOptionValue = obj.option_value;

                tempOptionValue.sensor = uint32(sensor);
                tempOptionValue.option = uint32(option);
                tempOptionValue.value = value;

                result = coder.ceval('kapi_SetOptionValue', obj.apiHandle, tempOptionValue);
                
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
            end
        end

        function [isOk, settings] = GetSensorSettings(obj, sensor)
            if coder.target('MATLAB')
                [result, settings] = kortexApiMexInterface('GetSensorSettings', obj.apiHandle, uint32(sensor));
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
            else
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                
                tempSensor = uint32(sensor);
                settings = obj.sensor_settings;
                
                result = coder.ceval('kapi_GetSensorSettings', obj.apiHandle, tempSensor, coder.ref(settings));
                
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
            end
        end

        function [isOk, sensor, resolution, frame_rate, bit_rate] = SetSensorSettings(obj, sensor, resolution, frame_rate, bit_rate)
            if coder.target('MATLAB')
                [result] = kortexApiMexInterface('SetSensorSettings', obj.apiHandle, uint32(sensor), uint32(resolution), uint32(frame_rate), uint32(bit_rate));
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
            else
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                
                tempSensorSettings = obj.sensor_settings;
                tempSensorSettings.sensor = sensor;
                tempSensorSettings.resolution = resolution;
                tempSensorSettings.frame_rate = frame_rate;
                tempSensorSettings.bit_rate = bit_rate;
                
                result = coder.ceval('kapi_SetSensorSettings', obj.apiHandle, tempSensorSettings);
                
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
            end
        end
        
        function [isOk] = DoSensorFocusAction(obj, sensor, focus_action)
            if coder.target('MATLAB')
                [result] = kortexApiMexInterface('DoSensorFocusAction', obj.apiHandle, uint32(sensor), uint32(focus_action));
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
            else
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);

                tempFocusAction = obj.sensor_focus_action;
                result = coder.ceval('kapi_DoSensorFocusAction', obj.apiHandle, tempFocusAction);
                
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
            end
        end

        function [isOk] = SendCartesianPose(obj, cartesian_cmd, constraintType, speeds, duration)
            if coder.target('MATLAB')
                position = cartesian_cmd(1:3);
                orientation = cartesian_cmd(4:6);

                [result] = kortexApiMexInterface('ReachCartesianPose', obj.apiHandle, constraintType, speeds, duration, position, orientation);

                isOk = (result == KortexErrorCodes.SUB_ERROR_NONE);
            else
                translationSpeed = speeds(1);
                orientationSpeed = speeds(2);

                translationCmd = cartesian_cmd(1:3);
                orientationCmd = cartesian_cmd(4:6);

                result = uint32(KortexErrorCodes.SUB_ERROR_NONE); 

                result = coder.ceval('kapi_ReachCartesianPose', obj.apiHandle, constraintType,...
                                    translationSpeed, orientationSpeed, duration, coder.ref(translationCmd),...
                                    coder.ref(orientationCmd));
                isOk = (result == KortexErrorCodes.SUB_ERROR_NONE);
            end                
        end
        
        function [isOk] = SetIntrinsicParameters(obj, sensor, resolution, principal_point, focal_length, distortion_coeffs)
            if coder.target('MATLAB')
                
                [result] = kortexApiMexInterface('SetIntrinsicParameters', obj.apiHandle, sensor, resolution, principal_point, focal_length, distortion_coeffs);
                
                isOk = (result == KortexErrorCodes.SUB_ERROR_NONE);
            else
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE); 

                result = coder.ceval('kapi_SetIntrinsicParameters', obj.apiHandle, sensor,...
                                    resolution, principal_point, focal_length, distortion_coeffs,...
                                    coder.ref(orientationCmd));
                
                isOk = (result == KortexErrorCodes.SUB_ERROR_NONE);
            end                
        end
        
        function [isOk] = SetExtrinsicParameters(obj, rotation, translation)
            if coder.target('MATLAB')
                
                [result] = kortexApiMexInterface('SetExtrinsicParameters', obj.apiHandle, rotation, translation);

                isOk = (result == KortexErrorCodes.SUB_ERROR_NONE);
            else
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE); 

                result = coder.ceval('kapi_SetExtrinsicParameters', obj.apiHandle, rotation, translation);
                
                isOk = (result == KortexErrorCodes.SUB_ERROR_NONE);
            end                
        end

        %Send a generic command to the robot. A generic command is a command that takes no parameters and that returns no data except a status flag and an error description.
        function [isOk] = SendGenericCommand(obj, command)
            if coder.target('MATLAB')
                [result] = kortexApiMexInterface(command, obj.apiHandle);
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
            end
        end

        function [isOk] = SendJointAngles(obj, joint_cmd, constraintType, speed, duration)
            if coder.target('MATLAB')
                [result] = kortexApiMexInterface('ReachJointAngles', obj.apiHandle, constraintType, speed, duration, joint_cmd(1:uint32(obj.nbrJointActuators)));  

                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;  
            else
                
                count = uint32(obj.nbrJointActuators);

                result = uint32(KortexErrorCodes.SUB_ERROR_NONE); 
                result = coder.ceval('kapi_ReachJointAngles', obj.apiHandle, constraintType,...
                                    speed, duration, coder.ref(joint_cmd), count);
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;  
            end
        end

        function [isOk] = SendPreComputedTrajectory(obj, position, velocity, acceleration, timestamp_sec, step_count)
            continuity_mode = int32(1); % hard coded to position because we only support that now.

            % Even though the input is hard-coded to 7 joints, only read the necessary number of joints
            tempPosition = position([1:obj.nbrJointActuators], [1:step_count]);
            tempVelocity = velocity([1:obj.nbrJointActuators], [1:step_count]);
            tempAcceleration = acceleration([1:obj.nbrJointActuators], [1:step_count]);
            tempTimestamp = timestamp_sec([1:1], [1:step_count]);
            if coder.target('MATLAB')
                [result] = kortexApiMexInterface('PlayPreComputedTrajectory', obj.apiHandle, uint32(step_count), continuity_mode, tempPosition, tempVelocity, tempAcceleration, tempTimestamp);
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
            else
                count = uint32(obj.nbrJointActuators);

                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                result = coder.ceval('kapi_PlayPreComputedTrajectory', obj.apiHandle, continuity_mode,...
                                    coder.ref(tempPosition), coder.ref(tempVelocity), coder.ref(tempAcceleration), coder.ref(tempTimestamp), count,...
                                    step_count);
                
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
            end
        end

        function [isOk, baseFeedback, jointsFeedback, interconnectFeedback] = SendRefreshFeedback(obj)
            if coder.target('MATLAB')
                [result, baseFeedback, jointsFeedback, interconnectFeedback] = kortexApiMexInterface('RefreshFeedback', obj.apiHandle);
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;  
            else
                tempBaseFeedback = obj.baseFeedback;
                tempActuatorFeedback = obj.actuatorFeedback;
                tempToolFeedback = obj.toolFeedback;
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                
                result = coder.ceval('kapi_RefreshFeedback', obj.apiHandle, coder.ref(tempBaseFeedback),...
                                    coder.ref(tempActuatorFeedback), coder.ref(tempToolFeedback));
                
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
                
                baseFeedback = tempBaseFeedback;
                jointsFeedback = tempActuatorFeedback;
                interconnectFeedback = tempToolFeedback;
            end
        end
        
        function [isOk] = SendToolCommand(obj, tool_cmd_mode, duration, tool_cmd)
            if coder.target('MATLAB')    
                [result] = kortexApiMexInterface('SendToolCommand', obj.apiHandle, tool_cmd_mode, duration, tool_cmd);
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
            else
                count = uint32(obj.nbrToolActuators);
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);

                result = coder.ceval('kapi_SendToolCommand', obj.apiHandle, tool_cmd_mode,...
                                    duration, coder.ref(tool_cmd), count);
                
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;  
            end
        end

        function [isOk] = SetAdmittance(obj, admittanceMode)
            if coder.target('MATLAB')
                [result] = kortexApiMexInterface('SetAdmittance', obj.apiHandle, admittanceMode);

                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
            else
                admittance_mode = uint32(admittanceMode);
                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                
                result = coder.ceval('kapi_SetAdmittance', obj.apiHandle, coder.ref(errorInfo), admittance_mode);
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
            end
        end


    end

    methods(Static, Access = protected)

        function header = getHeaderImpl
           header = matlab.system.display.Header(mfilename('class'), ...
               'Title','Kortex properties', ...
               'Text','A detailed description of those parameters can be found in the Kortex documentation.');
        end

        function groups = getPropertyGroupsImpl           
            ConfigurationSection = matlab.system.display.Section(...
                'Title','Configuration',...
                'PropertyList',{'nbrJointActuators','nbrToolActuators','hasVision'});

            ConnectionSection = matlab.system.display.Section(...
                'Title','Connection',...
                'PropertyList',{'ip_address','user', 'password', 'session_connection_timeout', 'session_ctrl_timeout'});

            robotTab = matlab.system.display.SectionGroup(...
                'Title','Robot', ...
                'Sections',[ConfigurationSection,ConnectionSection]);

            groups = robotTab;
        end
    end

    methods(Access = protected)
        function validatePropertiesImpl(obj)
            
            obj.property_validation = true;
            
            if(isinteger(int32(obj.nbrJointActuators)))
                if obj.nbrJointActuators > obj.nbrMaxJointActuators || obj.nbrJointActuators < 0 
                    if coder.target('MATLAB')
                        warning('nbrJointActuators must be between 1 and %d', obj.nbrMaxJointActuators)
                    end
                    obj.property_validation = false;
                end
            else
                if coder.target('MATLAB')
                    warning('nbrJointActuators must be an integer')
                end
            end
            
            if(isinteger(int32(obj.nbrToolActuators)))
                if obj.nbrToolActuators > obj.nbrMaxToolActuators
                    if coder.target('MATLAB')
                        warning('nbrToolActuators must be between 1 and %d', obj.nbrMaxToolActuators)
                    end
                    obj.property_validation = false;
                end
            else
                if coder.target('MATLAB')
                    warning('nbrToolActuators must be an integer')
                end
                obj.property_validation = false;
            end
            
            if(isinteger(int32(obj.session_connection_timeout)))
                if obj.session_connection_timeout < 0
                    if coder.target('MATLAB') 
                        warning('session_connection_timeout must be positive')
                    end
                    obj.property_validation = false;
                end
            else
                if coder.target('MATLAB')
                    warning('session_connection_timeout must be an integer')
                end
                obj.property_validation = false;
            end
            
            if(isinteger(int32(obj.session_ctrl_timeout)))
                if obj.session_ctrl_timeout < 0 
                    if coder.target('MATLAB')
                        warning('session_ctrl_timeout must be positive')
                    end
                    obj.property_validation = false;
                end
            else
                if coder.target('MATLAB')
                    warning('session_ctrl_timeout must be an integer')
                end
                obj.property_validation = false;
            end
        end        
       
        function isOk = setupImpl(obj)
            isOk = false;
            
            %We only try to initialize the API if all the input properties have been validated.
            if obj.property_validation
                
                [result] = CreateRobotApisWrapper(obj);
                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;
                pause(0.25)
            
                if (~isOk)
                    return;
                end
            else
                return;
            end
        end

        function releaseImpl(obj)
            DestroyRobotApisWrapper(obj);
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = ['KORTEX ', 'Main'];
        end

        function [varargout] = stepImpl(obj, varargin)
            step_status = obj.isInitialized;
            
            obj.cmdToProcess         = varargin{1};
            obj.processCmdSignal     = varargin{2};

            obj.precompute_trj_position = varargin{3};
            obj.precompute_trj_velocity = varargin{4};
            obj.precompute_trj_acceleration = varargin{5};
            obj.precompute_trj_timestamp = varargin{6};
            
            obj.precompute_count = varargin{7};

            obj.joint_constraint = varargin{8};
            obj.joint_cmd = varargin{9};

            obj.cartesian_constraint = varargin{10};
            obj.cartesian_cmd = varargin{11};
            
            obj.tool_constraint = varargin{12};
            obj.tool_cmd = varargin{13};

            isOk = true;
            obj.idCmdInProgress = enumCmd.undefined;
            if obj.isInitialized
                if obj.processCmdSignal

                    obj.isCmdInProgress = true;
                    switch (obj.cmdToProcess)
                        case enumCmd.clear_faults
                            obj.idCmdInProgress = enumCmd.clear_faults;
                            
                            if coder.target('MATLAB')
                                [isOk] = SendGenericCommand(obj, 'ClearFaults');
                            else
                                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                                result = coder.ceval('kapi_ClearFaults', obj.apiHandle);
                                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
                            end  
                        case enumCmd.emergency_stop
                            obj.idCmdInProgress = enumCmd.emergency_stop;

                            if coder.target('MATLAB')
                                [isOk] = SendGenericCommand(obj, 'ApplyEmergencyStop');
                            else
                                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                                result = coder.ceval('kapi_ApplyEmergencyStop', obj.apiHandle);
                                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
                            end 
                        case enumCmd.stop_action
                            obj.idCmdInProgress = enumCmd.stop_action;

                            if coder.target('MATLAB')
                                [isOk] = SendGenericCommand(obj, 'StopAction');
                            else
                                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                                result = coder.ceval('kapi_StopAction', obj.apiHandle);
                                isOk = result == KortexErrorCodes.SUB_ERROR_NONE; 
                            end
                        case enumCmd.pause_action
                            obj.idCmdInProgress = enumCmd.pause_action;

                            if coder.target('MATLAB')
                                [isOk] = SendGenericCommand(obj, 'PauseAction');
                            else
                                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                                result = coder.ceval('kapi_PauseAction', obj.apiHandle);
                                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;  
                            end
                        case enumCmd.resume_action
                            obj.idCmdInProgress = enumCmd.resume_action;

                            if coder.target('MATLAB')
                                [isOk] = SendGenericCommand(obj, 'ResumeAction');
                            else
                                result = uint32(KortexErrorCodes.SUB_ERROR_NONE);
                                result = coder.ceval('kapi_ResumeAction', obj.apiHandle);
                                isOk = result == KortexErrorCodes.SUB_ERROR_NONE;  
                            end
                            
                        case enumCmd.joint_reach
                            obj.idCmdInProgress = enumCmd.joint_reach;

                            constraintType = int32(obj.joint_constraint(1));
                            duration = obj.joint_constraint(2);
                            speed = obj.joint_constraint(2);
                            
                            [isOk] = SendJointAngles(obj, obj.joint_cmd, constraintType, speed, duration);
                        case enumCmd.cartesian_reach
                            obj.idCmdInProgress = enumCmd.cartesian_reach;

                            constraintType = int32( obj.cartesian_constraint(1) );
                            duration = obj.cartesian_constraint(2);
                            speeds = obj.cartesian_constraint(3:4);
                            
                            [isOk] = SendCartesianPose(obj, obj.cartesian_cmd, constraintType, speeds, duration);
                        case enumCmd.tool_reach
                            obj.idCmdInProgress = enumCmd.tool_reach;
                            [isOk] = SendToolCommand( obj, int32(enumToolMode.toolMode_reach), obj.tool_constraint(1), obj.tool_cmd );
                            
                        case enumCmd.tool_speed
                            obj.idCmdInProgress = enumCmd.tool_speed;
                            [isOk] = SendToolCommand( obj, int32(enumToolMode.toolMode_speed), obj.tool_constraint(1), obj.tool_cmd );
                            
                        case enumCmd.precomputed_joint_trj
                            obj.idCmdInProgress = enumCmd.precomputed_joint_trj;
            
                            position = obj.precompute_trj_position;            % [nbrJointActuator : nbrTimeStep]
                            velocity = obj.precompute_trj_velocity;            % [nbrJointActuator : nbrTimeStep]
                            acceleration = obj.precompute_trj_acceleration;    % [nbrJointActuator : nbrTimeStep]
                            timestamp = obj.precompute_trj_timestamp;          % [1 : nbrTimeStep]
                            step_count = obj.precompute_count;                 % Number of steps to read (out of the max 25000)

                            [isOk] = SendPreComputedTrajectory(obj, position, velocity, acceleration, timestamp, step_count);
                        otherwise
                            obj.isCmdInProgress = false;
                          
                            if coder.target('MATLAB')
                                [~, obj.last_ErrorName] = kortexApiMexInterface('GetErrorName', obj);
                                warning(obj.last_ErrorName)
                            end
                    end
                end
            else
                if coder.target('MATLAB')
                    warning('Robot not initialized.')
                end
            end

            step_status = step_status && isOk;
            
            if(obj.hasVision)
                [isOk, obj.sensor_settings] = obj.GetSensorSettings(enum_Sensor.SENSOR_COLOR);
                [isOk, obj.intrinsic_info] = obj.GetIntrinsicParameters(enum_Sensor.SENSOR_COLOR);
                [isOk, obj.extrinsic_info] = obj.GetExtrinsicParameters();
            end
            [isOk, obj.baseFeedback, jointsFeedback, interconnectFeedback] = SendRefreshFeedback(obj);
            step_status = step_status && isOk;
            
            moving_status = int32(0);
            
            if obj.isCmdInProgress
                [isOk, moving_status] = GetMovementStatus(obj);
                step_status = step_status && isOk;
            end

            is_moving = true;
            if ~obj.processCmdSignal
                is_moving = (moving_status ~= 0);
                
                if ~is_moving
                    obj.isCmdInProgress = false;
                end
            end

            

            if coder.target('MATLAB')
                [~,tempError] = kortexApiMexInterface('GetLastError', obj.apiHandle);
            end
            
            varargout = {...
                step_status,...
                uint32(obj.idCmdInProgress),...
                uint32(moving_status),...
                jointsFeedback,...
                obj.baseFeedback,...
                interconnectFeedback,...
                obj.intrinsic_info,...
                obj.extrinsic_info,...
                obj.sensor_settings,...
                is_moving...
            };
        end

        function nbrInput = getNumInputsImpl(obj)
            nbrInput = 13;
        end

        function varargout = getInputNamesImpl(obj)
            varargout{1} = 'cmdToProcess'; 
            varargout{2} = 'processCmdSignal'; 
            varargout{3} = 'precompute_trj_position'; 
            varargout{4} = 'precompute_trj_velocity'; 
            varargout{5} = 'precompute_trj_acceleration'; 
            varargout{6} = 'precompute_trj_timestamp';
            varargout{7} = 'precompute_count';
            varargout{8} = 'joint_constraint';
            varargout{9} = 'joint_cmd';
            varargout{10} = 'cartesian_constraint';
            varargout{11} = 'cartesian_cmd';
            varargout{12} = 'tool_constraint';
            varargout{13} = 'tool_cmd';
        end

        function nbrOutput = getNumOutputsImpl(obj)
            nbrOutput = 10;
        end

        function varargout = getOutputNamesImpl(obj)
            varargout{1} = 'status'; 
            varargout{2} = 'cmd_in_progress';
            varargout{3} = 'moving_status';
            varargout{4} = 'joint_feedback';
            varargout{5} = 'base_feedback';
            varargout{6} = 'tool_feedback';
            varargout{7} = 'intrinsic_parameters';
            varargout{8} = 'extrinsic_parameters';
            varargout{9} = 'vision_settings';
            varargout{10} = 'is_moving';
        end

        function varargout = getOutputDataTypeImpl(obj)
            varargout{1} = 'logical';  
            varargout{2} = 'uint32';
            varargout{3} = 'uint32';
            varargout{4} = 'ActuatorsFeedback';
            varargout{5} = 'BaseFeedback';
            varargout{6} = 'InterconnectFeedback';
            varargout{7} = 'IntrinsicParameters';
            varargout{8} = 'ExtrinsicParameters';
            varargout{9} = 'SensorSettings';
            varargout{10} = 'logical';
        end

        function varargout = isOutputComplexImpl(obj)
            varargout{1} = false; 
            varargout{2} = false;
            varargout{3} = false;
            varargout{4} = false;
            varargout{5} = false;
            varargout{6} = false;
            varargout{7} = false;
            varargout{8} = false;
            varargout{9} = false;
            varargout{10} = false;
        end

        function varargout = isOutputFixedSizeImpl(obj)
            varargout{1} = true; 
            varargout{2} = true;
            varargout{3} = true;
            varargout{4} = true;
            varargout{5} = true;
            varargout{6} = true;
            varargout{7} = true;
            varargout{8} = true;
            varargout{9} = true;
            varargout{10} = true;
        end

        function varargout = getOutputSizeImpl(obj)
            varargout{1} = 1; 
            varargout{2} = 1; 
            varargout{3} = 1; 
            varargout{4} = 1;
            varargout{5} = 1;
            varargout{6} = 1;
            varargout{7} = 1;
            varargout{8} = 1;
            varargout{9} = 1;
            varargout{10} = 1;
        end
    end
end
