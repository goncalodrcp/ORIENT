classdef kObjTrajectoryFeeder < matlab.System & matlab.system.mixin.Propagates ...
        & kortex_ros2_build & matlab.system.mixin.CustomIcon

    properties
    end
    
    properties (Nontunable, Logical)
    end
    
    properties (Nontunable)
        nbrJointActuators = 7;
    end
   
    properties (Access = private)
        in_nextCmdReadyToProcess = false;

        guide_list_max_capacity = 100;
        
        %Element count of the list.
        guide_list_max_size = 0;

        %Max capacity of the list.
        guide_list = zeros(100,1, 'uint32');
        
        simulation_ended = false;
        
        currentGuideIndex = 0;
        currentCartIndex = 0;
        currentAngIndex = 0;
        currentPrecompIndex = 0;
        currentToolReachIndex = 0;
        currentToolSpeedIndex = 0;

        nextWayPointType = 0;

        %Max capacity of the list.
        cart_list_max_capacity = 10;
        %Size of the list.
        cart_list_size = 0;
        trjCartWayPoints = repmat({struct('cmd', enumCmd.cartesian_reach, 'wayPoint', [0,0,0,0,0,0], 'constraint', [0, 0, 0, 0])}, 10, 1);

        %Max capacity of the list.
        ang_list_max_capacity = 10;
        %Size of the list.
        ang_list_size = 0;
        trjAngWayPoints = repmat({struct('cmd', enumCmd.joint_reach, 'wayPoint', [0,0,0,0,0,0,0], 'constraint', [0, 0])}, 10, 1);

        %Max capacity of the list.
        toolreach_list_max_capacity = 10;
        %Size of the list.
        toolreach_list_size = 0;
        trjToolReachWayPoints = repmat({struct('cmd', enumCmd.tool_reach, 'wayPoint', 0, 'constraint', 0)}, 10, 1);

        %Max capacity of the list.
        toolspeed_list_max_capacity = 10;
        %Size of the list.
        toolspeed_list_size = 0;
        trjToolSpeedWayPoints = repmat({struct('cmd', enumCmd.tool_speed, 'wayPoint', 0, 'constraint', 0)}, 10, 1);

        %Max capacity of the list.
        precomp_list_max_capacity = 10;
        %Size of the list.
        precomp_list_size = 0;
        trjPreCompWayPoints = repmat({struct('cmd', enumCmd.tool_speed, 'wayPoint', zeros(25000, 29), 'count', 0, 'convertToDeg', false)}, 10, 1);
        
    end

    methods
        function obj = kObjSimplifiedApi(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods(Access = protected)
        
        function wayPoint = createCartesianWayPoint(obj, x, y, z, tx, ty, tz, constraintType,duration,  TranslationSpeed, OrientationSpeed)
            wayPoint = struct('cmd', enumCmd.cartesian_reach, 'wayPoint', [x, y, z, tx, ty, tz], 'constraint', [constraintType, duration, TranslationSpeed, OrientationSpeed]);
        end
        
        function wayPoint = create6DofJointWayPoint(obj, j1, j2, j3, j4, j5, j6, constraintType, constraintValue)
            wayPoint = struct('cmd', enumCmd.joint_reach, 'wayPoint', [j1, j2, j3, j4, j5, j6, 0], 'constraint', [constraintType, constraintValue]);
        end

        function wayPoint = create7DofJointWayPoint(obj, j1, j2, j3, j4, j5, j6, j7, constraintType, constraintValue)
            wayPoint = struct('cmd', enumCmd.joint_reach, 'wayPoint', [j1, j2, j3, j4, j5, j6, j7], 'constraint', [constraintType, constraintValue]);
        end

        function wayPoint = createToolWayPoint(obj, mode,  command, duration)
            wayPoint = struct('cmd', mode, 'wayPoint', command, 'constraint', duration);
        end

        function wayPoint = createPrecomputedTrajectoryWayPoint(obj, trajectoryPoints, convertToDeg, count)
            temp = zeros(25000, 29);
            [p,q] = size(trajectoryPoints);
            temp(1:p, 1:q) = trajectoryPoints;
            wayPoint = struct('cmd', enumCmd.precomputed_joint_trj, 'wayPoint', temp, 'count', count, 'convertToDeg', convertToDeg);
        end

        function wayPoint = createSetSensorSettingsWayPoint(obj, ~, convertToDeg)
            wayPoint = struct('cmd', enumCmd.vision_set_sensor_settings, 'sensor', sensor, 'resolution', resolution, 'frame_rate', frame_rate, 'bit_rate', bit_rate);
        end

        %% setup / release object functions
        function setupImpl(obj)

            wayPoint = create6DofJointWayPoint(obj, 60, 0, 0, 0, 0, 0, 0, 0);
            AddWaypoint(obj, wayPoint);
            
            wayPoint = create6DofJointWayPoint(obj, 90, 0, 0, 0, 0, 0, 0, 0);
            AddWaypoint(obj, wayPoint);
            
            wayPoint = create6DofJointWayPoint(obj, 30, 0, 0, 0, 0, 0, 0, 0);
            AddWaypoint(obj, wayPoint);
            
            wayPoint = create6DofJointWayPoint(obj, 0, 15, 230, 0, 55, 90, 0, 0);
            AddWaypoint(obj, wayPoint);
%             
%             wayPoint = createToolWayPoint(obj,enumCmd.tool_speed, 0.9, 0);
%             AddWaypoint(obj, wayPoint);
% 
% 
%             wayPoint = createToolWayPoint(obj,enumCmd.tool_speed, -0.9, 0);
%             AddWaypoint(obj, wayPoint);
%             
            wayPoint = createCartesianWayPoint(obj, 0.577, 0.001, 0.638, 90.0, 0, 90.0, 0, 0, 0, 0);
            AddWaypoint(obj, wayPoint);
            
            wayPoint = createCartesianWayPoint(obj, 0.577, 0.301, 0.638, 90.0, 0, 90.0, 0, 0, 0, 0);
            AddWaypoint(obj, wayPoint);
            
            wayPoint = createCartesianWayPoint(obj, 0.577, -0.301, 0.638, 90.0, 0, 90.0, 0, 0, 0, 0);
            AddWaypoint(obj, wayPoint);
%             
%             wayPoint = create7DofJointWayPoint(obj, 360, 15, 180, 230, 0, 55, 90, 0, 0);
%             AddWaypoint(obj, wayPoint);
%                         
%             wayPoint = createToolWayPoint(obj,enumCmd.tool_reach, 0.5, 0);
%             AddWaypoint(obj, wayPoint);
%             
            wayPoint = create6DofJointWayPoint(obj, 0, 0, 0, 0, 0, 0, 0, 0);
            AddWaypoint(obj, wayPoint);
% 
%             wayPoint = create7DofJointWayPoint(obj, 0, 0, 0, 0, 0, 0, 0, 0, 0);
%             AddWaypoint(obj, wayPoint);
%             
            if coder.target('MATLAB')
                load('trajectory.mat');
            else
                traj = coder.load('trajectory.mat');
                precomputedTrajectory = traj.precomputedTrajectory;
            end
            [point_count, item] = size(precomputedTrajectory);
            wayPoint = createPrecomputedTrajectoryWayPoint(obj, precomputedTrajectory, true, point_count);
            AddWaypoint(obj, wayPoint);
%             
%             wayPoint = create6DofJointWayPoint(obj, 90, 0, 0, 0, 0, 0, 0, 0);
%             AddWaypoint(obj, wayPoint);
            
            wayPoint = create6DofJointWayPoint(obj, 0, 0, 0, 0, 0, 0, 0, 0);
            AddWaypoint(obj, wayPoint);
%             
%             wayPoint = create7DofJointWayPoint(obj, 0, 0, 0, 0, 0, 0, 0, 0, 0);
%             AddWaypoint(obj, wayPoint);
        end

        function AddWaypoint(obj, waypoint)
            obj.guide_list_max_size = obj.guide_list_max_size + 1;
            obj.guide_list(obj.guide_list_max_size) = waypoint.cmd;
            
            switch (waypoint.cmd)
                case enumCmd.joint_reach
                    obj.ang_list_size = obj.ang_list_size + 1;
                    obj.trjAngWayPoints{obj.ang_list_size} = waypoint;
                    
                case enumCmd.cartesian_reach
                    obj.cart_list_size = obj.cart_list_size + 1;
                    obj.trjCartWayPoints{obj.cart_list_size} = waypoint;
                    
                case enumCmd.tool_reach
                    obj.toolreach_list_size = obj.toolreach_list_size + 1;
                    obj.trjToolReachWayPoints{obj.toolreach_list_size} = waypoint;
                
                case enumCmd.tool_speed
                    obj.toolspeed_list_size = obj.toolspeed_list_size + 1;
                    obj.trjToolSpeedWayPoints{obj.toolspeed_list_size} = waypoint;

                case enumCmd.precomputed_joint_trj
                    obj.precomp_list_size = obj.precomp_list_size + 1;
                    obj.trjPreCompWayPoints{obj.precomp_list_size} = waypoint;

            end
        end
        
        function releaseImpl(obj)
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = ["Trajectory" "Feeder"];
        end

        function [varargout] = stepImpl(obj, varargin)
            
            obj.in_nextCmdReadyToProcess = varargin{1};
            
            cmdToProcess = enumCmd.undefined;
            processCmdSignal = false;
            
            cartesian_cmd = [0, 0, 0, 0, 0, 0];
            cartesian_constraint = [0, 0, 0, 0];
            joint_cmd = [0, 0, 0, 0, 0, 0, 0];
            joint_constraint = [0, 0];
            
            precompute_trj_position = zeros(7, 25000);
            precompute_trj_velocity = zeros(7, 25000);
            precompute_trj_acceleration = zeros(7, 25000);
            precompute_trj_timestamp = zeros(1, 25000);
            precompute_count = 0;
            
            tool_cmd = 0;
            tool_constraint = 0;
            
            %If there is next waypoint to send and the simulation is still on
            if obj.in_nextCmdReadyToProcess && ~obj.simulation_ended
                obj.currentGuideIndex = obj.currentGuideIndex + 1;
%                 obj.guide_list = obj.guide_list(2:end);

                %If all the waypoint from the list have been executed.
                if obj.currentGuideIndex > obj.guide_list_max_size
                    obj.currentGuideIndex = -1;
                    obj.simulation_ended = true;
                    processCmdSignal = false;
                else
                    obj.nextWayPointType = obj.guide_list(obj.currentGuideIndex);
                    processCmdSignal = true;

                    switch (obj.nextWayPointType)
                        case enumCmd.joint_reach
                            obj.currentAngIndex = obj.currentAngIndex + 1;
                            joint_cmd = obj.trjAngWayPoints{obj.currentAngIndex}.wayPoint;
                            joint_constraint = obj.trjAngWayPoints{obj.currentAngIndex}.constraint;
                            cmdToProcess = enumCmd.joint_reach;
                        
                        case enumCmd.cartesian_reach
                            obj.currentCartIndex = obj.currentCartIndex + 1;
                            cartesian_cmd = obj.trjCartWayPoints{obj.currentCartIndex}.wayPoint;
                            cartesian_constraint = obj.trjCartWayPoints{obj.currentCartIndex}.constraint;
                            cmdToProcess = enumCmd.cartesian_reach;

                        case enumCmd.precomputed_joint_trj
                            obj.currentPrecompIndex = obj.currentPrecompIndex + 1;
                            precompute_trj_timestamp = obj.trjPreCompWayPoints{obj.currentPrecompIndex}.wayPoint(:, 1).';
                            % 7 is always used for trajectories since output is fixed in size
                            % In the case of having less than 7 joints, the values for non-existant joints wil be left to zero.
                            precompute_trj_position(1:obj.nbrJointActuators, :) =     rad2deg(obj.trjPreCompWayPoints{obj.currentPrecompIndex}.wayPoint(:, 2:2+obj.nbrJointActuators-1).');
                            precompute_trj_velocity(1:obj.nbrJointActuators, :) =     rad2deg(obj.trjPreCompWayPoints{obj.currentPrecompIndex}.wayPoint(:, 9:9+obj.nbrJointActuators-1).');
                            precompute_trj_acceleration(1:obj.nbrJointActuators, :) = rad2deg(obj.trjPreCompWayPoints{obj.currentPrecompIndex}.wayPoint(:, 16:16+obj.nbrJointActuators-1).');
                            precompute_count = obj.trjPreCompWayPoints{obj.currentPrecompIndex}.count;
                            cmdToProcess = enumCmd.precomputed_joint_trj;        
                        case enumCmd.tool_reach
                            obj.currentToolReachIndex = obj.currentToolReachIndex + 1;
                            tool_cmd = obj.trjToolReachWayPoints{obj.currentToolReachIndex}.wayPoint;
                            tool_constraint = obj.trjToolReachWayPoints{obj.currentToolReachIndex}.constraint;
                            cmdToProcess = enumCmd.tool_reach;        
                        case enumCmd.tool_speed
                            obj.currentToolSpeedIndex = obj.currentToolSpeedIndex + 1;
                            tool_cmd = obj.trjToolSpeedWayPoints{obj.currentToolSpeedIndex}.wayPoint;
                            tool_constraint = obj.trjToolSpeedWayPoints{obj.currentToolSpeedIndex}.constraint;
                            cmdToProcess = enumCmd.tool_speed;        
                        otherwise
                            disp('not supported yet !!!')
                            cmdToProcess = enumCmd.undefined;
                            processCmdSignal = false;
                    end
                end
            end
  
            varargout = { ...
                double(cmdToProcess),...
                processCmdSignal,...
                precompute_trj_position,...
                precompute_trj_velocity,...
                precompute_trj_acceleration,...
                precompute_trj_timestamp,...
                precompute_count,...
                joint_constraint,...
                joint_cmd,...
                cartesian_constraint,...
                cartesian_cmd,...
                tool_constraint,...
                tool_cmd,...
                obj.simulation_ended...
                };
        end
        
        function nbrInput = getNumInputsImpl(obj)
            nbrInput = 1;
        end

        function varargout = getInputNamesImpl(obj)
            varargout{1} = 'is_ready';
        end

        function varargout = getOutputNamesImpl(obj)
            
            varargout = cell(1, nargout);
            
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
            varargout{14} = 'simulation_ended';
        end

        function nbrOutput = getNumOutputsImpl(obj)
            nbrOutput = 14;
        end

        function varargout = getOutputDataTypeImpl(obj)
            varargout = cell(1, nargout);

            varargout{1} = 'double';
            varargout{2} = 'logical';
            varargout{3} = 'double';
            varargout{4} = 'double';
            varargout{5} = 'double';
            varargout{6} = 'double';
            varargout{7} = 'double';
            varargout{8} = 'double';
            varargout{9} = 'double';
            varargout{10} = 'double';
            varargout{11} = 'double';
            varargout{12} = 'double';
            varargout{13} = 'double';
            varargout{14} = 'logical';
        end

        function varargout = isOutputComplexImpl(obj)
            varargout = cell(1, nargout);
            
            for index = 1:nargout
                varargout{index} = false;
            end
        end

        function varargout = isOutputFixedSizeImpl(obj)
            varargout = cell(1, nargout);
            
            varargout{1} = true;
            varargout{2} = true;

            varargout{3} = false;
            varargout{4} = false;
            varargout{5} = false;
            varargout{6} = false;
            
            varargout{7} = true;
            varargout{8} = true;
            varargout{9} = true;
            varargout{10} = true;
            varargout{11} = true;
            varargout{12} = true;
            varargout{13} = true;
            varargout{14} = true;
        end
            
                
        function varargout = getOutputSizeImpl(obj)
            varargout = cell(1, nargout);
            
            varargout{1} = 1;
            varargout{2} = 1;
            
            varargout{3} = [7 25000];
            varargout{4} = [7 25000];
            varargout{5} = [7 25000];
            varargout{6} = [1 25000];

            varargout{7} = 1;
            varargout{8} = [1 2];
            varargout{9} = [1 7];
            varargout{10} = [1 4];
            varargout{11} = [1 6];
            varargout{12} = [1 1];
            varargout{13} = [1 1];
            varargout{14} = 1;
        end

    end
end
