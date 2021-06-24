% initializes a set of bus objects in the MATLAB base workspace

% Bus object: 

%% base bus definition
clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'arm_state';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'uint32';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'arm_voltage';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'double';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'arm_current';
fields_array(3).Dimensions = 1;
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'double';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

fields_array(4) = Simulink.BusElement;
fields_array(4).Name = 'temperature_cpu';
fields_array(4).Dimensions = 1;
fields_array(4).DimensionsMode = 'Fixed';
fields_array(4).DataType = 'double';
fields_array(4).SampleTime = -1;
fields_array(4).Complexity = 'real';
fields_array(4).SamplingMode = 'Sample based';

fields_array(5) = Simulink.BusElement;
fields_array(5).Name = 'temperature_ambient';
fields_array(5).Dimensions = 1;
fields_array(5).DimensionsMode = 'Fixed';
fields_array(5).DataType = 'double';
fields_array(5).SampleTime = -1;
fields_array(5).Complexity = 'real';
fields_array(5).SamplingMode = 'Sample based';

fields_array(6) = Simulink.BusElement;
fields_array(6).Name = 'imu_acceleration';
fields_array(6).Dimensions = [1 3];
fields_array(6).DimensionsMode = 'Fixed';
fields_array(6).DataType = 'double';
fields_array(6).SampleTime = -1;
fields_array(6).Complexity = 'real';
fields_array(6).SamplingMode = 'Sample based';

fields_array(7) = Simulink.BusElement;
fields_array(7).Name = 'imu_angular_velocity';
fields_array(7).Dimensions = [1 3];
fields_array(7).DimensionsMode = 'Fixed';
fields_array(7).DataType = 'double';
fields_array(7).SampleTime = -1;
fields_array(7).Complexity = 'real';
fields_array(7).SamplingMode = 'Sample based';

fields_array(8) = Simulink.BusElement;
fields_array(8).Name = 'tool_pose';
fields_array(8).Dimensions = [1 6];
fields_array(8).DimensionsMode = 'Fixed';
fields_array(8).DataType = 'double';
fields_array(8).SampleTime = -1;
fields_array(8).Complexity = 'real';
fields_array(8).SamplingMode = 'Sample based';

fields_array(9) = Simulink.BusElement;
fields_array(9).Name = 'tool_twist';
fields_array(9).Dimensions = [1 6];
fields_array(9).DimensionsMode = 'Fixed';
fields_array(9).DataType = 'double';
fields_array(9).SampleTime = -1;
fields_array(9).Complexity = 'real';
fields_array(9).SamplingMode = 'Sample based';

fields_array(10) = Simulink.BusElement;
fields_array(10).Name = 'tool_external_wrench_force';
fields_array(10).Dimensions = [1 3];
fields_array(10).DimensionsMode = 'Fixed';
fields_array(10).DataType = 'double';
fields_array(10).SampleTime = -1;
fields_array(10).Complexity = 'real';
fields_array(10).SamplingMode = 'Sample based';

fields_array(11) = Simulink.BusElement;
fields_array(11).Name = 'tool_external_wrench_torque';
fields_array(11).Dimensions = [1 3];
fields_array(11).DimensionsMode = 'Fixed';
fields_array(11).DataType = 'double';
fields_array(11).SampleTime = -1;
fields_array(11).Complexity = 'real';
fields_array(11).SamplingMode = 'Sample based';

fields_array(12) = Simulink.BusElement;
fields_array(12).Name = 'fault_bank_a';
fields_array(12).Dimensions = 1;
fields_array(12).DimensionsMode = 'Fixed';
fields_array(12).DataType = 'uint32';
fields_array(12).SampleTime = -1;
fields_array(12).Complexity = 'real';
fields_array(12).SamplingMode = 'Sample based';

fields_array(13) = Simulink.BusElement;
fields_array(13).Name = 'fault_bank_b';
fields_array(13).Dimensions = 1;
fields_array(13).DimensionsMode = 'Fixed';
fields_array(13).DataType = 'uint32';
fields_array(13).SampleTime = -1;
fields_array(13).Complexity = 'real';
fields_array(13).SamplingMode = 'Sample based';

fields_array(14) = Simulink.BusElement;
fields_array(14).Name = 'warning_bank_a';
fields_array(14).Dimensions = 1;
fields_array(14).DimensionsMode = 'Fixed';
fields_array(14).DataType = 'uint32';
fields_array(14).SampleTime = -1;
fields_array(14).Complexity = 'real';
fields_array(14).SamplingMode = 'Sample based';

fields_array(15) = Simulink.BusElement;
fields_array(15).Name = 'warning_bank_b';
fields_array(15).Dimensions = 1;
fields_array(15).DimensionsMode = 'Fixed';
fields_array(15).DataType = 'uint32';
fields_array(15).SampleTime = -1;
fields_array(15).Complexity = 'real';
fields_array(15).SamplingMode = 'Sample based';

Bus_BaseFeedback = Simulink.Bus;
Bus_BaseFeedback.HeaderFile = '';
Bus_BaseFeedback.Description = sprintf('base feedback struct bus');
Bus_BaseFeedback.Elements = fields_array;


%% joint actuators bus definition
max_nbr_actuators = 7;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'status_flags';
fields_array(1).Dimensions = [1 max_nbr_actuators];
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'uint32';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'jitter_comm';
fields_array(2).Dimensions = [1 max_nbr_actuators];
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'uint32';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'position';
fields_array(3).Dimensions = [1 max_nbr_actuators];
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'double';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

fields_array(4) = Simulink.BusElement;
fields_array(4).Name = 'velocity';
fields_array(4).Dimensions = [1 max_nbr_actuators];
fields_array(4).DimensionsMode = 'Fixed';
fields_array(4).DataType = 'double';
fields_array(4).SampleTime = -1;
fields_array(4).Complexity = 'real';
fields_array(4).SamplingMode = 'Sample based';

fields_array(5) = Simulink.BusElement;
fields_array(5).Name = 'torque';
fields_array(5).Dimensions = [1 max_nbr_actuators];
fields_array(5).DimensionsMode = 'Fixed';
fields_array(5).DataType = 'double';
fields_array(5).SampleTime = -1;
fields_array(5).Complexity = 'real';
fields_array(5).SamplingMode = 'Sample based';

fields_array(6) = Simulink.BusElement;
fields_array(6).Name = 'current_motor';
fields_array(6).Dimensions = [1 max_nbr_actuators];
fields_array(6).DimensionsMode = 'Fixed';
fields_array(6).DataType = 'double';
fields_array(6).SampleTime = -1;
fields_array(6).Complexity = 'real';
fields_array(6).SamplingMode = 'Sample based';

fields_array(7) = Simulink.BusElement;
fields_array(7).Name = 'voltage';
fields_array(7).Dimensions = [1 max_nbr_actuators];
fields_array(7).DimensionsMode = 'Fixed';
fields_array(7).DataType = 'double';
fields_array(7).SampleTime = -1;
fields_array(7).Complexity = 'real';
fields_array(7).SamplingMode = 'Sample based';

fields_array(8) = Simulink.BusElement;
fields_array(8).Name = 'temperature_motor';
fields_array(8).Dimensions = [1 max_nbr_actuators];
fields_array(8).DimensionsMode = 'Fixed';
fields_array(8).DataType = 'double';
fields_array(8).SampleTime = -1;
fields_array(8).Complexity = 'real';
fields_array(8).SamplingMode = 'Sample based';

fields_array(9) = Simulink.BusElement;
fields_array(9).Name = 'temperature_core';
fields_array(9).Dimensions = [1 max_nbr_actuators];
fields_array(9).DimensionsMode = 'Fixed';
fields_array(9).DataType = 'double';
fields_array(9).SampleTime = -1;
fields_array(9).Complexity = 'real';
fields_array(9).SamplingMode = 'Sample based';

fields_array(10) = Simulink.BusElement;
fields_array(10).Name = 'fault_bank_a';
fields_array(10).Dimensions = [1 max_nbr_actuators];
fields_array(10).DimensionsMode = 'Fixed';
fields_array(10).DataType = 'uint32';
fields_array(10).SampleTime = -1;
fields_array(10).Complexity = 'real';
fields_array(10).SamplingMode = 'Sample based';

fields_array(11) = Simulink.BusElement;
fields_array(11).Name = 'fault_bank_b';
fields_array(11).Dimensions = [1 max_nbr_actuators];
fields_array(11).DimensionsMode = 'Fixed';
fields_array(11).DataType = 'uint32';
fields_array(11).SampleTime = -1;
fields_array(11).Complexity = 'real';
fields_array(11).SamplingMode = 'Sample based';

fields_array(12) = Simulink.BusElement;
fields_array(12).Name = 'warning_bank_a';
fields_array(12).Dimensions = [1 max_nbr_actuators];
fields_array(12).DimensionsMode = 'Fixed';
fields_array(12).DataType = 'uint32';
fields_array(12).SampleTime = -1;
fields_array(12).Complexity = 'real';
fields_array(12).SamplingMode = 'Sample based';

fields_array(13) = Simulink.BusElement;
fields_array(13).Name = 'warning_bank_b';
fields_array(13).Dimensions = [1 max_nbr_actuators];
fields_array(13).DimensionsMode = 'Fixed';
fields_array(13).DataType = 'uint32';
fields_array(13).SampleTime = -1;
fields_array(13).Complexity = 'real';
fields_array(13).SamplingMode = 'Sample based';

Bus_JointsFeedback = Simulink.Bus;
Bus_JointsFeedback.HeaderFile = '';
Bus_JointsFeedback.Description = sprintf('joints feedback struct bus');
Bus_JointsFeedback.Elements = fields_array;

%% gripper bus definition
max_motor = 10;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'feedback_id';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'uint32';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'status_flags';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'uint32';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'fault_bank_a';
fields_array(3).Dimensions = 1;
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'uint32';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

fields_array(4) = Simulink.BusElement;
fields_array(4).Name = 'fault_bank_b';
fields_array(4).Dimensions = 1;
fields_array(4).DimensionsMode = 'Fixed';
fields_array(4).DataType = 'uint32';
fields_array(4).SampleTime = -1;
fields_array(4).Complexity = 'real';
fields_array(4).SamplingMode = 'Sample based';

fields_array(5) = Simulink.BusElement;
fields_array(5).Name = 'warning_bank_a';
fields_array(5).Dimensions = 1;
fields_array(5).DimensionsMode = 'Fixed';
fields_array(5).DataType = 'uint32';
fields_array(5).SampleTime = -1;
fields_array(5).Complexity = 'real';
fields_array(5).SamplingMode = 'Sample based';

fields_array(6) = Simulink.BusElement;
fields_array(6).Name = 'warning_bank_b';
fields_array(6).Dimensions = 1;
fields_array(6).DimensionsMode = 'Fixed';
fields_array(6).DataType = 'uint32';
fields_array(6).SampleTime = -1;
fields_array(6).Complexity = 'real';
fields_array(6).SamplingMode = 'Sample based';

fields_array(7) = Simulink.BusElement;
fields_array(7).Name = 'motor_count';
fields_array(7).Dimensions = 1;
fields_array(7).DimensionsMode = 'Fixed';
fields_array(7).DataType = 'uint32';
fields_array(7).SampleTime = -1;
fields_array(7).Complexity = 'real';
fields_array(7).SamplingMode = 'Sample based';

fields_array(8) = Simulink.BusElement;
fields_array(8).Name = 'motor_id';
fields_array(8).Dimensions = [1 max_motor];
fields_array(8).DimensionsMode = 'Fixed';
fields_array(8).DataType = 'uint32';
fields_array(8).SampleTime = -1;
fields_array(8).Complexity = 'real';
fields_array(8).SamplingMode = 'Sample based';

fields_array(9) = Simulink.BusElement;
fields_array(9).Name = 'motor_position';
fields_array(9).Dimensions = [1 max_motor];
fields_array(9).DimensionsMode = 'Fixed';
fields_array(9).DataType = 'double';
fields_array(9).SampleTime = -1;
fields_array(9).Complexity = 'real';
fields_array(9).SamplingMode = 'Sample based';

fields_array(10) = Simulink.BusElement;
fields_array(10).Name = 'motor_velocity';
fields_array(10).Dimensions = [1 max_motor];
fields_array(10).DimensionsMode = 'Fixed';
fields_array(10).DataType = 'double';
fields_array(10).SampleTime = -1;
fields_array(10).Complexity = 'real';
fields_array(10).SamplingMode = 'Sample based';

fields_array(11) = Simulink.BusElement;
fields_array(11).Name = 'motor_current_motor';
fields_array(11).Dimensions = [1 max_motor];
fields_array(11).DimensionsMode = 'Fixed';
fields_array(11).DataType = 'double';
fields_array(11).SampleTime = -1;
fields_array(11).Complexity = 'real';
fields_array(11).SamplingMode = 'Sample based';

fields_array(12) = Simulink.BusElement;
fields_array(12).Name = 'motor_voltage';
fields_array(12).Dimensions = [1 max_motor];
fields_array(12).DimensionsMode = 'Fixed';
fields_array(12).DataType = 'double';
fields_array(12).SampleTime = -1;
fields_array(12).Complexity = 'real';
fields_array(12).SamplingMode = 'Sample based';

fields_array(13) = Simulink.BusElement;
fields_array(13).Name = 'motor_temperature_motor';
fields_array(13).Dimensions = [1 max_motor];
fields_array(13).DimensionsMode = 'Fixed';
fields_array(13).DataType = 'double';
fields_array(13).SampleTime = -1;
fields_array(13).Complexity = 'real';
fields_array(13).SamplingMode = 'Sample based';

Bus_Gripper_Feedback = Simulink.Bus;
Bus_Gripper_Feedback.HeaderFile = '';
Bus_Gripper_Feedback.Description = sprintf('gripper struct bus');
Bus_Gripper_Feedback.Elements = fields_array;


%% tool bus definition
clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'feedback_id';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'uint32';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'status_flags';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'uint32';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'jitter_comm';
fields_array(3).Dimensions = 1;
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'uint32';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

fields_array(4) = Simulink.BusElement;
fields_array(4).Name = 'imu_acceleration_x';
fields_array(4).Dimensions = 1;
fields_array(4).DimensionsMode = 'Fixed';
fields_array(4).DataType = 'double';
fields_array(4).SampleTime = -1;
fields_array(4).Complexity = 'real';
fields_array(4).SamplingMode = 'Sample based';

fields_array(5) = Simulink.BusElement;
fields_array(5).Name = 'imu_acceleration_y';
fields_array(5).Dimensions = 1;
fields_array(5).DimensionsMode = 'Fixed';
fields_array(5).DataType = 'double';
fields_array(5).SampleTime = -1;
fields_array(5).Complexity = 'real';
fields_array(5).SamplingMode = 'Sample based';

fields_array(6) = Simulink.BusElement;
fields_array(6).Name = 'imu_acceleration_z';
fields_array(6).Dimensions = 1;
fields_array(6).DimensionsMode = 'Fixed';
fields_array(6).DataType = 'double';
fields_array(6).SampleTime = -1;
fields_array(6).Complexity = 'real';
fields_array(6).SamplingMode = 'Sample based';

fields_array(7) = Simulink.BusElement;
fields_array(7).Name = 'imu_angular_velocity_x';
fields_array(7).Dimensions = 1;
fields_array(7).DimensionsMode = 'Fixed';
fields_array(7).DataType = 'double';
fields_array(7).SampleTime = -1;
fields_array(7).Complexity = 'real';
fields_array(7).SamplingMode = 'Sample based';

fields_array(8) = Simulink.BusElement;
fields_array(8).Name = 'imu_angular_velocity_y';
fields_array(8).Dimensions = 1;
fields_array(8).DimensionsMode = 'Fixed';
fields_array(8).DataType = 'double';
fields_array(8).SampleTime = -1;
fields_array(8).Complexity = 'real';
fields_array(8).SamplingMode = 'Sample based';

fields_array(9) = Simulink.BusElement;
fields_array(9).Name = 'imu_angular_velocity_z';
fields_array(9).Dimensions = 1;
fields_array(9).DimensionsMode = 'Fixed';
fields_array(9).DataType = 'double';
fields_array(9).SampleTime = -1;
fields_array(9).Complexity = 'real';
fields_array(9).SamplingMode = 'Sample based';

fields_array(10) = Simulink.BusElement;
fields_array(10).Name = 'voltage';
fields_array(10).Dimensions = 1;
fields_array(10).DimensionsMode = 'Fixed';
fields_array(10).DataType = 'double';
fields_array(10).SampleTime = -1;
fields_array(10).Complexity = 'real';
fields_array(10).SamplingMode = 'Sample based';

fields_array(11) = Simulink.BusElement;
fields_array(11).Name = 'temperature_core';
fields_array(11).Dimensions = 1;
fields_array(11).DimensionsMode = 'Fixed';
fields_array(11).DataType = 'double';
fields_array(11).SampleTime = -1;
fields_array(11).Complexity = 'real';
fields_array(11).SamplingMode = 'Sample based';

fields_array(12) = Simulink.BusElement;
fields_array(12).Name = 'fault_bank_a';
fields_array(12).Dimensions = 1;
fields_array(12).DimensionsMode = 'Fixed';
fields_array(12).DataType = 'uint32';
fields_array(12).SampleTime = -1;
fields_array(12).Complexity = 'real';
fields_array(12).SamplingMode = 'Sample based';

fields_array(13) = Simulink.BusElement;
fields_array(13).Name = 'fault_bank_b';
fields_array(13).Dimensions = 1;
fields_array(13).DimensionsMode = 'Fixed';
fields_array(13).DataType = 'uint32';
fields_array(13).SampleTime = -1;
fields_array(13).Complexity = 'real';
fields_array(13).SamplingMode = 'Sample based';

fields_array(14) = Simulink.BusElement;
fields_array(14).Name = 'warning_bank_a';
fields_array(14).Dimensions = 1;
fields_array(14).DimensionsMode = 'Fixed';
fields_array(14).DataType = 'uint32';
fields_array(14).SampleTime = -1;
fields_array(14).Complexity = 'real';
fields_array(14).SamplingMode = 'Sample based';

fields_array(15) = Simulink.BusElement;
fields_array(15).Name = 'warning_bank_b';
fields_array(15).Dimensions = 1;
fields_array(15).DimensionsMode = 'Fixed';
fields_array(15).DataType = 'uint32';
fields_array(15).SampleTime = -1;
fields_array(15).Complexity = 'real';
fields_array(15).SamplingMode = 'Sample based';

fields_array(16) = Simulink.BusElement;
fields_array(16).Name = 'gripper_feedback';
fields_array(16).DataType = 'Bus: Bus_Gripper_Feedback';
fields_array(16).SampleTime = -1;
fields_array(16).SamplingMode = 'Sample based';


Bus_ToolFeedback = Simulink.Bus;
Bus_ToolFeedback.HeaderFile = '';
Bus_ToolFeedback.Description = sprintf('tool feedback struct bus');
Bus_ToolFeedback.Elements = fields_array;

busName_BaseFeedback   = 'Bus_BaseFeedback';
busName_JointsFeedback = 'Bus_JointsFeedback';
busName_ToolFeedback   = 'Bus_ToolFeedback';
busName_GripperFeedback   = 'Bus_Gripper_Feedback';

assignin('base', busName_BaseFeedback,   Bus_BaseFeedback);
assignin('base', busName_JointsFeedback, Bus_JointsFeedback);
assignin('base', busName_ToolFeedback,   Bus_ToolFeedback);
assignin('base', busName_GripperFeedback,   Bus_Gripper_Feedback);

