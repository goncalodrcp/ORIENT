% initializes a set of bus objects in the MATLAB base workspace

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'sensor';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'uint32';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'resolution';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'uint32';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'frame_rate';
fields_array(3).Dimensions = 1;
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'uint32';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

fields_array(4) = Simulink.BusElement;
fields_array(4).Name = 'bit_rate';
fields_array(4).Dimensions = 1;
fields_array(4).DimensionsMode = 'Fixed';
fields_array(4).DataType = 'uint32';
fields_array(4).SampleTime = -1;
fields_array(4).Complexity = 'real';
fields_array(4).SamplingMode = 'Sample based';

Bus_ColorSettings = Simulink.Bus;
Bus_ColorSettings.HeaderFile = '';
Bus_ColorSettings.Description = sprintf('color settings struct bus');
Bus_ColorSettings.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'sensor';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'uint32';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'resolution';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'uint32';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'frame_rate';
fields_array(3).Dimensions = 1;
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'uint32';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

fields_array(4) = Simulink.BusElement;
fields_array(4).Name = 'bit_rate';
fields_array(4).Dimensions = 1;
fields_array(4).DimensionsMode = 'Fixed';
fields_array(4).DataType = 'uint32';
fields_array(4).SampleTime = -1;
fields_array(4).Complexity = 'real';
fields_array(4).SamplingMode = 'Sample based';

Bus_DepthSettings = Simulink.Bus;
Bus_DepthSettings.HeaderFile = '';
Bus_DepthSettings.Description = sprintf('depth settings struct bus');
Bus_DepthSettings.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'brightness';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'double';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'contrast';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'double';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'saturation';
fields_array(3).Dimensions = 1;
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'double';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

Bus_ColorOption = Simulink.Bus;
Bus_ColorOption.HeaderFile = '';
Bus_ColorOption.Description = sprintf('color option struct bus');
Bus_ColorOption.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'exposure';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'double';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'gain';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'double';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'visual_preset';
fields_array(3).Dimensions = 1;
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'double';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

fields_array(4) = Simulink.BusElement;
fields_array(4).Name = 'frames_queue_size';
fields_array(4).Dimensions = 1;
fields_array(4).DimensionsMode = 'Fixed';
fields_array(4).DataType = 'double';
fields_array(4).SampleTime = -1;
fields_array(4).Complexity = 'real';
fields_array(4).SamplingMode = 'Sample based';

fields_array(5) = Simulink.BusElement;
fields_array(5).Name = 'depth_units';
fields_array(5).Dimensions = 1;
fields_array(5).DimensionsMode = 'Fixed';
fields_array(5).DataType = 'double';
fields_array(5).SampleTime = -1;
fields_array(5).Complexity = 'real';
fields_array(5).SamplingMode = 'Sample based';

fields_array(6) = Simulink.BusElement;
fields_array(6).Name = 'enable_auto_exposure';
fields_array(6).Dimensions = 1;
fields_array(6).DimensionsMode = 'Fixed';
fields_array(6).DataType = 'double';
fields_array(6).SampleTime = -1;
fields_array(6).Complexity = 'real';
fields_array(6).SamplingMode = 'Sample based';

fields_array(7) = Simulink.BusElement;
fields_array(7).Name = 'error_polling_enable';
fields_array(7).Dimensions = 1;
fields_array(7).DimensionsMode = 'Fixed';
fields_array(7).DataType = 'double';
fields_array(7).SampleTime = -1;
fields_array(7).Complexity = 'real';
fields_array(7).SamplingMode = 'Sample based';

fields_array(8) = Simulink.BusElement;
fields_array(8).Name = 'output_trigger_enable';
fields_array(8).Dimensions = 1;
fields_array(8).DimensionsMode = 'Fixed';
fields_array(8).DataType = 'double';
fields_array(8).SampleTime = -1;
fields_array(8).Complexity = 'real';
fields_array(8).SamplingMode = 'Sample based';

Bus_DepthOption = Simulink.Bus;
Bus_DepthOption.HeaderFile = '';
Bus_DepthOption.Description = sprintf('depth option struct bus');
Bus_DepthOption.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'sensor';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'uint32';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'resolution';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'uint32';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'principal_point_x';
fields_array(3).Dimensions = 1;
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'double';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

fields_array(4) = Simulink.BusElement;
fields_array(4).Name = 'principal_point_y';
fields_array(4).Dimensions = 1;
fields_array(4).DimensionsMode = 'Fixed';
fields_array(4).DataType = 'double';
fields_array(4).SampleTime = -1;
fields_array(4).Complexity = 'real';
fields_array(4).SamplingMode = 'Sample based';

fields_array(5) = Simulink.BusElement;
fields_array(5).Name = 'focal_length_x';
fields_array(5).Dimensions = 1;
fields_array(5).DimensionsMode = 'Fixed';
fields_array(5).DataType = 'double';
fields_array(5).SampleTime = -1;
fields_array(5).Complexity = 'real';
fields_array(5).SamplingMode = 'Sample based';

fields_array(6) = Simulink.BusElement;
fields_array(6).Name = 'focal_length_y';
fields_array(6).Dimensions = 1;
fields_array(6).DimensionsMode = 'Fixed';
fields_array(6).DataType = 'double';
fields_array(6).SampleTime = -1;
fields_array(6).Complexity = 'real';
fields_array(6).SamplingMode = 'Sample based';

fields_array(7) = Simulink.BusElement;
fields_array(7).Name = 'k1';
fields_array(7).Dimensions = 1;
fields_array(7).DimensionsMode = 'Fixed';
fields_array(7).DataType = 'double';
fields_array(7).SampleTime = -1;
fields_array(7).Complexity = 'real';
fields_array(7).SamplingMode = 'Sample based';

fields_array(8) = Simulink.BusElement;
fields_array(8).Name = 'k2';
fields_array(8).Dimensions = 1;
fields_array(8).DimensionsMode = 'Fixed';
fields_array(8).DataType = 'double';
fields_array(8).SampleTime = -1;
fields_array(8).Complexity = 'real';
fields_array(8).SamplingMode = 'Sample based';

fields_array(9) = Simulink.BusElement;
fields_array(9).Name = 'k3';
fields_array(9).Dimensions = 1;
fields_array(9).DimensionsMode = 'Fixed';
fields_array(9).DataType = 'double';
fields_array(9).SampleTime = -1;
fields_array(9).Complexity = 'real';
fields_array(9).SamplingMode = 'Sample based';

fields_array(10) = Simulink.BusElement;
fields_array(10).Name = 'p1';
fields_array(10).Dimensions = 1;
fields_array(10).DimensionsMode = 'Fixed';
fields_array(10).DataType = 'double';
fields_array(10).SampleTime = -1;
fields_array(10).Complexity = 'real';
fields_array(10).SamplingMode = 'Sample based';

fields_array(11) = Simulink.BusElement;
fields_array(11).Name = 'p2';
fields_array(11).Dimensions = 1;
fields_array(11).DimensionsMode = 'Fixed';
fields_array(11).DataType = 'double';
fields_array(11).SampleTime = -1;
fields_array(11).Complexity = 'real';
fields_array(11).SamplingMode = 'Sample based';

Bus_Intrinsic = Simulink.Bus;
Bus_Intrinsic.HeaderFile = '';
Bus_Intrinsic.Description = sprintf('intrinsic parameters struct bus');
Bus_Intrinsic.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'x';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'double';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'y';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'double';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'z';
fields_array(3).Dimensions = 1;
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'double';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

Bus_Translation = Simulink.Bus;
Bus_Translation.HeaderFile = '';
Bus_Translation.Description = sprintf('translation vector struct bus');
Bus_Translation.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'row_1';
fields_array(1).Dimensions = [1 3];
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'double';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'row_2';
fields_array(2).Dimensions = [1 3];
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'double';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'row_3';
fields_array(3).Dimensions = [1 3];
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'double';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

Bus_Rotation = Simulink.Bus;
Bus_Rotation.HeaderFile = '';
Bus_Rotation.Description = sprintf('rotation matrix struct bus');
Bus_Rotation.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'rotation';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'Bus: Bus_Rotation';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'translation';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'Bus: Bus_Translation';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

Bus_Extrinsic = Simulink.Bus;
Bus_Extrinsic.HeaderFile = '';
Bus_Extrinsic.Description = sprintf('extrinsic parameters struct bus');
Bus_Extrinsic.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'color_intrinsic';
fields_array(1).DataType = 'Bus: Bus_Intrinsic';
fields_array(1).SampleTime = -1;
fields_array(1).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'depth_intrinsic';
fields_array(3).DataType = 'Bus: Bus_Intrinsic';
fields_array(3).SampleTime = -1;
fields_array(3).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'extrinsic';
fields_array(2).DataType = 'Bus: Bus_Extrinsic';
fields_array(2).SampleTime = -1;
fields_array(2).SamplingMode = 'Sample based';

Bus_Calibration = Simulink.Bus;
Bus_Calibration.HeaderFile = '';
Bus_Calibration.Description = sprintf('calibration struct bus');
Bus_Calibration.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'color_settings';
fields_array(1).DataType = 'Bus: Bus_ColorSettings';
fields_array(1).SampleTime = -1;
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'depth_settings';
fields_array(2).DataType = 'Bus: Bus_DepthSettings';
fields_array(2).SampleTime = -1;
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'color_option';
fields_array(3).DataType = 'Bus: Bus_ColorOption';
fields_array(3).SampleTime = -1;
fields_array(3).SamplingMode = 'Sample based';

fields_array(4) = Simulink.BusElement;
fields_array(4).Name = 'depth_option';
fields_array(4).DataType = 'Bus: Bus_DepthOption';
fields_array(4).SampleTime = -1;
fields_array(4).SamplingMode = 'Sample based';

fields_array(5) = Simulink.BusElement;
fields_array(5).Name = 'calibration_parameters';
fields_array(5).DataType = 'Bus: Bus_Calibration';
fields_array(5).SampleTime = -1;
fields_array(5).SamplingMode = 'Sample based';

Bus_VisionConfig = Simulink.Bus;
Bus_VisionConfig.HeaderFile = '';
Bus_VisionConfig.Description = sprintf('vision config struct bus');
Bus_VisionConfig.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'sensor';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'uint32';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'option';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'uint32';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'value';
fields_array(3).Dimensions = 1;
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'double';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

Bus_OptionValue = Simulink.Bus;
Bus_OptionValue.HeaderFile = '';
Bus_OptionValue.Description = sprintf('Option value from a sensors of the camera.');
Bus_OptionValue.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'sensor';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'uint32';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'option';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'uint32';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

Bus_OptionIdentifier = Simulink.Bus;
Bus_OptionIdentifier.HeaderFile = '';
Bus_OptionIdentifier.Description = sprintf('Option ID.');
Bus_OptionIdentifier.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'sensor';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'uint32';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'focus_action';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'uint32';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

Bus_SensorFocusAction = Simulink.Bus;
Bus_SensorFocusAction.HeaderFile = '';
Bus_SensorFocusAction.Description = sprintf('A focus action that will be applied on the camera.');
Bus_SensorFocusAction.Elements = fields_array;


busName_VisionConfig = 'Bus_VisionConfig';
busName_SensorSettings = 'Bus_ColorSettings';
busName_DepthSettings = 'Bus_DepthSettings';
busName_ColorOption = 'Bus_ColorOption';
busName_DepthOption = 'Bus_DepthOption';
busName_Calibration = 'Bus_Calibration';
busName_Intrinsic = 'Bus_Intrinsic';
busName_Extrinsic = 'Bus_Extrinsic';
busName_Rotation = 'Bus_Rotation';
busName_Translation = 'Bus_Translation';
busName_OptionValue = 'Bus_OptionValue';
busName_OptionIdentifier = 'Bus_OptionIdentifier';
busName_SensorFocusAction = 'Bus_SensorFocusAction';


assignin('base', busName_VisionConfig, Bus_VisionConfig);
assignin('base', busName_SensorSettings, Bus_ColorSettings);
assignin('base', busName_DepthSettings, Bus_DepthSettings);
assignin('base', busName_ColorOption, Bus_ColorOption);
assignin('base', busName_DepthOption, Bus_DepthOption);
assignin('base', busName_Intrinsic, Bus_Intrinsic);
assignin('base', busName_Extrinsic, Bus_Extrinsic);
assignin('base', busName_Calibration, Bus_Calibration);
assignin('base', busName_Rotation, Bus_Rotation);
assignin('base', busName_Translation, Bus_Translation);
assignin('base', busName_OptionValue, Bus_OptionValue);
assignin('base', busName_OptionIdentifier, Bus_OptionIdentifier);
assignin('base', busName_SensorFocusAction, Bus_SensorFocusAction);


