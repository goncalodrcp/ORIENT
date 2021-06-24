function [ busName_IntrinsicParam ] = CreateIntrinsicBuses()
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
fields_array(7).Name = 'distortion_coeffs';
fields_array(7).Dimensions = [1 5];
fields_array(7).DimensionsMode = 'Fixed';
fields_array(7).DataType = 'double';
fields_array(7).SampleTime = -1;
fields_array(7).Complexity = 'real';
fields_array(7).SamplingMode = 'Sample based';


Bus_IntrinsicParam = Simulink.Bus;
Bus_IntrinsicParam.HeaderFile = '';
Bus_IntrinsicParam.Description = sprintf('intrinsic parameters struct bus');
Bus_IntrinsicParam.Elements = fields_array;

busName_IntrinsicParam = 'Bus_IntrinsicParam';

assignin('base', busName_IntrinsicParam, Bus_IntrinsicParam);
