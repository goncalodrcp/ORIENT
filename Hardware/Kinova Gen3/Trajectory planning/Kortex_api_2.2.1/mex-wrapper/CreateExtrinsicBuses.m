function [ busName_ExtrinsicParam ] = CreateExtrinsicBuses()
% initializes a set of bus objects in the MATLAB base workspace

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 't_x';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'double';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 't_y';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'double';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 't_z';
fields_array(3).Dimensions = 1;
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'double';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

Bus_TranslationParam = Simulink.Bus;
Bus_TranslationParam.HeaderFile = '';
Bus_TranslationParam.Description = sprintf('translation vector struct bus');
Bus_TranslationParam.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'row1';
fields_array(1).Dimensions = [1 3];
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'double';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'row2';
fields_array(2).Dimensions = [1 3];
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'double';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'row3';
fields_array(3).Dimensions = [1 3];
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'double';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

Bus_RotationParam = Simulink.Bus;
Bus_RotationParam.HeaderFile = '';
Bus_RotationParam.Description = sprintf('rotation matrix struct bus');
Bus_RotationParam.Elements = fields_array;

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'rotation';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'Bus: Bus_RotationParam';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'translation';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'Bus: Bus_TranslationParam';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

Bus_ExtrinsicParam = Simulink.Bus;
Bus_ExtrinsicParam.HeaderFile = '';
Bus_ExtrinsicParam.Description = sprintf('extrinsic parameters struct bus');
Bus_ExtrinsicParam.Elements = fields_array;

busName_ExtrinsicParam = 'Bus_ExtrinsicParam';
busName_TranslationParam = 'Bus_TranslationParam';
busName_RotationParam = 'Bus_RotationParam';

assignin('base', busName_ExtrinsicParam, Bus_ExtrinsicParam);
assignin('base', busName_TranslationParam, Bus_TranslationParam);
assignin('base', busName_RotationParam, Bus_RotationParam);
