% initializes a set of bus objects in the MATLAB base workspace

clear fields_array;
fields_array(1) = Simulink.BusElement;
fields_array(1).Name = 'code';
fields_array(1).Dimensions = 1;
fields_array(1).DimensionsMode = 'Fixed';
fields_array(1).DataType = 'uint32';
fields_array(1).SampleTime = -1;
fields_array(1).Complexity = 'real';
fields_array(1).SamplingMode = 'Sample based';

fields_array(2) = Simulink.BusElement;
fields_array(2).Name = 'sub_code';
fields_array(2).Dimensions = 1;
fields_array(2).DimensionsMode = 'Fixed';
fields_array(2).DataType = 'uint32';
fields_array(2).SampleTime = -1;
fields_array(2).Complexity = 'real';
fields_array(2).SamplingMode = 'Sample based';

fields_array(3) = Simulink.BusElement;
fields_array(3).Name = 'description';
fields_array(3).Dimensions = 512;
fields_array(3).DimensionsMode = 'Fixed';
fields_array(3).DataType = 'int8';
fields_array(3).SampleTime = -1;
fields_array(3).Complexity = 'real';
fields_array(3).SamplingMode = 'Sample based';

Bus_Error = Simulink.Bus;
Bus_Error.HeaderFile = '';
Bus_Error.Description = sprintf('error struct bus');
Bus_Error.Elements = fields_array;

busName_Error = 'Bus_Error';

assignin('base', busName_Error, Bus_Error);