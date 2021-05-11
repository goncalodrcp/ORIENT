function [result] = GetEnumValueSensor(enum_value_name)
    if strcmp(enum_value_name, 'SENSOR COLOR')
        result = 1;
    elseif strcmp(enum_value_name, 'SENSOR DEPTH')
        result = 2;
    else
        result = 0;
    end
end