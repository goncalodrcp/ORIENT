function [result] = GetEnumValueBitRate(enum_value_name)
    if strcmp(enum_value_name, '10 MBPS')
        result = 1;
    elseif strcmp(enum_value_name, '15 MBPS')
        result = 2;
    elseif strcmp(enum_value_name, '20 MBPS')
        result = 3;
    elseif strcmp(enum_value_name, '25 MBPS')
        result = 4;
    else
        result = 0;
    end
end