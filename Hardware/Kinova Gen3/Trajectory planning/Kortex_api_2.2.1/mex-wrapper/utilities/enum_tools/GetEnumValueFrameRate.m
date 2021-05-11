function [result] = GetEnumValueFrameRate(enum_value_name)
    if strcmp(enum_value_name, '6 FPS')
        result = 1;
    elseif strcmp(enum_value_name, '15 FPS')
        result = 2;
    elseif strcmp(enum_value_name, '30 FPS')
        result = 3;
    else
        result = 0;
    end
end