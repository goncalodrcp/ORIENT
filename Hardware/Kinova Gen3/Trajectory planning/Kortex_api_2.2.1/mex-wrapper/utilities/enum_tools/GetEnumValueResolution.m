function [result] = GetEnumValueResolution(enum_value_name)
    if strcmp(enum_value_name, '320x240')
        result = 1;
    elseif strcmp(enum_value_name, '424x240')
        result = 2;
    elseif strcmp(enum_value_name, '480x270')
        result = 3;
    elseif strcmp(enum_value_name, '640x480')
        result = 4;
    elseif strcmp(enum_value_name, '1280x720')
        result = 5;
    elseif strcmp(enum_value_name, '1920x1080')
        result = 6;
    else
        result = 0;
    end
end