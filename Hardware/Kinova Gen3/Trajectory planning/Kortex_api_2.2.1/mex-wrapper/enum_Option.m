classdef enum_Option < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        OPTION_UNSPECIFIED(0),
		OPTION_BACKLIGHT_COMPENSATION(1),
		OPTION_BRIGHTNESS(2),
		OPTION_CONTRAST(3),
		OPTION_EXPOSURE(4),
		OPTION_GAIN(5),
		OPTION_GAMMA(6),
		OPTION_HUE(7),
		OPTION_SATURATION(8),
		OPTION_SHARPNESS(9),
		OPTION_WHITE_BALANCE(10),
		OPTION_ENABLE_AUTO_EXPOSURE(11),
		OPTION_ENABLE_AUTO_WHITE_BALANCE(12),
		OPTION_VISUAL_PRESET(13),
		OPTION_LASER_POWER(14),
		OPTION_ACCURACY(15),
		OPTION_MOTION_RANGE(16),
		OPTION_FILTER_OPTION(17),
		OPTION_CONFIDENCE_THRESHOLD(18),
		OPTION_EMITTER_ENABLED(19),
		OPTION_FRAMES_QUEUE_SIZE(20),
		OPTION_TOTAL_FRAME_DROPS(21),
		OPTION_AUTO_EXPOSURE_MODE(22),
		OPTION_POWER_LINE_FREQUENCY(23),
		OPTION_ASIC_TEMPERATURE(24),
		OPTION_ERROR_POLLING_ENABLED(25),
		OPTION_PROJECTOR_TEMPERATURE(26),
		OPTION_OUTPUT_TRIGGER_ENABLED(27),
		OPTION_MOTION_MODULE_TEMPERATURE(28),
		OPTION_DEPTH_UNITS(29),
		OPTION_ENABLE_MOTION_CORRECTION(30),
		OPTION_AUTO_EXPOSURE_PRIORITY(31),
		OPTION_COLOR_SCHEME(32),
		OPTION_HISTOGRAM_EQUALIZATION_ENABLED(33),
		OPTION_MIN_DISTANCE(34),
		OPTION_MAX_DISTANCE(35),
		OPTION_TEXTURE_SOURCE(36),
		OPTION_FILTER_MAGNITUDE(37),
		OPTION_FILTER_SMOOTH_ALPHA(38),
		OPTION_FILTER_SMOOTH_DELTA(39),
		OPTION_HOLES_FILL(40),
		OPTION_STEREO_BASELINE(41),
		OPTION_AUTO_EXPOSURE_CONVERGE_STEP(42)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = enum_Option.OPTION_UNSPECIFIED;
        end

        function dScope = getDataScope()
            % GETDATASCOPE  Specifies whether the data type definition should be imported from,
            %               or exported to, a header file during code generation.
            dScope = 'Imported';
        end

        function desc = getDescription()
            % GETDESCRIPTION  Returns a description of the enumeration.
            desc = '';
        end
        
        function headerFile = getHeaderFile()
            % GETHEADERFILE  Specifies the name of a header file. 
            headerFile = 'kortex_wrapper_data.h';
        end
        
        function flag = addClassNameToEnumNames()
            % ADDCLASSNAMETOENUMNAMES  Indicate whether code generator applies the class name as a prefix
            %                          to the enumeration.
            flag = false;
        end

    end

end
