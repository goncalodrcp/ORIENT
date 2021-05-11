classdef enum_TrajectoryContinuityMode < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        TRAJECTORY_CONTINUITY_MODE_UNDEFINED(0),
		TRAJECTORY_CONTINUITY_MODE_POSITION(1),
		TRAJECTORY_CONTINUITY_MODE_SPEED(2),
		TRAJECTORY_CONTINUITY_MODE_ACCELERATION(3)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = enum_TrajectoryContinuityMode.TRAJECTORY_CONTINUITY_MODE_UNDEFINED;
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
