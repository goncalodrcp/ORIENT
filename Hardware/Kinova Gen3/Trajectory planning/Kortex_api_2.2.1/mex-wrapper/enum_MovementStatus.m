classdef enum_MovementStatus < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        MOVEMENT_STATUS_IDLE(0),
		MOVEMENT_STATUS_RUNNING(1),
		MOVEMENT_STATUS_PAUSED(2),
		MOVEMENT_STATUS_ABORTED(3),
		MOVEMENT_STATUS_COMPLETED(4),
		PRE_PROCESSING(5)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = enum_MovementStatus.MOVEMENT_STATUS_IDLE;
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
