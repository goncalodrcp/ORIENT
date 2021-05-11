classdef enum_FocusAction < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        FOCUSACTION_UNSPECIFIED(0),
		FOCUSACTION_START_CONTINUOUS_FOCUS(1),
		FOCUSACTION_PAUSE_CONTINUOUS_FOCUS(2),
		FOCUSACTION_FOCUS_NOW(3),
		FOCUSACTION_DISABLE_FOCUS(4),
		FOCUSACTION_SET_FOCUS_POINT(5),
		FOCUSACTION_SET_MANUAL_FOCUS(6)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = enum_FocusAction.FOCUSACTION_UNSPECIFIED;
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
