function configModelForKinovaROS(modelName)
% Configures the currently active Simulink model for PostGeneration command
%   This will force Code Generation Only parameters to ON, 
%   and set the Post Code Generation command to call kortexPostCodeGeneration(h)
%   Note that the 'ert_make_rtw_hook.m' hook file will be called in the 
%   Post Code Generation step
    arguments
        modelName (1,:) char = bdroot
    end
    
    set_param(modelName, 'GenCodeOnly', 'on')
    set_param(modelName, 'PostCodeGenCommand', 'kortexPostGeneration(h)');
end