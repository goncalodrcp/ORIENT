/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*/


// Mex entry point (interface) file for Kortex API wrapper

#include <sstream>
#include <cstring>
#include <map>
#include <vector>
#include <mex.h>

#include <kortexApiWrapper.h>

#include <mexConverter.h>

// Constant managing the size of 'static std::vector<mxClassID> requiredParams' in mexFunction
const size_t INITIAL_REQUIRED_PARAMS_SIZE = 25;

// Dimension size of Principal Point in Matlab
const auto PRINCIPAL_POINT_DIMENSION_SIZE = 2u;

// Dimension size of Focal Length in Matlab
const auto FOCAL_LENGTH_DIMENSION_SIZE = 2u;

// Dimension size of Distortion Coefficients in Matlab
const auto DISTORTIION_COEFFICIENTS_DIMENSION_SIZE = 5u;

//Function that checks the parameter count each time a mex command is called.
uint32_t checkInputParametersLength( int numParamsIn, int numParamsRequired) 
{
    if (numParamsIn - 1 != numParamsRequired)
    {
        return KortexErrorCodes::ERROR_INVALID_PARAM;
    }
    return KortexErrorCodes::SUB_ERROR_NONE;
}

//Function that validates the parameters each time a mex function is called.
uint32_t checkInputParameters(int numParamsIn, int numParamsRequired, const mxArray *pParamsInArray[], std::vector<mxClassID>& pParamsRequiredArray)
{
    const auto ret = checkInputParametersLength(numParamsIn, numParamsRequired);
    
    if (ret != KortexErrorCodes::SUB_ERROR_NONE){
        return ret;
    }

    if (pParamsInArray == NULL)
    {
        return KortexErrorCodes::ERROR_INVALID_PARAM;
    }

    for (int i=0; i<numParamsRequired; i++)
    {
        // Skip the command name (index 0)
        mxClassID category = mxGetClassID(pParamsInArray[i + 1]);

        if (category != pParamsRequiredArray[i])
        {
            return KortexErrorCodes::ERROR_INVALID_PARAM;
        }
    }

    return KortexErrorCodes::SUB_ERROR_NONE;
}

//Function that sets the error code each time a mex function is called.
void setReturnedStatus(mxArray* plhs[], uint32_t& plhs_index, uint32_t error_code)
{
    // Return status
    plhs[plhs_index] = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *mxGetUint32s(plhs[plhs_index++]) = error_code;
}


//The gateway function
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Vector used to validate input parameters (size and types)
    static std::vector<mxClassID> requiredParams;
    
    if (requiredParams.empty())
        requiredParams.reserve(INITIAL_REQUIRED_PARAMS_SIZE);

    uint32_t in_apiHandle = 0;

    uint32_t error_code = KortexErrorCodes::ERROR_UNSUPPORTED_METHOD;
    bool hasVision = false;

    // Get function name 
    char command[128];
    mxGetString(prhs[0], command, 128);

    if (!strcmp(command, "GetLastError"))
    {
        requiredParams = { mxUINT32_CLASS };

        uint32_t last_error;

        if ( !checkInputParameters(nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);

            error_code = kapi_GetLastError(in_apiHandle, &last_error);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);

        plhs[plhs_index] = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
        *mxGetUint32s(plhs[plhs_index++]) = last_error;
    }
    else if (!strcmp(command, "RefreshFeedback"))
    {
        BaseFeedback            out_baseFeedback = {0};
        ActuatorsFeedback       out_actuatorsFeedback = {0};
        InterconnectFeedback    out_interconnectFeedback = {0};

        requiredParams = { mxUINT32_CLASS };

        if ( !checkInputParameters(nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);

            error_code = kapi_RefreshFeedback(in_apiHandle, &out_baseFeedback, &out_actuatorsFeedback, &out_interconnectFeedback);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);

        plhs[plhs_index++] = KortexMatlab::createBaseFeedbackStructMatrix(&out_baseFeedback);
        plhs[plhs_index++] = KortexMatlab::createActuatorsFeedbackStructMatrix(MAX_JOINTS, &out_actuatorsFeedback);
        plhs[plhs_index++] = KortexMatlab::createInterconnectFeedbackStructMatrix(&out_interconnectFeedback);
    }
    else if (
        !strcmp(command, "StopAction") || 
        !strcmp(command, "PauseAction") || 
        !strcmp(command, "ResumeAction") || 
        !strcmp(command, "ClearFaults") || 
        !strcmp(command, "ApplyEmergencyStop")
        )
    {
        requiredParams = { mxUINT32_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);

            if (!strcmp(command, "StopAction"))
            {
                error_code = kapi_StopAction(in_apiHandle);
            }
            else if (!strcmp(command, "PauseAction"))
            {
                error_code = kapi_PauseAction(in_apiHandle);
            }
            else if (!strcmp(command, "ResumeAction"))
            {
                error_code = kapi_ResumeAction(in_apiHandle);
            }
            else if (!strcmp(command, "ClearFaults"))
            {
                error_code = kapi_ClearFaults(in_apiHandle);
            }
            else if (!strcmp(command, "ApplyEmergencyStop"))
            {
                error_code = kapi_ApplyEmergencyStop(in_apiHandle);
            }
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
    }
    else if (!strcmp(command, "GetMovementStatus"))
    {
        int32_t out_movementStatus = MOVEMENT_STATUS_IDLE;

        requiredParams = { mxUINT32_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);

            error_code = kapi_GetMovementStatus(in_apiHandle, &out_movementStatus);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);

        plhs[plhs_index] = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
        *mxGetInt32s(plhs[plhs_index++]) = static_cast<int32_t>(out_movementStatus);
    }
    else if (!strcmp(command, "GetErrorName"))
    {
        MovementStatus out_movementStatus = MOVEMENT_STATUS_IDLE;
        static char name[MAX_ENUM_NAME_SIZE];

        requiredParams = { mxUINT32_CLASS, mxUINT32_CLASS };

        size_t buflen;
        uint32_t prhs_index = 0;
        in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);

        uint32_t prhs_error_code = 0;
        prhs_error_code = *mxGetUint32s(prhs[++prhs_index]);

        error_code = kapi_GetErrorName(in_apiHandle,  prhs_error_code, name);

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);

        plhs[plhs_index++] = mxCreateString(name);
    }
    else if (!strcmp(command, "ReachCartesianPose"))
    {
        requiredParams = { mxUINT32_CLASS, mxINT32_CLASS, mxDOUBLE_CLASS, mxDOUBLE_CLASS, mxDOUBLE_CLASS, mxDOUBLE_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            in_apiHandle                = *mxGetUint32s(prhs[++prhs_index]);
            CartesianConstraintType in_constraintType = (CartesianConstraintType)*mxGetInt32s(prhs[++prhs_index]);
            double* in_pSpeed           = mxGetDoubles(prhs[++prhs_index]);
            bool nbrSpeedOk             = ((size_t)mxGetN(prhs[prhs_index]) == 2);  // [translation, orientation]
            double in_duration          = *mxGetDoubles(prhs[++prhs_index]);
            double* in_pPosition        = mxGetDoubles(prhs[++prhs_index]);
            bool nbrPositionOk          = ((size_t)mxGetN(prhs[prhs_index]) == 3);  // [x, y, z]
            double* in_pOrientation     = mxGetDoubles(prhs[++prhs_index]);
            bool nbrOrientationOk       = ((size_t)mxGetN(prhs[prhs_index]) == 3);  // [theta_x, theta_y, theta_z]

            // Validate size of input arrays before calling the wrapper function
            if (nbrSpeedOk && nbrPositionOk && nbrOrientationOk)
            {
                error_code = kapi_ReachCartesianPose(in_apiHandle,  in_constraintType, in_pSpeed[0], in_pSpeed[1], in_duration, in_pPosition, in_pOrientation);
            }
            else
            {
                error_code = KortexErrorCodes::ERROR_INVALID_PARAM;
            }

        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
    }
    else if (!strcmp(command, "ReachJointAngles"))
    {
        requiredParams = { mxUINT32_CLASS, mxINT32_CLASS, mxDOUBLE_CLASS, mxDOUBLE_CLASS, mxDOUBLE_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            in_apiHandle                = *mxGetUint32s(prhs[++prhs_index]);
            JointConstraintType in_constraintType = (JointConstraintType)*mxGetInt32s(prhs[++prhs_index]);
            double in_speed             = *mxGetDoubles(prhs[++prhs_index]);
            double in_duration          = *mxGetDoubles(prhs[++prhs_index]);
            double* in_pJointAngles     = mxGetDoubles(prhs[++prhs_index]);
            uint32_t nbrJointAngles     = (size_t)mxGetN(prhs[prhs_index]);
            bool nbrJointAnglesOk       = (nbrJointAngles <= MAX_JOINTS);

            // Validate size of input arrays before calling the wrapper function
            error_code = nbrJointAnglesOk;
            if (!error_code)
            {
                std::stringstream errorStream;
                errorStream << "kortexApiMexInterface - Invalid array arguments";
            }
            else
            {
                error_code = kapi_ReachJointAngles(in_apiHandle, in_constraintType, in_speed, in_duration, in_pJointAngles, nbrJointAngles);
            }
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);   // Inserts status information at plhs[plhs_index] and increments plhs_index
    }
    else if (!strcmp(command, "SendJointSpeedCommand"))
    {
        requiredParams = { mxUINT32_CLASS, mxDOUBLE_CLASS, mxDOUBLE_CLASS, mxUINT32_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            in_apiHandle                = *mxGetUint32s(prhs[++prhs_index]);
            double in_duration          = *mxGetDoubles(prhs[++prhs_index]);
            double* in_pJointSpeeds     = mxGetDoubles(prhs[++prhs_index]);
            uint32_t nbrJointSpeeds     = (size_t)mxGetN(prhs[prhs_index]);
            bool nbrJointSpeedsOk       = (nbrJointSpeeds <= MAX_JOINTS);

            if (nbrJointSpeedsOk)
            {
                error_code = kapi_SendJointSpeedCommand(in_apiHandle,  in_duration, in_pJointSpeeds, nbrJointSpeeds);
            }
            else
            {
                error_code = KortexErrorCodes::ERROR_INVALID_PARAM;
            }
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
    }
    else if (!strcmp(command, "SendToolCommand"))
    {
        requiredParams = { mxUINT32_CLASS, mxINT32_CLASS, mxDOUBLE_CLASS, mxDOUBLE_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            in_apiHandle                = *mxGetUint32s(prhs[++prhs_index]);
            ToolMode in_mode            = static_cast<ToolMode>( *mxGetInt32s(prhs[++prhs_index]) );
            double in_duration          = *mxGetDoubles(prhs[++prhs_index]);
            double* in_pToolCommands    = mxGetDoubles(prhs[++prhs_index]);
            uint32_t nbrToolActuators   = (size_t)mxGetN(prhs[prhs_index]);

            error_code = kapi_SendToolCommand(in_apiHandle,  in_mode, in_duration, in_pToolCommands, nbrToolActuators);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code); // Inserts status information at plhs[plhs_index] and increments plhs_index

    }
    else if (!strcmp(command, "SetAdmittance"))
    {
        requiredParams = { mxUINT32_CLASS, mxINT32_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            in_apiHandle                = *mxGetUint32s(prhs[++prhs_index]);
            AdmittanceMode in_mode      = (AdmittanceMode)*mxGetInt32s(prhs[++prhs_index]);

            error_code = kapi_SetAdmittance(in_apiHandle,  in_mode);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
    }
    else if (!strcmp(command, "PlayPreComputedTrajectory"))
    {
        requiredParams = { mxUINT32_CLASS, mxUINT32_CLASS, mxINT32_CLASS, mxDOUBLE_CLASS, mxDOUBLE_CLASS, mxDOUBLE_CLASS, mxDOUBLE_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            in_apiHandle      = *mxGetUint32s(prhs[++prhs_index]);
            
            uint32_t position_count = *mxGetUint32s(prhs[++prhs_index]);;

            TrajectoryContinuityMode in_mode = (TrajectoryContinuityMode)*mxGetInt32s(prhs[++prhs_index]);

            double* in_pPosition        = mxGetDoubles(prhs[++prhs_index]);
            uint32_t nbrPosition_Joint  = (size_t)mxGetM(prhs[prhs_index]);
            bool nbrPositionOk          = (nbrPosition_Joint <= MAX_JOINTS && position_count <= MAX_PRE_COMPUTED_TRAJECTORY_ELEMENTS);

            double* in_pVelocity        = mxGetDoubles(prhs[++prhs_index]);
            uint32_t nbrVelocity_Joint  = (size_t)mxGetM(prhs[prhs_index]);
            bool nbrVelocityOk          = (nbrVelocity_Joint <= MAX_JOINTS && position_count <= MAX_PRE_COMPUTED_TRAJECTORY_ELEMENTS);

            double* in_pAcceleration        = mxGetDoubles(prhs[++prhs_index]);
            uint32_t nbrAcceleration_Joint  = (size_t)mxGetM(prhs[prhs_index]);
            bool nbrAccelerationOk          = (nbrAcceleration_Joint <= MAX_JOINTS && position_count <= MAX_PRE_COMPUTED_TRAJECTORY_ELEMENTS);

            double* in_pTimestamp       = mxGetDoubles(prhs[++prhs_index]);
            bool nbrTimestampOk         = ((size_t)mxGetM(prhs[prhs_index]) == 1 && position_count <= MAX_PRE_COMPUTED_TRAJECTORY_ELEMENTS);

            // Make sure input arrays have the same size for their 'Joint' dimension
            bool nbrJointAnglesOk       = nbrPositionOk && nbrVelocityOk && nbrAccelerationOk && nbrTimestampOk;
            uint32_t nbrJointAngles     = nbrPosition_Joint;

            // Make sure input arrays have the same size for their 'Step' dimension
            bool nbrStepsOk         = nbrPositionOk && nbrVelocityOk && nbrAccelerationOk && nbrTimestampOk;

            // Validate size of input arrays before calling the wrapper function
            if (nbrJointAnglesOk && nbrStepsOk)
            {
                error_code = kapi_PlayPreComputedTrajectory(in_apiHandle,  in_mode, in_pPosition, in_pVelocity, in_pAcceleration, 
                                                     in_pTimestamp, nbrJointAngles, position_count);
            }
            else
            {
                error_code = KortexErrorCodes::ERROR_INVALID_PARAM;
            }
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
    }
    else if (!strcmp(command, "CreateRobotApisWrapper"))
    {
        uint32_t out_apiHandle = 0;
        uint32_t visionDeviceID = 0;
        uint32_t error_code = 0;

        requiredParams = { mxCHAR_CLASS, mxCHAR_CLASS, mxCHAR_CLASS, mxUINT32_CLASS, mxUINT32_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;

            /* NOTE: The C-style string is always terminated with a NULL character and stored in column-major order. */
            char* in_pIpAddress    = mxArrayToUTF8String(prhs[++prhs_index]);
            char* in_pUsername     = mxArrayToUTF8String(prhs[++prhs_index]);
            char* in_pPassword     = mxArrayToUTF8String(prhs[++prhs_index]);
            uint32_t in_sessionTimeoutMs = *mxGetUint32s(prhs[++prhs_index]);
            uint32_t in_controlTimeoutMs = *mxGetUint32s(prhs[++prhs_index]);

            if( nullptr == in_pIpAddress
                    || nullptr == in_pUsername
                    || nullptr == in_pPassword
                )
            {
                throw std::runtime_error("Out of memory. Not enough space to create string for ipaddress, username or password");
            }

            error_code = kapi_CreateRobotApisWrapper(&out_apiHandle, in_pIpAddress, in_pUsername, in_pPassword, in_sessionTimeoutMs, in_controlTimeoutMs);

            mxFree(in_pIpAddress);
            mxFree(in_pUsername);
            mxFree(in_pPassword);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);

        plhs[plhs_index] = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
        *mxGetUint32s(plhs[plhs_index++]) = out_apiHandle;

        plhs[plhs_index] = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
        *mxGetUint32s(plhs[plhs_index++]) = visionDeviceID;
    }
    else if (!strcmp(command, "DestroyRobotApisWrapper"))
    {
        requiredParams = { mxUINT32_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);

            error_code = kapi_DestroyRobotApisWrapper(in_apiHandle);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
    }
    else if (!strcmp(command, "GetJointCount"))
    {
        uint32_t out_jointCount = 0;

        requiredParams = { mxUINT32_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);
            error_code = kapi_GetJointCount(in_apiHandle,  &out_jointCount);
        }
        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);

        plhs[plhs_index] = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
        *mxGetUint32s(plhs[plhs_index++]) = out_jointCount;
    }
    else if (!strcmp(command, "SetSensorSettings"))
    {
        requiredParams = { mxUINT32_CLASS, mxUINT32_CLASS, mxUINT32_CLASS, mxUINT32_CLASS, mxUINT32_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;

            SensorSettings settings;

            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);

            settings.sensor = *mxGetUint32s(prhs[++prhs_index]);
            settings.resolution = *mxGetUint32s(prhs[++prhs_index]);
            settings.frame_rate = *mxGetUint32s(prhs[++prhs_index]);
            settings.bit_rate = *mxGetUint32s(prhs[++prhs_index]);

            error_code = kapi_SetSensorSettings(in_apiHandle,  settings);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
    }
    else if (!strcmp(command, "DoSensorFocusAction"))
    {
        requiredParams = { mxUINT32_CLASS, mxUINT32_CLASS, mxUINT32_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;

            SensorFocusAction action;

            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);

            action.sensor = *mxGetUint32s(prhs[++prhs_index]);
            action.focus_action = *mxGetUint32s(prhs[++prhs_index]);
            

            error_code = kapi_DoSensorFocusAction(in_apiHandle,  action);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
    }
    else if (!strcmp(command, "SetOptionValue"))
    {
        requiredParams = { mxUINT32_CLASS, mxUINT32_CLASS, mxUINT32_CLASS, mxDOUBLE_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;

            OptionValue option_value;

            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);

            option_value.sensor = *mxGetUint32s(prhs[++prhs_index]);
            option_value.option = *mxGetUint32s(prhs[++prhs_index]);
            option_value.value  = *mxGetDoubles(prhs[++prhs_index]);

            error_code = kapi_SetOptionValue(in_apiHandle,  option_value);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
    }
    else if (!strcmp(command, "InitVision"))
    {
        requiredParams = { mxUINT32_CLASS };

        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;

            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);

            error_code = kapi_InitVision(in_apiHandle);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
    }
    else if (!strcmp(command, "GetSensorSettings"))
    {
        requiredParams = { mxUINT32_CLASS, mxUINT32_CLASS };

        SensorSettings result;
        
        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            
            uint32_t tempID;

            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);
            
            tempID = *mxGetUint32s(prhs[++prhs_index]);
            
            error_code = kapi_GetSensorSettings(in_apiHandle,  tempID, &result);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
            
        plhs[plhs_index] = KortexMatlab::createSensorSettingsStructMatrix(&result);
    }
    else if (!strcmp(command, "GetOptionValue"))
    {
        requiredParams = { mxUINT32_CLASS, mxUINT32_CLASS, mxUINT32_CLASS };

        OptionValue result;
        OptionIdentifier input;
        
        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            
            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);
            
            input.sensor = *mxGetUint32s(prhs[++prhs_index]);
            input.option = *mxGetUint32s(prhs[++prhs_index]);
            error_code = kapi_GetOptionValue(in_apiHandle,  input, &result);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
        plhs[plhs_index++] = KortexMatlab::createOptionValueStructMatrix(&result);
    }
    else if (!strcmp(command, "GetIntrinsicParameters"))
    {
        requiredParams = { mxUINT32_CLASS, mxUINT32_CLASS };

        IntrinsicParameters result;
        uint32_t input;
        
        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            
            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);
            
            input = *mxGetUint32s(prhs[++prhs_index]);
            error_code = kapi_GetIntrinsicParameters(in_apiHandle, input, &result);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
        
        plhs[plhs_index++] = KortexMatlab::createIntrinsicStructMatrix(&result);
    }
    else if (!strcmp(command, "GetExtrinsicParameters"))
    {
        requiredParams = { mxUINT32_CLASS };

        ExtrinsicParameters result;
        uint32_t input;
        
        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            
            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);
            
            error_code = kapi_GetExtrinsicParameters(in_apiHandle, &result);
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
        
        plhs[plhs_index++] = KortexMatlab::createExtrinsicStructMatrix(&result);
    }
    else if (!strcmp(command, "SetIntrinsicParameters"))
    {
        requiredParams = { mxUINT32_CLASS , mxUINT32_CLASS, mxUINT32_CLASS, mxDOUBLE_CLASS, mxDOUBLE_CLASS, mxDOUBLE_CLASS};

        IntrinsicParameters input;
        
        uint32_t sensor;
        uint32_t resolution;
        
        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            
            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);
            sensor       = *mxGetUint32s(prhs[++prhs_index]);
            resolution   = *mxGetUint32s(prhs[++prhs_index]);

            double* principal_point = mxGetDoubles(prhs[++prhs_index]);
            const auto principal_point_count = mxGetN(prhs[prhs_index]);

            double* focal_length = mxGetDoubles(prhs[++prhs_index]);
            const auto focal_length_count = mxGetN(prhs[prhs_index]);
            
            double* distortion_coeffs = mxGetDoubles(prhs[++prhs_index]);
            const auto distortion_coeffs_count = mxGetN(prhs[prhs_index]);
            
            if (principal_point_count == PRINCIPAL_POINT_DIMENSION_SIZE && 
                focal_length_count == FOCAL_LENGTH_DIMENSION_SIZE && 
                distortion_coeffs_count == DISTORTIION_COEFFICIENTS_DIMENSION_SIZE)
            {
                input.sensor = static_cast<Sensor>(sensor);
                input.resolution = static_cast<Resolution>(resolution);
                
                input.principal_point_x = principal_point[0];
                input.principal_point_y = principal_point[1];
                
                input.focal_length_x = focal_length[0];
                input.focal_length_y = focal_length[1];
                
                input.distortion_coeffs.k1 = distortion_coeffs[0];
                input.distortion_coeffs.k2 = distortion_coeffs[1];
                input.distortion_coeffs.k3 = distortion_coeffs[2];
                input.distortion_coeffs.p1 = distortion_coeffs[3];
                input.distortion_coeffs.p2 = distortion_coeffs[4];

                error_code = kapi_SetIntrinsicParameters(in_apiHandle, sensor, input);
            }
            else
            {
                error_code = KortexErrorCodes::ERROR_INVALID_PARAM;
            }
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
    }
    else if (!strcmp(command, "SetExtrinsicParameters"))
    {
        requiredParams = { mxUINT32_CLASS , mxDOUBLE_CLASS, mxDOUBLE_CLASS};

        ExtrinsicParameters input;
        
        if ( KortexErrorCodes::SUB_ERROR_NONE == checkInputParameters( nrhs, requiredParams.size(), prhs, requiredParams) )
        {
            uint32_t prhs_index = 0;
            
            in_apiHandle = *mxGetUint32s(prhs[++prhs_index]);
            
            double* rotation          = mxGetDoubles(prhs[++prhs_index]);
            bool rotation_count_check = ((size_t)mxGetN(prhs[prhs_index]) == 9);

            double* translation          = mxGetDoubles(prhs[++prhs_index]);
            bool translation_count_check = ((size_t)mxGetN(prhs[prhs_index]) == 3);

            if(rotation_count_check && translation_count_check)
            {
                input.rotation.row1.column1 = rotation[0];
                input.rotation.row1.column2 = rotation[1];
                input.rotation.row1.column3 = rotation[2];
                input.rotation.row2.column1 = rotation[3];
                input.rotation.row2.column2 = rotation[4];
                input.rotation.row2.column3 = rotation[5];
                input.rotation.row3.column1 = rotation[6];
                input.rotation.row3.column2 = rotation[7];
                input.rotation.row3.column3 = rotation[8];

                input.translation.x = translation[0];
                input.translation.y = translation[1];
                input.translation.z = translation[2];

                error_code = kapi_SetExtrinsicParameters(in_apiHandle, input);
            }
            else
            {
                error_code = KortexErrorCodes::ERROR_INVALID_PARAM;
            }
        }

        uint32_t plhs_index = 0;
        setReturnedStatus(plhs, plhs_index, error_code);
    }
    else
    {
        uint32_t plhs_index = 0;

        setReturnedStatus(plhs, plhs_index, error_code);

        mexPrintf("kortexApiMexInterface : Command not defined\n");
    }
}
