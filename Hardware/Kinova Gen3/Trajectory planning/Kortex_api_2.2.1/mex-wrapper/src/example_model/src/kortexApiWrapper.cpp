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

#include <cstring>
#include <iostream>
#include <vector>
#include <math.h>

#include <SimplifiedApi.h>
#include <IArmBaseApi.h>

#include "kortexApiWrapper.h"
#include "proto_converter.h"

// Uncomment those lines if you want console output(Windows only)
// #if defined(_START_CONSOLE_)
// #include <Windows.h>
// #endif

using namespace std;
using namespace Kinova::Api;
using namespace ApiHelper;

const uint32_t MAX_API_INSTANCE = 50;

const char* NO_INIT_MESSAGE   = "API not Initialized";
const char* NO_HANDLE_MESSAGE = "Unknown API Handle";
const char* NO_VISION_MODULE  = "Vision module not found";

bool isInitialized = false;
std::map< int, std::unique_ptr<ArmBaseSimplifiedApi> > m_mapApi;

uint32_t ValidateApi(uint32_t target)
{
    try 
    {
        if(isInitialized)
        {
            auto it = m_mapApi.find(target);

            if (it != m_mapApi.end())
            {
                 return KortexErrorCodes::SUB_ERROR_NONE;
            }
            
        }
        
    }
    catch(std::exception const& stdex)
    {
        return KortexErrorCodes::ERROR_METHOD_FAILED;
    }

    return KortexErrorCodes::ERROR_INVALID_PARAM;
};

// **************
// **  Common  **
// **************

uint32_t kapi_CreateRobotApisWrapper(uint32_t *out_apiHandle, char* in_pIpAddress, char* in_pUsername,
                                    char* in_pPassword, uint32_t in_sessionTimeoutMs, uint32_t in_controlTimeoutMs)
{
    unsigned int lastHandle = 0;

    try
    {
//         Uncomment those lines if you want console output(Windows only)
//         #if defined(_START_CONSOLE_)
//         AllocConsole();
//         freopen("CONOUT$", "w", stdout);
//         freopen("CONOUT$", "w", stderr);
//         #endif

        isInitialized = false;
        
        //Check for the next available api handle.
        for(unsigned int i = 1; i < MAX_API_INSTANCE; i++)
        {
            if (m_mapApi.count(i) == 0)
            {
                lastHandle = i;
                auto uPtr = std::unique_ptr<ArmBaseSimplifiedApi>( new ArmBaseSimplifiedApi(in_pIpAddress) );
                m_mapApi[i] = std::move(uPtr);
                m_mapApi[i]->GetApiRef()->OpenSession({in_pUsername, in_pPassword, in_sessionTimeoutMs, in_controlTimeoutMs});
                m_mapApi[i]->GetApiRef()->InitVision();
                *out_apiHandle = i;
                
                isInitialized = true;

                return KortexErrorCodes::SUB_ERROR_NONE;
            }
        }
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        return ex.getErrorInfo().getError().error_sub_code();
    }
    catch (const std::exception &ex)
    {
        return KortexErrorCodes::ERROR_METHOD_FAILED;
    }

    m_mapApi[lastHandle].reset(nullptr);
    m_mapApi.erase(lastHandle);
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_InitVision(uint32_t in_apiHandle)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            auto result = m_mapApi[in_apiHandle]->GetApiRef()->InitVision();
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch(std::exception const& stdex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }
        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_DestroyRobotApisWrapper(uint32_t in_apiHandle)
{
    try
    {
//         Uncomment those lines if you want console output(Windows only)
//         #if defined(_START_CONSOLE_)
//         FreeConsole();
//         #endif

        if(isInitialized)
        {
            m_mapApi[in_apiHandle].reset(nullptr);
            m_mapApi.erase(in_apiHandle);
             return KortexErrorCodes::SUB_ERROR_NONE;
        }    
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        return ex.getErrorInfo().getError().error_sub_code();
    }
    catch (const std::exception &ex)
    {
        return KortexErrorCodes::ERROR_METHOD_FAILED;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_GetJointCount(uint32_t in_apiHandle, uint32_t* out_pNbrJoints)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            auto result = m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->GetActuatorCount();
            (*out_pNbrJoints) = proto_to_c(result);
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

// *****************
// **  Low-Level  **
// *****************
uint32_t kapi_RefreshFeedback(uint32_t in_apiHandle, BaseFeedback *out_pBaseFeedback, ActuatorsFeedback* out_pActuatorsFeedback,  InterconnectFeedback* out_pInterconnectFeedback)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {         
            auto actuator_count = m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->GetActuatorCount().count();
            auto result = m_mapApi[in_apiHandle]->GetApiRef()->BaseCyclicServices()->RefreshFeedback();

            proto_to_c(result, out_pBaseFeedback, out_pActuatorsFeedback, out_pInterconnectFeedback, actuator_count);
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

// ******************
// **  High-Level  **
// ******************

uint32_t kapi_ReachCartesianPose(uint32_t in_apiHandle, int32_t in_constraintType, 
        double in_translationSpeed, double in_orientationSpeed, double in_duration, 
        double* in_pPosition, double* in_pOrientation)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->ExecuteAction(c_to_proto(in_constraintType, in_translationSpeed, in_orientationSpeed, in_duration, in_pPosition, in_pOrientation));
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_ReachJointAngles(uint32_t in_apiHandle, int32_t in_constraintType, double in_speed, double in_duration,
                           double* in_pJointAngles, uint32_t in_nbrJoints)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->ExecuteAction(c_to_proto(in_constraintType, in_speed, in_duration, in_pJointAngles, in_nbrJoints));
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_SendJointSpeedCommand(uint32_t in_apiHandle, double in_duration, double* in_pJointSpeeds, uint32_t in_nbrJoints)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->SendJointSpeedsCommand(c_to_proto(in_duration, in_pJointSpeeds, in_nbrJoints));
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
};

uint32_t kapi_SendWrenchCommand             (uint32_t in_apiHandle, double in_duration, double* in_pForce, double* torque) 
{ 
    return KortexErrorCodes::SUB_ERROR_NONE;
};

uint32_t kapi_SendJointTorqueCommand        (uint32_t in_apiHandle, double in_duration, double* in_pJointTorques, uint32_t in_nbrJoints) 
{ 
    return KortexErrorCodes::SUB_ERROR_NONE;
};

uint32_t kapi_SendToolCommand(uint32_t in_apiHandle, int in_mode, uint32_t duration, double* in_pCommand, uint32_t in_nbrToolActuators)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->ExecuteAction(c_to_proto(in_mode, duration, in_pCommand, in_nbrToolActuators));   
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_PlayPreComputedTrajectory(uint32_t in_apiHandle, int in_mode, double* in_pPosition, double* in_pVelocity, 
                                    double* in_pAcceleration, double* in_pTimestamp, uint32_t in_nbrJoints, uint32_t in_nbrSteps)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            auto trajectory = c_to_proto(in_mode, in_pPosition, in_pVelocity, in_pAcceleration, in_pTimestamp, in_nbrJoints, in_nbrSteps);
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->PlayPreComputedJointTrajectory(trajectory);
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_PauseAction(uint32_t in_apiHandle)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->PauseAction(); 
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_ResumeAction(uint32_t in_apiHandle)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->ResumeAction(); 
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_StopAction (uint32_t in_apiHandle) 
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->StopAction(); 
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_SetAdmittance(uint32_t in_apiHandle, AdmittanceMode in_admittanceMode)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->SetAdmittance(c_to_proto(in_admittanceMode));
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_SetServoingMode(uint32_t in_apiHandle, ServoingMode in_servoingMode)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->SetServoingMode(c_to_proto(in_servoingMode));
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_ClearFaults(uint32_t in_apiHandle)
{ 
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->ClearFaults(); 
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_ApplyEmergencyStop(uint32_t in_apiHandle)
{ 
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->ApplyEmergencyStop(); 
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_Reboot(uint32_t in_apiHandle)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            m_mapApi[in_apiHandle]->GetApiRef()->BaseServices()->Reboot(); 
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_GetMovementStatus(uint32_t in_apiHandle, int32_t* out_pMovementStatus)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            auto trjState = m_mapApi[in_apiHandle]->GetTrajectoryManagerRef()->PollTrajectoryStatus();
            
            switch ( trjState.GetState().state )
            {
                case TrajectoryState::State::eIdle:
                    *out_pMovementStatus = (int32_t)MOVEMENT_STATUS_IDLE;
                break;
                case TrajectoryState::State::eRunning:
                    *out_pMovementStatus = (int32_t)MOVEMENT_STATUS_RUNNING;
                break;
                case TrajectoryState::State::ePaused:
                    *out_pMovementStatus = (int32_t)MOVEMENT_STATUS_PAUSED;
                break;
                case TrajectoryState::State::ePrecomputedTrajectoryProcessing:
                    *out_pMovementStatus = (int32_t)PRE_PROCESSING;
                break;
            }
            return KortexErrorCodes::SUB_ERROR_NONE;
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch(std::exception const& stdex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }
    }

    return KortexErrorCodes::ERROR_INVALID_PARAM;
};

uint32_t kapi_GetLastError(uint32_t in_apiHandle, uint32_t *error)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            *error = m_mapApi[in_apiHandle]->GetTrajectoryManagerRef()->GetLastError();
             return KortexErrorCodes::SUB_ERROR_NONE;
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
};

uint32_t kapi_GetErrorName(uint32_t in_apiHandle, uint32_t error_code, char* name)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            const google::protobuf::EnumDescriptor *descriptor = Kinova::Api::SubErrorCodes_descriptor();
            std::string tempName = descriptor->FindValueByNumber(error_code)->name();

            strncpy(name, tempName.c_str(), MAX_ENUM_NAME_SIZE);
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
};

uint32_t kapi_SetSensorSettings(uint32_t in_apiHandle, SensorSettings settings)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            bool has_vision = m_mapApi[in_apiHandle]->GetApiRef()->HasVisionModule();
            uint32_t ID = 0;

            if(has_vision)
            {
                ID = m_mapApi[in_apiHandle]->GetApiRef()->GetVisionModuleID();

                m_mapApi[in_apiHandle]->GetApiRef()->VisionConfigServices()->SetSensorSettings(c_to_proto(settings), ID);    
            }
            else
            {
                return KortexErrorCodes::ERROR_DEVICE_DISCONNECTED;
            }
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_GetSensorSettings(uint32_t in_apiHandle, uint32_t sensorID, SensorSettings *settings)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            bool has_vision = m_mapApi[in_apiHandle]->GetApiRef()->HasVisionModule();
            uint32_t ID = 0;

            if(has_vision)
            {
                ID = m_mapApi[in_apiHandle]->GetApiRef()->GetVisionModuleID();

                VisionConfig::SensorIdentifier tempID = VisionConfig::SensorIdentifier();
                tempID.set_sensor((VisionConfig::Sensor)sensorID); 

                auto result = m_mapApi[in_apiHandle]->GetApiRef()->VisionConfigServices()->GetSensorSettings(tempID, ID);

                *settings = proto_to_c(result);
            }
            else
            {
                return KortexErrorCodes::ERROR_DEVICE_DISCONNECTED;
            }
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_SetOptionValue(uint32_t in_apiHandle, OptionValue value)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            bool has_vision = m_mapApi[in_apiHandle]->GetApiRef()->HasVisionModule();
            uint32_t ID = 0;

            if(has_vision)
            {
                ID = m_mapApi[in_apiHandle]->GetApiRef()->GetVisionModuleID();
                
                m_mapApi[in_apiHandle]->GetApiRef()->VisionConfigServices()->SetOptionValue(c_to_proto(value), ID);
            }
            else
            {
                return KortexErrorCodes::ERROR_DEVICE_DISCONNECTED;
            }
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_GetOptionValue(uint32_t in_apiHandle, OptionIdentifier optionID, OptionValue *value)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            bool has_vision = m_mapApi[in_apiHandle]->GetApiRef()->HasVisionModule();
            uint32_t ID = 0;

            if(has_vision)
            {
                ID = m_mapApi[in_apiHandle]->GetApiRef()->GetVisionModuleID();
                
                VisionConfig::OptionIdentifier tempID = VisionConfig::OptionIdentifier();
                tempID.set_sensor((VisionConfig::Sensor)optionID.sensor);
                tempID.set_option((VisionConfig::Option)optionID.option);

                auto result = m_mapApi[in_apiHandle]->GetApiRef()->VisionConfigServices()->GetOptionValue(tempID, ID);
                
                *value = proto_to_c(result);
            }
            else
            {
                return KortexErrorCodes::ERROR_DEVICE_DISCONNECTED;
            }
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_GetOptionInformation(uint32_t in_apiHandle, OptionIdentifier optionID, OptionInformation *optionInfo)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            bool has_vision = m_mapApi[in_apiHandle]->GetApiRef()->HasVisionModule();
            uint32_t ID = 0;

            if(has_vision)
            {
                ID = m_mapApi[in_apiHandle]->GetApiRef()->GetVisionModuleID();
                
                VisionConfig::OptionIdentifier tempID = VisionConfig::OptionIdentifier();
                tempID.set_sensor((VisionConfig::Sensor)optionID.sensor);
                tempID.set_option((VisionConfig::Option)optionID.option);

                auto result = m_mapApi[in_apiHandle]->GetApiRef()->VisionConfigServices()->GetOptionInformation(tempID, ID);

                *optionInfo = proto_to_c(result);
            }
            else
            {
                return KortexErrorCodes::ERROR_DEVICE_DISCONNECTED;
            }
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_DoSensorFocusAction(uint32_t in_apiHandle, SensorFocusAction action)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            bool has_vision = m_mapApi[in_apiHandle]->GetApiRef()->HasVisionModule();
            uint32_t ID = 0;

            if(has_vision)
            {
                ID = m_mapApi[in_apiHandle]->GetApiRef()->GetVisionModuleID();
                
                m_mapApi[in_apiHandle]->GetApiRef()->VisionConfigServices()->DoSensorFocusAction(c_to_proto(action), ID);
            }
            else
            {
                return KortexErrorCodes::ERROR_DEVICE_DISCONNECTED;
            }
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_GetIntrinsicParameters(uint32_t in_apiHandle, uint32_t sensorID, IntrinsicParameters *params)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            bool has_vision = m_mapApi[in_apiHandle]->GetApiRef()->HasVisionModule();
            uint32_t ID = 0;

            if(has_vision)
            {
                ID = m_mapApi[in_apiHandle]->GetApiRef()->GetVisionModuleID();
                
                VisionConfig::SensorIdentifier tempID = VisionConfig::SensorIdentifier();
                tempID.set_sensor((VisionConfig::Sensor)sensorID); 

                auto result = m_mapApi[in_apiHandle]->GetApiRef()->VisionConfigServices()->GetIntrinsicParameters(tempID, ID);

                *params = proto_to_c(result);
            }
            else
            {
                return KortexErrorCodes::ERROR_DEVICE_DISCONNECTED;
            }
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_GetExtrinsicParameters (uint32_t in_apiHandle, ExtrinsicParameters *params)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            bool has_vision = m_mapApi[in_apiHandle]->GetApiRef()->HasVisionModule();
            uint32_t ID = 0;

            if(has_vision)
            {
                ID = m_mapApi[in_apiHandle]->GetApiRef()->GetVisionModuleID();

                auto result = m_mapApi[in_apiHandle]->GetApiRef()->VisionConfigServices()->GetExtrinsicParameters(ID);

                *params = proto_to_c(result);

            }
            else
            {
                return KortexErrorCodes::ERROR_DEVICE_DISCONNECTED;
            }
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_SetIntrinsicParameters(uint32_t in_apiHandle, uint32_t sensorID, IntrinsicParameters params)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            bool has_vision = m_mapApi[in_apiHandle]->GetApiRef()->HasVisionModule();
            uint32_t ID = 0;

            if(has_vision)
            {
                ID = m_mapApi[in_apiHandle]->GetApiRef()->GetVisionModuleID();

                m_mapApi[in_apiHandle]->GetApiRef()->VisionConfigServices()->SetIntrinsicParameters(c_to_proto(params), ID);    
            }
            else
            {
                return KortexErrorCodes::ERROR_DEVICE_DISCONNECTED;
            }
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}

uint32_t kapi_SetExtrinsicParameters(uint32_t in_apiHandle, ExtrinsicParameters params)
{
    if(ValidateApi(in_apiHandle) == KortexErrorCodes::SUB_ERROR_NONE)
    {
        try
        {
            bool has_vision = m_mapApi[in_apiHandle]->GetApiRef()->HasVisionModule();
            uint32_t ID = 0;

            if(has_vision)
            {
                ID = m_mapApi[in_apiHandle]->GetApiRef()->GetVisionModuleID();

                m_mapApi[in_apiHandle]->GetApiRef()->VisionConfigServices()->SetExtrinsicParameters(c_to_proto(params), ID);    
            }
            else
            {
                return KortexErrorCodes::ERROR_DEVICE_DISCONNECTED;
            }
        }
        catch(Kinova::Api::KDetailedException& ex)
        {
            return ex.getErrorInfo().getError().error_sub_code();
        }
        catch (const std::exception &ex)
        {
            return KortexErrorCodes::ERROR_METHOD_FAILED;
        }

        return KortexErrorCodes::SUB_ERROR_NONE;
    }
    return KortexErrorCodes::ERROR_INVALID_PARAM;
}
