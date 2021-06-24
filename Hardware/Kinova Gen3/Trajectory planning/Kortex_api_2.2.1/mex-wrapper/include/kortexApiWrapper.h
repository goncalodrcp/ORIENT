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

#ifndef KORTEX_API_WRAPPER_H_
#define KORTEX_API_WRAPPER_H_

#include "kortex_wrapper_data.h"

#ifdef __cplusplus
extern "C" {
#endif

    // **************
    // **  Common  **
    // **************
    uint32_t kapi_CreateRobotApisWrapper    (uint32_t* out_apiHandle, char* in_pIpAddress, char* in_pUsername, char* in_pPassword, uint32_t in_sessionTimeoutMs, uint32_t in_controlTimeoutMs);
    uint32_t kapi_DestroyRobotApisWrapper   (uint32_t in_apiHandle);
    uint32_t kapi_GetJointCount             (uint32_t in_apiHandle, uint32_t* out_pNbrJoints);
    uint32_t kapi_InitVision                (uint32_t in_apiHandle);

    // *****************
    // **  Low-Level  **
    // *****************
    uint32_t kapi_RefreshFeedback   (uint32_t in_apiHandle, BaseFeedback *out_pBaseFeedback, ActuatorsFeedback* out_pActuatorsFeedback,  InterconnectFeedback* out_pInterconnectFeedback);
    

    // ******************
    // **  High-Level  **
    // ******************

    uint32_t kapi_ReachCartesianPose            (uint32_t in_apiHandle, int32_t in_constraintType, double in_translationSpeed, double in_orientationSpeed, double in_duration, double* in_pPosition, double* in_pOrientation);
    uint32_t kapi_ReachJointAngles              (uint32_t in_apiHandle, int32_t in_constraintType, double in_speed, double in_duration, double* in_pJointAngles, uint32_t in_nbrJoints);

    uint32_t kapi_SendJointSpeedCommand         (uint32_t in_apiHandle, double in_duration, double* in_pJointSpeeds, uint32_t in_nbrJoints);

    uint32_t kapi_SendWrenchCommand             (uint32_t in_apiHandle, double in_duration, double* in_pForce, double* torque);  // (v2.0)
    uint32_t kapi_SendJointTorqueCommand        (uint32_t in_apiHandle, double in_duration, double* in_pJointTorques, uint32_t in_nbrJoints);    // (v2.0)

    uint32_t kapi_SendToolCommand               (uint32_t in_apiHandle, int in_mode, uint32_t duration, double* in_pCommand, uint32_t in_nbrToolActuators);
    
    uint32_t kapi_PlayPreComputedTrajectory     (uint32_t in_apiHandle, int in_mode, double* in_pPosition, double* in_pVelocity, double* in_pAcceleration, double* in_pTimestamp, uint32_t in_nbrJoints, uint32_t in_nbrSteps);

    uint32_t kapi_PauseAction                   (uint32_t in_apiHandle);
    uint32_t kapi_ResumeAction                  (uint32_t in_apiHandle);
    uint32_t kapi_StopAction                    (uint32_t in_apiHandle);
    
    uint32_t kapi_SetAdmittance                 (uint32_t in_apiHandle, enum AdmittanceMode in_admittanceMode);

    uint32_t kapi_SetServoingMode               (uint32_t in_apiHandle, enum ServoingMode in_servoingMode);

    uint32_t kapi_ClearFaults                   (uint32_t in_apiHandle);
    uint32_t kapi_ApplyEmergencyStop            (uint32_t in_apiHandle);
    uint32_t kapi_Reboot                        (uint32_t in_apiHandle);

    uint32_t kapi_GetMovementStatus             (uint32_t in_apiHandle, int32_t* out_pMovementStatus);

    uint32_t kapi_GetLastError                  (uint32_t in_apiHandle, uint32_t* error);
    uint32_t kapi_GetErrorName                  (uint32_t in_apiHandle, uint32_t error_code, char* name);

    uint32_t kapi_SetSensorSettings             (uint32_t in_apiHandle, SensorSettings settings);
    uint32_t kapi_GetSensorSettings             (uint32_t in_apiHandle, uint32_t sensorID, SensorSettings* settings);
    uint32_t kapi_SetOptionValue                (uint32_t in_apiHandle, OptionValue value);
    uint32_t kapi_GetOptionValue                (uint32_t in_apiHandle, OptionIdentifier optionID, OptionValue* value);
    uint32_t kapi_GetOptionInformation          (uint32_t in_apiHandle, OptionIdentifier optionID, OptionInformation* optionInfo);
    uint32_t kapi_DoSensorFocusAction           (uint32_t in_apiHandle, SensorFocusAction action);
    uint32_t kapi_GetIntrinsicParameters        (uint32_t in_apiHandle, uint32_t sensorID, IntrinsicParameters* params);
    uint32_t kapi_GetExtrinsicParameters        (uint32_t in_apiHandle, ExtrinsicParameters* params);

    uint32_t kapi_SetIntrinsicParameters        (uint32_t in_apiHandle, uint32_t sensorID, IntrinsicParameters params);
    uint32_t kapi_SetExtrinsicParameters        (uint32_t in_apiHandle, ExtrinsicParameters params);

    
#ifdef __cplusplus
}
#endif //#ifdef __cplusplus

#endif //include guard
