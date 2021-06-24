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

#ifndef _WRAPPER_PROTO_CONVERTER_
#define _WRAPPER_PROTO_CONVERTER_

#include <Frame.pb.h>
#include <Base.pb.h>
#include <BaseCyclic.pb.h>
#include <Session.pb.h>
#include <VisionConfig.pb.h>
#include <DeviceManager.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include "kortex_wrapper_data.h"

uint32_t proto_to_c(Kinova::Api::Base::ActuatorInformation target);
void proto_to_c(Kinova::Api::BaseCyclic::Feedback target, BaseFeedback *out_pBaseFeedback, ActuatorsFeedback* out_pActuatorsFeedback, InterconnectFeedback* out_pInterconnectFeedback, size_t actuator_count);
SensorSettings proto_to_c(Kinova::Api::VisionConfig::SensorSettings settings);
OptionValue proto_to_c(Kinova::Api::VisionConfig::OptionValue input);
OptionInformation proto_to_c(Kinova::Api::VisionConfig::OptionInformation input);
IntrinsicParameters proto_to_c(Kinova::Api::VisionConfig::IntrinsicParameters input);
ExtrinsicParameters proto_to_c(Kinova::Api::VisionConfig::ExtrinsicParameters input);

Kinova::Api::BaseCyclic::Command c_to_proto(ActuatorsCommand* in_pActuatorsCmd, InterconnectCommand* in_pInterconnectCmd, size_t actuator_count);
Kinova::Api::Base::Action c_to_proto(int32_t in_constraintType, double in_translationSpeed, double in_orientationSpeed, 
                                     double in_duration, double* in_pPosition, double* in_pOrientation);
Kinova::Api::Base::Action c_to_proto(int32_t in_constraintType, double in_speed, double in_duration, double* in_pJointAngles, uint32_t in_nbrJoints);
Kinova::Api::Base::Action c_to_proto(int in_mode, uint32_t duration, double* in_pCommand, uint32_t in_nbrToolActuators);

Kinova::Api::Base::JointSpeeds c_to_proto(uint32_t in_duration, double* in_pJointSpeeds, uint32_t in_nbrJoints);
Kinova::Api::Base::PreComputedJointTrajectory c_to_proto(int in_mode, double* in_pPosition, double* in_pVelocity, double* in_pAcceleration, double* in_pTimestamp, uint32_t in_nbrJoints, uint32_t in_nbrSteps);

Kinova::Api::Base::Admittance c_to_proto(AdmittanceMode in_admittanceMode);
Kinova::Api::Base::ServoingModeInformation c_to_proto(ServoingMode in_servoingMode);

Kinova::Api::VisionConfig::SensorSettings c_to_proto(SensorSettings settings);
Kinova::Api::VisionConfig::OptionValue c_to_proto(OptionValue input);

Kinova::Api::VisionConfig::SensorFocusAction c_to_proto(SensorFocusAction input);

Kinova::Api::VisionConfig::IntrinsicParameters c_to_proto(IntrinsicParameters input);
Kinova::Api::VisionConfig::ExtrinsicParameters c_to_proto(ExtrinsicParameters input);

#endif //INCLUDE GUARDS

