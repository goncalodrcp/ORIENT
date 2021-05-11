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

#include "proto_converter.h"
#include <iostream>
using namespace std;

uint32_t proto_to_c(Kinova::Api::Base::ActuatorInformation target)
{
    return target.count();
}

Kinova::Api::BaseCyclic::Command c_to_proto(ActuatorsCommand* in_pActuatorsCmd,  InterconnectCommand* in_pInterconnectCmd, size_t actuator_count)
{
    auto command = Kinova::Api::BaseCyclic::Command();

    for(size_t i = 0; i < actuator_count; i++)
    {   
        auto actuator = command.add_actuators();

        actuator->set_flags(in_pActuatorsCmd->flags[i]);
        actuator->set_position(in_pActuatorsCmd->position[i]);
        actuator->set_velocity(in_pActuatorsCmd->velocity[i]);
        actuator->set_torque_joint(in_pActuatorsCmd->torque_joint[i]);
        actuator->set_current_motor(in_pActuatorsCmd->current_motor[i]);
    }

    return command;
}

void proto_to_c(Kinova::Api::BaseCyclic::Feedback target, BaseFeedback *out_pBaseFeedback, ActuatorsFeedback* out_pActuatorsFeedback, InterconnectFeedback* out_pInterconnectFeedback, size_t actuator_count)
{
    out_pBaseFeedback->arm_state = uint32_t( target.base().active_state() );

    out_pBaseFeedback->arm_voltage = target.base().arm_voltage();
    out_pBaseFeedback->arm_current = target.base().arm_current();
    out_pBaseFeedback->temperature_cpu = target.base().temperature_cpu();
    out_pBaseFeedback->temperature_ambient = target.base().temperature_ambient();
    
    out_pBaseFeedback->imu_acceleration[0] = target.base().imu_acceleration_x();
    out_pBaseFeedback->imu_acceleration[1] = target.base().imu_acceleration_y();
    out_pBaseFeedback->imu_acceleration[2] = target.base().imu_acceleration_z();
    
    out_pBaseFeedback->imu_angular_velocity[0] = target.base().imu_angular_velocity_x();
    out_pBaseFeedback->imu_angular_velocity[1] = target.base().imu_angular_velocity_y();
    out_pBaseFeedback->imu_angular_velocity[2] = target.base().imu_angular_velocity_z();
    
    out_pBaseFeedback->tool_pose[0] = target.base().tool_pose_x();
    out_pBaseFeedback->tool_pose[1] = target.base().tool_pose_y();
    out_pBaseFeedback->tool_pose[2] = target.base().tool_pose_z();
    out_pBaseFeedback->tool_pose[3] = target.base().tool_pose_theta_x();
    out_pBaseFeedback->tool_pose[4] = target.base().tool_pose_theta_y();
    out_pBaseFeedback->tool_pose[5] = target.base().tool_pose_theta_z();

    out_pBaseFeedback->tool_twist[0] = target.base().tool_twist_linear_x();
    out_pBaseFeedback->tool_twist[1] = target.base().tool_twist_linear_y();
    out_pBaseFeedback->tool_twist[2] = target.base().tool_twist_linear_z();
    out_pBaseFeedback->tool_twist[3] = target.base().tool_twist_angular_x();
    out_pBaseFeedback->tool_twist[4] = target.base().tool_twist_angular_y();
    out_pBaseFeedback->tool_twist[5] = target.base().tool_twist_angular_z();
    
    out_pBaseFeedback->tool_external_wrench_force[0] = target.base().tool_external_wrench_force_x();
    out_pBaseFeedback->tool_external_wrench_force[1] = target.base().tool_external_wrench_force_y();
    out_pBaseFeedback->tool_external_wrench_force[2] = target.base().tool_external_wrench_force_z();
    
    out_pBaseFeedback->tool_external_wrench_torque[0] = target.base().tool_external_wrench_torque_x();
    out_pBaseFeedback->tool_external_wrench_torque[1] = target.base().tool_external_wrench_torque_y();
    out_pBaseFeedback->tool_external_wrench_torque[2] = target.base().tool_external_wrench_torque_z();
    
    out_pBaseFeedback->fault_bank_a = target.base().fault_bank_a();
    out_pBaseFeedback->fault_bank_b = target.base().fault_bank_b();
    out_pBaseFeedback->warning_bank_a = target.base().warning_bank_a();
    out_pBaseFeedback->warning_bank_b = target.base().warning_bank_b();

    for(unsigned int i = 0; i < actuator_count; i++)
    {   
        out_pActuatorsFeedback->status_flags[i] = target.actuators(i).status_flags();
        out_pActuatorsFeedback->jitter_comm[i] = target.actuators(i).jitter_comm();
        out_pActuatorsFeedback->position[i] = target.actuators(i).position();
        out_pActuatorsFeedback->velocity[i] = target.actuators(i).velocity();
        out_pActuatorsFeedback->torque[i] = target.actuators(i).torque();
        out_pActuatorsFeedback->current_motor[i] = target.actuators(i).current_motor();
        out_pActuatorsFeedback->voltage[i] = target.actuators(i).voltage();
        out_pActuatorsFeedback->temperature_motor[i] = target.actuators(i).temperature_motor();
        out_pActuatorsFeedback->temperature_core[i] = target.actuators(i).temperature_core();

        out_pActuatorsFeedback->fault_bank_a[i] = target.actuators(i).fault_bank_a();
        out_pActuatorsFeedback->fault_bank_b[i] = target.actuators(i).fault_bank_b();
        out_pActuatorsFeedback->warning_bank_a[i] = target.actuators(i).warning_bank_a();
        out_pActuatorsFeedback->warning_bank_b[i] = target.actuators(i).warning_bank_b();
    }

    out_pInterconnectFeedback->feedback_id = target.interconnect().feedback_id().identifier();
    out_pInterconnectFeedback->status_flags = target.interconnect().status_flags();
    out_pInterconnectFeedback->jitter_comm = target.interconnect().jitter_comm();
    
    out_pInterconnectFeedback->imu_acceleration_x = target.interconnect().imu_acceleration_x();
    out_pInterconnectFeedback->imu_acceleration_y = target.interconnect().imu_acceleration_y();
    out_pInterconnectFeedback->imu_acceleration_z = target.interconnect().imu_acceleration_z();

    out_pInterconnectFeedback->imu_angular_velocity_x = target.interconnect().imu_angular_velocity_x();
    out_pInterconnectFeedback->imu_angular_velocity_y = target.interconnect().imu_angular_velocity_y();
    out_pInterconnectFeedback->imu_angular_velocity_z = target.interconnect().imu_angular_velocity_z();

    out_pInterconnectFeedback->voltage = target.interconnect().voltage();
    out_pInterconnectFeedback->temperature_core = target.interconnect().temperature_core();
            
    out_pInterconnectFeedback->fault_bank_a = target.interconnect().fault_bank_a();
    out_pInterconnectFeedback->fault_bank_b = target.interconnect().fault_bank_b();
    out_pInterconnectFeedback->warning_bank_a = target.interconnect().warning_bank_a();
    out_pInterconnectFeedback->warning_bank_b = target.interconnect().warning_bank_b();

    if(target.interconnect().has_gripper_feedback())
    {
        out_pInterconnectFeedback->gripper_feedback.feedback_id = target.interconnect().gripper_feedback().feedback_id().identifier();
        out_pInterconnectFeedback->gripper_feedback.status_flags = target.interconnect().gripper_feedback().status_flags();

        out_pInterconnectFeedback->gripper_feedback.fault_bank_a = target.interconnect().gripper_feedback().fault_bank_a();
        out_pInterconnectFeedback->gripper_feedback.fault_bank_b = target.interconnect().gripper_feedback().fault_bank_b();
        out_pInterconnectFeedback->gripper_feedback.warning_bank_a = target.interconnect().gripper_feedback().warning_bank_a();
        out_pInterconnectFeedback->gripper_feedback.warning_bank_b = target.interconnect().gripper_feedback().warning_bank_b();
        out_pInterconnectFeedback->gripper_feedback.motor_count = target.interconnect().gripper_feedback().motor_size();

        if(out_pInterconnectFeedback->gripper_feedback.motor_count >= GRIPPER_MAX_MOTOR_COUNT)
        {
            out_pInterconnectFeedback->gripper_feedback.motor_count = GRIPPER_MAX_MOTOR_COUNT;
        }

        for(uint32_t i = 0; i < out_pInterconnectFeedback->gripper_feedback.motor_count; i++)
        {
            out_pInterconnectFeedback->gripper_feedback.motor[i].motor_id = target.interconnect().gripper_feedback().motor(i).motor_id();
            
            out_pInterconnectFeedback->gripper_feedback.motor[i].position = target.interconnect().gripper_feedback().motor(i).position();
            out_pInterconnectFeedback->gripper_feedback.motor[i].velocity = target.interconnect().gripper_feedback().motor(i).velocity();
            out_pInterconnectFeedback->gripper_feedback.motor[i].current_motor = target.interconnect().gripper_feedback().motor(i).current_motor();
            out_pInterconnectFeedback->gripper_feedback.motor[i].voltage = target.interconnect().gripper_feedback().motor(i).voltage();
            out_pInterconnectFeedback->gripper_feedback.motor[i].temperature_motor = target.interconnect().gripper_feedback().motor(i).temperature_motor();
        }
    }
}

Kinova::Api::Base::Action c_to_proto(int32_t in_constraintType, double in_translationSpeed, double in_orientationSpeed, 
                                     double in_duration, double* in_pPosition, double* in_pOrientation)
{
    auto action = Kinova::Api::Base::Action();
    
    auto constrainedPose = action.mutable_reach_pose();
    auto pose = constrainedPose->mutable_target_pose();
    auto constraint = constrainedPose->mutable_constraint();

    switch(in_constraintType)
    {
        case static_cast<int32_t>(CARTESIAN_CONSTRAINT_DURATION):
            constraint->set_duration(in_duration);
            break;
        case static_cast<int32_t>(CARTESIAN_CONSTRAINT_SPEED):
        {
            auto speed = constraint->mutable_speed();

            speed->set_translation(in_translationSpeed);
            speed->set_orientation(in_orientationSpeed);
            break;
        }
        case static_cast<int32_t>(CARTESIAN_NO_CONSTRAINT):
        {
            break;
        }
        default:
        {
            throw std::runtime_error("unknown constraint");
        }
    }


    pose->set_x(in_pPosition[0]);
    pose->set_y(in_pPosition[1]);
    pose->set_z(in_pPosition[2]);

    pose->set_theta_x(in_pOrientation[0]);
    pose->set_theta_y(in_pOrientation[1]);
    pose->set_theta_z(in_pOrientation[2]);

    return action;
}

Kinova::Api::Base::Action c_to_proto(int in_mode, uint32_t duration, double* in_pCommand, uint32_t in_nbrToolActuators)
{
    auto action = Kinova::Api::Base::Action();
    auto gripper_cmd = action.mutable_send_gripper_command();

    gripper_cmd->set_duration(duration);
    gripper_cmd->set_mode(static_cast<Kinova::Api::Base::GripperMode>(in_mode));
    
    auto gripper = gripper_cmd->mutable_gripper();
    
    for(unsigned int i = 0; i < in_nbrToolActuators; ++i)
    {
        auto finger = gripper->add_finger();
        finger->set_finger_identifier(i+1);
        finger->set_value(in_pCommand[i]);
    }
    
    return action;
}

Kinova::Api::Base::Action c_to_proto(int32_t in_constraintType, double in_speed, double in_duration, double* in_pJointAngles, uint32_t in_nbrJoints)
{
    auto action = Kinova::Api::Base::Action();

    auto reachJointAngles = action.mutable_reach_joint_angles();
    auto jointAngles = reachJointAngles->mutable_joint_angles();
    auto constraint = reachJointAngles->mutable_constraint();

    constraint->set_type((Kinova::Api::Base::JointTrajectoryConstraintType)in_constraintType);

    if(in_constraintType == static_cast<int32_t>(JOINT_CONSTRAINT_DURATION))
    {
        constraint->set_type(Kinova::Api::Base::JOINT_CONSTRAINT_DURATION);
        constraint->set_value(in_duration);
    }
    else if(in_constraintType == static_cast<int32_t>(JOINT_CONSTRAINT_SPEED))
    {
        constraint->set_type(Kinova::Api::Base::JOINT_CONSTRAINT_SPEED);
        constraint->set_value(in_speed);
    }
    else if (in_constraintType == static_cast<int32_t>(UNSPECIFIED_JOINT_CONSTRAINT))
    {
        constraint->set_type(Kinova::Api::Base::UNSPECIFIED_JOINT_CONSTRAINT);
        constraint->set_value(0);
    }
    else
    {
        throw std::runtime_error("unknown constraint");
    }

    for(uint32_t i = 0 ; i < in_nbrJoints; ++i)
    {
        auto jointAngle = jointAngles->add_joint_angles();
        jointAngle->set_joint_identifier(i);
        jointAngle->set_value(in_pJointAngles[i]);
    }

    return action;
}

Kinova::Api::Base::JointSpeeds c_to_proto(uint32_t in_duration, double* in_pJointSpeeds, uint32_t in_nbrJoints)
{
    auto command = Kinova::Api::Base::JointSpeeds();

    command.set_duration(in_duration);
    
    for(uint32_t i = 0; i < in_nbrJoints; i++)
    {   
        auto speed = command.add_joint_speeds();

        speed->set_joint_identifier(i);
        speed->set_value(in_pJointSpeeds[i]);
    }

    return command;
}

Kinova::Api::Base::PreComputedJointTrajectory c_to_proto(int in_mode, double* in_pPosition, double* in_pVelocity, 
                                                         double* in_pAcceleration, double* in_pTimestamp, uint32_t in_nbrJoints, uint32_t in_nbrSteps)
{
    auto input = Kinova::Api::Base::PreComputedJointTrajectory();

    input.set_mode((Kinova::Api::Base::TrajectoryContinuityMode)in_mode);

    Kinova::Api::Base::PreComputedJointTrajectoryElement *element;

    for (uint32_t timestep=0; timestep < in_nbrSteps; timestep++)
    {
        element = input.add_trajectory_elements();

        // Set all joint values
        for (uint32_t jointIndex=0; jointIndex < in_nbrJoints; jointIndex++)
        {
            element->add_joint_angles(in_pPosition[timestep * in_nbrJoints + jointIndex]);
            element->add_joint_speeds(in_pVelocity[timestep * in_nbrJoints + jointIndex]);
            element->add_joint_accelerations(in_pAcceleration[timestep * in_nbrJoints + jointIndex]);
        }

        element->set_time_from_start(float(in_pTimestamp[timestep]));
    }

    return input;
}

Kinova::Api::Base::Admittance c_to_proto(AdmittanceMode in_admittanceMode)
{
    auto input = Kinova::Api::Base::Admittance();
    
    input.set_admittance_mode((Kinova::Api::Base::AdmittanceMode)in_admittanceMode);

    return input;
}

Kinova::Api::Base::ServoingModeInformation c_to_proto(ServoingMode in_servoingMode)
{
    auto input = Kinova::Api::Base::ServoingModeInformation();
    
    input.set_servoing_mode((Kinova::Api::Base::ServoingMode)in_servoingMode);

    return input;
}

Kinova::Api::VisionConfig::SensorSettings c_to_proto(SensorSettings settings)
{
    auto result = Kinova::Api::VisionConfig::SensorSettings();
    
    result.set_sensor((Kinova::Api::VisionConfig::Sensor)settings.sensor);
    result.set_resolution((Kinova::Api::VisionConfig::Resolution)settings.resolution);
    result.set_frame_rate((Kinova::Api::VisionConfig::FrameRate)settings.frame_rate);
    result.set_bit_rate((Kinova::Api::VisionConfig::BitRate)settings.bit_rate);

    return result;
}

SensorSettings proto_to_c(Kinova::Api::VisionConfig::SensorSettings settings)
{
    SensorSettings result;

    result.sensor = static_cast<Sensor>(settings.sensor());
    result.resolution = static_cast<Resolution>(settings.resolution());
    result.frame_rate = static_cast<FrameRate>(settings.frame_rate());
    result.bit_rate = static_cast<BitRate>(settings.bit_rate());

    return result;
}

Kinova::Api::VisionConfig::OptionValue c_to_proto(OptionValue input)
{
    auto result = Kinova::Api::VisionConfig::OptionValue();

    result.set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
    result.set_option((Kinova::Api::VisionConfig::Option)input.option);
    result.set_value(input.value);

    return result;
}

OptionValue proto_to_c(Kinova::Api::VisionConfig::OptionValue input)
{
    OptionValue result;

    result.sensor = static_cast<Sensor>(input.sensor());
    result.option = static_cast<Option>(input.option());
    result.value = input.value();

    return result;
}

OptionInformation proto_to_c(Kinova::Api::VisionConfig::OptionInformation input)
{
    OptionInformation result;

    result.sensor = static_cast<Sensor>(input.sensor());
    result.option = static_cast<Option>(input.option());

    result.supported = input.supported();
    result.read_only = input.read_only();
    result.minimum = input.minimum();
    result.maximum = input.maximum();
    result.step = input.step();
    result.default_value = input.default_value();

    return result;
}

Kinova::Api::VisionConfig::SensorFocusAction c_to_proto(SensorFocusAction input)
{
    auto result = Kinova::Api::VisionConfig::SensorFocusAction();

    result.set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
    result.set_focus_action((Kinova::Api::VisionConfig::FocusAction)input.focus_action);

    return result;
}

IntrinsicParameters proto_to_c(Kinova::Api::VisionConfig::IntrinsicParameters input)
{
    IntrinsicParameters result;

    result.sensor = static_cast<Sensor>(input.sensor());
    result.resolution = static_cast<Resolution>(input.resolution());

    result.principal_point_x = input.principal_point_x();
    result.principal_point_y = input.principal_point_y();
    result.focal_length_x = input.focal_length_x();
    result.focal_length_y = input.focal_length_y();

    result.distortion_coeffs.k1 = input.distortion_coeffs().k1();
    result.distortion_coeffs.k2 = input.distortion_coeffs().k2();
    result.distortion_coeffs.k3 = input.distortion_coeffs().k3();
    result.distortion_coeffs.p1 = input.distortion_coeffs().p1();
    result.distortion_coeffs.p2 = input.distortion_coeffs().p2();
    
    return result;
}

ExtrinsicParameters proto_to_c(Kinova::Api::VisionConfig::ExtrinsicParameters input)
{
    ExtrinsicParameters result;

    result.rotation.row1.column1 = input.rotation().row1().column1();
    result.rotation.row1.column2 = input.rotation().row1().column2();
    result.rotation.row1.column3 = input.rotation().row1().column3();
    result.rotation.row2.column1 = input.rotation().row2().column1();
    result.rotation.row2.column2 = input.rotation().row2().column2();
    result.rotation.row2.column3 = input.rotation().row2().column3();
    result.rotation.row3.column1 = input.rotation().row3().column1();
    result.rotation.row3.column2 = input.rotation().row3().column2();
    result.rotation.row3.column3 = input.rotation().row3().column3();

    result.translation.x = input.translation().t_x();
    result.translation.y = input.translation().t_y();
    result.translation.z = input.translation().t_z();
    
    return result;
}

Kinova::Api::VisionConfig::IntrinsicParameters c_to_proto(IntrinsicParameters input)
{
    auto params = Kinova::Api::VisionConfig::IntrinsicParameters();

    params.set_sensor(static_cast<Kinova::Api::VisionConfig::Sensor>(input.sensor));
    params.set_resolution(static_cast<Kinova::Api::VisionConfig::Resolution>(input.resolution));

    params.set_principal_point_x(input.principal_point_x);
    params.set_principal_point_y(input.principal_point_y);
    params.set_focal_length_x(input.focal_length_x);
    params.set_focal_length_y(input.focal_length_y);

    params.mutable_distortion_coeffs()->set_k1(input.distortion_coeffs.k1);
    params.mutable_distortion_coeffs()->set_k2(input.distortion_coeffs.k2);
    params.mutable_distortion_coeffs()->set_k3(input.distortion_coeffs.k3);
    params.mutable_distortion_coeffs()->set_p1(input.distortion_coeffs.p1);
    params.mutable_distortion_coeffs()->set_p2(input.distortion_coeffs.p2);

    return params;
}

Kinova::Api::VisionConfig::ExtrinsicParameters c_to_proto(ExtrinsicParameters input)
{
    auto params = Kinova::Api::VisionConfig::ExtrinsicParameters();

    params.mutable_rotation()->mutable_row1()->set_column1(input.rotation.row1.column1);
    params.mutable_rotation()->mutable_row1()->set_column2(input.rotation.row1.column2);
    params.mutable_rotation()->mutable_row1()->set_column3(input.rotation.row1.column3);

    params.mutable_rotation()->mutable_row2()->set_column1(input.rotation.row2.column1);
    params.mutable_rotation()->mutable_row2()->set_column2(input.rotation.row2.column2);
    params.mutable_rotation()->mutable_row2()->set_column3(input.rotation.row2.column3);

    params.mutable_rotation()->mutable_row3()->set_column1(input.rotation.row3.column1);
    params.mutable_rotation()->mutable_row3()->set_column2(input.rotation.row3.column2);
    params.mutable_rotation()->mutable_row3()->set_column3(input.rotation.row3.column3);

    params.mutable_translation()->set_t_x(input.translation.x);
    params.mutable_translation()->set_t_y(input.translation.y);
    params.mutable_translation()->set_t_z(input.translation.z);
    
    return params;
}

