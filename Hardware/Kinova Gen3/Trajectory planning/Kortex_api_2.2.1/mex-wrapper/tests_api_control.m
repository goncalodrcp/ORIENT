function tests = tests_api_control
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    disp('SETUP');

    testCase.TestData.IP_ADDRESS = '192.168.1.10';
    testCase.TestData.ActuatorCount = uint32(7);

    testCase.TestData.RefreshFeedbackIteration = 10;
    testCase.TestData.StopActionIteration = 10;
    testCase.TestData.PauseActionIteration = 10;
    testCase.TestData.MovementStatusIteration = 10;
    testCase.TestData.SetAdmittanceIteration = 10;
    testCase.TestData.SetServoingModeIteration = 10;

    testCase.TestData.JointSpeedIteration  = 10;
    testCase.TestData.JointSpeedTolerance  = 0.9;
    testCase.TestData.JointSpeedDuration   = 5;    %seconds
    testCase.TestData.JointSpeedCommand3   = 10;
    testCase.TestData.JointSpeedCommand5   = 10;
    testCase.TestData.JointSpeedCommand7   = 10;

    testCase.TestData.JointPositionIteration  = 10;
    testCase.TestData.JointPositionTolerance  = 0.9;
    
    testCase.TestData.CartesianPoseIteration  = 10;
    testCase.TestData.CartesianTolerance  = 0.9;
end

function TestRefreshFeedback(testCase)
    
    [result, tempHandle, ~] = kortexApiMexInterface('CreateRobotApisWrapper', testCase.TestData.IP_ADDRESS, 'admin', 'admin', uint32(60000), uint32(2000));
    
    %Validate that the connection has been established correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);

    for counter = 1:testCase.TestData.RefreshFeedbackIteration
        [result, baseFeedback, jointsFeedback, interconnectFeedback] = kortexApiMexInterface('RefreshFeedback', tempHandle);

    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);

        % BASE FEEDBACK
        testCase.assertInstanceOf(baseFeedback.arm_state, 'uint32');
        testCase.assertInstanceOf(baseFeedback.arm_voltage, 'double');
        testCase.assertInstanceOf(baseFeedback.arm_current, 'double');
        testCase.assertInstanceOf(baseFeedback.temperature_cpu, 'double');
        testCase.assertInstanceOf(baseFeedback.temperature_ambient, 'double');
        
        testCase.assertInstanceOf(baseFeedback.imu_acceleration, 'double');
        testCase.assertLength(baseFeedback.imu_acceleration, 3);

        testCase.assertInstanceOf(baseFeedback.imu_angular_velocity, 'double');
        testCase.assertLength(baseFeedback.imu_angular_velocity, 3);

        testCase.assertInstanceOf(baseFeedback.tool_pose, 'double');
        testCase.assertLength(baseFeedback.tool_pose, 6);

        testCase.assertInstanceOf(baseFeedback.tool_twist, 'double');
        testCase.assertLength(baseFeedback.tool_twist, 6);

        testCase.assertInstanceOf(baseFeedback.fault_bank_a, 'uint32');
        testCase.assertInstanceOf(baseFeedback.fault_bank_b, 'uint32');

        testCase.assertInstanceOf(baseFeedback.warning_bank_a, 'uint32');
        testCase.assertInstanceOf(baseFeedback.warning_bank_b, 'uint32');

        % JOINT FEEDBACK
        testCase.assertInstanceOf(jointsFeedback.status_flags, 'uint32');
        testCase.assertLength(jointsFeedback.status_flags, testCase.TestData.ActuatorCount);

        testCase.assertInstanceOf(jointsFeedback.jitter_comm, 'uint32');
        testCase.assertLength(jointsFeedback.jitter_comm, testCase.TestData.ActuatorCount);

        testCase.assertInstanceOf(jointsFeedback.position, 'double');
        testCase.assertLength(jointsFeedback.position, testCase.TestData.ActuatorCount);

        testCase.assertInstanceOf(jointsFeedback.velocity, 'double');
        testCase.assertLength(jointsFeedback.velocity, testCase.TestData.ActuatorCount);

        testCase.assertInstanceOf(jointsFeedback.torque, 'double');
        testCase.assertLength(jointsFeedback.torque, testCase.TestData.ActuatorCount);

        testCase.assertInstanceOf(jointsFeedback.current_motor, 'double');
        testCase.assertLength(jointsFeedback.current_motor, testCase.TestData.ActuatorCount);

        testCase.assertInstanceOf(jointsFeedback.voltage, 'double');
        testCase.assertLength(jointsFeedback.voltage, testCase.TestData.ActuatorCount);

        testCase.assertInstanceOf(jointsFeedback.temperature_motor, 'double');
        testCase.assertLength(jointsFeedback.temperature_motor, testCase.TestData.ActuatorCount);

        testCase.assertInstanceOf(jointsFeedback.temperature_core, 'double');
        testCase.assertLength(jointsFeedback.temperature_core, testCase.TestData.ActuatorCount);

        testCase.assertInstanceOf(jointsFeedback.fault_bank_a, 'uint32');
        testCase.assertLength(jointsFeedback.fault_bank_a, testCase.TestData.ActuatorCount);

        testCase.assertInstanceOf(jointsFeedback.fault_bank_b, 'uint32');
        testCase.assertLength(jointsFeedback.fault_bank_b, testCase.TestData.ActuatorCount);

        testCase.assertInstanceOf(jointsFeedback.warning_bank_a, 'uint32');
        testCase.assertLength(jointsFeedback.warning_bank_a, testCase.TestData.ActuatorCount);

        testCase.assertInstanceOf(jointsFeedback.warning_bank_b, 'uint32');
        testCase.assertLength(jointsFeedback.warning_bank_b, testCase.TestData.ActuatorCount);

        % INTERCONNECT FEEDBACK
        testCase.assertInstanceOf(interconnectFeedback.feedback_id, 'uint32');
        testCase.assertInstanceOf(interconnectFeedback.status_flags, 'uint32');
        testCase.assertInstanceOf(interconnectFeedback.jitter_comm, 'uint32');

        testCase.assertInstanceOf(interconnectFeedback.imu_acceleration_x, 'double');
        testCase.assertInstanceOf(interconnectFeedback.imu_acceleration_y, 'double');
        testCase.assertInstanceOf(interconnectFeedback.imu_acceleration_z, 'double');

        testCase.assertInstanceOf(interconnectFeedback.imu_angular_velocity_x, 'double');
        testCase.assertInstanceOf(interconnectFeedback.imu_angular_velocity_y, 'double');
        testCase.assertInstanceOf(interconnectFeedback.imu_angular_velocity_z, 'double');

        testCase.assertInstanceOf(interconnectFeedback.voltage, 'double');
        testCase.assertInstanceOf(interconnectFeedback.temperature_core, 'double');

        testCase.assertInstanceOf(interconnectFeedback.fault_bank_a, 'uint32');
        testCase.assertInstanceOf(interconnectFeedback.fault_bank_b, 'uint32');

        testCase.assertInstanceOf(interconnectFeedback.warning_bank_a, 'uint32');
        testCase.assertInstanceOf(interconnectFeedback.warning_bank_b, 'uint32');

        testCase.assertInstanceOf(interconnectFeedback.difference_count_a, 'double');
        testCase.assertInstanceOf(interconnectFeedback.difference_count_b, 'double');

        testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.feedback_id, 'uint32');
        testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.status_flags, 'uint32');
        testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.fault_bank_a, 'uint32');
        testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.fault_bank_b, 'uint32');
        testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.warning_bank_a, 'uint32');
        testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.warning_bank_b, 'uint32');
        testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.motor_count, 'uint32');
        
        for i = 1:interconnectFeedback.gripper_feedback.motor_count
            testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.motor(i).motor_id, 'uint32');
            testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.motor(i).status_flags, 'uint32');
            testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.motor(i).jitter_comm, 'uint32');
            testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.motor(i).position, 'double');
            testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.motor(i).velocity, 'double');
            testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.motor(i).force, 'double');
            testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.motor(i).current_motor, 'double');
            testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.motor(i).voltage, 'double');
            testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.motor(i).temperature_core, 'double');
            testCase.assertInstanceOf(interconnectFeedback.gripper_feedback.motor(i).temperature_motor, 'double');
        end
    end

    [result] = kortexApiMexInterface('DestroyRobotApisWrapper', uint32(tempHandle));

    %Validate that the connection has been closed correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
end

function TestSendJointSpeedCommand(testCase)
    [result, tempHandle] = kortexApiMexInterface('CreateRobotApisWrapper', testCase.TestData.IP_ADDRESS, 'admin', 'admin', uint32(60000), uint32(2000));
    
    %Validate that the connection has been established correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);

    [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [0,0,50,0,50,0,50]);
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    
    pause(10);

    for counter = 1:testCase.TestData.JointSpeedIteration

        [result, ~, jointsFeedback, ~] = kortexApiMexInterface('RefreshFeedback', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        InitPosition3 = jointsFeedback.position(3);
        InitPosition5 = jointsFeedback.position(5);
        InitPosition7 = jointsFeedback.position(7);

        [result] = kortexApiMexInterface('SendJointSpeedCommand', tempHandle, 0,[0, 0 ,testCase.TestData.JointSpeedCommand3, ...
                                              0, testCase.TestData.JointSpeedCommand5, 0, testCase.TestData.JointSpeedCommand7], testCase.TestData.ActuatorCount);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        pause(testCase.TestData.JointSpeedDuration);
        [result] = kortexApiMexInterface('SendJointSpeedCommand', tempHandle, 0,[0,0,0,0,0,0,0], testCase.TestData.ActuatorCount);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        [result, ~, jointsFeedback, ~] = kortexApiMexInterface('RefreshFeedback', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        positionReached3 = jointsFeedback.position(3);
        positionReached5 = jointsFeedback.position(5);
        positionReached7 = jointsFeedback.position(7);

        % We verify that the actuator goes to the expected position (velocity * time)
        verifyEqual(testCase,positionReached3, mod(InitPosition3 + (testCase.TestData.JointSpeedDuration * testCase.TestData.JointSpeedCommand3), 360), ...
                   'RelTol',testCase.TestData.JointSpeedTolerance, 'Unable to reach target position on joint #3 using speed control.');

        verifyEqual(testCase,positionReached5, mod(InitPosition5 + (testCase.TestData.JointSpeedDuration * testCase.TestData.JointSpeedCommand5), 360), ...
                   'RelTol',testCase.TestData.JointSpeedTolerance, 'Unable to reach target position on joint #5 using speed control.');

        verifyEqual(testCase,positionReached7, mod(InitPosition7 + (testCase.TestData.JointSpeedDuration * testCase.TestData.JointSpeedCommand7), 360), ...
                   'RelTol',testCase.TestData.JointSpeedTolerance, 'Unable to reach target position on joint #7 using speed control.');

        pause(1);

        [result] = kortexApiMexInterface('SendJointSpeedCommand', tempHandle, 0,[0,0,-1 * testCase.TestData.JointSpeedCommand3,0, ...
                                              -1 * testCase.TestData.JointSpeedCommand5, 0, ...
                                              -1 * testCase.TestData.JointSpeedCommand7], testCase.TestData.ActuatorCount);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(testCase.TestData.JointSpeedDuration);

        [result] = kortexApiMexInterface('SendJointSpeedCommand', tempHandle, 0,[0,0,0,0,0,0,0], testCase.TestData.ActuatorCount);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        [result, ~, jointsFeedback, ~] = kortexApiMexInterface('RefreshFeedback', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        positionReached3 = jointsFeedback.position(3);
        positionReached5 = jointsFeedback.position(5);
        positionReached7 = jointsFeedback.position(7);

        % We verify that the actuators go back to the initial position (-velocity * time)
        verifyEqual(testCase,positionReached3, InitPosition3, ...
                   'RelTol',testCase.TestData.JointSpeedTolerance, 'Unable to get back to the original 3 position.');
        verifyEqual(testCase,positionReached5, InitPosition5, ...
                   'RelTol',testCase.TestData.JointSpeedTolerance, 'Unable to get back to the original 5 position.');
        verifyEqual(testCase,positionReached7, InitPosition7, ...
                   'RelTol',testCase.TestData.JointSpeedTolerance, 'Unable to get back to the original 7 position.');
    end

    [result] = kortexApiMexInterface('DestroyRobotApisWrapper', uint32(tempHandle));
        
    %Validate that the connection has been closed correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
end

function TestSendJointReachCommand(testCase)
    [result, tempHandle] = kortexApiMexInterface('CreateRobotApisWrapper', testCase.TestData.IP_ADDRESS, 'admin', 'admin', uint32(60000), uint32(2000));
    
    %Validate that the connection has been established correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);

    [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [0,0,20,0,20,0,20]);
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    
    pause(10);

    for counter = 1:testCase.TestData.JointPositionIteration

        [result, ~, jointsFeedback, ~] = kortexApiMexInterface('RefreshFeedback', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        InitPosition3 = jointsFeedback.position(3);
        InitPosition5 = jointsFeedback.position(5);
        InitPosition7 = jointsFeedback.position(7);

        [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [0,0,60,0,60,0,60]);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(5);
        
        [result, ~, jointsFeedback, ~] = kortexApiMexInterface('RefreshFeedback', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        positionReached3 = jointsFeedback.position(3);
        positionReached5 = jointsFeedback.position(5);
        positionReached7 = jointsFeedback.position(7);

        verifyEqual(testCase,positionReached3, 50, ...
                   'RelTol',testCase.TestData.JointPositionTolerance, 'Unable to reach target position on joint #3.');

        verifyEqual(testCase,positionReached5, 50, ...
                   'RelTol',testCase.TestData.JointPositionTolerance, 'Unable to reach target position on joint #5.');

        verifyEqual(testCase,positionReached7, 50, ...
                   'RelTol',testCase.TestData.JointPositionTolerance, 'Unable to reach target position on joint #7.');

        pause(1);

        [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [0,0,20,0,20,0,20]);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(5);
        
        [result, ~, jointsFeedback, ~] = kortexApiMexInterface('RefreshFeedback', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        positionReached3 = jointsFeedback.position(3);
        positionReached5 = jointsFeedback.position(5);
        positionReached7 = jointsFeedback.position(7);

        verifyEqual(testCase,positionReached3, InitPosition3, ...
                   'RelTol',testCase.TestData.JointSpeedTolerance, 'Unable to get back to the original position on joint #3.');
        verifyEqual(testCase,positionReached5, InitPosition5, ...
                   'RelTol',testCase.TestData.JointSpeedTolerance, 'Unable to get back to the original position on joint #5.');
        verifyEqual(testCase,positionReached7, InitPosition7, ...
                   'RelTol',testCase.TestData.JointSpeedTolerance, 'Unable to get back to the original position on joint #7.');
    end

    [result] = kortexApiMexInterface('DestroyRobotApisWrapper', uint32(tempHandle));

    %Validate that the connection has been closed correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
end

function TestSendCartesianPose(testCase)
    [result, tempHandle] = kortexApiMexInterface('CreateRobotApisWrapper', testCase.TestData.IP_ADDRESS, 'admin', 'admin', uint32(60000), uint32(2000));
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);

    [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [0,0,0,0,0,0,0]);
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    
    pause(10);
    
    [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [360,15,180,230,0,55,90]);
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);

    pause(10);
    
    for counter = 1:testCase.TestData.CartesianPoseIteration

        [result, baseFeedback, ~, ~] = kortexApiMexInterface('RefreshFeedback', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        InitX = baseFeedback.tool_pose(1);
        InitY = baseFeedback.tool_pose(2);
        InitZ = baseFeedback.tool_pose(3);

        InitThetaX = baseFeedback.tool_pose(4);
        InitThetaY = baseFeedback.tool_pose(5);
        InitThetaZ = baseFeedback.tool_pose(6);

        [result] = kortexApiMexInterface('ReachCartesianPose', tempHandle, int32(0), [0, 0], 0, [InitX + 0.1, InitY, InitZ], [InitThetaX, InitThetaY,InitThetaZ]);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(3);
        
        [result, baseFeedback, ~, ~] = kortexApiMexInterface('RefreshFeedback', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        verifyEqual(testCase,baseFeedback.tool_pose(1), InitX + 0.1, ...
                   'RelTol',testCase.TestData.CartesianTolerance, 'Unable to reach target pose.');

        [result] = kortexApiMexInterface('ReachCartesianPose', tempHandle, int32(0), [0, 0], 0, [InitX, InitY, InitZ], [InitThetaX, InitThetaY,InitThetaZ]);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(3);
        
        verifyEqual(testCase,baseFeedback.tool_pose(1), InitX + 0.1, ...
                   'RelTol',testCase.TestData.CartesianTolerance, 'Unable to get back to init pose.');
        
    end

    [result] = kortexApiMexInterface('DestroyRobotApisWrapper', uint32(tempHandle));

    %Validate that the connection has been closed correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
end

function TestStopAction(testCase)
    
    [result, tempHandle] = kortexApiMexInterface('CreateRobotApisWrapper', testCase.TestData.IP_ADDRESS, 'admin', 'admin', uint32(60000), uint32(2000));
    
    %Validate that the connection has been established correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);

    [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [0,0,20,0,20,0,20]);
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    
    pause(10);
    
    for counter = 1:testCase.TestData.StopActionIteration
        [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [0,0,20,0,20,0,20]);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(5);
        
        [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [0,0,180,0,20,0,20]);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(2);
        [result] = kortexApiMexInterface('StopAction', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(0.5)

        
    end

    [result] = kortexApiMexInterface('DestroyRobotApisWrapper', uint32(tempHandle));

    %Validate that the connection has been closed correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
end

function TestPauseResumeAction(testCase)
    
    [result, tempHandle] = kortexApiMexInterface('CreateRobotApisWrapper', testCase.TestData.IP_ADDRESS, 'admin', 'admin', uint32(60000), uint32(2000));
    
    %Validate that the connection has been established correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    
    [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [0,0,20,0,20,0,20]);
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    
    pause(10);

    for counter = 1:testCase.TestData.PauseActionIteration
        [result] = kortexApiMexInterface('SendToolCommand', tempHandle, int32(enumToolMode.toolMode_reach), 0, 0);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(2);
        
        [result] = kortexApiMexInterface('SendToolCommand', tempHandle, int32(enumToolMode.toolMode_reach), 0, 0.9);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(0.3);
        
        [result] = kortexApiMexInterface('PauseAction', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(0.3);
        
        [result] = kortexApiMexInterface('ResumeAction', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(0.3);
    end

    [result] = kortexApiMexInterface('DestroyRobotApisWrapper', uint32(tempHandle));

    %Validate that the connection has been closed correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
end

function TestGetMovementStatus(testCase)
    
    [result, tempHandle] = kortexApiMexInterface('CreateRobotApisWrapper', testCase.TestData.IP_ADDRESS, 'admin', 'admin', uint32(60000), uint32(2000));
    
    %Validate that the connection has been established correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    
    [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [0,0,20,0,20,0,20]);
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    
    pause(3);

    for counter = 1:testCase.TestData.MovementStatusIteration
        [result] = kortexApiMexInterface('SendToolCommand', tempHandle, int32(enumToolMode.toolMode_reach), 0, 0);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(2);
        
        [result] = kortexApiMexInterface('SendToolCommand', tempHandle, int32(enumToolMode.toolMode_reach), 0, 0.9);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(0.05);
        
        [result, moving_status] = kortexApiMexInterface('GetMovementStatus', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        testCase.assertEqual(moving_status, int32(1));
        
        pause(0.3);
        
        [result] = kortexApiMexInterface('PauseAction', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(0.05);
        
        [result, moving_status] = kortexApiMexInterface('GetMovementStatus', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        testCase.assertEqual(moving_status, int32(2));
        
        pause(0.3);
        
        [result] = kortexApiMexInterface('ResumeAction', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(2);
        
        [result, moving_status] = kortexApiMexInterface('GetMovementStatus', tempHandle);
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        testCase.assertEqual(moving_status, int32(0));
        
    end

    [result] = kortexApiMexInterface('DestroyRobotApisWrapper', uint32(tempHandle));

    %Validate that the connection has been closed correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
end

function TestSetAdmittance(testCase)
    
    [result, tempHandle] = kortexApiMexInterface('CreateRobotApisWrapper', testCase.TestData.IP_ADDRESS, 'admin', 'admin', uint32(60000), uint32(2000));
    
    %Validate that the connection has been established correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    
    [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [0,0,20,0,20,0,20]);
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    
    pause(4);

    for counter = 1:testCase.TestData.SetAdmittanceIteration
        %Set to joint admittance
        [result] = kortexApiMexInterface('SetAdmittance', tempHandle, int32(2));
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(1);
        
        %Disable admittance
        [result] = kortexApiMexInterface('SetAdmittance', tempHandle, int32(4));
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    end

    [result] = kortexApiMexInterface('DestroyRobotApisWrapper', uint32(tempHandle));

    %Validate that the connection has been closed correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
end

function TestSetServoingMode(testCase)
    
    [result, tempHandle] = kortexApiMexInterface('CreateRobotApisWrapper', testCase.TestData.IP_ADDRESS, 'admin', 'admin', uint32(60000), uint32(2000));
    
    %Validate that the connection has been established correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    
    [result] = kortexApiMexInterface('ReachJointAngles', tempHandle, int32(0), 0, 0, [0,0,20,0,20,0,20]);
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    
    pause(4);

    for counter = 1:testCase.TestData.SetServoingModeIteration
        %Set to low level
        [result] = kortexApiMexInterface('SetServoingMode', tempHandle, int32(3));
        
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
        
        pause(1);
        
        %Get back to single level
        [result] = kortexApiMexInterface('SetServoingMode', tempHandle, int32(2));
        
        testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);
    end

    [result] = kortexApiMexInterface('DestroyRobotApisWrapper', uint32(tempHandle));

    %Validate that the connection has been closed correctly without error.
    testCase.assertEqual(result, KortexErrorCodes.SUB_ERROR_NONE);

end

function teardownOnce(testCase)
    disp('TEAR DOWN');
end