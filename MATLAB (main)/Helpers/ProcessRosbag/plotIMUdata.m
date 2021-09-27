function plotIMUdata(t,acc,angvel,quat)

% Acceleration
figure;
sgtitle('Accelerometer')
subplot(3,1,1);
plot(t,acc(1,:))
title('X-Axis');
subplot(3,1,2); 
plot(t,acc(2,:))
title('Y-Axis');
subplot(3,1,3); 
plot(t,acc(3,:))
title('Z-Axis');

% Angular Velocity
figure;
sgtitle('Gyroscope')
subplot(3,1,1);
plot(t,angvel(1,:))
title('X-Axis');
subplot(3,1,2); 
plot(t,angvel(2,:))
title('Y-Axis');
subplot(3,1,3); 
plot(t,angvel(3,:))
title('Z-Axis');

%Estimated orientation in Euler angles - Embedded EKF
orientationIMU_euler = quat2eul(quat)*180/pi;
figure;
sgtitle('Orientation of the IMU in Euler angles - Estimated by internal EKF')
subplot(3,1,1);
plot(t,orientationIMU_euler(:,1))
title('Z-Axis');
subplot(3,1,2); 
plot(t,orientationIMU_euler(:,2))
title('Y-Axis');
subplot(3,1,3); 
plot(t,orientationIMU_euler(:,3))
title('X-Axis');

%Estimated orientation in quaternions - Embedded EKF
figure;
sgtitle('Orientation of the IMU in quaternions - Estimated by internal EKF')
subplot(4,1,1); 
plot(t,quat(:,1)); %qW
title('qW');
subplot(4,1,2); 
plot(t,quat(:,2)); %qX
title('qX');
subplot(4,1,3); 
plot(t,quat(:,3)); %qY
title('qY');
subplot(4,1,4); 
plot(t,quat(:,4)); %qZ
title('qZ');
 
end

