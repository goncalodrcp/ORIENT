function plotRawIMU(t,acc,angvel)

% Acceleration
figure;
sgtitle('Accelerometer')
subplot(3,1,1);
plot(t,acc(1,:))
xlabel('t (s)')
ylabel('Acceleration (m/s^2)')
title('X-Axis');
subplot(3,1,2); 
plot(t,acc(2,:))
xlabel('t (s)')
ylabel('Acceleration (m/s^2)')
title('Y-Axis');
subplot(3,1,3); 
plot(t,acc(3,:))
xlabel('t (s)')
ylabel('Acceleration (m/s^2)')
title('Z-Axis');

% Angular Velocity
figure;
sgtitle('Gyroscope')
subplot(3,1,1);
plot(t,angvel(1,:))
xlabel('t (s)')
ylabel('Angular velocity (rad/s)')
title('X-Axis');
subplot(3,1,2); 
plot(t,angvel(2,:))
xlabel('t (s)')
ylabel('Angular velocity (rad/s)')
title('Y-Axis');
subplot(3,1,3); 
plot(t,angvel(3,:))
xlabel('t (s)')
ylabel('Angular velocity (rad/s)')
title('Z-Axis');

end

