function plot_bag2(bag_path)

bag = rosbag(bag_path);

poses_topic = select(bag,'Topic','/vrpn_client_node/Robot_1/pose');
poses_msg = readMessages(poses_topic,'DataFormat','struct');

time = linspace(0, bag.EndTime-bag.StartTime,length(poses_msg)); 

qx = zeros(1,length(poses_msg));
qy = zeros(1,length(poses_msg));
qz = zeros(1,length(poses_msg));
qw = zeros(1,length(poses_msg));

x = zeros(1,length(poses_msg));
y = zeros(1,length(poses_msg));
z = zeros(1,length(poses_msg));


for i = 1:length(poses_msg)
    poses(i) = poses_msg{i}.Pose;
%     position(i) = poses(i).Position;
%     orientation(i) = poses(i).Orientation;
    x(i) = poses_msg{i}.Pose.Position.X;
    y(i) = poses_msg{i}.Pose.Position.Y;
    z(i) = poses_msg{i}.Pose.Position.Z;

    qx(i) = poses_msg{i}.Pose.Orientation.X;
    qy(i) = poses_msg{i}.Pose.Orientation.Y;
    qz(i) = poses_msg{i}.Pose.Orientation.Z;
    qw(i) = poses_msg{i}.Pose.Orientation.W;

end

position = [x', y', z'];
orientation = [ qw', qx', qy', qz'];
% orientation_euler = quat2eul(orientation) * 180/pi;

orientation_euler = zeros(length(poses_msg), 3);
q_ref = [qw(1) qx(1) qy(1) qz(1)]; % take the first quaternion as reference

for i = 1:length(poses_msg) % go through all samples
   q_aux = [qw(i) qx(i) qy(i) qz(i)]; 
   q_error = quatmultiply(quatconj(q_ref), q_aux); % quaternion difference is done by multiplying by the conjugate
   orientation_euler(i,:) = quat2eul(q_error, 'ZYX') * 180/pi; % get Euler angles [psi theta phi]
end

figure
plot(time, x - x(1));
title('Position x'); xlabel('time [s]'); ylabel('x [m]'); grid on; grid minor;
figure
plot(time, y - y(1));
title('Position y'); xlabel('time [s]'); ylabel('y [m]'); grid on; grid minor;
figure
plot(time, z - z(1));
title('Position z'); xlabel('time [s]'); ylabel('z [m]'); grid on; grid minor;

figure
plot(time, orientation_euler(:,1));
title('Orientation Yaw'); xlabel('time [s]'); ylabel('\psi [deg]'); grid on; grid minor;
figure
plot(time, orientation_euler(:,2));
title('Orientation Pitch'); xlabel('time [s]'); ylabel('\theta [deg]'); grid on; grid minor;
figure
plot(time, orientation_euler(:,3));
title('Orientation Roll'); xlabel('time [s]'); ylabel('\phi [deg]'); grid on; grid minor;


end