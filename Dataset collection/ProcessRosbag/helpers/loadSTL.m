addpath('D:\IST\ORIENT_repos\ORIENT\Hardware\Kinova Gen3\Trajectory planning\Mesh files');
gen3 = loadrobot('kinovaGen3','DataFormat','row','Gravity',[0 0 -9.81]);
% Add a "dummy" gripper link
%gripperLength = 0.101; % Gripper length in meters
gripperLength = 0.097; % Gripper length in meters
gripperBody = rigidBody('Gripper');
gripperJoint = rigidBodyJoint('GripperLink','fixed');
T = rotm2tform([0 1 0;0 0 1;1 0 0]) * trvec2tform([gripperLength 0 0]);
setFixedTransform(gripperJoint,T); % Move and orient the gripper
gripperBody.Joint = gripperJoint;
addBody(gen3,gripperBody,'EndEffector_Link');
% Add a "dummy" mesh
addVisual(gen3.Bodies{9},'Mesh','mount_v2.stl', ... 
              trvec2tform([0 0 0]) * axang2tform([0 1 0 -pi/2]) * axang2tform([0 0 1 pi]));