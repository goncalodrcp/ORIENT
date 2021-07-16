
gripper = rigidBodyTree;

%End effector
EEBody = rigidBody('End_Effector');
EELink = rigidBodyJoint('EELink','fixed');
T = rotm2tform([1 0 0;0 1 0;0 0 1]) * trvec2tform([0 0 0]); %coincident with base (world) frame
setFixedTransform(EELink,T); % Move and orient the gripper
EEBody.Joint = EELink;
addBody(gripper,EEBody,'base');

%IMU
imuBody = rigidBody('IMU');
imuLink = rigidBodyJoint('imuLink','fixed');
ee2IMU_length = 0.05;
T = rotm2tform([0 1 0;0 0 1;1 0 0]) * trvec2tform([0 ee2IMU_length 0]);
setFixedTransform(imuLink,T); % Move and orient the gripper
imuBody.Joint = imuLink;
addBody(gripper,imuBody,'End_Effector');

%Camera
camBody = rigidBody('Cam');
camLink = rigidBodyJoint('camLink','fixed');
IMU2cam_length = 0.05;
T = rotm2tform([1 0 0;0 1 0;0 0 1]) * trvec2tform([0 IMU2cam_length 0]);
setFixedTransform(camLink,T); % Move and orient the gripper
camBody.Joint = camLink;
addBody(gripper,camBody,'IMU');
fig = show(gripper); %Show configuration
hold on;

C_T_EE_1 = getTransform(gripper,randomConfiguration(gripper),'Cam');


% New transform
T_new = eul2tform([0 pi/6 0]);
setFixedTransform(EELink,T_new); % Move and orient the gripper
replaceJoint(gripper,'End_Effector',EELink);
%figure;
show(gripper); %Show configuration

C_T_EE_2 = getTransform(gripper,randomConfiguration(gripper),'Cam');


function T_inv = invSE3(T)
    
  T_inv = [tform2rotm(T)' -tform2rotm(T)*tform2trvec(T)'; 0 0 0 1];

end 
