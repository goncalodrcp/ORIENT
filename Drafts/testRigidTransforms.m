clear;
close all

%random point
w_p = [0.3 0.05 0.3];

%frame y
w_o = [0 0 0];

%frame w
w_o_y = [0.2 0 0];
%p_w = [w_o; w_p]; %point in frame w

%frame b 
w_o_b = [0.2 0.2 0.2]; %origin of frame b in frame w
%p_b = [w_o_b; p_w(2,:)];

%b_R_w = eye(3);
b_R_w = eul2rotm([pi/6 0 0]); %ZYX 

y_o_b = w_o_b - w_o_y;
y_p = w_p - w_o_y;

plotTransforms(w_o,rotm2quat(eye(3)),'FrameSize',0.05);
grid on;
hold on;
xlim ([-0.1 0.5])
ylim ([-0.1 0.5])
zlim auto
plotTransforms(w_o_y,rotm2quat(eye(3)),'FrameSize',0.05);
plotTransforms(w_o_b,rotm2quat(b_R_w),'FrameSize',0.05);
plot3(w_p(:,1),w_p(:,2),w_p(:,3),'-o')
%plot3(p_b(:,1),p_b(:,2),p_b(:,3),'-o')
%plot3(p_b(:,1),p_b(:,2),p_b(:,3),'-o')

b_p_1 = b_R_w*(y_p') - b_R_w*(y_o_b');
b_p_2 = b_R_w*w_p'-b_R_w*w_o_b';

%% Plot transformation estimated by Kalibr
clear;
close all

%Calibration done in Kalibr (4/10/2021)
% Transformation (cam0):
%-----------------------
%T_ci:  (imu0 to cam0): 
%[[ 0.02380328  0.99967821  0.00876872 -0.02115072]
% [-0.9996539   0.02370258  0.01141338 -0.00470428]
% [ 0.01120186 -0.00903736  0.99989642  0.00017564]
% [ 0.          0.          0.          1.        ]]
%T_ic:  (cam0 to imu0): 
%[[ 0.02380328 -0.9996539   0.01120186 -0.00420117]
% [ 0.99967821  0.02370258 -0.00903736  0.021257  ]
% [ 0.00876872  0.01141338  0.99989642  0.00006353]
% [ 0.          0.          0.          1.        ]]

%Transformation between the IMU frame and Camera frame: c_p = c_T_i*i_p
c_T_i = [0.02380328  0.99967821  0.00876872 -0.02115072;
        -0.9996539   0.02370258  0.01141338 -0.00470428;
        0.01120186 -0.00903736  0.99989642  0.00017564;
        0.          0.          0.          1.];

%Transformation between the Camera frame and IMU frame: i_p = i_T_c*c_p
i_T_c = invSE3(c_T_i);

c_o = [0 0 0]; %camera frame location

plotTransforms(c_o,rotm2quat(eye(3)),'FrameSize',0.05);
grid on;
hold on;
xlim ([-0.1 0.1])
ylim ([-0.1 0.1])
zlim ([-0.1 0.1])
plotTransforms(tform2trvec(c_T_i),tform2quat(c_T_i),'FrameSize',0.05);



%% Auxiliary functions 

function T_inv = invSE3(T)
    
  T_inv = [tform2rotm(T)' -tform2rotm(T)*tform2trvec(T)'; 0 0 0 1];

end 


