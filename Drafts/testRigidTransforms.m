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


