function [ParamFilter, state_camera_struct] = mainExperiment_init_state_camera

% Init Filter
P0amers = diag([1;1;1]*1.e-3); %initial landmark covariance
R = 1.0^2*eye(2); %measurement noise for one landmark
ParamFilter.NbAmers = 30; %nominal number of landmarks in the state
ParamFilter.NbAmersMin = ParamFilter.NbAmers;
ParamFilter.EcartPixelMax = 40; %it was 20

% init covariance
p0Rot = (0.01*pi/180)^2;
p0v =  1.e-4;
p0x =  1.e-8;
p0omegab = 1.e-6;
p0ab = 1.e-6;
P0 = diag([p0Rot*ones(3,1);p0v*ones(3,1);p0x*ones(3,1);...
    p0omegab*ones(3,1);p0ab*ones(3,1)]);
%%

% process noises
q_omega = (1.6968e-4)^2*200;
q_a = (2e-3)^2*200;
q_omegab = (1.9393e-5)^2*200;
q_ab = (3e-3)^2*200;
Q = diag([q_omega*ones(3,1);q_a*ones(3,1);q_omegab* ...
    ones(3,1);q_ab*ones(3,1)]);
Qc = chol(Q);
P0 = blkdiag(P0,kron(eye(ParamFilter.NbAmers),P0amers));

%%

%depending on the chosen camera %cam0
% ParamFilter.chiC = [0.0, 0.0, 0.0, 0.0;
%     0.0, 0.0, 0.0, 0.0;
%     0.0, 0.0, 0.0, 0.0;
%     0.0, 0.0, 0.0, 1.0]; % camera pose // extrinsics

ParamFilter.chiC = [1.0, 0.0, 0.0, 0.0;
    0.0, 1.0, 0.0, 0.0;
    0.0, 0.0, 1.0, 0.0;
    0.0, 0.0, 0.0, 1.0]; % camera pose // extrinsics

calib_fileID = fopen('calib.txt', 'r' );
%calib_fileID = fopen( neurocams3_data([], 'calib.txt'), 'r' );
calib_param = fscanf(calib_fileID,'%f,%f,%f,%f,%f,%f,%f,%f,%f');
% calib_param = fscanf(calib_fileID,'%f %f %f %f %f %f %f %f %f');

fx = calib_param(1);
fy = calib_param(2);
cx = calib_param(3);
cy = calib_param(4);

k1 = calib_param(5);
k2 = calib_param(6);
p1 = calib_param(7);
p2 = calib_param(8);

ParamFilter.Pi = [fx 0 0; 0 fy 0; cx, cy 1]';%camera calibration matrix
ParamFilter.cameraParams = cameraParameters('IntrinsicMatrix', ParamFilter.Pi',...
    'RadialDistortion',[k1, k2],...
    'TangentialDistortion',[p1, p2]);


state_camera_struct.P0 = P0;
state_camera_struct.Q = Q;
state_camera_struct.Qc = Qc;
state_camera_struct.R = R;