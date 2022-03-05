  function [ParamFilter, state_camera_struct] = mainExperiment_init_state_camera

% Init Filter
P0amers = diag([1;1;1]*1.e-3); %initial landmark covariance
R = 1.0^2*eye(2); %measurement noise for one landmark
ParamFilter.NbAmers = 30; %nominal number of landmarks in the state
ParamFilter.NbAmersMin = ParamFilter.NbAmers;
%ParamFilter.NbAmersMin = 10;
ParamFilter.EcartPixelMax = 25; %the starting value is 20

% init covariance
p0Rot = (0.01*pi/180)^2;
p0v =  1.e-4;
p0x =  1.e-8;
p0omegab = 1.e-6;
p0ab = 1.e-6;
P0 = diag([p0Rot*ones(3,1);p0v*ones(3,1);p0x*ones(3,1);...
    p0omegab*ones(3,1);p0ab*ones(3,1)]);
%%

% process noises - this works with im_X_A10_v18 only...
q_omega = (2e-5)^2*200; %2e-5 works best
q_a = (2e-2)^2*200;
q_omegab = (2e-5)^2*200;
q_ab = (3e-2)^2*200;
%this works - sort of
% q_omega = (0.000060038)^2*200;
% q_a = (0.000553)^2*200;
% q_omegab = (0.0000578)^2*200;
% q_ab = (0.00032)^2*200;
%trying to switch noises from gyro and accelerometer
% q_omega = (0.0000553)^2*200;
% q_a = (0.0060038)^2*200;
% q_ab = (0.000578)^2*200;
% q_omegab = (0.000032)^2*200;
Q = diag([q_omega*ones(3,1);q_a*ones(3,1);q_omegab* ...
    ones(3,1);q_ab*ones(3,1)]);
Qc = chol(Q);
P0 = blkdiag(P0,kron(eye(ParamFilter.NbAmers),P0amers));

%%

%depending on the chosen camera %cam0
%ParamFilter.chiC = [1.0, 0.0, 0.0, 0.0;
%    0.0, 1.0, 0.0, 0.0;
%    0.0, 0.0, 1.0, 0.0;
%    0.0, 0.0, 0.0, 1.0]; % camera pose // extrinsics

%HARCODED - ADD FUNCTION TO READ EXTRINSICS
%  ParamFilter.chiC = [-0.02002943 -0.99977726 -0.00665278 -0.04822244;
%                      0.99974576 -0.01995899 -0.01049043  0.02329326;
%                      0.01035531 -0.0068612   0.99992284 -0.05070335;
%                      0.0          0.0         0.0          1.0       ]; % camera pose // extrinsics
                
% ParamFilter.chiC = [-0.02002943  0.99974576  0.01035531 -0.02372815;
%                     -0.99977726 -0.01995899 -0.0068612  -0.04809467;
%                     -0.00665278 -0.01049043  0.99992284  0.05062299;
%                      0.0          0.0          0.0          1.0     ];
                 
%Assuming the IMU frame has the same orientation has the camera frame
%Lets say the IMU frame is 2cm behind the camera frame in the X-axis
ParamFilter.chiC = [1.0, 0.0, 0.0, -0.02;
    0.0, 1.0, 0.0, 0.0;
    0.0, 0.0, 1.0, 0.0;
    0.0, 0.0, 0.0, 1.0]; % camera pose // extrinsics 

%calib_fileID = fopen('calib.txt', 'r' );
%calib_fileID = fopen( neurocams3_data([], 'calib.txt'), 'r' );
%calib_param = fscanf(calib_fileID,'%f,%f,%f,%f,%f,%f,%f,%f,%f');
% calib_param = fscanf(calib_fileID,'%f %f %f %f %f %f %f %f %f');

%HARDCODED - ADD FUNCTION TO READ CAMERA PARAMETERS

%From the experiments of 24/07
%cameraparameters = [1146.400485 1142.440508 497.044978 498.755012];
%distortionCoeffs = [-0.310282 0.146101 0.001371 -0.000252];

%From the experiments of 14/10
cameraparameters = [1129.723097 1130.712255 978.656074 773.305876];
distortionCoeffs = [-0.269803 0.068608 0.000538 0.000525];
calib_param = [cameraparameters, distortionCoeffs];

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