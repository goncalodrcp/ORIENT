function [ParamFilter] = init_filter(NLand,NLandMin,pixelErr,noiseIMU,camParam,T_IC)

%%

%Noise parameters of IMU (Process noise) - standard deviations
gyro_noise_density = noiseIMU(1);  %[ rad / s / sqrt(Hz) ]   ( gyro "white noise" )
gyro_random_walk = noiseIMU(2);   %[ rad / s^2 / sqrt(Hz) ] ( gyro bias diffusion )
acc_noise_density = noiseIMU(3);  %[ m / s^2 / sqrt(Hz) ]   ( accel "white noise" )
acc_random_walk = noiseIMU(4);   %[ m / s^3 / sqrt(Hz) ].  ( accel bias diffusion )

%Camera specifications
K = [camParam(1,1) 0 camParam(1,3);0 camParam(1,2) camParam(1,4);0 0 1]; %intrinsics matrix
%Distortion model
%Radial parameters
k1 = camParam(2,1);
k2 = camParam(2,2);
%Tangencial parameters
p1 = camParam(2,3);
p2 = camParam(2,4);

%Process noise - Q
q_omega = (gyro_noise_density)^2*200;
q_a = (acc_noise_density)^2*200;
q_omegab = (gyro_random_walk)^2*200;
q_ab = (acc_random_walk)^2*200;
Q = diag([q_omega*ones(3,1);q_a*ones(3,1);q_omegab* ...
    ones(3,1);q_ab*ones(3,1)]);

%Measurement noise - W
W = 1.0^2*eye(2); %measurement noise for one landmark

%%%%
%% Initialize parameters

ParamFilter.NbAmers = NLand; %nominal number of landmarks in the state
ParamFilter.NbAmersMin = NLandMin;
ParamFilter.EcartPixelMax = pixelErr;

%depending on the chosen camera %cam0
ParamFilter.T_IC = T_IC; % camera pose
ParamFilter.K = K;%camera calibration matrix
%Why do they transpose K? (below)
%Testing without the trasnpose
%Apparently that's the format that MATLAB uses.
ParamFilter.cameraParams = cameraParameters('IntrinsicMatrix', ParamFilter.K',...
    'RadialDistortion',[k1, k2],...
    'TangentialDistortion',[p1, p2]);
ParamFilter.Q = Q; %Process noise covariance matrix
ParamFilter.W = W; %Measurement noise covariance matrix


end

