%% Main experiment
%
%
%data comes from the EUROC MAV dataset V1_01_medium
clear;
close all;
addpath ../LieGroupToolbox
addpath ../filters
addpath ../data

%Assign directory where images can be found
dirImage = '../V1_02_medium/mav0/cam0/data/';
%IMU and ground-truth data
fileData = 'DATA_MEDIUM.mat';
%Image names and time instants
fileImages = 'fileImages_MEDIUM.mat';

%Frequency of acquisition
freqIMU = 200; %Hz
freqCam = 20; %Hz

offset = 881;
tMin = 1081; % starting IMU time
tImagesMin = 109; % starting camera time

NbStepsMax = 8000;

%% Definition of global parameters and data structures

[ParamGlobal,IMU_data,CAM_data,GT_data] = init_IMU_CAM(tMin,tImagesMin,fileData,fileImages);

%% Correct offset

[GT_data.tReal, GT_data.trajReal] = correctOffset(GT_data.tReal, GT_data.trajReal, offset);

%%

obsTimes = zeros(length(ParamGlobal.t),1);
obsTimes(1:freqIMU/freqCam:end) = 1; % IMU is 10 times faster than camera

%% Initialze Filter

%Use 'readmatrix' function with combination with
%'delimitedTextImportOptions' to set import requirements
%import sensor parameters from .yaml files. First convert them into a .txt
%file
%Example:
%opts = delimitedTextImportOptions('CommentStyle','%');
%opts = setvartype(opts,'double');
%A = readmatrix('modelParam.txt',opts);

%General parameters
NLand = 30; %number of landmarks
NLandMin = NLand; %minimum number of landmarks
pixelErr = 20; %pixel error threshold

%Import parameters
opts = delimitedTextImportOptions('CommentStyle','%');
opts = setvartype(opts,'double');
noiseIMU = readmatrix('IMU_noise_param.txt',opts); %array with IMU noise parameters
T_IC = readmatrix('extrinsics.txt',opts); %extrinsics -> Transformation from camera frame to IMU (body) frame
camParam = readmatrix('camParameters.txt',opts); %instrincs and distortion model
%Camera resolution
res = [752 480];

%Initialize filter
ParamFilter = init_filter(NLand,NLandMin,pixelErr,noiseIMU,camParam,T_IC);

%% State initialization

%Number of steps
NbSteps = ParamGlobal.NbSteps;
%Ground truth trajectory
trajReal = GT_data.trajReal;

%Initial covariance
P0amers = diag([1;1;1]*1.e-3); %initial landmark covariance
p0Rot = (0.01*pi/180)^2;
p0v =  1.e-4;
p0x =  1.e-8;
p0omegab = 1.e-6;
p0ab = 1.e-6;
P0 = diag([p0Rot*ones(3,1);p0v*ones(3,1);p0x*ones(3,1);...
    p0omegab*ones(3,1);p0ab*ones(3,1)]);
P0 = blkdiag(P0,kron(eye(ParamFilter.NbAmers),P0amers));

%Initialisation of the state is obtained following [Mur-Artal,2017],
load('../data/ORB_SLAM_init.mat');
Rot0 = eul2rotm([trajReal.psi(1),trajReal.theta(1),trajReal.phi(1)]);
x0 = trajReal.x(:,1);
v0 = trajReal.v(:,1);
omega_b0 = orb_slam.omega_b;
a_b0 = orb_slam.a_b;
PosAmers0 = orb_slam.PosAmers;

%%%%%%
trackerMain = orb_slam.trackerMain;
trackerBis = orb_slam.trackerBis;
myTracks = orb_slam.myTracks;10
%%%%%%

Q = ParamFilter.Q; %Process noise covariance
Qc = chol(Q); %cholesky decomposition

%Left-UKF-LG
trajL = initTraj(NbSteps);
[stateL_0,PL_0] = stateStruct(Rot0,v0,x0,omega_b0,a_b0,PosAmers0,P0);
SL_0 = chol(PL_0);
chiL_0 = state2chi(stateL_0.Rot,stateL_0.v,stateL_0.o,stateL_0.pLand);
% RotL = Rot0;
% vL = v0;
% xL = x0;
% omega_bL = omega_b0;
% a_bL = a_b0;
% PosAmersL = PosAmers0;
% P_L = P0;
% S_L = chol(P_L);
% chiL = state2chi(RotL,vL,xL,PosAmersL);


%Right-UKF-LG
trajR = initTraj(NbSteps);
[stateR_0,PR_0] = stateStruct(Rot0,v0,x0,omega_b0,a_b0,PosAmers0,P0);
SR_0 = chol(PR_0);
chiR_0 = state2chi(stateR_0.Rot,stateR_0.v,stateR_0.o,stateR_0.pLand);
% trajR = initTraj(NbSteps);
% RotR = Rot0;
% vR = v0;
% xR = x0;
% omega_bR = omega_b0;ParamFilter
% a_bR = a_b0;
% PosAmersR = PosAmers0;
% P_R = P0;
% S_R = chol(P_R);
% chiR = state2chi(RotR,vR,xR,PosAmersR);

%SE(3)-UKF
trajRef = initTraj(NbSteps);
[stateRef_0,PRef_0] = stateStruct(Rot0,v0,x0,omega_b0,a_b0,PosAmers0,P0);
PRef_0 = blkdiag(PRef_0);
SRef_0 = chol(PRef_0);
chiRef_0 = [stateRef_0.Rot stateRef_0.o;0 0 0 1];
xidotRef_0 = zeros(6,1);
% trajRef = initTraj(NbSteps);
% RotRef = Rot0;
% vRef = v0;
% xRef = x0;
% omega_bRef = omega_b0;
% a_bRef = a_b0;
% PosAmersRef = PosAmers0;
% P_Ref = blkdiag(P0);
% S_Ref = chol(P_Ref);
% chiRef = [RotRef xRef;0 0 0 1];
% xidotRef = zeros(6,1);

%UKF
trajU = initTraj(NbSteps);
[stateU_0,PU_0] = stateStruct(Rot0,v0,x0,omega_b0,a_b0,PosAmers0,P0);
PU_0 = blkdiag(PU_0);
SU_0 = chol(PU_0);
% trajU = initTraj(NbSteps);
% RotU = Rot0;
% vU = v0;
% xU = x0;
% omega_bU = omega_b0;
% a_bU = a_b0;
% PosAmersU = PosAmers0;
% P_U = blkdiag(P0);
% S_U = chol(P_U);

%IEKF
trajI = initTraj(NbSteps);
[stateI_0,PI_0] = stateStruct(Rot0,v0,x0,omega_b0,a_b0,PosAmers0,P0);
PI_0 = blkdiag(PI_0);
% trajI = initTraj(NbSteps);
% RotI = Rot0;
% vI = v0;
% xI = x0;
% omega_bI = omega_b0;
% a_bI = a_b0;
% PosAmersI = PosAmers0;
% P_I = blkdiag(P0);

%% Filtering
IdxImage = 2; % image index
for i = 2:NbStepsMax
    % propagation
    dt = t(i)-t(i-1);
    omega_i = omega(:,i); %Input angular velocity from gyroscope
    acc_i = acc(:,i); %Input acceleration from accelerometer
    
    chiAntR = chiR; % Save non propagated state
    RotR = RotR*expSO3((omega_i-omega_bR)*dt); %Propagate mean (see time discretization)
    vR = vR+(RotR*(acc_i-a_bR)+g)*dt; %Propagate mean (see time discretization)
    xR = xR+vR*dt; %Propagate mean (see time discretization)
    chiR = state2chi(RotR,vR,xR,PosAmersR); %State to Chi again (including the propagated changes)
    S_R = rukfPropagation(dt,chiR,chiAntR,omega_bR,a_bR,S_R,omega_i,...
        acc_i,Qc,g); %Sigma point generation and covariance propagation
    
    % propagation for others filters
    chiAntL = chiL;
    RotL = RotL*expSO3((omega_i-omega_bL)*dt);
    vL = vL+(RotL*(acc_i-a_bL)+g)*dt;
    xL = xL+vL*dt;
    chiL = state2chi(RotL,vL,xL,PosAmersL);
    S_L = lukfPropagation(dt,chiL,chiAntL,omega_bL,a_bL,S_L,omega_i,...
        acc_i,Qc,g);
    
    chiAntRef = [RotRef,xRef;zeros(1,3) 1];
    vAnt = vRef;
    RotRef = RotRef*expSO3((omega_i-omega_bRef)*dt);
    vRef = vRef+(RotRef*(acc_i-a_bRef)+g)*dt;
    xRef = xRef+vRef*dt;
    xidotRef = xidotRef + [omega_i-omega_bRef;vRef]*dt;
    chiRef = [RotRef,xRef;zeros(1,3) 1];
    S_Ref = ukfRefPropagation(dt,chiRef,chiAntRef,vAnt,omega_bRef,...
        a_bRef,S_Ref,omega_i,acc_i,Qc,g);
    
    RotAntU = RotU;
    RotU = RotU*expSO3((omega_i-omega_bU)*dt);
    vU = vU+(RotU*(acc_i-a_bU)+g)*dt;
    xU = xU+vU*dt;
    chiU = [RotU,xU;zeros(1,3) 1];
    S_U = ukfPropagation(dt,RotU,RotAntU,vAnt,omega_bU,a_bU,S_U,omega_i,...
        acc_i,Qc,g);
    
    [RotI,vI,xI,PosAmersI,P_I] = iekfPropagation(dt,RotI,vI,xI,...
        omega_bI,a_bI,PosAmersI,P_I,omega_i,acc_i,Q,g);
    chiI = state2chi(RotI,vI,xI,PosAmersI);
    
    % if measurement
    if obsTimes(i) == 1
        % track points in image
        [y,yAmers,trackerMain,trackerBis,pointsMain,validityMain,...
            myTracks,pointsBis] = ...
            ObserveLandmarks(trackerMain,trackerBis,dirImage,IdxImage,...
            fileImages,ParamFilter,RotR,xR,PosAmersR,i,S_R,myTracks);
        
        % update state
        param.yAmers = yAmers;
        [chiR,omega_bR,a_bR,S_R,pC_1,pC_2] = rukfUpdate(chiR,omega_bR,...
            a_bR,S_R,y,param,R,ParamFilter);
        [RotR,vR,xR,PosAmersR] = chi2state(chiR);
        
        % update other filters
        chiL = state2chi(RotL,vL,xL,PosAmersL);
        [chiL,omega_bL,a_bL,S_L] = lukfUpdate(chiL,omega_bL,...
            a_bL,S_L,y,param,R,ParamFilter);
        [RotL,vL,xL,PosAmersL] = chi2state(chiL);
        
        param.PosAmers = PosAmersRef;
        chiRef = state2chi(RotRef,vRef,xRef,PosAmersRef);
        [chiRef,vRef,PosAmersRef,omega_bRef,a_bRef,S_Ref,xidotRef] = ukfRefUpdate(chiRef,vRef,omega_bRef,...
            a_bRef,S_Ref,y,param,R,ParamFilter,PosAmersRef,xidotRef);
        RotRef = chiRef(1:3,1:3);
        xRef = chiRef(1:3,4);
        
        param.PosAmers = PosAmersU;
        chiU = state2chi(RotU,vU,xU,PosAmersU);
        [RotU,vU,xU,PosAmersU,omega_bU,a_bU,S_U] = ukfUpdate(RotU,vU,xU,omega_bU,...
            a_bU,S_U,y,param,R,ParamFilter,PosAmersU);
        
        [chiI,omega_bI,a_bI,P_I] = iekfUpdate(chiI,omega_bI,a_bI,...
            P_I,y,R,ParamFilter,yAmers);
        [RotI,vI,xI,PosAmersI] = chi2state(chiI);
        
        % save trajectory
        trajR = updateTraj(trajR,RotR,vR,xR,omega_bR,a_bR,i);
        trajL = updateTraj(trajL,RotL,vL,xL,omega_bL,a_bL,i);
        trajRef = updateTraj(trajRef,RotRef,vRef,xRef,omega_bRef,a_bRef,i);
        trajU = updateTraj(trajU,RotU,vU,xU,omega_bU,a_bU,i);
        trajI = updateTraj(trajI,RotI,vI,xI,omega_bI,a_bI,i);
        
        % remplace non visible landmarks
        [S_R,PosAmersR,ParamFilter,trackerBis,myTracks,PosAmersNew,...
            IdxAmersNew,trackCov,pointsMain,validityMain] = manageAmers(S_R,...
            PosAmersR,ParamFilter,ParamGlobal,trackerBis,...
            trajR,i,pointsMain,validityMain,IdxImage,myTracks,pointsBis);
        chiR = state2chi(RotR,vR,xR,PosAmersR);
        
        setPoints(trackerMain,pointsMain,validityMain);
        
        % remplace non visible landmarks for others filters with same
        % landmarks
        if isempty(IdxAmersNew) == 0
            P_L = S_L'*S_L;
            P_Ref = S_Ref'*S_Ref;
            P_U = S_U'*S_U;
            for jj = 1:length(IdxAmersNew)
                idx = IdxAmersNew(jj);
                idxP = 15+(3*idx-2:3*idx);
                P_L(:,idxP) = 0;
                P_L(idxP,:) = 0;
                P_L(idxP,idxP) = trackCov{jj};
                PosAmersL(:,idx) = PosAmersNew(jj,:)';
                P_Ref(:,idxP) = 0;
                P_Ref(idxP,:) = 0;
                P_Ref(idxP,idxP) = trackCov{jj};
                PosAmersRef(:,idx) = PosAmersNew(jj,:)';
                P_U(:,idxP) = 0;
                P_U(idxP,:) = 0;
                P_U(idxP,idxP) = trackCov{jj};
                PosAmersU(:,idx) = PosAmersNew(jj,:)';
                P_I(:,idxP) = 0;
                P_I(idxP,:) = 0;
                P_I(idxP,idxP) = trackCov{jj};
                PosAmersI(:,idx) = PosAmersNew(jj,:)';
            end
            S_L = chol(P_L);
            chiL = state2chi(RotL,vL,xL,PosAmersL);
            S_Ref = chol(P_Ref);
            chiRef = [RotRef,xRef;zeros(1,3) 1];
            S_U = chol(P_U);
            chiI = state2chi(RotI,vI,xI,PosAmersI);
        end
        disp(i/200);
        IdxImage = IdxImage+1;
    else
        trajR = updateTraj(trajR,RotR,vR,xR,omega_bR,a_bR,i);
        trajL = updateTraj(trajL,RotL,vL,xL,omega_bL,a_bL,i);
        trajRef = updateTraj(trajRef,RotRef,vRef,xRef,omega_bRef,a_bRef,i);
        trajU = updateTraj(trajU,RotU,vU,xU,omega_bU,a_bU,i);
        trajI = updateTraj(trajI,RotI,vI,xI,omega_bI,a_bI,i);
    end
end

%% Plots
errorR = computeError(trajR,trajReal,i);
errorL = computeError(trajL,trajReal,i);
errorU = computeError(trajU,trajReal,i);
errorRef = computeError(trajRef,trajReal,i);
errorI = computeError(trajI,trajReal,i);

figure;hold on;
plot(t(2:i-1)-t(2),errorR.errorR);
plot(t(2:i-1)-t(2),errorL.errorR);
plot(t(2:i-1)-t(2),errorRef.errorR);
plot(t(2:i-1)-t(2),errorU.errorR);
plot(t(2:i-1)-t(2),errorI.errorR);
disp(sqrt(mean([errorR.errorR errorL.errorR  errorRef.errorR errorU.errorR errorI.errorR].^2)));
legend('R-UKF-LG','L-UKF-LG','SE(3)-UKF','UKF','IEKF')
xlabel('t (s)')
ylabel('RMSE attitdude (°)')
title('RMSE on attitude as function of time')
figure
hold on;
plot(t(2:i-1)-t(2),errorR.errorX);
plot(t(2:i-1)-t(2),errorL.errorX);
plot(t(2:i-1)-t(2),errorRef.errorX);
plot(t(2:i-1)-t(2),errorU.errorX);
plot(t(2:i-1)-t(2),errorI.errorX);
disp(sqrt(mean([errorR.errorX  errorL.errorX errorRef.errorX errorU.errorX errorI.errorX].^2)));
legend('R-UKF-LG','L-UKF-LG','SE(3)-UKF','UKF','IEKF')
xlabel('t (s)')
ylabel('RMSE position (m)')
title('RMSE on position as function of time')