function [trajs, i] = mainExperiment_loop(orb_slam, IMU_img_struct, state_camera_struct, NbStepsMax, obsTimes, ParamFilter, ParamGlobal)

trajReal = IMU_img_struct.trajReal;
NbSteps = IMU_img_struct.NbSteps;
t = IMU_img_struct.t;
omega = IMU_img_struct.omega;
acc = IMU_img_struct.acc;
g = IMU_img_struct.g;

P0 = state_camera_struct.P0;
Q = state_camera_struct.Q;
Qc = state_camera_struct.Qc;
R = state_camera_struct.R;

dirImage = ParamGlobal.dirImage;
fileImages = ParamGlobal.fileImages;


% Rot0 = eul2rotm([trajReal.psi(1),trajReal.theta(1),trajReal.phi(1)]);
Rot0 = eul2rotm([0 0 0]);
%x0 = trajReal.x(:,1)/100;
%x0 = trajReal.x(:,1); %doesnt work well most of the time
x0 = [0;0;0];
%x0 = [-0.05;-0.05;-0.05];
v0 = trajReal.v(:,1);
omega_b0 = orb_slam.omega_b;
a_b0 = orb_slam.a_b;
PosAmers0 = orb_slam.PosAmers;
trackerMain = orb_slam.trackerMain;
trackerBis = orb_slam.trackerBis;
myTracks = orb_slam.myTracks;

IdxImage = 2; % image index

trajL = initTraj(NbSteps);
RotL = Rot0;
vL = v0;
xL = x0;
omega_bL = omega_b0;
a_bL = a_b0;
PosAmersL = PosAmers0;
P_L = P0;
S_L = chol(P_L);
chiL = state2chi(RotL,vL,xL,PosAmersL);

trajR = initTraj(NbSteps);
RotR = Rot0;
vR = v0;
xR = x0;
omega_bR = omega_b0;
a_bR = a_b0;
PosAmersR = PosAmers0;
P_R = P0;
S_R = chol(P_R);
chiR = state2chi(RotR,vR,xR,PosAmersR);

trajRef = initTraj(NbSteps);
RotRef = Rot0;
vRef = v0;
xRef = x0;
omega_bRef = omega_b0;
a_bRef = a_b0;
PosAmersRef = PosAmers0;
P_Ref = blkdiag(P0);
S_Ref = chol(P_Ref);
chiRef = [RotRef xRef;0 0 0 1];
xidotRef = zeros(6,1);

trajU = initTraj(NbSteps);
RotU = Rot0;
vU = v0;
xU = x0;
omega_bU = omega_b0;
a_bU = a_b0;
PosAmersU = PosAmers0;
P_U = blkdiag(P0);
S_U = chol(P_U);

trajI = initTraj(NbSteps);
RotI = Rot0;
vI = v0;
xI = x0;
omega_bI = omega_b0;
a_bI = a_b0;
PosAmersI = PosAmers0;
P_I = blkdiag(P0);

%% Filtering
for i = 2:NbStepsMax
    
    if i == 1000
        flag = 1;
    end 
    
    % propagation
    dt = t(i)-t(i-1);
    omega_i = omega(:,i);
    acc_i = acc(:,i);
    
    chiAntR = chiR;
    RotR = RotR*expSO3((omega_i-omega_bR)*dt);
    vR = vR+(RotR*(acc_i-a_bR)+g)*dt;
    xR = xR+vR*dt;
    chiR = state2chi(RotR,vR,xR,PosAmersR);
    S_R = rukfPropagation(dt,chiR,chiAntR,omega_bR,a_bR,S_R,omega_i,...
        acc_i,Qc,g);
    
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
        % track points in image - using R-UKF
        [y,yAmers,trackerMain,trackerBis,pointsMain,validityMain,...
            myTracks,pointsBis] = ...
            ObserveLandmarks(trackerMain,trackerBis,dirImage,IdxImage,...
            fileImages,ParamFilter,RotR,xR,PosAmersR,i,S_R,myTracks);
        
        %Use L-UKF
%         [y,yAmers,trackerMain,trackerBis,pointsMain,validityMain,...
%             myTracks,pointsBis] = ...
%             ObserveLandmarks(trackerMain,trackerBis,dirImage,IdxImage,...
%             fileImages,ParamFilter,RotL,xL,PosAmersL,i,S_L,myTracks);
     
        %try different approach - with "standard" UKF
%         [y,yAmers,trackerMain,trackerBis,pointsMain,validityMain,...
%             myTracks,pointsBis] = ...
%             ObserveLandmarks(trackerMain,trackerBis,dirImage,IdxImage,...
%             fileImages,ParamFilter,RotU,xU,PosAmersU,i,S_U,myTracks);
        
        % update state
        param.yAmers = yAmers;
        [chiR,omega_bR,a_bR,S_R] = rukfUpdate(chiR,omega_bR,...
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
        
        %Same but for L-UKF
%         [S_L,PosAmersL,ParamFilter,trackerBis,myTracks,PosAmersNew,...
%             IdxAmersNew,trackCov,pointsMain,validityMain] = manageAmers(S_L,...
%             PosAmersL,ParamFilter,ParamGlobal,trackerBis,...
%             trajL,i,pointsMain,validityMain,IdxImage,myTracks,pointsBis);
%         chiL = state2chi(RotL,vL,xL,PosAmersL);
        
        setPoints(trackerMain,pointsMain,validityMain);
        
        % remplace non visible landmarks for others filters with same
        % landmarks
        if isempty(IdxAmersNew) == 0
            %P_R = S_R'*S_R;
            P_L = S_L'*S_L;
            P_Ref = S_Ref'*S_Ref;
            P_U = S_U'*S_U;
            for jj = 1:length(IdxAmersNew)
                idx = IdxAmersNew(jj);
                idxP = 15+(3*idx-2:3*idx);
%                 P_R(:,idxP) = 0;
%                 P_R(idxP,:) = 0;
%                 P_R(idxP,idxP) = trackCov{jj};
%                 PosAmersR(:,idx) = PosAmersNew(jj,:);
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
%             S_R = chol(P_R);
%             chiR = state2chi(RotR,vR,xR,PosAmersR);
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

trajs.trajR = trajR;
trajs.trajL = trajL;
trajs.trajRef = trajRef;
trajs.trajU = trajU;
trajs.trajI = trajI;
