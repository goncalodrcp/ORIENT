function [interpInfo,ikInfo,genTime] = rotationInterpAndIK(numWayPoints, orientations, toolPosition, wayPointTimes, ts, frameID, ik, ikWeights, ikInitGuess)
%Trajectory generation and following function
%INPUTS: numWayPoints - number of waypoints in the trajectoty;
% wayPoints - 3x(numWayPoints) matrix cointaining the orientations
% expressed in Euler Angles (ZYX);
% toolPosition - (X,Y,Z) coordinates of the gripper frame relative to the
% base frame
% wayPointTimes - array of time instants at which the robot reaches the
% waypoints (1xnumWayPoins)
% ts - Sampling frequency (Hz) for interpolation
% frameID - String containing the name of the frame relative to the base
% frame that we wish compute the transformation.
% ik - Inverse kinematics object for the rigid body tree (Kinova Gen3)
% ikWeights - array containing the weights for each joint angle for the IK
% numeric algorithm
% ikInitGuess - Initial guess of the robot configuration
%
% OUTPUTS:
% interpInfo - structure containing the results of the interpolation
% (SLERP)
% ikInfo - structure containing the IK results
% genTime - time duration of the execution
%

% Initialize rotational trajectory variables for orientation, velocity and
% acceleration;
rot = [];
angv = [];
angacc = [];

%Initializse joint angles and time vector
jointAnglesIK=[];

%Get each orientation in quaternions
Q = eul2quat(orientations(:,1:end)'); 

i=1;
tic;
for w = 1:numWayPoints-1
    
    % Get the initial and final rotations and times for the segment
    qi = Q(w,:);
    qf = Q(w+1,:);
    timeInterval = wayPointTimes(w:w+1); %Get waypoints time instants
    if w==1
        segmentTrajTimes = timeInterval(1):ts:timeInterval(2);
    else
        segmentTrajTimes = timeInterval(1)+ts:ts:timeInterval(2);
    end
    
    waypointAccelTimeSeg = diff(timeInterval)/4;
 
    %Polynomial time scaling - cubic
    %[s,sd,sdd] = cubicpolytraj([0 1],timeInterval,segmentTrajTimes);
    % Trapezoidal time scaling
    %[s,sd,sdd] = trapveltraj([0 1],numel(segmentTrajTimes)); %O envio para o kinova foi feito usando isto!!!!
    [s,sd,sdd] = trapveltraj([0 1],numel(segmentTrajTimes),'AccelTime',waypointAccelTimeSeg,'EndTime',diff(timeInterval));
    % Find the quaternions from trajectory generation
    [R, omega, alpha] = rottraj(qi, qf, timeInterval, segmentTrajTimes,'TimeScaling',[s;sd;sdd]);  
    % Save this trajectory interpolation
    rot = [rot R];
    angv = [angv omega];
    angacc = [angacc alpha];
    
    % Trajectory following loop   
    for idx = 1:numel(segmentTrajTimes) 
        % Solve IK
        tgtPose = trvec2tform(toolPosition) * quat2tform(R(:,idx)');
        % Plot
        %plotTransforms(tform2trvec(tgtPose),tform2quat(tgtPose),'FrameSize',0.05);
        [config,info] = ik(frameID,tgtPose,ikWeights,ikInitGuess);
        ikInitGuess = config;
        poseErrorIK(i) = info.PoseErrorNorm;
        % Save joint angles of IK
        jointAnglesIK(i,:) = config;
        timesIK(i) = segmentTrajTimes(idx);
        i=i+1;
    end
   
end

%Compute joint velocity and acceleration using numerical differentiation
angularVelIK=diff(jointAnglesIK)/ts;
angularVelIK(1,:) = 0;
angularVelIK(end+1,:) = 0;
     
angularAccIK=diff(angularVelIK)/ts;
angularAccIK(1,:) = 0;
angularAccIK(end+1,:) = 0;

%Output
interpInfo.Rot = rot;
interpInfo.AngVel = angv;
interpInfo.AngAcc = angacc;
ikInfo.jointAngles = jointAnglesIK;
ikInfo.jointVel = angularVelIK;
ikInfo.jointAcc = angularAccIK;
ikInfo.time = timesIK;
ikInfo.error = poseErrorIK;
genTime = toc;
 
end

