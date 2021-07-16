function [interpInfo] = rotationInterpolation(numWayPoints, orientations, wayPointTimes, ts)

rot = [];
angv = [];
angacc = [];

%Initializse joint angles and time vector
jointAnglesIK=[];

%Get each orientation in quaternions
rotMatrix = eul2rotm(orientations');
for i=1:numWayPoints
    Q(:,i) = rotm2quat(rotMatrix(:,:,i)*[1 0 0;0 -1 0;0 0 -1]);
end
Q = Q'; 

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

end

