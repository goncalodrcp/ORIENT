function [trajectory] = compute1kHzTrajectory(ikInfo,waypointTimes,trajTimes)

jointAnglesIK = ikInfo.jointAngles;
jointVelocityIK = ikInfo.jointVel;
jointAccIK = ikInfo.jointAcc;

numJoints = size(jointAnglesIK,2);

%Joint angles home? or first guess from IK
jointWaypoints = [];
for i=1:length(waypointTimes)
    jointWaypoint = jointAnglesIK(trajTimes == waypointTimes(i),:);
    jointWaypoints = [jointWaypoints jointWaypoint'];
end

%% Trapezoidal velocity trajectory - in radians
%jointWaypoints = [jointAnglesIK(1,:)' jointAnglesIK(end,:)'];
%Last method used in experiments of 24/07
% sampleTime = 0.001; %1 millisecond/ 1kHz
% numSamples = waypointTimes(end)/sampleTime + 1; 
% [qJoint,qdJoint,qddJoint] = trapveltraj(jointWaypoints,numSamples, ... 
%                     'EndTime',repmat(diff(waypointTimes),[numJoints 1]));
%                 
% trajTimes = linspace(0,waypointTimes(end),numSamples); 
                
%[qJoint,qdJoint,qddJoint] = trapveltraj(jointWaypoints,numel(trajTimes), ... 
%                    'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
%                    'EndTime',repmat(diff(waypointTimes),[numJoints 1]));

%% Interpolation with Modified Akima - in degrees
% Method used in experiments of 14/10
sampleTime = 0.001; % 1ms sampling time for new trajectory
trajTimes = 0:sampleTime:waypointTimes(end);
t = ikInfo.time; %at 50ms
qJoint = interp1(t,jointAnglesIK,trajTimes,'makima');
qdJoint = interp1(t,jointVelocityIK,trajTimes,'makima');
qddJoint = interp1(t,jointAccIK,trajTimes,'makima');

% Trajectory ready to send to the robot
trajectory.angles = qJoint;
trajectory.velocity = qdJoint;
trajectory.acceleration = qddJoint;
trajectory.time = trajTimes;


end

