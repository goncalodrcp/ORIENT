function [trajectory] = compute1kHzTrajectory(ikInfo,waypointTimes,trajTimes)

jointAnglesIK = ikInfo.jointAngles;

numJoints = size(jointAnglesIK,2);

%Joint angles home? or first guess from IK
jointWaypoints = [];
for i=1:length(waypointTimes)
    jointWaypoint = jointAnglesIK(trajTimes == waypointTimes(i),:);
    jointWaypoints = [jointWaypoints jointWaypoint'];
end

%jointWaypoints = [jointAnglesIK(1,:)' jointAnglesIK(end,:)'];
sampleTime = 0.001; %1 millisecond/ 1kHz
numSamples = waypointTimes(end)/sampleTime + 1; 
[qJoint,qdJoint,qddJoint] = trapveltraj(jointWaypoints,numSamples, ... 
                    'EndTime',repmat(diff(waypointTimes),[numJoints 1]));
                
%[qJoint,qdJoint,qddJoint] = trapveltraj(jointWaypoints,numel(trajTimes), ... 
%                    'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
%                    'EndTime',repmat(diff(waypointTimes),[numJoints 1]));

trajTimes = linspace(0,waypointTimes(end),numSamples); 

% Trajectory ready to send to the robot
trajectory.angles = qJoint;
trajectory.velocity = qdJoint;
trajectory.acceleration = qddJoint;
trajectory.time = trajTimes;


end

