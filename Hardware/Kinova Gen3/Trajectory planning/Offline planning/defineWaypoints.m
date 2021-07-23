function [waypointTrajectory] = defineWaypoints(orientationWaypoints,axis,position,waypointTimes,ts)

% Get number of waypoints
numWaypoints = length(orientationWaypoints);

% Initialize orientation of each axis
orientations = zeros(3,numWaypoints);

% Because the order of rotation for euler angles is ZYX
if axis == 'X'
    orientations(3,:) = orientationWaypoints;
elseif axis == 'Y'
    orientations(2,:) = orientationWaypoints;
elseif axis == 'Z'
    orientations(1,:) = orientationWaypoints;
else
    disp('Invalid axis inserted');
end
           
%Waypoints - they should remain the same since we're only performing
%rotation
% Positions (X Y Z)
waypointsXYZ = repmat(position,1,numWaypoints); 
           
% Array of waypoint times
duration = waypointTimes(end); % Define total duration of the movement (s)

% Trajectory sample times
trajTimes = 0:ts:duration;

% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;

% Generate trajectory
waypointTrajectory.orientations = orientations;
waypointTrajectory.position = waypointsXYZ;
waypointTrajectory.trajtimes = trajTimes;
waypointTrajectory.accelTimes = waypointAccelTimes;

end

