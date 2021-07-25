function helperPlotIK(ikInfo,ts,waypointTimes,velocityLimit,accelerationLimit)

jointAnglesIK = ikInfo.jointAngles;
timesIK = ikInfo.time;

numJoints = size(jointAnglesIK,2);

numWaypoints = length(waypointTimes);

% Plot Inverse Kinematics joint angles
% Compare joint angles
% Plot each joint trajectory
angularVelIK=diff(jointAnglesIK)/ts;
angularVelIK(1,:) = 0;
angularVelIK(end+1,:) = 0;
     
angularAccIK=diff(angularVelIK)/ts;
angularAccIK(1,:) = 0;
angularAccIK(end+1,:) = 0;

anglePlot = figure;
velPlot = figure;
accPlot = figure;

for idx=1:numJoints
    
    % Angle plot
    figure(anglePlot), hold on;
    plot(timesIK,jointAnglesIK(:,idx));
    for wIdx = 1:numWaypoints
        xline(waypointTimes(wIdx),'k--'); 
    end
     title('Inverse Kinematics results - angle'); 
     xlabel('Time [s]');
     ylabel('Joint Angle [rad]');
     
     %Velocity plot
     figure(velPlot), hold on;
     plot(timesIK,angularVelIK(:,idx));
     yline(velocityLimit,'r','Max. Speed'); %Speed limit
     for wIdx = 1:numWaypoints
        xline(waypointTimes(wIdx),'k--'); 
     end
     title('Inverse Kinematics results - velocity'); 
     xlabel('Time [s]');
     ylabel('Angular velocity [rad/s]');
     
     % Acceleration plot
     figure(accPlot), hold on;
     plot(timesIK,angularAccIK(:,idx));
     yline(accelerationLimit,'r','Max. Acceleration'); %Acceleration limit
     for wIdx = 1:numWaypoints
        xline(waypointTimes(wIdx),'k--'); 
     end
     title('Inverse Kinematics results - acceleration'); 
     xlabel('Time [s]');
     ylabel('Angular acceleration [rad/s^2]');
          
end

end

