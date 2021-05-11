function plotInterpolation(t,waypointTimes,q,varargin)

%q - orientation represented in euler angles ZYX
%qd - angular velocity [x y z]
%qdd - angular acceleration [x y z]

% Find the order of the trajectory (how many derivatives are provided)
order = 1; % Position only
if nargin > 2 
    order = 2; % Position and velocity
    if nargin > 3 
       order = 3; % Position, velocity, and acceleration 
    end
end

numCoords = 3;
axisName = ['Z','Y','X'];
axisOrder = [3 2 1];

% Loop through all the coordinates and plot
for idx = 1:numCoords
    figure; 
    % Always plot time and position only
    subplot(order,1,1), hold on;
    plot(t,q(:,idx));
    plotTimeLines(waypointTimes);
    ylabel('Angle[rad]');
    title("Rotation interpolation trajetory with trapezoidal time scaling " + axisName(idx) + "-Axis")
    
    % Plot velocity if provided
    if nargin > 2
        subplot(order,1,2), hold on;
        qd = varargin{1};
        plot(t,qd(axisOrder(idx),:));
        plotTimeLines(waypointTimes);
        ylabel('Angular velocity [rad/s]');
        
            % Plot acceleration if provided
            if nargin > 3
                subplot(order,1,3), hold on;
                qdd = varargin{2};
                plot(t,qdd(axisOrder(idx),:));
                plotTimeLines(waypointTimes);
                ylabel('Angular acceleration [rad/s^2]');
            end     
    end
    
    % Finally, label the time axis
    xlabel('Time');
   
end


end


% Helper function to plot vertical lines for the waypoint times
function plotTimeLines(t)
    for idx = 1:numel(t)
       xline(t(idx),'r--'); 
    end
end

