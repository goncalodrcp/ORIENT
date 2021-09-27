function replayTrajectoryKinova(gen3,jointTrajectory)

numJoints = 7;
eeName = 'Gripper';
%
angleTraj = jointTrajectory.angles';
timestamp = jointTrajectory.time;

% Create figure and hold it
figure
set(gcf,'Visible','on');
show(gen3, angleTraj(1,:));
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
% Loop through values at specified interval and update figure
%count=1;
 for i = 2:100:length(timestamp) %change the step to increase animation speed
   transform(:,:,i) = getTransform(gen3,angleTraj(i,:),eeName); 
   plotTransforms(tform2trvec(transform(:,:,i)),tform2quat(transform(:,:,i)),'FrameSize',0.05);
   % Display manipulator model
   show(gen3, angleTraj(i,:), 'Frames', 'off', 'PreservePlot', false);
   title(['Trajectory at t = ' num2str(timestamp(i))]);
   % Update figure
   drawnow
   %frames(count)=getframe(gcf); %store frames for a video
   %count = count + 1;
 end

end

