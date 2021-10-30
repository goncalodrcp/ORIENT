function [orb_slam, initHelper] = mainExperiment_init_orbSlam(ParamFilter, ParamGlobal)

num_tracks = 200; %it was 100

orb_slam.omega_b = [0;0;0];
orb_slam.a_b = [0;0;0];
%orb_slam.a_b = [0.007678;-0.007325;0.045866]; %data from LPMS sensor

cameraParams = ParamFilter.cameraParams;
dirImage = ParamGlobal.dirImage;
fileImages = ParamGlobal.fileImages;

%image = undistortImage( imread( [dirImage fileImages{1}] ), cameraParams );
image = rgb2gray(imread(fileImages{1})); 
%image = im2double(image);
%image = im2uint8(image);
image = undistortImage(image,cameraParams);
init_points = detectMinEigenFeatures(image); %detetar novos pontos de interesse
init_points = selectUniform(init_points,num_tracks+30,size(image)); %selcioanr novos potnos

for i=1:num_tracks
    orb_slam.myTracks(i) = pointTrack(1, init_points(i).Location);
end

orb_slam.trackerBis = vision.PointTracker('MaxBidirectionalError',3); %Typical value between 0 and 3 pixels.
initialize(orb_slam.trackerBis,init_points(1:num_tracks).Location,image);

%init_points = init_points(num_tracks + 1 : end); %WHY THIS???
init_points = selectUniform(init_points,30,size(image)); %selcioanr novos potnos

orb_slam.trackerMain = vision.PointTracker('MaxBidirectionalError',3); %Typical value between 0 and 3 pixels.
initialize(orb_slam.trackerMain,init_points.Location,image);

initHelper = init_points.Location;

orb_slam.PosAmers = zeros(3,30);

for i=1:30
    orb_slam.PosAmers(1,i) = (init_points.Location(i,1) - ParamFilter.Pi(1,3)) / ParamFilter.Pi(1,1);
    orb_slam.PosAmers(2,i) = (init_points.Location(i,2) - ParamFilter.Pi(2,3)) / ParamFilter.Pi(2,2);
    
%     orb_slam.PosAmers(1,i) = init_points.Location(i,1) / ParamFilter.Pi(1,1);
%     orb_slam.PosAmers(2,i) = init_points.Location(i,2) / ParamFilter.Pi(2,2);

    orb_slam.PosAmers(3,i) = 1;

end


