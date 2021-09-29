function trackerTest(ParamGlobal, orb_slam, ParamFilter)

figure;


% TrackerMain
for i = 2: 50
  
    image = strcat(ParamGlobal.dirImage,ParamGlobal.fileImages{i});
    image = undistortImage(imread(image),ParamFilter.cameraParams);

    [pointsMain,validityMain] = orb_slam.trackerMain.step(image);

    pointImage = insertMarker(image,pointsMain,'+','Color','white');

    imshow(pointImage);
end

%% TrackerBis

for i = 2: 50

    image = strcat(ParamGlobal.dirImage,ParamGlobal.fileImages{i});
    image = undistortImage(imread(image),ParamFilter.cameraParams);

    [pointsMain,validityMain] = orb_slam.trackerBis.step(image);

    pointImage = insertMarker(image,pointsMain,'+','Color','red');

    imshow(pointImage);
end