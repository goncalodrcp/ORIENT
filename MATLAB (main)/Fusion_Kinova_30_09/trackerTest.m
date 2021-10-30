function trackerTest(ParamGlobal, orb_slam, ParamFilter)

figure;

% TrackerMain
for i = 2: 200

    image = ParamGlobal.fileImages{i};    
    image = undistortImage(image,ParamFilter.cameraParams);

    [pointsMain,validityMain] = orb_slam.trackerMain.step(image);

    pointImage = insertMarker(image,pointsMain,'+','Color','green');

    imshow(pointImage);
end

%% TrackerBis

figure;
for i = 2: 200

    image = ParamGlobal.fileImages{i};    
    image = undistortImage(image,ParamFilter.cameraParams);

    [pointsMain,validityMain] = orb_slam.trackerBis.step(image);

    pointImage = insertMarker(image,pointsMain,'+','Color','red');

    imshow(pointImage);
end