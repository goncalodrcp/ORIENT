function [trackerBis,myTracks] = razTrackerBis(ParamGlobal,ParamFilter,I,IdxImage,myTracks)
% reset trackerBis when number of points tracked is too small

%%para evitar os hardcodes
num_tracks = 200;
rel_freq = 46;

fileImages = ParamGlobal.fileImages;
cameraParams = ParamFilter.cameraParams;
dirImage = ParamGlobal.dirImage;

if(IdxImage < 20)
    max = IdxImage-1;
else 
    max = 20;
end

idxImage = IdxImage-(0:1:max); %trakcing on previous images
idxReal = I-(0:rel_freq:rel_freq*length(idxImage)-rel_freq);
% image = strcat(dirImage,int2str(fileImages(idxImage(1))),'.png');
% image = fileImages{IdxImage};
%image = strcat(dirImage,fileImages{IdxImage});
%image = undistortImage(imread(image),cameraParams);
image = im2uint8(fileImages{IdxImage});
image = undistortImage(image,cameraParams);

trackerBis = vision.PointTracker('MaxBidirectionalError',1);
points = detectMinEigenFeatures(image);
points = selectUniform(points,num_tracks,size(image));
Location = points.Location;
initialize(trackerBis,Location,image);
trackerInit = clone(trackerBis);
for i = 1:length(Location)
    pT = pointTrack(1,Location(i,:));
    myTracks(i) = pT;
    myTracks(i).ViewIds = idxReal(1);
end
for i = 2:length(idxImage)
%     image = strcat(dirImage,int2str(fileImages(idxImage(i))),'.png');
    %image = fileImages{IdxImage};  %%%%FIZ PORCARIA AQUI
%     image = fileImages{idxImage(i)};
    %image = strcat(dirImage,fileImages{idxImage(i)});
    image = im2uint8(fileImages{IdxImage});
    image = undistortImage(image,cameraParams);
    [points, validity] = trackerInit.step(image);
    for ii = 1:length(validity)
        if(validity(ii) == 1)
            myTracks(ii).ViewIds = [idxReal(i) myTracks(ii).ViewIds];
            myTracks(ii).Points = [points(ii,:); myTracks(ii).Points];
        end
    end
end
end

