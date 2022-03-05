clear;
close all;

%%

path = 'D:\IST\ORIENT_repos\Tests\ThesisSW\ESIM_test\Fusion_Kinova_30_09\Results\';
fileName = 'res_Z_A20_v18_MF10_featuretracking.mat';
load([path fileName]);

for i=1:length(trackedFeatures)
   
    numLandmarks(i) = length(trackedFeatures{i}.yAmers);
    frame(i) = trackedFeatures{i}.idImage;
    
    
%     camMeasuremnts(:,i) = trackedFeatures{i}.y;
%     predictedMeasMean(:,i) = trackedFeatures{i}.ybar;
%     landmarksId(:,i) = trackedFeatures{i}.yAmers;
    
end


%%

plot(frame,numLandmarks)
xlabel('Frame number');
ylabel('Landmarks tracked');
ylim([0 35]);



%%

%Retrieve parameters from the filter
cameraParams = ParamFilter.cameraParams;
dirImage = ParamGlobal.dirImage;
fileImages = ParamGlobal.fileImages;

%Image number (random)
i = 350;
%Landmark id
landmarksID = trackedFeatures{i-1}.yAmers;
%Measurements from the KLT tracker
camMeas = trackedFeatures{i-1}.y;
%Predicted measurement mean
predictedMeasMean = trackedFeatures{i-1}.ybar;

image = rgb2gray(imread(fileImages{i}));  
image = undistortImage(image,cameraParams);

camMeas = reshape(camMeas,[2,length(camMeas)/2]);
predictedMeasMean = reshape(predictedMeasMean,[2,length(predictedMeasMean)/2]);

figure; 
%image = insertMarker(image,camMeas','+','Color','green','size',5); %tracked points
%image = insertMarker(image,predictedMeasMean','x','Color','red','size',5); %predicted measurements
imshow(image);
hold on;
plot(camMeas(1,:),camMeas(2,:),'+','Color','green','MarkerSize',10,'LineWidth',2);
plot(predictedMeasMean(1,:),predictedMeasMean(2,:),'X','Color','red','MarkerSize',10,'LineWidth',2);
legend('Tracked landmarks','Predicted measurements');


