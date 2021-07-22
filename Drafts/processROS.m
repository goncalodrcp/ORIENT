clear;
close all
%% Process ROSBAG images

bag = rosbag('images.bag');
bIm = select(bag,'Topic','/camera/image_raw');
msgStructs = readMessages(bIm);


for i=1:length(msgStructs)
    
    images{i} = readImage(msgStructs{i});
    images{i} = im2gray(images{i});
    images{i} = im2double(images{i});
    figure;
    imshow(images{i});
    
end

