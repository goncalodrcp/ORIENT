%% Make gif for ESim dataset

%dirImg = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\rotations\rotation_18_x\'; %Y rotation - yes thats correct
%dirImg = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\rotations\rotation_18_y\'; %X rotation - yes thats correct
dirImg = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\rotations\rotation_18_z\';

addpath(genpath(dirImg));

Images = readtable('images.txt');
fileImages = table2array(Images(:,2));
filename = 'zRotationSim.gif';
N = length(fileImages);
for i = 1:N
    imagePath = strcat(dirImg,fileImages{i});
    image = imread(imagePath);
    %[imind,cm] = rgb2ind(image,256); 
      % Write to the GIF File 
      if i == 1 
          imwrite(image,filename,'gif', 'Loopcount',inf,'DelayTime',0.1); 
      else 
          imwrite(image,filename,'gif','WriteMode','append','DelayTime',0.1); 
      end 
    
end


%% Make gif for Kinova dataset

%Image directory 
imageDir = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Feedback\Y-Axis\im_Y_A20_v18';
%Create image data store object
imds = imageDatastore(imageDir);
fileImages = imds.Files;
filename = 'yRotationKinova_A20_v18.gif';

N = length(fileImages);
for i = 1:N
    image = rgb2gray(imread(fileImages{i})); 
    %[imind,cm] = rgb2ind(image,256); 
      % Write to the GIF File 
      if i == 1 
          imwrite(image,filename,'gif', 'Loopcount',inf,'DelayTime',0.02); 
      else 
          imwrite(image,filename,'gif','WriteMode','append','DelayTime',0.02); 
      end 
    
end

