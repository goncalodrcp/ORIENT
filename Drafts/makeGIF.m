

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