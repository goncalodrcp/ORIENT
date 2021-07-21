function makeVideo(name,imgStruct)

video = VideoWriter(name); %create the video object
video.FrameRate = 4; %set frame rate (4Hz)
open(video); %open the file for writing
for i=1:length(imgStruct) %where N is the number of images
  I = imgStruct{i}; %read the next image
  writeVideo(video,I); %write the image to file
end
close(video); %close the file

end

