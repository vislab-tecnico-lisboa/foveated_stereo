workingDir=pwd;
imageNames = dir(fullfile(workingDir,'*.png'));
outputVideo = VideoWriter(fullfile(pwd,'shuttle_out.avi'));
outputVideo.FrameRate = 1;
outputVideo.Quality=100;
open(outputVideo)
filename='test.gif';
for ii = 1:length(imageNames)
   img = imread(fullfile(workingDir,imageNames(ii).name));
   writeVideo(outputVideo,img)
   
   [A,map] = rgb2ind(img,256); 
	if ii == 1;
		imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.1);
	else
		imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.1);
    end
end
close(outputVideo);

