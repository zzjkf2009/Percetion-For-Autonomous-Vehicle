%*@File ReadFrames.m
%*@Author Zejiang Zeng
%*@Copyright 2017.2-2017.5 University of Maryland, Zejiang Zeng (zzeng@terpmail.umd.edu)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read image from video
clear
v=VideoReader('../Data/project_video.mp4');
for n=700:800
   filename=strcat('frame',num2str(n),'.jpg');
   b=read(v,n);
   imwrite(b,filename);
end
%%
clear
for ii=200:300
   
try
    filename=strcat('frame',num2str(ii),'.jpg');
    currentimage=imread(filename);
    %[CornerImage,LenaImage]=Mainfunction(currentimage);
    [OutImage]=mainloop(currentimage,ii);

    %[LenaImage]=Method2(ii);
    
    filename_lane=strcat('Lane',num2str(ii),'.jpg');
%     filename_ID=strcat('ID',num2str(ii),'.jpg');
    imwrite(OutImage,filename_lane);

catch 
    ii
    continue;
end
end
%%
clear 

writerObj = VideoWriter('LaneDection.avi');
writerObj.FrameRate = 30;
open(writerObj);
 for ii = 700:800
     try
    filename=strcat('Lane',num2str(ii),'.jpg');
%    img = imread(images{ii});
   img=imread(filename);
   writeVideo(writerObj,img)
     catch
         continue;
     end
 end
close(writerObj)
% v=VideoReader('Lena.avi');
%%
clear
Text_position=[650 640];
 text='Straight ahead';
for ii=302:500
   
try
    filename=strcat('lane',num2str(ii),'.jpg');
    currentimage=imread(filename);
    %[CornerImage,LenaImage]=Mainfunction(currentimage);
   

    %[LenaImage]=Method2(ii);
     FinalFrame=insertText(currentimage,Text_position,text,'FontSize',20,'BoxColor','white','TextColor','black');
   % filename_lane=strcat('Lane',num2str(ii),'.jpg');
%     filename_ID=strcat('ID',num2str(ii),'.jpg');
    imwrite(FinalFrame,filename);

catch 
    ii
    continue;
end
end