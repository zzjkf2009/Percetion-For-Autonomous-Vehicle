clear
close all
clc

detector = vision.CascadeObjectDetector('CarDetector2.xml');
% videoFileReader  = vision.VideoFileReader('../Dataset/simple.avi','VideoOutputDataType','uint8');
% img      = step(videoFileReader);

img = imread('frame1.jpg'); 

bbox = step(detector,img); 
% [selectedBbox,selectedScore] = selectStrongestBbox(bbox,score);
[Numbbox,~]=size(bbox); 
% for kk=Numbbox:-1:1
%     if (bbox(kk,2)<300)||(bbox(kk,1)<230)||(bbox(kk,3)>90||(bbox(kk,1)>600))
%         bbox(kk,:)=[];
%     end
% end
% [selectedBbox,selectedScore] = selectStrongestBbox(bbox,score);
[New_Numbbox,~]=size(bbox);
for mm=1:New_Numbbox
carname=strcat('car',num2str(mm));
img = insertObjectAnnotation(img,'rectangle',bbox(mm,:),carname);
end
figure; imshow(img), title('Original Image')
%imwrite(detectedImg,'out1.jpg');

centroid=zeros(New_Numbbox,2);
for i=1:New_Numbbox
centroid(i,:)=[bbox(i,1)+round(bbox(i,3)/2),bbox(i,2)+round(bbox(i,3)/2)];
end
old_bbox=bbox;
oldcentroid=centroid;
old_Numbbox=New_Numbbox;

%%
close all
for ff=2:300
imgname=strcat('frame',num2str(ff),'.jpg');
savename=strcat('out',num2str(ff),'.jpg');
videoFrame = imread(imgname); 
 bbox = step(detector,videoFrame); 
[Numbbox,~]=size(bbox);
for kk=Numbbox:-1:1
    if (bbox(kk,2)<300)||(bbox(kk,1)<230)||(bbox(kk,3)>90||(bbox(kk,1)>600))
        bbox(kk,:)=[];
    end
end
[New_Numbbox,~]=size(bbox);
centroid=zeros(New_Numbbox,2);
for i=1:New_Numbbox
centroid(i,:)=[bbox(i,1)+round(bbox(i,3)/2),bbox(i,2)+round(bbox(i,3)/2)];
end
index=zeros(old_Numbbox,1);
for nn=1:old_Numbbox
    centroid_x=centroid(:,1)-oldcentroid(nn,1);
    centroid_y=centroid(:,2)-oldcentroid(nn,2);
    centroid_xy=sqrt(centroid_x.*centroid_x+centroid_y.*centroid_y);
try
index(nn)=find(centroid_xy<20);
catch
    index(nn)=0;
end
end

for mm=1:old_Numbbox
    carname=strcat('car',num2str(mm));
    if(index(mm)~=0)    
   
    videoFrame=insertObjectAnnotation(videoFrame,'rectangle',bbox(index(mm,:),:),carname);
    else 
   
   videoFrame=insertObjectAnnotation(videoFrame,'rectangle',old_bbox(mm,:),carname);
    end
end
for ko=1:4
    if(index(ko)~=0)
        old_bbox(ko,:)=bbox(index(ko),:);
    else
         old_bbox(ko,:)= old_bbox(ko,:);
    end
end
for ii=1:4
oldcentroid(ii,:)=[old_bbox(ii,1)+round(old_bbox(ii,3)/2),old_bbox(ii,2)+round(old_bbox(ii,3)/2)];
end
 imwrite(videoFrame,savename);
 figure, imshow(videoFrame); title(savename);
 end
%%
videoPlayer  = vision.VideoPlayer('Position',...
    [100 100 [size(img, 2), size(img, 1)]]);
videoFWriter = vision.VideoFileWriter('track the car_test.avi','FrameRate',30);
while ~isDone(videoFileReader)
    videoFrame = step(videoFileReader);
    bbox = step(detector,videoFrame); 
[Numbbox,~]=size(bbox);
for kk=Numbbox:-1:1
    if (bbox(kk,2)<300)||(bbox(kk,1)<230)||(bbox(kk,3)>90||(bbox(kk,1)>600))
        bbox(kk,:)=[];
    end
end
[New_Numbbox,~]=size(bbox);
centroid=zeros(New_Numbbox,2);
for i=1:New_Numbbox
centroid(i,:)=[bbox(i,1)+round(bbox(i,3)/2),bbox(i,2)+round(bbox(i,3)/2)];
end
index=zeros(4,1);
for nn=1:old_Numbbox
       centroid_x=centroid(:,1)-oldcentroid(nn,1);
    centroid_y=centroid(:,2)-oldcentroid(nn,2);
    centroid_xy=sqrt(centroid_x.*centroid_x+centroid_y.*centroid_y);
try
index(nn)=find(centroid_xy<20);
catch
    index(nn)=0;
end
end

for mm=1:old_Numbbox
    carname=strcat('car',num2str(mm));
    if(index(mm)~=0)    
   
    videoFrame=insertObjectAnnotation(videoFrame,'rectangle',bbox(index(mm,:),:),carname);
    else 
   
    videoFrame=insertObjectAnnotation(videoFrame,'rectangle',old_bbox(mm,:),carname);
    end
end
for ko=1:4
    if(index(ko)~=0)
        old_bbox(ko,:)=bbox(index(ko),:);
    else
         old_bbox(ko,:)= old_bbox(ko,:);
    end
end
for ii=1:4
oldcentroid(ii,:)=[old_bbox(ii,1)+round(old_bbox(ii,3)/2),old_bbox(ii,2)+round(old_bbox(ii,3)/2)];
end
    step(videoPlayer, videoFrame);
    step(videoFWriter,videoFrame);
end
release(videoFileReader);
release(videoPlayer);
release(videoFWriter);