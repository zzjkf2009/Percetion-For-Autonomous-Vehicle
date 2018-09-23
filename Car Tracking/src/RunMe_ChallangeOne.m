clear
%close all
clc

detector = vision.CascadeObjectDetector('CarDetector2.xml');
% videoFileReader  = vision.VideoFileReader('../Dataset/simple.avi','VideoOutputDataType','uint8');
% img      = step(videoFileReader);
for n=800:800
imagename=strcat('frame',num2str(n),'.jpg');
img = imread(imagename); 
bbox = step(detector,img); 
[Numbbox,~]=size(bbox); 
for kk=Numbbox:-1:1
    if (bbox(kk,3)>100)
        bbox(kk,:)=[];
    end
end

%
bbox(6,:)=[];
bbox(2,:)=[];
bbox(1,:)=[];
%
[New_Numbbox,~]=size(bbox);
% % resize the opential car image and save them 
% close all
% for loop=1:New_Numbbox
%     cropfilename=strcat('crop_challange_',num2str(n),'_',num2str(loop),'.png');
%     OutImage_1=imcrop(img,bbox(loop,:));
%     OutImage_2=imresize(OutImage_1,[64 64]);
%     imwrite(OutImage_2,cropfilename);
% end

for mm=1:New_Numbbox
carname=strcat('car',num2str(mm));
img = insertObjectAnnotation(img,'rectangle',bbox(mm,:),carname);
end
imagename=strcat('out',num2str(n),'.jpg');
figure; imshow(img), title(imagename)
imwrite(img,imagename);
centroid=zeros(New_Numbbox,2);
for i=1:New_Numbbox
centroid(i,:)=[bbox(i,1)+round(bbox(i,3)/2),bbox(i,2)+round(bbox(i,3)/2)];
end
old_bbox=bbox;
oldcentroid=centroid;
old_Numbbox=New_Numbbox;
end




%%
%close all
for ff=801:802
imagename=strcat('frame',num2str(ff),'.jpg');
img = imread(imagename); 
bbox = step(detector,img); 
[Numbbox,~]=size(bbox); 
for kk=Numbbox:-1:1
    if (bbox(kk,3)>100)
        bbox(kk,:)=[];
    end
end

%
% bbox(6,:)=[];
% bbox(2,:)=[];
% Find the centroid for susscive frams
[New_Numbbox,~]=size(bbox);
centroid=zeros(New_Numbbox,2);
for i=1:New_Numbbox
centroid(i,:)=[bbox(i,1)+round(bbox(i,3)/2),bbox(i,2)+round(bbox(i,3)/2)];
end

% find if the centroid from previous frame is close to the current frame

index=zeros(old_Numbbox,1);
for nn=1:old_Numbbox 
    centroid_x=centroid(:,1)-oldcentroid(nn,1);
    centroid_y=centroid(:,2)-oldcentroid(nn,2);
    centroid_xy=sqrt(centroid_x.*centroid_x+centroid_y.*centroid_y);
try
    k=find(centroid_xy<20);
    [Numclose,~]=size(k);
    if Numclose==1
       index(nn)=k;
    else
        if Numclose>1
        [~,mink] =min(centroid_xy);
         index(nn)=mink;
        end
    end
catch
    index(nn)=0;
end
end


for mm=1:old_Numbbox
    %carname=strcat('car',num2str(mm));
     carname=strcat('car');
    if(index(mm)~=0)    
   
    img=insertObjectAnnotation(img,'rectangle',bbox(index(mm,:),:),carname);
    
    else 
   
   img=insertObjectAnnotation(img,'rectangle',old_bbox(mm,:),carname);
  
    end
end
imagename=strcat('out',num2str(ff),'.jpg');
figure; imshow(img), title(imagename)
% imwrite(img,imagename);
centroid=zeros(New_Numbbox,2);
for i=1:New_Numbbox
centroid(i,:)=[bbox(i,1)+round(bbox(i,3)/2),bbox(i,2)+round(bbox(i,3)/2)];
end
old_bbox=bbox;
oldcentroid=centroid;
old_Numbbox=New_Numbbox;
 end
%%
clear
close all
clc

detector = vision.CascadeObjectDetector('CarDetector2.xml');
img = imread('frame800.jpg'); 
bbox = step(detector,img); 
[Numbbox,~]=size(bbox);
for kk=Numbbox:-1:1
    if (bbox(kk,2)<300)||(bbox(kk,1)<230)||(bbox(kk,3)>100||(bbox(kk,1)>600))
        bbox(kk,:)=[];
    end
end
% [selectedBbox,selectedScore] = selectStrongestBbox(bbox,score);
detectedImg = insertObjectAnnotation(img,'rectangle',bbox(1,:),'car1');
detectedImg = insertObjectAnnotation(detectedImg,'rectangle',bbox(2,:),'car2');
detectedImg = insertObjectAnnotation(detectedImg,'rectangle',bbox(3,:),'car3');
detectedImg = insertObjectAnnotation(detectedImg,'rectangle',bbox(4,:),'car4');
figure; imshow(detectedImg);
[New_Numbbox,~]=size(bbox);
%%
% % resize the opential car image and save them 
% close all
% for loop=1:New_Numbbox
%     cropfilename=strcat('crop_62_',num2str(loop),'.png');
%     OutImage_1=imcrop(img,bbox(loop,:));
%     OutImage_2=imresize(OutImage_1,[64 64]);
%     imwrite(OutImage_2,cropfilename);
% end

%%
% identify car features to track
points1=detectMinEigenFeatures(rgb2gray(img),'ROI',bbox(1,:));
points2=detectMinEigenFeatures(rgb2gray(img),'ROI',bbox(2,:));
points3=detectMinEigenFeatures(rgb2gray(img),'ROI',bbox(3,:));
points4=detectMinEigenFeatures(rgb2gray(img),'ROI',bbox(4,:));
points=vertcat(points1,points2,points3,points4);
%points = detectHarrisFeatures(rgb2gray(img),'ROI',bbox(3,:),'FilterSize',3,'MinQuality', 0.02);
%points = detectFASTFeatures(rgb2gray(img),'ROI',bbox(3,:),'MinQuality', 0.02);
%points=detectSURFFeatures(rgb2gray(img),'ROI',bbox(3,:),'NumScaleLevels' ,5);
%display the detected points
figure, imshow(img), hold on, title('Detected Features');
plot(points);

% Create a point tracker and enable the bidirectional error constraint to
% make it more robust in the presence of noise and clutter.
pointTracker = vision.PointTracker;
% Initialize the tracker with the initial point locations and the initial
% video frame.
points = points.Location;
initialize(pointTracker, points, img);

% Create a video player object for displaying video frames.
videoPlayer  = vision.VideoPlayer('Position',...
    [100 100 [size(img, 2), size(img, 1)]+30]);

% Track the Face
% Track the points from frame to frame, and use
% |estimateGeometricTransform| function to estimate the motion of the face.

% Make a copy of the points to be used for computing the geometric
% transformation between the points in the previous and the current frames
oldPoints = points;

bboxPoints1 = bbox2points(bbox(1, :));
bboxPoints2 = bbox2points(bbox(2 ,:));
bboxPoints3 = bbox2points(bbox(3 ,:));
bboxPoints4 = bbox2points(bbox(4 ,:));

videoFileReader = vision.VideoFileReader('../Dataset/challenge.avi','VideoOutputDataType','uint8');
%videoFWriter = vision.VideoFileWriter('track the car_KLT.avi','FrameRate',30);
while ~isDone(videoFileReader)
    % get the next frame
    videoFrame = step(videoFileReader);

    % Track the points. Note that some points may be lost.
    [points, isFound,scores] = step(pointTracker, videoFrame);
    visiblePoints = points(isFound, :);
   
    oldInliers = oldPoints(isFound, :);
    %oldInliers = oldPoints;
    
    if size(visiblePoints, 1) >= 2 % need at least 2 points
        
        % Estimate the geometric transformation between the old points
        % and the new points and eliminate outliers
        [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
            oldInliers, visiblePoints, 'similarity', 'MaxDistance', 0.5);

        % Apply the transformation to the bounding box points
        bboxPoints1 = transformPointsForward(xform, bboxPoints1);
        bboxPoints2 = transformPointsForward(xform, bboxPoints2);
        bboxPoints3 = transformPointsForward(xform, bboxPoints3);
        bboxPoints4 = transformPointsForward(xform, bboxPoints4);
        
                
        % Insert a bounding box around the object being tracked
        bboxPolygon1 = reshape(bboxPoints1', 1, []);
        bboxPolygon2 = reshape(bboxPoints2', 1, []);
        bboxPolygon3 = reshape(bboxPoints3', 1, []);
        bboxPolygon4 = reshape(bboxPoints4', 1, []);
        
        videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon1, ...
            'LineWidth', 2);
        videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon2, ...
            'LineWidth', 2);
        videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon3, ...
            'LineWidth', 2);
        videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon4, ...
            'LineWidth', 2);
                
        % Display tracked points
        videoFrame = insertMarker(videoFrame, visiblePoints, '+', ...
            'Color', 'white');       
        
        % Reset the points
        oldPoints = visiblePoints;
        setPoints(pointTracker, oldPoints);        
     end
    
    % Display the annotated video frame using the video player object
    step(videoPlayer, videoFrame);
    step(videoFWriter,videoFrame);
end

Clean up
release(videoFileReader);
release(videoPlayer);
release(pointTracker);
%release(videoFWriter);