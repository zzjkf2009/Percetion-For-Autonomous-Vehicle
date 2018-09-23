%*@File ReadFrames.m
%*@Author Zejiang Zeng
%*@Copyright 2017.2-2017.5 University of Maryland, Zejiang Zeng (zzeng@terpmail.umd.edu)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read image from video

%v=VideoReader('../Dataset/simple.avi');
v=VideoReader('../Dataset/challenge.avi');
for n=800:850
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

writerObj = VideoWriter('track_the car_cloest_bbox_2.avi');
writerObj.FrameRate = 30;
open(writerObj);
 for ii = 1:300
     try
    filename=strcat('out',num2str(ii),'.jpg');
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
%%
% Create Mat file for images 

for n=3:358
   % PositiveCars.Cars{n} = [1,1,64,64];
     testPossitive.Testcars(n,1) = 1;
     testPossitive.Testcars(n,2) = 1;
     testPossitive.Testcars(n,3) = 64;
     testPossitive.Testcars(n,4) = 64; 
end
%%
i=0;
for n=5:490
    try
    Nof=n+100;
    filename=strcat('C:\Users\Zejiang Zeng\Desktop\HomeWork\EMPM673-Project1-master\ENPM673-Project3\CarTracking\Dataset\vehicles\\GTI_MiddleClose\image0',num2str(Nof),'.png');
    imread(filename);
    testPossitive.imageFilename{n-i} = filename;
    catch
        i=i+1;
    end
end

%%
i=0;
for n=2212:3059
    try
    Nof=n-2112;
    filename=strcat('C:\Users\Zejiang Zeng\Desktop\HomeWork\EMPM673-Project1-master\ENPM673-Project3\CarTracking\Dataset\vehicles\GTI_Right\image0',num2str(Nof),'.png');
    imread(filename);
    PositiveCars.imageFilename{n-i} = filename;
    catch
        i=i+1;
    end
end
%%
negativeFolder = fullfile('c:\','Users','Zejiang Zeng','Desktop',...
    'HomeWork','EMPM673-Project1-master','ENPM673-Project3','CarTracking','Dataset','non-vehicles','GTI');
negativeImages = imageDatastore(negativeFolder);
% trainCascadeObjectDetector('CarDetector2.xml',PositiveCars, ...
%     negativeFolder,'FeatureType','HOG','NumCascadeStages',15);

trainCascadeObjectDetector('CarDetector2.xml',testPossitive, ...
    negativeFolder,'FalseAlarmRate',0.2,'FeatureType','HOG','NumCascadeStages',5);
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

videoFileReader = vision.VideoFileReader('../Dataset/simple.avi','VideoOutputDataType','uint8');
videoFWriter = vision.VideoFileWriter('track the car_KLT.avi','FrameRate',30);
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
release(videoFWriter);