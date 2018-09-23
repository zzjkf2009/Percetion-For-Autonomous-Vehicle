%file RunMe.m
%Author: Zejiang Zeng
%*@Copyright 2017.2-2017.5 University of Maryland, Zejiang Zeng (zzeng@terpmail.umd.edu)


%%
clear 
close all
% read the image amd remove the noise
frame_num=270;
current_frame=strcat('frame',num2str(frame_num),'.jpg');
OriginalImage=imread(current_frame);
%figure, imshow(OriginalImage);
%remove the noise using median filter
GrayImage=rgb2gray(OriginalImage);
NoNoise=medfilt2(GrayImage);
% figure,imshow(NoNoise);
% Iblur1 = imGaussFilter(NoNoise,2);
% figure,imshow(Iblur1)
%%
% % Image enhencement
% pout_imadjust = imadjust(Iblur1);
% pout_histeq = histeq(Iblur1);
% figure,imshow(pout_imadjust)
% Iblur2 = imGaussFilter(pout_imadjust,2);
% figure,imshow(Iblur2)
%%
% % generate horizontal edge emphasis kernel
% h = fspecial('sobel');
% % invert kernel to detect vertical edges
% h = h';
% edge = imfilter(NoNoise,h);
% BW_edge=im2bw(edge,0.1);
% figure,imshow(BW_edge)

%%
BW_edge_canny=edge(NoNoise,'Canny',0.5);
figure,imshow(BW_edge_canny)

%% 
%Mask out the top half of the image
[width,length,~]=size(OriginalImage);
x=[length/6 length*5/6 length*5/6 length/6 length/6];
y=[round(width*2/3) round(width*2/3) width width round(width*2/3)];
%y=[round(width/2) round(width/2) width width round(width/2)];
Mask=poly2mask(x,y,width,length);
I_Masked = BW_edge_canny.*Mask;
%I_Masked = BW_edge.*Mask;
figure,imshow(I_Masked)

% se=strel('disk',7);
% closeBW=imclose(I_Masked,se);
% figure,imshow(closeBW);
%%
% Using Hough transform to find the lines
%[H,T,R]=hough(closeBW);
[H,T,R]=hough(I_Masked); 
P  = houghpeaks(H,25,'threshold',ceil(0.1*max(H(:))));
%lines = houghlines(closeBW,T,R,P,'FillGap',20,'MinLength',20);
lines = houghlines(I_Masked,T,R,P,'FillGap',20,'MinLength',20);
Pre_lines=lines;
figure,imshow(OriginalImage), hold on
% max_len = 0;
[~,NumLines]=size(lines);
%%
%Fliter the lines into two grows, right and left lane candidate
for i=NumLines:-1:1
    if abs(lines(i).theta)>70
        lines(i)=[];

    elseif  lines(i).theta>0 && lines(i).point1(1)>(length/2)
         lines(i)=[];
    end
end  
[~,NewLines]=size(lines);
p_i=1;
n_i=1;
for j=1:NewLines
    if lines(j).theta>0
        left_lane(p_i)=lines(j);
        p_i=p_i+1;
    else
        right_lane(n_i)=lines(j);
        n_i=n_i+1;
    end
end
%%
% Extrapolate lines
[~,NumLL]=size(left_lane);
for LL_i=1:NumLL
LL_point1(LL_i)=left_lane(LL_i).point1(2);
LL_point2(LL_i)=left_lane(LL_i).point2(2);
end
[~,LL_index_low]=max(LL_point1);
[~,LL_index_high]=min(LL_point2);
LL_point_low=left_lane(LL_index_low).point1;
LL_point_high=left_lane(LL_index_high).point2;
LL_point=[LL_point_low;LL_point_high]';
RL_point1_y=LL_point(2,1);
RL_point2_y=LL_point(2,2);

[~,NumRL]=size(right_lane);
RL_theta=[];
for RL_i=1:NumRL
    RL_theta=[RL_theta right_lane(RL_i).theta];
end
RL_theta_sorted=sort(RL_theta,'descend');
midtheta=RL_theta_sorted(ceil(NumRL/2));
[~,RL_index]=find(RL_theta==midtheta);
RL_index=1;
if n_i>1
RL_point1_x=(right_lane(RL_index).rho-RL_point1_y*sind(right_lane(RL_index).theta))/cosd(right_lane(RL_index).theta);
RL_point2_x=(right_lane(RL_index).rho-RL_point2_y*sind(right_lane(RL_index).theta))/cosd(right_lane(RL_index).theta);
RL_point=[RL_point1_x,RL_point2_x;RL_point1_y,RL_point2_y];
end

for k = 1:NewLines
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','black');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','blue');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

  % Determine the endpoints of the longest line segment
%    len = norm(lines(k).point1 - lines(k).point2);
%    if ( len > max_len)
%       max_len = len;
%       xy_long = xy;
%    end
end
% plot(xy_long(:,1),xy_long(:,2),'LineWidth',5,'Color','cyan');
% plot(LL_point(1,:),LL_point(2,:),'LineWidth',5,'Color','red');
% hold on
% if n_i>1
% plot(RL_point(1,:),RL_point(2,:),'LineWidth',5,'Color','red');
% end

%%
OutImage=insertShape(OriginalImage,'Line',[LL_point_low,LL_point_high],'LineWidth',10,'Color','red');
if n_i>1
    OutImage=insertShape(OutImage,'Line',[RL_point1_x,RL_point1_y,RL_point2_x,RL_point2_y],'LineWidth',10,'Color','red');
end
%  filename_lane=strcat('Lane',num2str(frame_num),'.jpg');
%  imwrite(OutImage,filename_lane);
 %%
 low_middle=(LL_point_low(1)+RL_point1_x)/2
 high_middle=(LL_point_high(1)+RL_point2_x)/2
 Text_position=[650 640];
 if (low_middle-high_middle)>30
     text='Left turn ahead';
     FinalFrame=insertText(OutImage,Text_position,text,'FontSize',20,'BoxColor','white','TextColor','black');
 end
 if (low_middle-high_middle)<-30
      text='Right turn ahead';
     FinalFrame=insertText(OutImage,Text_position,text,'FontSize',20,'BoxColor','white','TextColor','black');
 end
if abs(low_middle-high_middle)<30
         text='Stright ahead';
     FinalFrame=insertText(OutImage,Text_position,text,'FontSize',20,'BoxColor','white','TextColor','black');
end
 figure,imshow(FinalFrame)

 