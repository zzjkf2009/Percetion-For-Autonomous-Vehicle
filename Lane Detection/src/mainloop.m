function [FinalFrame]=mainloop(OriginalImage,numberofframe)
%file RunMe.m
%Author: Zejiang
%*@Copyright 2017.2-2017.5 University of Maryland, Zejiang Zeng (zzeng@terpmail.umd.edu)

close all
% read the image amd remove the noise
%OriginalImage=imread('frame70.jpg');
%figure, imshow(OriginalImage);
%remove the noise using median filter
GrayImage=rgb2gray(OriginalImage);
NoNoise=medfilt2(GrayImage);

BW_edge_canny=edge(NoNoise,'Canny',0.5);
%figure,imshow(BW_edge_canny)

%Mask out the top half of the image
[width,length,~]=size(OriginalImage);
x=[length/6 length*5/6 length*5/6 length/6 length/6];
y=[round(width*2/3) round(width*2/3) width width round(width*2/3)];
Mask=poly2mask(x,y,width,length);
I_Masked = BW_edge_canny.*Mask;
%figure,imshow(I_Masked)

se=strel('disk',7);
closeBW=imclose(I_Masked,se);
%figure,imshow(closeBW);
%%
% Using Hough transform to find the lines
[H,T,R]=hough(closeBW);
P  = houghpeaks(H,20,'threshold',ceil(0.1*max(H(:))));
lines = houghlines(closeBW,T,R,P,'FillGap',20,'MinLength',20);
%figure,imshow(OriginalImage), hold on
[~,NumLines]=size(lines);

%Fliter the lines into two grows, right and left lane candidate
for i=NumLines:-1:1
    if abs(lines(i).theta)>70
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
if n_i>1
RL_point1_x=(right_lane(1).rho-RL_point1_y*sind(right_lane(1).theta))/cosd(right_lane(1).theta);
RL_point2_x=(right_lane(1).rho-RL_point2_y*sind(right_lane(1).theta))/cosd(right_lane(1).theta);
RL_point=[RL_point1_x,RL_point2_x;RL_point1_y,RL_point2_y];
end
% 
% plot(LL_point(1,:),LL_point(2,:),'LineWidth',5,'Color','red');
% hold on
% if n_i>1
% plot(RL_point(1,:),RL_point(2,:),'LineWidth',5,'Color','red');
% end


OutImage=insertShape(OriginalImage,'Line',[LL_point_low,LL_point_high],'LineWidth',10,'Color','red');
if n_i>1
    OutImage=insertShape(OutImage,'Line',[RL_point1_x,RL_point1_y,RL_point2_x,RL_point2_y],'LineWidth',10,'Color','red');
else
    numberofframe
end
low_middle=(LL_point_low(1)+RL_point1_x)/2;
 high_middle=(LL_point_high(1)+RL_point2_x)/2;
 Text_position=[650 640];
 if (low_middle-high_middle)>20
     text='Left turn ahead';
     FinalFrame=insertText(OutImage,Text_position,text,'FontSize',20,'BoxColor','white','TextColor','black');
 end
 if (low_middle-high_middle)<-20
      text='Right turn ahead';
     FinalFrame=insertText(OutImage,Text_position,text,'FontSize',20,'BoxColor','white','TextColor','black');
 end
if abs(low_middle-high_middle)<20
         text='Stright ahead';
     FinalFrame=insertText(OutImage,Text_position,text,'FontSize',20,'BoxColor','white','TextColor','black');
end

end