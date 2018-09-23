# Percetion-For-Autonomous-Vehicle

## Overview
Those are the percetion projects for autonomous vehicle , including Lane detection, Traffic Sign Detection, Car tracking and Vision Odometry.


## Lane Detection
### Assumuptions
- The vechicle keeps driving in a same lane
- The lanes are visiable
- The front vehicle are far enough

### Pipeline
- Gray Scale Transformation
- Gaussian Smoothing
- Canny Edge Detection
- ROI (Region of Interest) Based Edge Filtering
- Hough Transformation
- Lane Extrapolation
- Curvature Prediction

### Result
More details can be found in this report: [Lane detecion report](https://github.com/zzjkf2009/Percetion-For-Autonomous-Vehicle/blob/master/Lane%20Detection/LaneDetection.pdf). There is the video result for this traditional method.
- [![Lane Detection](http://img.youtube.com/vi/KkB9RftPOQQ/0.jpg)](http://www.youtube.com/watch?v=KkB9RftPOQQ)
### State of art approach for Lane Detection
- Spatial As Deep: Spatial CNN for Traffic Scene Understanding:
[Spatial CNN](https://arxiv.org/abs/1712.06080)
[Blog](https://www.cnblogs.com/guoyaohua/p/8940871.html)
- Dual-View CNN:
[DVCNN](https://ieeexplore.ieee.org/document/7535517/)
- Towards End-to-End Lane Detection: an Instance Segmentation Approach:
[Segmentation Approach](https://arxiv.org/abs/1802.05591)

## Car Tracking

### Pipeline
- Haar Cascade classcifation: car detection
- Closest Bounding Box: multi-object tracking
- or KLT tracker

### Result
More details can be found in this report:[Car tracking](https://github.com/zzjkf2009/Percetion-For-Autonomous-Vehicle/blob/master/Car%20Tracking/Car%20tracking.pdf)
Video result:
- [![Lane Detection](http://img.youtube.com/vi/2HRDgvAQjFI/0.jpg)](http://www.youtube.com/watch?v=2HRDgvAQjFI)
