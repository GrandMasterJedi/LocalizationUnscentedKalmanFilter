# Unscented Kalman Filter for Vehicle Localization


[image1]: ./example/dataset1_output.PNG "out1"
[image2]: ./example/LidarNIS.png "lidar"
[image3]: ./example/RadarNIS.png "radar"

The Unscented algorithm (sampling based Kalman Filter) is used to estimate the state of a vehicle assuming a "Constant Turn Rate and Velocity" (CRTV) model from noisy lidar and radar measurement. The state is composed of vehicle position ((x,y) Cartesian coordinates), velocity (assumed constant), yaw and yaw rate (assumed constant). 

The following images show the output of the Kalman Filter prediction (green dots) based on the the simulated dataset of noisy lidar and radar measurement (blue and red dots).

![alt text][image1]


This project is my solution to term 2.2 of the Udacity Self-Driving Car Engineer Nanodegree Program. Check the original project repository for more information. 

Based on the simulated dataset 1 (Udacity Term 2 Simulator), the RMSE for the state vector are below project targets. The NIS (Normalized Innovation Squared) performance measure follows their theoretical value (Chi2 distribution). Below is the NIS for both radar and lidar measurement prediction plotted with their 95% theoretical value (in orange). For only a few prediction points, this measure exceeds this upper limit. 

![alt text][image2]
![alt text][image3]



---
## Resources & Dependencies
* This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)
* This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 
* Udacity Self-Driving Car [Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) 
* Udacity project [master repository] (https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project)
* Project [utilities repository](https://github.com/udacity/CarND-Mercedes-SF-Utilities) 
* [Project rubric](https://review.udacity.com/#!/projects/284/view)

