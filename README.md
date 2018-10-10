# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program



## Files in the Github src Folder
The files you need to work with are in the src folder of the github repository.

main.cpp - reads in data, calls a function to run the Unscented Kalman filter, calls a function to calculate RMSE
ukf.cpp - initializes the Unscented Kalman filter, calls the predict and update function, defines the predict and update functions
tools.cpp- function to calculate RMSE
The only files you need to modify are ukf.cpp and tools.cpp.

## Data
The data file information is provided by the simulator and is the same data files from EKF. Again each line in the data file represents either a lidar or radar measurement marked by "L" or "R" on the starting line. The next columns are either the two lidar position measurements (x,y) or the three radar position measurements (rho, phi, rho_dot). Then comes the time stamp and finally the ground truth values for x, y, vx, vy, yaw, yawrate.

Although the data set contains values for yaw and yawrate ground truth, there is no need to use these values. main.cpp does not use these values, and you are only expected to calculate RMSE for x, y vx and vy. You can compare your vx and vy RMSE values from the UKF project and the EKF project. For UKF, vx and vy RMSE should be lower than for EKF; this is because we are using a more detailed motion model and UKF is also known for handling non-linear equations better than EKF.




## Tips
### Check out the coding quizzes and coding quiz answers from the lesson
Use the coding quizzes from the lecture to help guide you. You have already implemented the prediction step and radar update step for the unscented Kalman filter. In the project, you will also need to code the update step for lidar.
### Normalize Angles
Don't forget to normalize angles so that angles are between -\pi−π and \piπ. The lectures explained how to do this.
### Don't Forget to Tune Parameters and Initialize Variables
In the starter code, we have given values for the process noise and measurement noise. You will need to tune the process noise parameters std_a_ and std_yawdd_ in order to get your solution working on both datasets. The measurement noise parameters for lidar and radar should be left as given.
You will also need to initialize your state vector x and state covariance matrix P with appropriate values.
If you are having trouble finding appropriate values for your parameters, consider analyzing the data file first. Between time intervals, how much does the object tend to accelerate? What is the maximum acceleration? What is the standard deviation of the acceleration? You can calculate approximate accelerations by dividing changes in velocity by the change in time.
### Check for Divide By Zero
Check for divides by zero.
### Debug
If you implement your solution based on the code taught in the unscented Kalman filter lesson and also find appropriate parameters, you can reach the required RMSE values in the rubric! If you find your code hangs, try adding print statements to figure out why. Your code might be correct but you might need to do more parameter tuning or adjust your initialization values.
###Ideas for Standing out
Use NIS to help tune your parameters
Visualize the ground truth, sensor measurements, and your Kalman filter results
Compare your UKF and EKF project results. Both projects use the same data file. RMSE, especially for v_x and v_y should be lower for the UKF project than the EKF project. Why might that be?



---
## Resources
* Udacity Self-Driving Car [Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) 
* Udacity project [master repository] (https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project)
* Project [utilities repository](https://github.com/udacity/CarND-Mercedes-SF-Utilities) 
* [Project rubric](https://review.udacity.com/#!/projects/284/view)

