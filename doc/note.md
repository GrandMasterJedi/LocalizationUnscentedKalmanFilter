









# Notes


## Constant turn rate and velocity magnitude (CTRV) model

* Integral of the change in velocity from time k to k+1? 0
* integral of the yaw rate from time k to k+1? yawRate * dt


Only one object to track. 



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







PROJECT DESCRIPTION
The project "unscented Kalman filter" is based on the same structure as the extended Kalman filter.
It uses a main file that calls a function called ProcessMeasurement. Anything important happens in this function. The function is part of the class ukf.


C++ QUIZZES
The quizzes including the solutions of them are included in the file ukf.cpp. They are individual functions, which don't need any special environment. The solution of the quizzes are given here and also the expected results.
The quizzes can easily evaluated: if every value of the student solution (vectors and matrices) differs less than 0.001 from the original solution, the quizz is passed, otherwise failed.



PROJECT PASSING CRITERIA
There are several criteria that must be fulfilled to pass the project.

- The overall processing chain (prediction, laser update or radar update depending on measurement type) must be correct.
- The student is not allowed to use values from the future to reason about the current state.
- It must be possible to run the project in three different modes: considering laser only, with considering radar only, or with using both sensors.
- For every mode, the overall RMSE (2d position only) may not be more than 10% increased to what the original solution is able to reach (this number depends on the individual measurement sequence)
- The RMSE of laser AND radar must be lower than radar only or laser only
- The NIS of radar measurements must be between 0.35 and 7.81 in at least 80% of all radar update steps.


PROJECT GRADING
- I recommend a hall of fame for the lowest overall RMSE using laser AND radar.
- I recommend to ask students to improve the initialization procedure and evaluate the RMSE during the first 20 steps.








