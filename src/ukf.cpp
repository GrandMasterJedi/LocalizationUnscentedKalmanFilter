#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector. 5 states: px, py, v, yaw, yawd
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // Note that results will be sensitive to this value
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  

  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  ///* Weights of sigma points for mean and covariance prediction
  weights_ = VectorXd(2*n_aug_+1);

  ///* Sigma point spreading parameter set to reccommended value 
  lambda_ = 3 - n_aug_;

  //set initial values for state vector (realistically as in lecture example)
  // x_ <<   5.7441,  1.3800,   2.2049,   0.5015,   0.3528;

  //// set initial value for covariance matrix
  //P_ <<   0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
  //        -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
  //        0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
  //        -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
  //        -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  ///* time when the state is true, in us
  // time_us_ = 0.0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage mp) {
  // TODO: Complete this function! Make sure you switch between lidar and radar measurements.

  if (!is_initialized_) {

    if (mp.sensor_type_ == MeasurementPackage::RADAR) 
    {

      //set initial values for state vector (realistically as in lecture example)
      x_ <<   .0,   .0,   .0,   .0,   .0;
//       x_ <<   5.7441,  1.3800,   2.2049,   0.5015,   0.3528;

      // set initial value for covariance matrix. Initial step res
      P_ <<  0.1,  .0,  .0,  .0, .0,
              .0, 0.1,  .0,  .0, .0,
              .0,  .0, 1.0,  .0, .0,
              .0,  .0,  .0, 1.0, .0,
              .0,  .0,  .0,  .0, 1.0;
//       P_ <<   0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
//              -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
//              0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
//              -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
//              -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

      // Convert radar from polar to cartesian coordinates and initialize state.
      double rho = mp.raw_measurements_[0]; // Range - radial distance from origin in polar coordinate
      double phi = mp.raw_measurements_[1]; // Bearing - angle between rho and axis

      double x =  rho * cos(phi);
      double y =  rho * sin(phi); 
    
      // // Constant velocity
      // double rho_dot = mp.raw_measurements_[2]; // Radial Velocity - rho rate
      // double vx = rho_dot * cos(phi);
      // double vy = rho_dot * sin(phi);
      // initial value set to the first measurement
      x_(0) = x;
      x_(1) = y; 

    }
    else if (mp.sensor_type_ == MeasurementPackage::LASER) 
    {
      x_(0) = mp.raw_measurements_[0];
      x_(1) = mp.raw_measurements_[1];
    }

    time_us_ = mp.timestamp_;

    is_initialized_ = true;

    // done initializing, no need to predict or update
    return;

  }
  
  // compute time elapsed between measurement
  double dt = (mp.timestamp_ - time_us_)/1e6; // expressed in seconds
  time_us_ = mp.timestamp_; 

  // predict and update
  Prediction(dt);

  if (mp.sensor_type_ == MeasurementPackage::RADAR) UpdateRadar(mp);
  if (mp.sensor_type_ == MeasurementPackage::LASER) UpdateLidar(mp);

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {
  /** Estimate the object's location. Modify the state vector, x_. 
  Predict sigma points, the state, and the state covariance matrix.
  */


  // Augmented state vector, with two noise components (velocity effect)
  VectorXd X_aug = VectorXd(n_aug_);
  X_aug.head(5) = x_;
  X_aug(5) = 0;
  X_aug(6) = 0;
  // for (int j = n_aug_-1; j>4; j--) {
  //   //Assign mean innvation compoent to zero
  //   x_aug_.coeff(j) = 0;
  // }
  
  // Covariance Matrix of sigma points
  // Note that P_ is set during initialization of measurement step  
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // Matrix of sigma points - Augmented with innovation
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  Xsig_aug.col(0)  = X_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = X_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = X_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  //########################################################################
  // Predict sigma points - Implementation as during lecture notes
  // avoid division by zero, write predicted sigma points into right column
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero (absolute value of yawd)
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
    }
    else {
      // when yawd is zero, it is a straight line, hypotenuse
        px_p = p_x + v*dt*cos(yaw);
        py_p = p_y + v*dt*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*dt;
    double yawd_p = yawd;

    //add noise. Effect of process noise on each state. It depends on delta t
    px_p = px_p + 0.5*nu_a*dt*dt * cos(yaw);
    py_p = py_p + 0.5*nu_a*dt*dt * sin(yaw);
    v_p = v_p + nu_a*dt;

    yaw_p = yaw_p + 0.5*nu_yawdd*dt*dt;
    yawd_p = yawd_p + nu_yawdd*dt;

    //write predicted sigma point into right column
    // predicted values for the 5 states (the remaining two are process noise, not states)
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //########################################################################
  // Predict Mean Covariance from sigma points (Implementation as in lecture notes)

  // set weights
  // weights depends on lambda as the sigma points are derived proportionally to lambda
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  // predicted state mean. 
  // It is weighted average of the predicted sigma points over the 2*n+1 columns
  x_.fill(0.0); 
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  // it is also weighted covariance formula over sigma points
  P_.fill(0.0); 
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;  // (5,1)
    
    // note: the following normalization is needed because the difference of 
    // two angle may be 2pi +- r. We are interested only in the small angle r
    // the normalization is applied to the 4th state that is the yaw
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;  //(5,1) x (1,5)
  }

}


void UKF::UpdateLidar(MeasurementPackage mp) {
  /** Use lidar data to update the belief about the object's position. 
  Modify the state vector, x_, and covariance, P_. Calculate the lidar NIS.
  */

  // Collect new measurement. Note that if LASER
  // meas_package.raw_measurements_ = VectorXd(2); see main.cpp line 71
  VectorXd z = mp.raw_measurements_;  // (2,1)

  int nz = 2; // lidar measures px and py coordinates
  MatrixXd Zsig = MatrixXd(nz, 2*n_aug_+1); // (2,15)

  // Note that lidar measurement is in the same space as the prediction for state x
  // however with only two states instead of 5
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

    Zsig(0,i) = Xsig_pred_(0,i); //p_x       
    Zsig(1,i) = Xsig_pred_(1,i); //p_y;   
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(nz); // (2,1)
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);  // (2,1)
  }
  // z_pred(0) = x_(0);
  // z_pred(1) = x_(1);
  

  // measurement error covariance matrix S. Here it is (2,2) and not (5,5) as P
  MatrixXd S = MatrixXd(nz, nz);  // (2,2)
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {     
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;    // (2,1)
    S = S + weights_(i) * z_diff  * z_diff.transpose(); // (2,1) x (1,2) 
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(nz,nz);
  R <<    std_laspx_*std_laspx_,  0,
          0,                      std_laspy_*std_laspy_;

  S = S + R;

  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, nz); // (5,2)
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;  // (2,1)
    VectorXd x_diff = Xsig_pred_.col(i) - x_;  // (5,1)
   
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    //   (5,2) =     (5,1) * (1,2)                 
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();   // (5,2) * (2,2)

  //residual
  VectorXd z_diff = z - z_pred;   // (2,1)

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;  // (5,2) * (2,1)
  P_ = P_ - K*S*K.transpose(); 
  //  (5,5) =     (5,2) (2,2) (2,5)
   
  // Calculate NIS and assign
  NISlidar_ = z_diff.transpose() * S.inverse() * z_diff;
  // (1,2) (2,2)(2,1)

  cout << "Lidar NIS: " << NISlidar_ << endl;

}


void UKF::UpdateRadar(MeasurementPackage mp) {
  /** Use radar data to update the belief about the object's position. 
  Modify the state vector, x_, and covariance, P_. Calculate the radar NIS.
  */

  // Collect new measurement
  VectorXd z = mp.raw_measurements_;  // (3,1)

  int nz = 3; // radar measures radius (r), angle (phi) and change in r (r_dot)
  MatrixXd Zsig = MatrixXd(nz, 2*n_aug_+1); // (3,15)

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model. convert to radial (polar) coordinate as in radar measurement
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(nz);       // (3,1)
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(nz,nz);   // (3,3)
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred; 

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose(); // (3,1) * (1,3)
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(nz,nz);
  R <<    std_radr_*std_radr_,  0,                        0,
          0,                    std_radphi_*std_radphi_,  0,
          0,                    0,                        std_radrd_*std_radrd_;
  S = S + R; // (3,3)

  //cross correlation matrix Tc between measurement and prediction error
  MatrixXd Tc = MatrixXd(n_x_, nz);  // (5,3)
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //loop over 2n+1 sigma points
    //residual, recalculation of above
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization. Only take ramaining part of difference between two angles
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;   // (5,1)

    //angle normalization, in the state vector, the 4th state is the yaw 
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    //  (5,3)  = (5,3) + c * (5,1) * (1,3)
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();   // (5,3) * (3,3)

  //residual
  VectorXd z_diff = z - z_pred;  // (3,1)

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff; // (5,3)*(3,1)
  P_ = P_ - K*S*K.transpose();
  // (5,3) (3,3) (3,5)

  // Calculate NIS and assign
  NISradar_ = z_diff.transpose() * S.inverse() * z_diff; // (1,3)*(3,3)*(3,1)
  cout << "Radar NIS: " << NISradar_ << endl;

}
