#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

	// initialized to false in order to initialize all the variables at the first sample
	is_initialized_ = false;

	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	previous_timestamp_ = 0.0;

	// initial state vector
	x_ = VectorXd(5);
	x_.fill(0.0);
	/*****************************************************************************
     *  Process noise
     ****************************************************************************/

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 2;

    /*****************************************************************************
     *  Laser measurement noise
     ****************************************************************************/

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    /*****************************************************************************
     *  Radar measurement noise
     ****************************************************************************/

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

	/********************************************************************
	* 
	********************************************************************/

	// initial covariance matrix
	P_ = MatrixXd(5, 5);

	P_ << 1, 0, 0, 0, 0,
		  0, 1, 0, 0, 0,
		  0, 0, 1, 0, 0,
		  0, 0, 0, 1, 0,
		  0, 0, 0, 0, 1;

	H_laser_ = MatrixXd(2,5);
	H_laser_ << 1, 0, 0, 0, 0,
				0, 1, 0, 0, 0;

	R_laser_ = MatrixXd(2, 2);
    R_laser_ << pow(std_laspx_,2),	0,
				0,					pow(std_laspy_,2);

    R_radar_ = MatrixXd(3, 3);
    R_radar_ << pow(std_radr_,2),	0,					0,
				0,					pow(std_radphi_,2), 0,
				0,					0,					pow(std_radrd_,2);

	Q_ = MatrixXd(2, 2);
    Q_ << pow(std_a_,2),	0,
		  0,				pow(std_yawdd_,2);
	
	/********************************************************************
	* Sigma points parameters
	********************************************************************/
	///* State dimension
	n_x_ = 5;

	///* Augmented state dimension
	n_aug_ = 7;

	///* Measurement dimension
	n_z_ = 3;

	///* Sigma point spreading parameter
	lambda_ = 3 - n_aug_;

	///* Weights of sigma points for mean/variance prediction
	weights_ = VectorXd(2*n_aug_+1);
	weights_.fill(0.0);

	weights_(0) = lambda_/(lambda_+n_aug_);
	for(int i = 1;i<weights_.size();i++)
		weights_(i) = 0.5/(lambda_+n_aug_);
	
	//weights_.segment(1, 2 * n_aug_).fill(0.5 / (n_aug_ + lambda_));
    //weights_(0) = lambda_ / (lambda_ + n_aug_);

	///* Augmented Sigma points matrix
	Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
	Xsig_aug_.fill(0.0);

	///* Augmented Sigma points matrix
	Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
	Xsig_pred_.fill(0.0);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
	/**
	TODO:

	Complete this function! Make sure you switch between lidar and radar
	measurements.
	*/
	if(!is_initialized_){

		if(meas_package.sensor_type_ == MeasurementPackage::LASER){
			x_(0) = meas_package.raw_measurements_[0];
			x_(1) = meas_package.raw_measurements_[1];
		}
		else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){

			double rho =  meas_package.raw_measurements_[0];
			double phi =  meas_package.raw_measurements_[1];
			x_(0) = rho * cos(phi);
			x_(1) = rho * sin(phi);

			if(fabs(x_(0)) < 0.0001){
                x_(0) = 1;
                P_(0,0) = 1000;
            }
            if(fabs(x_(1)) < 0.0001){
                x_(1) = 1;
                P_(1,1) = 1000;
            }
		}
		previous_timestamp_ = meas_package.timestamp_;

		is_initialized_ = true;
		return;
	}

	double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = meas_package.timestamp_;

    Prediction(dt);

	if(meas_package.sensor_type_ == MeasurementPackage::LASER)
		UpdateLidar(meas_package);
	else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
		UpdateRadar(meas_package);
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {
	
	ComputeAugmentedSigmaPoints();
	PredictSigmaPoints(dt);
	PredictMeanAndVariance();

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

	VectorXd z = meas_package.raw_measurements_;

	MatrixXd Ht = H_laser_.transpose();
	MatrixXd PHt = P_ * Ht; 
	
	VectorXd y = z - H_laser_ * x_;
	MatrixXd S = H_laser_ * PHt + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd K = PHt * Si;
	
	//new estimate
	x_ = x_ + (K * y);

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_laser_) * P_;
	
	//compute NIS coefficient for the current laser measurement
	NIS_laser_ = y.transpose() * Si * y;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement. Used for testing purposes only
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z_,n_z_);
	S.fill(0.0);
	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z_);
	Tc.fill(0.0);
	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z_);
	z_pred.fill(0.0);

	MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
	Zsig.fill(0.0);

	TransformSigmaIntoMeasurement(&Zsig);
	
	//z_pred = Zsig * weights_;
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
	//calculate cross correlation matrix
	//calculate Kalman gain K;
	//update state mean and covariance matrix

    for(int i = 0;i<Zsig.cols();i++){
        VectorXd tmp_z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (tmp_z_diff(1)> M_PI) tmp_z_diff(1)-=2.*M_PI;
        while (tmp_z_diff(1)<-M_PI) tmp_z_diff(1)+=2.*M_PI;
    
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

		S = S + weights_(i) * tmp_z_diff * tmp_z_diff.transpose();
		Tc = Tc + weights_(i) * x_diff * tmp_z_diff.transpose();
    }
	S = S + R_radar_;

    VectorXd z_diff = meas_package.raw_measurements_-z_pred;
	
	//angle normalization
	while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
	MatrixXd K = Tc * S.inverse();

	x_ = x_ + K * z_diff;
    P_ = P_ - (K * S * K.transpose());

	NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}

MatrixXd UKF::ComputeSigmaPoints(){

	// define the sigma points matrix Xsig
	MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

	MatrixXd A = P_.llt().matrixL();
	MatrixXd tmpPMatrix = sqrt(lambda_ + n_x_)*A;
    

	// populate the sigma point matrix as: Xsig = [x_	x_+sqrt((lambda_+n_x_)*P_	x_-sqrt((lambda_+n_x_)*P_]	where:
	// x_ is the current status; lambda is the Design Parameter;n_x_ size of the state vector; P the process covariance matrix
    Xsig.col(0) = x_;
    for(int i = 0;i<tmpPMatrix.cols();i++){
        Xsig.col(i+1) = x_ + tmpPMatrix.col(i);
        Xsig.col(i+1+tmpPMatrix.cols()) = x_ - tmpPMatrix.col(i);
    }

	return Xsig;
}

void UKF::ComputeAugmentedSigmaPoints(){

	//create augmented mean vector
	VectorXd x_aug = VectorXd(7);
	x_aug.fill(0.0);
	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(7, 7);
	P_aug.fill(0.0);

	//create augmented mean state
	//create augmented covariance matrix
	//create square root matrix

	x_aug << x_.array(), 0.0, 0.0;

	P_aug.topLeftCorner(P_.rows(),P_.cols()) = P_;
	P_aug.bottomRightCorner(Q_.rows(),Q_.cols()) = Q_;
	
	MatrixXd A = P_aug.llt().matrixL();
	MatrixXd offset = A * sqrt(lambda_+n_aug_);
  
	Xsig_aug_.col(0) = x_aug;
	for(int i = 0;i<offset.cols();i++){
		Xsig_aug_.col(i+1) = x_aug + offset.col(i);
		Xsig_aug_.col(i+1+offset.cols()) = x_aug - offset.col(i);
	}
}

void UKF::PredictSigmaPoints(double dt){

	double dt_2 = pow(dt,2);

	for (int i=0; i<Xsig_aug_.cols();i++){
		
		double v = Xsig_aug_.col(i)(2);
		double phi = Xsig_aug_.col(i)(3);
		double phi_dot = Xsig_aug_.col(i)(4);
		double nu_a = Xsig_aug_.col(i)(5);
		double nu_phi_dotdot = Xsig_aug_.col(i)(6);
      
		VectorXd noise_comp = VectorXd(n_x_);
		VectorXd state_transition_comp = VectorXd(n_x_);

		noise_comp << 0.5*dt_2*cos(phi)*nu_a,
					  0.5*dt_2*sin(phi)*nu_a,
					  dt*nu_a,
					  0.5*dt_2*nu_phi_dotdot,
					  dt*nu_phi_dotdot;

		if(fabs(phi_dot) < 0.001){
			state_transition_comp(0) = v*cos(phi)*dt;
			state_transition_comp(1) = v*sin(phi)*dt;
		}
		else{
			state_transition_comp(0) = (v/phi_dot)*(sin(phi+phi_dot*dt)-sin(phi));
			state_transition_comp(1) = (v/phi_dot)*(-cos(phi+phi_dot*dt)+cos(phi));
		}
		state_transition_comp(2) = state_transition_comp(4) = 0;
		state_transition_comp(3) = phi_dot*dt;

		Xsig_pred_.col(i) = Xsig_aug_.col(i).head(5)+state_transition_comp + noise_comp;
	}
}

void UKF::PredictMeanAndVariance(){

	//predict state mean
	//predict state covariance matrix
	x_.fill(0.0);
	P_.fill(0.0);

	for(int i = 0;i<Xsig_pred_.cols();i++)
		x_ = x_ + weights_(i)*Xsig_pred_.col(i);
    
	//x = Xsig_pred*weights;
    
	for(int i = 0;i<Xsig_pred_.cols();i++){

	    // state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
		P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;

	}
}

void UKF::TransformSigmaIntoMeasurement(MatrixXd* Zsig_out) {

	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
	Zsig.fill(0.0);

	//transform sigma points into measurement space
    for(int i = 0;i<2 * n_aug_ + 1;i++){
        double px = Xsig_pred_(0,i);
        double py = Xsig_pred_(1,i);
        double v = Xsig_pred_(2,i);
        double xhi = Xsig_pred_(3,i);

        double rho = sqrt(pow(px,2)+ pow(py,2));
        double phi = atan2(py,px);
        double rho_dot = (px*cos(xhi) * v + py*sin(xhi) * v)/rho;
        
		if (rho != rho) {
            rho = 0;
        }
        if (phi != phi) {
            phi = 0;
        }
        if (rho_dot != rho_dot) {
            rho_dot = 0;
        }
		
		Zsig.col(i) << rho, phi, rho_dot;
    }
    
    *Zsig_out = Zsig;
}

//void UKF::ComputeSigmaRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) {
//
//	//mean predicted measurement
//	VectorXd z_pred = VectorXd(n_z_);
//	z_pred.fill(0.0);
//
//	//measurement covariance matrix S
//	MatrixXd S = MatrixXd(n_z_,n_z_);
//	S.fill(0.0);
//
//	//calculate mean predicted measurement
//	//calculate measurement covariance matrix S
//    
//	z_pred = Zsig_ * weights_;
//
//	/*
//	for (int i=0; i < 2*n_aug_+1; i++) 
//      z_pred = z_pred + weights_(i) * Zsig.col(i);
//	*/
//        
//    for(int i = 0;i<Zsig_.cols();i++){
//        //residual
//		VectorXd z_diff = Zsig_.col(i) - z_pred;
//
//		//angle normalization
//		while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
//		while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
//
//		S = S + weights_(i) * z_diff * z_diff.transpose();
//    }
//	
//	*z_out = z_pred;
//	*S_out = S+R_radar_;
//}