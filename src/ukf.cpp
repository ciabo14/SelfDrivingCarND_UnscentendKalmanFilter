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

	// initial state vector
	x_ = VectorXd(5);

	// initial covariance matrix
	P_ = MatrixXd(5, 5);

	P_ << 1, 0, 0, 0, 0,
		  0, 1, 0, 0, 0,
		  0, 0, 1, 0, 0,
		  0, 0, 0, 1, 0,
		  0, 0, 0, 0, 1;

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 30;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 30;

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

	/**
	TODO:

	Complete the initialization. See ukf.h for other member properties.

	Hint: one or more values initialized above might be wildly off...
	*/

	///* State dimension
	n_x_ = 5;

	///* Augmented state dimension
	n_aug_ = 7;

	///* Sigma point spreading parameter
	lambda_ = 3-n_x_;

	///* Weights of sigma points for mean/variance prediction
	weights_ = VectorXd(2*n_aug_+1);;

	weights_(0) = lambda_/(lambda_+n_aug_);
	for(int i = 1;i<weights_.size();i++)
		weights_(i) = 0.5/(lambda_+n_aug_);

	///* Augmented Sigma points matrix
	Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

	///* the current NIS for radar
	double NIS_radar_;

	previous_timestamp_ = 0.0;

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

		x_.fill(0.0);

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
                //P_(0,0) = 1000;
            }
            if(fabs(x_(1)) < 0.0001){
                x_(1) = 1;
                //P_(1,1) = 1000;
            }
		}

		is_initialized_ = true;
		return;
	}

	double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
    Prediction(dt);


}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {
	
	ComputeAugmentedSigmaPoints(&Xsig_aug_);
	PredictSigmaPoints(dt);
	PredictMeanAndVariance(&x_,&P_);

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
	/**
	TODO:

	Complete this function! Use lidar data to update the belief about the object's
	position. Modify the state vector, x_, and covariance, P_.

	You'll also need to calculate the lidar NIS.
	*/
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
	/**
	TODO:

	Complete this function! Use radar data to update the belief about the object's
	position. Modify the state vector, x_, and covariance, P_.

	You'll also need to calculate the radar NIS.
	*/
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

void UKF::ComputeAugmentedSigmaPoints(MatrixXd* Xsig_out){

	//create augmented mean vector
	VectorXd x_aug = VectorXd(7);

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(7, 7);

	//create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

	//create augmented mean state
	//create augmented covariance matrix
	//create square root matrix

	x_aug.head(5) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;

	MatrixXd Q = MatrixXd(2,2);
	Q << pow(std_a_,2),0,0,pow(std_yawdd_,2);

	P_aug.topLeftCorner(P_.rows(),P_.cols()) = P_;
	P_aug.bottomRightCorner(Q.rows(),Q.cols()) = Q;
  
	MatrixXd A = P_aug.llt().matrixL();
	MatrixXd tmpMatrix = sqrt(lambda_+n_aug_)*A;
  
	Xsig_aug.col(0) = x_aug;
	for(int i = 0;i<tmpMatrix.cols();i++){
		Xsig_aug.col(i+1) = x_aug + tmpMatrix.col(i);
		Xsig_aug.col(i+1+tmpMatrix.cols()) = x_aug - tmpMatrix.col(i);
	}
	*Xsig_out = Xsig_aug;
}

void UKF::PredictSigmaPoints(double dt){

	double dt_2 = pow(dt,2);
  
	for (int i=0; i<Xsig_aug_.cols();i++){
		double v = Xsig_aug_.col(i)(2);
		double phi = Xsig_aug_.col(i)(3);
		double phi_dot = Xsig_aug_.col(i)(4);
		double nu_a = Xsig_aug_.col(i)(5);
		double nu_phi_2dot = Xsig_aug_.col(i)(6);
      
		VectorXd noise_comp = VectorXd(n_x_);
		VectorXd state_transition_comp = VectorXd(n_x_);
		noise_comp << 0.5*dt_2*cos(phi)*nu_a,
					  0.5*dt_2*sin(phi)*nu_a,
					  dt*nu_a,
					  0.5*dt_2*nu_phi_2dot,
					  dt*nu_phi_2dot;
		if(phi_dot < 0.0001)
			state_transition_comp <<  v*cos(phi)*dt,
									  v*sin(phi)*dt,
									  0,
									  phi_dot*dt,
									  0;
      
		else
			state_transition_comp <<  (v/phi_dot)*(sin(phi+phi_dot*dt)-sin(phi)),
									  (v/phi_dot)*(-cos(phi+phi_dot*dt)+cos(phi)),
									  0,
									  phi_dot*dt,
									  0;   
		Xsig_pred_.col(i) = Xsig_aug_.col(i).head(5)+state_transition_comp + noise_comp;
	}
}

void UKF::PredictMeanAndVariance(VectorXd *x, MatrixXd *P){

	//predict state mean
	//predict state covariance matrix
    VectorXd state = *x;
	state.fill(0.0);
	VectorXd covariance = *P;
	covariance.fill(0.0);

	for(int i = 0;i<Xsig_pred_.cols();i++)
		state = state + weights_(i)*Xsig_pred_.col(i);
    
	//x = Xsig_pred*weights;
    
	MatrixXd tmp = MatrixXd(Xsig_pred_.rows(),Xsig_pred_.cols());
	MatrixXd tmp2 = MatrixXd(Xsig_pred_.rows(),Xsig_pred_.cols());

	for(int i = 0;i<Xsig_pred_.cols();i++){
		tmp.col(i) = Xsig_pred_.col(i) - state;
		tmp2.col(i) = weights_(i)*(Xsig_pred_.col(i) - state);
	}

	*x = state;
	*P = tmp2*tmp.transpose();//*(Xsig_pred - x).transpose();

}