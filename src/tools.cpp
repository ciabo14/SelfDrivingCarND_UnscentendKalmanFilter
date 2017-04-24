#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse.fill(0.0);

	// check the validity of the inputs 
    if(estimations.size() == 0 || estimations.size() != ground_truth.size())
    {
        cout << "Error with estimation and ground_truth size" << endl;
        return rmse;
    }
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){

		//rmse = rmse.array() + (estimations[i]-ground_truth[i]).array()*(estimations[i]-ground_truth[i]).array();
		VectorXd res = estimations[i] - ground_truth[i];
        res = res.array() * res.array();
        rmse += res;
	}
	//calculate the mean and the squared root
    rmse /= estimations.size();
    rmse = rmse.array().sqrt();

	return rmse;

}

VectorXd Tools::EvaluateNISIndex(const vector<double> lidar_nis_values, const vector<double> radar_nis_values, 
							     const vector<double> nis_values) {
	    //const float radar_95 = 7.815;
        const float radar_95 = 5.991;
        const float lidar_95 = 5.991;
		VectorXd nis_performances(3);
		nis_performances.fill(0.0);
		for(int i = 0; i < lidar_nis_values.size(); i++)
            if(lidar_nis_values[i] > lidar_95)
                nis_performances[0]+=1.0;

		for(int i = 0; i < radar_nis_values.size(); i++)
            if(radar_nis_values[i] > radar_95)
                nis_performances[1]+=1.0;

		for(int i = 0; i < nis_values.size(); i++)
            if(nis_values[i] > lidar_95)
                nis_performances[2]+=1.0;
		nis_performances[0] /=lidar_nis_values.size();
		nis_performances[1] /=radar_nis_values.size();
		nis_performances[2] /=nis_values.size();

		return nis_performances;
}
double Tools::NormalizeAngle(const double currentAngle) {

	double norm_val = currentAngle;

    //angle normalization

	//while (norm_val> M_PI) norm_val-=2.*M_PI;
	//while (norm_val<-M_PI) norm_val+=2.*M_PI;
	
	if (norm_val > M_PI){ 
		norm_val = fmod(norm_val,(2.*M_PI));
		if (norm_val != 0 && (norm_val>M_PI)) norm_val -= (2.*M_PI);
	}
    if (norm_val < -M_PI){
		norm_val = fmod(norm_val,(2.*M_PI));
		if (norm_val != 0 && (norm_val<-M_PI)) norm_val += (2.*M_PI);
			 
	}
	
	return norm_val;
}
