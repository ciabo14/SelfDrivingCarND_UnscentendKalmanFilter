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
	rmse << 0,0,0,0;

	// check the validity of the inputs 
    if(estimations.size() == 0 || estimations.size() != ground_truth.size())
    {
        cout << "Error with estimation and ground_truth size" << endl;
        return rmse;
    }
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){

		rmse = rmse.array() + (estimations[i]-ground_truth[i]).array()*(estimations[i]-ground_truth[i]).array();
		
	}
	//calculate the mean and the squared root
    rmse /= estimations.size();
    rmse = rmse.array().sqrt();

	return rmse;

}
