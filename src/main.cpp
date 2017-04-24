#include <fstream>
#include <array>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "system_configuration.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * This function will read all the measurement data from the input file and will create two lists: one of measurement and one of GT
  * @params {ifstream} in_file_ the input file ifstream
 * {vector<MeasurementPackage> *} meas_pack_list_out the vector where to place all the measurements to be used
 * {vector<MeasurementPackage>* } gt_pack_list_out the vector where to place all the gt values to be used
 * {SystemConfiguration} conf the SystemConfiguration to select which measurement to use
 */
void readInputFile(ifstream &in_file_, vector<MeasurementPackage> *meas_pack_list_out, vector<GroundTruthPackage> *gt_pack_list_out, SystemConfiguration conf){
	
	vector<MeasurementPackage> measurement_pack_list;
	vector<GroundTruthPackage> gt_pack_list;
	string line;

	// prep the measurement packages (each line represents a measurement at a timestamp)
	while (getline(in_file_, line)) {
		string sensor_type;
		MeasurementPackage meas_package;
		GroundTruthPackage gt_package;
		istringstream iss(line);
		long long timestamp;

		// reads first element from the current line
		iss >> sensor_type;

		if (sensor_type.compare("L") == 0) {
			// laser measurement

			// read measurements at this timestamp
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2);
			float px;
			float py;
			iss >> px;
			iss >> py;
			meas_package.raw_measurements_ << px, py;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			if(conf.laser)
				measurement_pack_list.push_back(meas_package);
		} else if (sensor_type.compare("R") == 0) {
			// radar measurement

			// read measurements at this timestamp
			meas_package.sensor_type_ = MeasurementPackage::RADAR;
			meas_package.raw_measurements_ = VectorXd(3);
			float ro;
			float phi;
			float ro_dot;
			iss >> ro;
			iss >> phi;
			iss >> ro_dot;
			meas_package.raw_measurements_ << ro, phi, ro_dot;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			if(conf.radar)
				measurement_pack_list.push_back(meas_package);
		}

		// read ground truth data to compare later
		float x_gt;
		float y_gt;
		float vx_gt;
		float vy_gt;
		iss >> x_gt;
		iss >> y_gt;
		iss >> vx_gt;
		iss >> vy_gt;
		gt_package.gt_values_ = VectorXd(4);
		gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
		if((sensor_type.compare("R") && conf.radar) || (sensor_type.compare("L") && conf.laser))
			gt_pack_list.push_back(gt_package);
	}
	*meas_pack_list_out = measurement_pack_list;
	*gt_pack_list_out = gt_pack_list;
}

/**
 * This function write in the outputfile the line referred to the current measurement
 * @params {ofstream&} out_file_ the file were to write the predicted state
 * {measurementPackage} meas_package the measurementPackage referred to the current measurement
 * {GroundTruthPackage} gt_package the gt package referred to the current measurement
 * {UKF} ukf the Unscented kalman filter from which select the predicted state
 */
void writeData(ofstream &out_file_, MeasurementPackage meas_package, GroundTruthPackage gt_package, UKF ukf){
	// timestamp
    out_file_  << meas_package.timestamp_ << "\t"; // pos1 - est

    // output the state vector
    out_file_  << ukf.x_(0) << "\t"; // pos1 - est
    out_file_  << ukf.x_(1) << "\t"; // pos2 - est
    out_file_  << ukf.x_(2) << "\t"; // vel_abs -est
    out_file_  << ukf.x_(3) << "\t"; // yaw_angle -est
    out_file_  << ukf.x_(4) << "\t"; // yaw_rate -est

    // output lidar and radar specific data
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // sensor type
      out_file_  << "lidar" << "\t";

      // NIS value
      out_file_  << ukf.NIS_laser_ << "\t";

      // output the lidar sensor measurement px and py
      out_file_  << meas_package.raw_measurements_(0) << "\t";
      out_file_  << meas_package.raw_measurements_(1) << "\t";

    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // sensor type
      out_file_  << "radar" << "\t";

      // NIS value
      out_file_  << ukf.NIS_radar_ << "\t";

      // output radar measurement in cartesian coordinates
      double ro = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      out_file_  << ro * cos(phi) << "\t"; // px measurement
      out_file_  << ro * sin(phi) << "\t"; // py measurement
    }

    // output the ground truth
    out_file_  << gt_package.gt_values_(0) << "\t";
	out_file_  << gt_package.gt_values_(1) << "\t";
    out_file_  << gt_package.gt_values_(2) << "\t";
    out_file_  << gt_package.gt_values_(3) << "\n";
}

/**
 * This function will parse the input and return an error if some input parameter is missing
 * @params {int} argc the input form the command line
 * {char *} argv[] the input form the command line
 */
SystemConfiguration ParseCMDLine(int argc, char *argv[]) {

	SystemConfiguration conf;

	string usage_instructions = "Usage instructions: \n";
	usage_instructions += argv[0];
	usage_instructions += "\n";
	usage_instructions += "path/to/input.txt\n";
	usage_instructions += "path/to/output.txt\n";
	usage_instructions += "path/to/NIS_output.txt\n";
	usage_instructions += "[r|l|b] for [radar only|laser only|both] \n";

	bool has_valid_args = false;

	// make sure the user has provided input and output files
	if (argc < 5) {
	cerr << usage_instructions << endl;
	}else if (argc == 5) {
	has_valid_args = true;
	}else if (argc > 5) {
	cerr << "Too many arguments.\n" << usage_instructions << endl;
	}

	if (!has_valid_args)
	exit(EXIT_FAILURE);
	
	try{
		conf.inputFile= argv[1];
		conf.outputFile = argv[2];
		conf.NISFile = argv[3];
		string sensorData = argv[4];

		if(sensorData.compare("r") == 0 || sensorData.compare("R") == 0 || sensorData.compare("radar") == 0){
			conf.radar = true;
			conf.laser = false;
		}
		else if(sensorData.compare("l") == 0 || sensorData.compare("L") == 0 || sensorData.compare("laser") == 0){
			conf.laser = true;
			conf.radar = false;
		}
		else if(sensorData.compare("b") == 0 || sensorData.compare("B") == 0 || sensorData.compare("both") == 0){
			conf.radar = true;
			conf.laser = true;
		}
		else
			throw "Not valid arg";

	}
	catch(exception &e)
	{
		cerr << "Please use a layout like the one described in the usage" << usage_instructions << endl;
		exit(EXIT_FAILURE);
	}

	return conf;
}

/**
 * This function will check the availability of the input and output files as well as the directories
 * @params {ifstream*} in_file the ifstream of the input file to be checked
 * {string*} in_name the input file name
 * {ofstream*} out_file the ofstream of the output file to be checked
 * {string*} out_name the output file name
 */
void check_files(ifstream& in_file, string& in_name, ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

/**
 * This function execute the UKF over all the measurements with a design for the noise parameters optimization
 * @params {string} input_file the name of the input file
 * {string} output_file the name of the output file
 * {double} std_a the value of the noise parameter std_a to be used
 * {double} std_yawdd the value of the noise parameter std_yawdd to be used 
 * {vector<double> *} lidar_nis_values_out the vector where to save the lidar NIS values 
 * {vector<double> *} radar_nis_values_out the vector where to save the radar NIS values 
 */
void execute_UKF_parametersOptimization(string input_file, string output_file, double std_a, double std_yawdd, 
										vector<double> * lidar_nis_values_out, vector<double> * radar_nis_values_out){

	SystemConfiguration conf;
	
	conf.inputFile = input_file;
	conf.outputFile = output_file;
	
	conf.radar = true;
	conf.laser = true;
	
	ifstream in_file_(conf.inputFile.c_str(), ifstream::in);

	ofstream out_file_(conf.outputFile.c_str(), ofstream::app);

	check_files(in_file_, conf.inputFile, out_file_, conf.outputFile);

	/**********************************************
	*  Read input file and set measurement       *
	**********************************************/

	vector<MeasurementPackage> measurement_pack_list;
	vector<GroundTruthPackage> gt_pack_list;

	vector<double> lidar_nis_values;
	vector<double> radar_nis_values;

	readInputFile(in_file_, &measurement_pack_list, &gt_pack_list, conf);
  
	// Create a UKF instance
	UKF ukf;
	ukf.Q_ << pow(std_a,2),	0,
			0,				pow(std_yawdd,2);


	// used to compute the RMSE later
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	// start filtering from the second frame (the speed is unknown in the first frame)

	size_t number_of_measurements = measurement_pack_list.size();

	// column names for output file
	out_file_ << "time_stamp" << "\t";  
	out_file_ << "px_state" << "\t";
	out_file_ << "py_state" << "\t";
	out_file_ << "v_state" << "\t";
	out_file_ << "yaw_angle_state" << "\t";
	out_file_ << "yaw_rate_state" << "\t";
	out_file_ << "sensor_type" << "\t";
	out_file_ << "NIS" << "\t";  
	out_file_ << "px_measured" << "\t";
	out_file_ << "py_measured" << "\t";
	out_file_ << "px_ground_truth" << "\t";
	out_file_ << "py_ground_truth" << "\t";
	out_file_ << "vx_ground_truth" << "\t";
	out_file_ << "vy_ground_truth" << "\n";


	for (size_t k = 0; k < number_of_measurements; ++k) {
		// Call the UKF-based fusion

		ukf.ProcessMeasurement(measurement_pack_list[k]);

		//writeData(out_file_,measurement_pack_list[k],gt_pack_list[k],ukf);

		// convert ukf x vector to cartesian to compare to ground truth
		VectorXd ukf_x_cartesian_ = VectorXd(4);
	
		double x_estimate_ = ukf.x_(0);
		double y_estimate_ = ukf.x_(1);
		double vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
		double vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));
    
		ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
    
		estimations.push_back(ukf_x_cartesian_);

		ground_truth.push_back(gt_pack_list[k].gt_values_);

		if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) 
			lidar_nis_values.push_back(ukf.NIS_laser_);
		else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR)
			radar_nis_values.push_back(ukf.NIS_radar_);

	}

	// compute the accuracy (RMSE)
	Tools tools;
	cout << "RMSE" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

	// close files
	if (out_file_.is_open()) {
		out_file_.close();
	}

	if (in_file_.is_open()) {
		in_file_.close();
	}
	*lidar_nis_values_out = lidar_nis_values;
	*radar_nis_values_out = radar_nis_values;

	cout << "Done!" << endl;
}

/**
 * This function write the NIS values for the lidar, for the radar and for the combination in the dedicated filename
 * @params {string} filename the name of the file where to write the NIS coefficients
 * {vector<double>} lidar_nis_values the NIS coefficients for the lidar prediction 
 * {vector<double>} radar_nis_values the NIS coefficients for the radar prediction
 * {vector<double>} nis_values the NIS coefficients for the entire system.
 */
void write_NIS(string filename, vector<double> lidar_nis_values, vector<double> radar_nis_values, vector<double> nis_values){

	ofstream NIS_out_file(filename, ofstream::app);

	for(int t = 0;t< lidar_nis_values.size() ;t++){
		NIS_out_file << lidar_nis_values[t];
		if(t<lidar_nis_values.size()-1)
			NIS_out_file << " - ";
	}
	NIS_out_file << endl;
	for(int t = 0;t< radar_nis_values.size();t++){
		NIS_out_file << radar_nis_values[t];
		if(t<radar_nis_values.size()-1)
			NIS_out_file << " - ";
	}
	NIS_out_file << endl;
	for(int t = 0;t< nis_values.size();t++){
		NIS_out_file << nis_values[t];
		if(t<nis_values.size()-1)
			NIS_out_file << " - ";
	}
	
	NIS_out_file.close();
}

/**
 * This function will goes throught the set of measurement read from the input file and compute the prediction state 
 * in the KF double step (prediction + update) for each measurement.
 * @params {int} argc the first input argument from the command line
 * {char*} argv[] the second input argument from the command line
 */
void execute_UKF(int argc, char* argv[]){

	SystemConfiguration conf;
	
	vector<double> nis_values;
	vector<double> lidar_nis_values;
	vector<double> radar_nis_values;

	#ifndef DEBUG
	conf = ParseCMDLine(argc, argv);
	/*check_arguments(argc, argv);
	in_file_name_ = argv[1];
	out_file_name_ = argv[2];*/

	#endif

	#ifdef DEBUG
	
	//conf.inputFile = "E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/obj_pose-laser-radar-synthetic-input.txt";
	//conf.outputFile = "E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/obj_pose-laser-radar-synthetic-output.txt";
	//conf.NISFile = "E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/obj_pose-laser-radar-synthetic-NIS.txt";
	//conf.inputFile = "E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/sample-laser-radar-measurement-data-1.txt";
	//conf.outputFile = "E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/sample-laser-radar-measurement-data-1-output.txt";
	//conf.NISFile = "E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/sample-laser-radar-measurement-data-1-NIS.txt";

	conf.inputFile = "E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/sample-laser-radar-measurement-data-2.txt";
	conf.outputFile = "E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/sample-laser-radar-measurement-data-2-output.txt";
	conf.NISFile = "E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/sample-laser-radar-measurement-data-2-NIS.txt";

	conf.radar = true;
	conf.laser = true;
										
	#endif
	ifstream in_file_(conf.inputFile.c_str(), ifstream::in);

	ofstream out_file_(conf.outputFile.c_str(), ofstream::app);

	check_files(in_file_, conf.inputFile, out_file_, conf.outputFile);

	vector<MeasurementPackage> measurement_pack_list;
	vector<GroundTruthPackage> gt_pack_list;

	readInputFile(in_file_, &measurement_pack_list, &gt_pack_list, conf);
  
	// Create a UKF instance
	UKF ukf;

	// used to compute the RMSE later
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	size_t number_of_measurements = measurement_pack_list.size();

	// column names for output file
	out_file_ << "time_stamp" << "\t";  
	out_file_ << "px_state" << "\t";
	out_file_ << "py_state" << "\t";
	out_file_ << "v_state" << "\t";
	out_file_ << "yaw_angle_state" << "\t";
	out_file_ << "yaw_rate_state" << "\t";
	out_file_ << "sensor_type" << "\t";
	out_file_ << "NIS" << "\t";  
	out_file_ << "px_measured" << "\t";
	out_file_ << "py_measured" << "\t";
	out_file_ << "px_ground_truth" << "\t";
	out_file_ << "py_ground_truth" << "\t";
	out_file_ << "vx_ground_truth" << "\t";
	out_file_ << "vy_ground_truth" << "\n";


	for (size_t k = 0; k < number_of_measurements; ++k) {
		// Call the UKF-based fusion

		ukf.ProcessMeasurement(measurement_pack_list[k]);
		writeData(out_file_,measurement_pack_list[k],gt_pack_list[k],ukf);
		// convert ukf x vector to cartesian to compare to ground truth
		VectorXd ukf_x_cartesian_ = VectorXd(4);
	
		double x_estimate_ = ukf.x_(0);
		double y_estimate_ = ukf.x_(1);
		double vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
		double vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));
    
		ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
    
		estimations.push_back(ukf_x_cartesian_);

		ground_truth.push_back(gt_pack_list[k].gt_values_);

		if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER){
			nis_values.push_back(ukf.NIS_laser_);
			lidar_nis_values.push_back(ukf.NIS_laser_);
		}
		else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR){
			nis_values.push_back(ukf.NIS_radar_);
			radar_nis_values.push_back(ukf.NIS_radar_);
		}

	}

	// compute the accuracy (RMSE)
	Tools tools;
	cout << "RMSE" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

	// close files
	if (out_file_.is_open()) {
		out_file_.close();
	}

	if (in_file_.is_open()) {
		in_file_.close();
	}
	write_NIS(conf.NISFile, lidar_nis_values, radar_nis_values, nis_values);
	cout << "Done!" << endl;
}

int main(int argc, char* argv[]) {
	
	/********************************************************************
	* Used during noise optimization only
	********************************************************************/
	//array<string,1> input_files = {
	//	"E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/sample-laser-radar-measurement-data-2.txt"//,
	//	//"E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/sample-laser-radar-measurement-data-1.txt",
	//	//"E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/obj_pose-laser-radar-synthetic-input.txt"							   };
	//};
	//array<string,3> output_files = {"E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/sample-laser-radar-measurement-data-2-output.txt",
	//								"E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/sample-laser-radar-measurement-data-1-output.txt",
	//								"E:/Self Driving Car Nanodegree/Term 2/UKF/SelfDrivingCarND_UnscentendKalmanFilter/data/obj_pose-laser-radar-synthetic-output.txt"};
	//	
	//vector<double> std_a_s; 
	//vector<double> std_yawdd_s;
	//float i = 0.8;
	//while(i>=0.2){
	//	std_a_s.push_back(i);
	//	i-=0.05;
	//}
	//float j = 1.2;
	//while(j>=0.2){
	//	std_yawdd_s.push_back(j);
	//	j-=0.05;
	//}
	//for(int i = 0;i<(input_files.size());i++)
	//	for(int j = 0;j<std_a_s.size();j++)
	//		for(int k = 0;k<std_yawdd_s.size();k++){
	//			cout << "=================" << input_files[i].substr(input_files[i].find_last_of("/")) << "=================";
	//			cout << "std_a_s=" << std_a_s[j] << " - std_yawdd=" << std_yawdd_s[k] << endl; 
	//			execute_UKF_parametersOptimization(input_files[i],output_files[i],std_a_s[j],std_yawdd_s[k],&nis_values);
	//			NIS_out_file <<"std_a_s=" << std_a_s[j] << " - std_yawdd=" << std_yawdd_s[k] << endl;
	//			for(int t = 0;t< nis_values.size() ;t++)
	//				NIS_out_file << nis_values[t] << " - ";
	//			NIS_out_file << endl;
	//		}
	//if (NIS_out_file.is_open()) {
	//	NIS_out_file.close();
	//}

	execute_UKF(argc, argv);

	return 0;
}