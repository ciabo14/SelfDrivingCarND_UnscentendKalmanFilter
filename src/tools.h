#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);
  Eigen::VectorXd EvaluateNISIndex(const std::vector<double> lidar_nis_values, const std::vector<double> radar_nis_values, const std::vector<double> nis_values);
  static double NormalizeAngle(const double currentAngle);

};

#endif /* TOOLS_H_ */
