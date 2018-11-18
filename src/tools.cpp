#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if ((estimations.size() == ground_truth.size()) && (estimations.size() != 0))
    {
      for (unsigned int i=0; i<estimations.size(); i++)
	{
	const VectorXd residual = estimations[i] - ground_truth[i];

	        // Coefficient-wise multiplication:
	rmse += (residual.array().pow(2)).matrix();
	}

      rmse = rmse/estimations.size();
      rmse = rmse.array().sqrt();
    }
  else
    {
      cout << "error RMSE size mismatch";
    }
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  Hj << 0,0,0,0,
		    0,0,0,0,
		    0,0,0,0;
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = c1*c2;

if ((px != 0) && (py != 0) && (fabs(c1) >= 0.0001))
{
	 //compute the Jacobian matrix
  Hj(0,0) = px/c2;
  Hj(0,1) = py/c2;
  Hj(1,0) = -py/c1;
  Hj(1,1) = px/c1;
  Hj(2,0) = py*(vx*py - vy*px)/c3;
  Hj(2,1) = px*(vy*px - vx*py)/c3;
  Hj(2,2) = px/c2;
  Hj(2,3) = py/c2;
}
else
{
    cout<< "error, Divide by zero";
}

return Hj;

}
