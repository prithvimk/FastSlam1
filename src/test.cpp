#include <eigen3/Eigen/Dense>
#include <iostream>

Eigen::MatrixXd motion_model(Eigen::MatrixXd x, Eigen::MatrixXd u) {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(3, 3);
  Eigen::MatrixXd B(3, 2);
  B <<  2 * cos(x(2, 0)), 0,
        2 * sin(x(2, 0)), 0,
        0.0, 2;
  
  x = F * x + B * u;

  //x(2, 0) = pi_2_pi(x(2, 0));

  return x;
}

int main() {
  Eigen::MatrixXd zp(2, 1);
    zp << 8, 
          2;
  std::cout << zp;
  return 0;
}