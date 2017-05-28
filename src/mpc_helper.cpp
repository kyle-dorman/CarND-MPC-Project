#include "mpc_helper.h"
#include <math.h>
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>

using namespace std;
using CppAD::AD;

namespace mpc
{

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Evaluate a polynomial.
AD<double> polyeval(Eigen::VectorXd coeffs, AD<double> x) {
  AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// take the derivative of a simple polynomial
Eigen::VectorXd polyderivative(Eigen::VectorXd coeffs) {
	assert(coeffs.size() > 1);
	Eigen::VectorXd result(coeffs.size() - 1);

	for (size_t i = 1; i < coeffs.size(); i++) {
		result[i - 1] = i * coeffs[i];
	}
	return result;
}

double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph_to_ms(double x) { return x * .447; }

}