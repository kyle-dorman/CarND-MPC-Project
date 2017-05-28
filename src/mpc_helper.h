#ifndef MPC_HELPER_H
#define MPC_HELPER_H

#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>

namespace mpc_helper
{
const int LATENCY = 100;

const size_t N = 15;
const double dt = 0.05;

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

// Evaluate a polynomial.
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);

// take the derivative of a simple polynomial
Eigen::VectorXd polyderivative(Eigen::VectorXd coeffs);

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);
double mph_to_ms(double x);

}

#endif /* MPC_HELPER_H */
