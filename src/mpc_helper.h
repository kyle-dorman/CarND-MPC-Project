#ifndef MPC_HELPER_H
#define MPC_HELPER_H

#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>

namespace mpc
{
const int LATENCY = 100;

const size_t N = 25;
const double dt = 0.05;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

const int LAG = (int)((double)LATENCY / 1000.0 / dt + 0.5);

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

// Evaluate a polynomial.
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);

Eigen::VectorXd polyfit(std::vector<double> xvals, std::vector<double> yvals,
                        int order);

// take the derivative of a simple polynomial
Eigen::VectorXd polyderivative(Eigen::VectorXd coeffs);

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);
double mph_to_ms(double x);

// helper to convert vector to eigen vector
Eigen::VectorXd toVectorXd(std::vector<double> &x);

void updateState(Eigen::VectorXd &state, double steering_angle, double throttle, double dt);

}

#endif /* MPC_HELPER_H */
