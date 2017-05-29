#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "mpc_helper.h"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using Eigen::VectorXd;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc_solver;

  h.onMessage([&mpc_solver](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double psi_unity = j[1]["psi_unity"];
          double v = mpc::mph_to_ms(j[1]["speed"]);
          double current_steering_angle = j[1]["steering_angle"];
          double current_throttle = j[1]["throttle"];

          double sin_psi = sin(psi);
          double cos_psi = cos(psi);

          // pts converted to vehicle coordinate space
          vector<double> next_x_vals(ptsx.size());
          vector<double> next_y_vals(ptsx.size());

          // translate and then rotate points to vehicle space
          for (size_t i=0; i<ptsx.size(); i++) {
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;

            next_x_vals[i] = (cos_psi * x) + (sin_psi * y);
            next_y_vals[i] = (-1.0 * sin_psi * x) + (cos_psi * y);
          }

          // fit a 3rd order polynomial to the points
          VectorXd coeffs = mpc::polyfit(next_x_vals, next_y_vals, 3);

          // project car coordinate system x, y, psi, and v into the future to account for LATENCY(100 ms)
          VectorXd current_state(4);
          current_state << 0, 0, 0, v;
          mpc::updateState(current_state, current_steering_angle, current_throttle, mpc::LATENCY/1000);
          double current_x = current_state[0];
          double current_y = current_state[1];
          double current_psi = current_state[2];
          double current_v = current_state[3];

          // calculate crosstrack error based on fit line
          double cte = mpc::polyeval(coeffs, current_x) - current_y;

          // calculate derivative of fit line
          VectorXd d_coeffs = mpc::polyderivative(coeffs);

          // calculate the desired orientation
          double d_psi = atan(mpc::polyeval(d_coeffs, current_x));
          double epsi = current_psi - d_psi;

          VectorXd state(8);
          state << current_x, current_y, current_psi, current_v, cte, epsi, current_steering_angle, current_throttle;

          /*
          * Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          vector<double> result = mpc_solver.Solve(state, coeffs);
          double steering_angle = -1.0 * result[0] / mpc::deg2rad(25.0);
          double throttle_value = result[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steering_angle;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals(mpc::N);
          vector<double> mpc_y_vals(mpc::N);
          for (size_t i = 0; i < mpc::N; i++) {
            mpc_x_vals[i] = result[2 + i];
            mpc_y_vals[i] = result[2 + i + mpc::N];
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(mpc::LATENCY));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
