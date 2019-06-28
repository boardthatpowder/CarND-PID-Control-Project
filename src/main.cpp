#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include "json.hpp"
#include "PID.h"

#define SAVE_DATA ;
#define APPLY_THROTTLE_CONTROL ;

// for convenience
using nlohmann::json;
using std::string;
using std::cout;
using std::cerr;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[]) {

  uWS::Hub h;

  PID pid_steering, pid_throttle;
  /**
   * TODO: Initialize the pid variable.
   */
  double init_Kp_steering = atof(argv[1]);  // steering
  double init_Ki_steering = atof(argv[2]);  // wheels alignment
  double init_Kd_steering = atof(argv[3]);  // trying not to oscillate around the line
  pid_steering.Init(init_Kp_steering, init_Ki_steering, init_Kd_steering);

  double init_Kp_throttle = atof(argv[4]);  // throttle 
  double init_Ki_throttle = atof(argv[5]);  // interval
  double init_Kd_throttle = atof(argv[6]);  // trying not to oscillate around the line
  pid_throttle.Init(init_Kp_throttle, init_Ki_throttle, init_Kd_throttle);


  #ifdef SAVE_DATA
    std::stringstream ss;
    string saveDir = "/Users/deanhart/Documents/udacity/self-driving-car-engineer-nanodegree/term2/github/CarND-PID-Control-Project/charts/";

  #ifdef APPLY_THROTTLE_CONTROL
    ss << saveDir << init_Kp_steering << "_" << init_Ki_steering << "_" << init_Kd_steering << "_" << init_Kp_throttle << "_" << init_Ki_throttle << "_" << init_Kd_throttle << ".csv";
  #else
    ss << saveDir << init_Kp_steering << "_" << init_Ki_steering << "_" << init_Kd_steering << ".csv";
  #endif

    string filename = ss.str();
    cout << "Saving data to " << filename << "\n";

    std::ofstream results (filename);

    results.open(filename, std::ios_base::app);
    results << "ts," << "cte," << "angle," << "steer_value," << "speed," << "throttle" << std::endl;
    results.close();
  #endif

  int ts = 0;

  h.onMessage([&pid_steering, &pid_throttle, &ts, &results, &filename](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          pid_steering.UpdateError(cte);
          steer_value = pid_steering.TotalError();
          if (steer_value<-1.0) {
            steer_value = -1.0;
          } else if (steer_value>1.0) {
            steer_value = 1.0;
          }

          double brake = 0;
          double throttle;

#ifdef APPLY_THROTTLE_CONTROL
          pid_throttle.UpdateError(steer_value);
          throttle = 1.0 - pid_throttle.TotalError();
          if (throttle<0) {
            brake = fabs(throttle);
            throttle = 0;
          } else if (throttle < 0.3) {
            throttle=0.3;
          } else if (throttle>1.0) {
            throttle = 1.0;
          }
#else
          throttle = 0.3;
#endif

#ifdef SAVE_DATA
          results.open(filename, std::ios_base::app);
          results << ts << "," << cte << "," << angle << "," << steer_value << "," << speed << "," << throttle << std::endl;
          results.close();
          ts++;

          if (cte<-10.0 || cte>10.0) {
            exit(0);
          }

          if (ts==4000) {
            exit(0);
          }
#endif

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          msgJson["brake"] = brake;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          cout << ts << " CTE:" << cte << " " << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    cout << "Listening to port " << port << std::endl;
  } else {
    cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}