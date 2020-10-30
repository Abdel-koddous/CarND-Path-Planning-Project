#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          //std::cout << previous_path_x << std::endl;

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // create a list of "widely" spaced (x,y) waypoints, spaced at 50m ?
          // use a spline to interpolate these waypoints with more points to control the speed of the car.

          vector<double> waypoints_x;
          vector<double> waypoints_y;

          // Use the previous path last x=2 points to start the new path
          // The new path is to be tangent to the new path
          // the reference point of the new path would be the last point in the previous path
          int previous_path_size = previous_path_x.size();
          double car_ref_x = previous_path_x[previous_path_size - 1];
          double car_ref_y = previous_path_y[previous_path_size - 1];

          double car_ref_x_prev = previous_path_x[previous_path_size - 2];
          double car_ref_y_prev = previous_path_y[previous_path_size - 2];

          double car_ref_yaw = atan2( car_ref_y - car_ref_y_prev, car_ref_x - car_ref_x_prev );
          
          waypoints_x.push_back( car_ref_x );
          waypoints_y.push_back( car_ref_y );

          waypoints_x.push_back( car_ref_x_prev );
          waypoints_y.push_back( car_ref_y_prev );

          // Introduce N more widely spaced points in the new path
          int N = 3; // Number of initial widely spaced waypoints
          double wide_dist = 50; // even distance between intial points

          for ( int i = 0; i < N; ++i )
          {
            double next_s = car_s + ( i+1 )*wide_dist;
            double next_d = car_d;
            vector<double> xy = getXY( next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            waypoints_x.push_back(xy[0]);
            waypoints_y.push_back(xy[1]);
          }
          // By the end of the for loop we have 2 + 3 points in waypoints_xy


          // Converting to car local reference
          for( int i=0; i<waypoints_x.size(); ++i )
          {
            // Shift current coordinates to car_ref
            double shift_x = waypoints_x[i] - car_ref_x;
            double shift_y = waypoints_y[i] - car_ref_y;

            // Computing coordinates in new local reference
            waypoints_x[i] = shift_x * cos(car_ref_yaw) + shift_y * sin(car_ref_yaw);
            waypoints_y[i] = -shift_x * sin(car_ref_yaw) + shift_y * cos(car_ref_yaw);
          }

          double target_speed = 49.5;
          double points_dist = (target_speed/2.237) * 0.02;

          tk::spline s;

          s.set_points( waypoints_x, waypoints_y);

          std::cout << "x = " << waypoints_x[0] << " y = " << waypoints_y[0] << std::endl;
          
          next_x_vals = waypoints_x;
          next_y_vals = waypoints_y;
          /*
          double dist_inc = 0.3;
          for (int i = 0; i < 50; ++i)
          {
            double next_s = car_s + ( i+1 )*dist_inc;
            double next_d = car_d;
            vector<double> xy = getXY( next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
          */
          // END


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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