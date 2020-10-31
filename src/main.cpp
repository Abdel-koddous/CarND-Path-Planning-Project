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
          // The new path is to be tangent to the old path
          // the reference point of the new path would be the last point in the previous path
          int previous_path_size = previous_path_x.size();
          std::cout << "Previous path size ==> " << previous_path_size << " Speed ==> " << car_speed << std::endl;
          
          double car_ref_x = car_x;
          double car_ref_y = car_y;
          double car_ref_yaw = deg2rad(car_yaw);

          if( previous_path_size < 2 ) 
          {

            // Estimating where the car was in the previous point ( before 0.02s ) using its speed and angle (NOT VALID IN INTIALLY)
            double car_ref_x_prev =  car_x - cos(car_yaw);
            double car_ref_y_prev =  car_y - sin(car_yaw);         
            
            waypoints_x.push_back( car_ref_x_prev );
            //waypoints_x.push_back( car_x );

            waypoints_y.push_back( car_ref_y_prev );
            //waypoints_y.push_back( car_y );

          }
          else
          {
            /*/ Debug 
            for(int i=0;i<previous_path_size;++i)
            {
              std::cout << "Previous path item i = " << previous_path_x[i] << std::endl;
            } */

            // Using the last couple of points in the previous path
            car_ref_x = previous_path_x[previous_path_size - 1];
            car_ref_y = previous_path_y[previous_path_size - 1];

            double car_ref_x_prev = previous_path_x[previous_path_size - 2];
            double car_ref_y_prev = previous_path_y[previous_path_size - 2];
            car_ref_yaw = atan2( car_ref_y - car_ref_y_prev, car_ref_x - car_ref_x_prev );

            waypoints_x.push_back( car_ref_x_prev );
            //waypoints_x.push_back( car_ref_x );

            waypoints_y.push_back( car_ref_y_prev );
            //waypoints_y.push_back( car_ref_y );

          }

          waypoints_x.push_back( car_ref_x );
          waypoints_y.push_back( car_ref_y );

  

          // Introduce N more widely spaced points in the new path
          int N = 3; // Number of initial widely spaced waypoints
          double wide_dist = 50; // even distance between intial points

          for ( int i = 0; i < N; ++i )
          {
            double next_s = car_s + ( i+1 )*wide_dist;
            vector<double> xy = getXY( next_s, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            waypoints_x.push_back(xy[0]);
            waypoints_y.push_back(xy[1]);
          }
          // By the end of the for loop we have 2 + 3 = 5 points in waypoints_xy

          std::cout << "------------ Waypoints ------------" << std::endl;

          for(int i = 0; i < waypoints_x.size(); ++i )
          {
            std::cout << "x = " << waypoints_x[i] << " y = " << waypoints_y[i] << std::endl;
          }
          std::cout << "-----------------------------------" << std::endl;

          // Converting the global coordinates to car local reference
          for( int i=0; i<waypoints_x.size(); ++i )
          {
            // Shift current coordinates to car_ref
            double shift_x = waypoints_x[i] - car_ref_x;
            double shift_y = waypoints_y[i] - car_ref_y;

            // Computing coordinates in new local reference
            waypoints_x[i] = shift_x * cos(car_ref_yaw) + shift_y * sin(car_ref_yaw);
            waypoints_y[i] = -shift_x * sin(car_ref_yaw) + shift_y * cos(car_ref_yaw);
          }

          std::cout << "------------ Waypoints in local coordinates------------" << std::endl;

          for(int i = 0; i < waypoints_x.size(); ++i )
          {
            std::cout << "x = " << waypoints_x[i] << " y = " << waypoints_y[i] << std::endl;
          }
          std::cout << "-----------------------------------" << std::endl;
          // Create an object from the class spline
          tk::spline s;


          s.set_points( waypoints_x, waypoints_y);


          
          // Start by using the points that were NOT CONSUMED in the previous path
          for (int i  = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          


          // Calculate points between two "widely" spread points of the spline

          double target_x = 50.0;
          double target_y = s(target_x);
          double distance_to_target = sqrt( (target_x*target_x) + (target_y*target_y) );

          double target_speed = 49.5;
          double points_dist = (target_speed/2.237) * 0.02;
          double NbOfPoints_to_target = distance_to_target/points_dist;

          //double x_add_on = 0;
          // Up to now the new path has previous_path_x.size() points in it
          // Let's generate new points to get its size back to the chosen standard value (here = 50)
          for (int i = 1; i <= 50 - previous_path_x.size(); i++)
          {
            
            //The Problem is not here ...
            double x_ref = i * points_dist;
            double y_ref =  s(x_ref);

            // Going back to global reference
            double next_point_x = x_ref*cos(car_ref_yaw) - y_ref*sin(car_ref_yaw);
            double next_point_y =  x_ref*sin(car_ref_yaw) + y_ref*cos(car_ref_yaw);

            next_point_x += car_ref_x;
            next_point_y += car_ref_y;

            next_x_vals.push_back(next_point_x);
            next_y_vals.push_back(next_point_y);

            std::cout << "x = " << next_point_x << "  - y = " << next_point_y << std::endl;
            
           /*
           double N = (distance_to_target/(0.02*target_speed/2.24));
           double x_point = x_add_on + (target_x)/N;
           double y_point = s(x_point);

           x_add_on = x_point;

           double x_ref = x_point;
           double y_ref = y_point;

           x_point = x_ref*cos(car_ref_yaw) - y_ref*sin(car_ref_yaw);
           y_point = x_ref*sin(car_ref_yaw) + y_ref*cos(car_ref_yaw);

           x_point += car_ref_x;
           y_point += car_ref_y;

           next_x_vals.push_back(x_point);
           next_y_vals.push_back(y_point);
           */

          }


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