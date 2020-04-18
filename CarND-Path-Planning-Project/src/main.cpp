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
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
          static double ctrl_speed = 0.0; // mph
          static double car_local_yaw = 0;
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // Find the car lanes from 'd'
          int car_lane = 0;
          if (car_d > 0 && car_d < 4)
          {
            car_lane = 0;
          }
          else if (car_d >= 4 && car_d < 8)
          {
            car_lane = 1;
          }
          else if (car_d >= 8 && car_d < 12)
          {
            car_lane = 2;
          }
          int prev_path_size = previous_path_x.size();
          // Find obstacles in each lane
          double obstacles_distances_lanes[3] = {100, 100, 100};
          double target_speed = 0;
          int lane = -1;
          {
            for (int i = 0; i < sensor_fusion.size(); ++i)
            {
              // find lane of car
              float d = sensor_fusion[i][6];

              if (d > 0 && d < 4)
              {
                lane = 0;
              }
              else if (d >= 4 && d < 8)
              {
                lane = 1;
              }
              else if (d >= 8 && d < 12)
              {
                lane = 2;
              }
              else
              {
                continue;
              }
              // find new position at 0.02 secs
              double speed = sqrt(std::pow(double(sensor_fusion[i][3]), 2) +
                                  std::pow(double(sensor_fusion[i][4]), 2));
              // Obstacles can be in front or rear
              double pos_s_1 = double(sensor_fusion[i][5]) + speed * 0.02;
              double pos_s_2 = double(sensor_fusion[i][5]) + speed;
              double pos_s = std::min(pos_s_1, pos_s_2);
              
              if ( pos_s_2 > car_s) {
                if (pos_s - car_s < 50 && car_lane != lane)
                {
                  if ( abs(obstacles_distances_lanes[lane]) > abs(pos_s - car_s) ) {
                    obstacles_distances_lanes[lane] = pos_s - car_s;
                  }
                }   
                else if (car_lane == lane) {
                  if ( (sensor_fusion[i][5] > car_s) && (obstacles_distances_lanes[lane] > pos_s_1 - car_s) ) {
                    obstacles_distances_lanes[lane] = pos_s_1 - car_s;
                  }
                }             
              }

            }
          }
          static double prev_obst_distance = 100;
          static bool lane_change = false;
          static int lane_change_dest = -1;

          double speed_diff = 0;
          const double MAX_SPEED = 50.0;
          const double MAX_ACC = .224;

          // Find target lane and speed
          lane = car_lane;
          // If a lane change is in progress, do not allow another 
          if (lane_change) {
            if (lane_change_dest == car_lane) {
              lane_change = false;
              lane_change_dest = -1;
            } else {
              lane = lane_change_dest;
              speed_diff += MAX_ACC;
            }
          }
          else {
            // Check obstacle distances in ego lane
            if (obstacles_distances_lanes[car_lane] < 40)
            {
              // Obstacle in ego lanes, check other lanes
              // Lane change is allowed only if 
              // 1. No obstacles in buffer zone front and rear 
              // 2. Ego vehicle is not executing a sharp turn
              if (car_lane > 0 && (obstacles_distances_lanes[(car_lane - 1)] > 50 || obstacles_distances_lanes[(car_lane - 1)] < -25) && ctrl_speed < 40.0 && 
                  (abs(car_local_yaw) < 0.03 || lane_change == true) ) {
                lane = car_lane - 1;
                lane_change_dest = lane;
                lane_change = true;
                speed_diff -= MAX_ACC;
              }
              else if (car_lane < 2 && (obstacles_distances_lanes[(car_lane + 1)] > 50 || obstacles_distances_lanes[(car_lane + 1)] < -25) && ctrl_speed < 40.0 && 
                (abs(car_local_yaw) < 0.03 || lane_change == true) ){
                lane = car_lane + 1;
                lane_change_dest = lane;
                lane_change = true;
                speed_diff -= MAX_ACC;
              }
              else
              {
                // No free lanes - check for obstacle distance and reduce/increase speed as required
                lane_change = false;
                if (obstacles_distances_lanes[car_lane] <= prev_obst_distance)
                {
                  if (obstacles_distances_lanes[car_lane] < 15 ) {
                    speed_diff -= 1.5*MAX_ACC;
                  }
                  else {
                    speed_diff -= MAX_ACC;
                  }
                  prev_obst_distance = obstacles_distances_lanes[car_lane];
                }
                else {
                  speed_diff += MAX_ACC;
                  prev_obst_distance = obstacles_distances_lanes[car_lane];                
                }
              }
            }
            else
            {
              // No obstacles .. continue in same lane
              lane_change = false;
              speed_diff += MAX_ACC;
              prev_obst_distance = 100;
            } // End of find lanes
          }

          printf("distances %6.2f %6.2f %6.2f yaw %6.2f lane change %d lane %d speed diff %6.2f\n", 
              obstacles_distances_lanes[0], obstacles_distances_lanes[1], obstacles_distances_lanes[2], car_local_yaw, lane_change, car_lane, speed_diff);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          { // Find lane curvature

            vector<double> x_pts;
            vector<double> y_pts;

            double car_x_cur = car_x;
            double car_y_cur = car_y;
            double car_yaw_cur = deg2rad(car_yaw);

            if (prev_path_size < 2)
            {
              // Use current position and estimated previous position
              x_pts.push_back(car_x - cos(car_yaw));
              y_pts.push_back(car_y - sin(car_yaw));
              x_pts.push_back(car_x);
              y_pts.push_back(car_y);
            }
            else
            {
              // Paths from previous path to provide continuity
              car_x_cur = previous_path_x[prev_path_size - 1];
              car_y_cur = previous_path_y[prev_path_size - 1];

              double tmp_x = previous_path_x[prev_path_size - 2];
              double tmp_y = previous_path_y[prev_path_size - 2];
              car_yaw_cur = atan2(car_y_cur - tmp_y, car_x_cur - tmp_x);

              x_pts.push_back(tmp_x);
              y_pts.push_back(tmp_y);
              x_pts.push_back(car_x_cur);
              y_pts.push_back(car_y_cur);
              //x_pts.push_back(car_x);
              //y_pts.push_back(car_y);
            }

            // Setting up target points in the future based on frenet co-ordinates
            // If lane change in progress, smooth the trajectory a bit
            double wp_init = 30.0;
            if (lane_change == true) {
              wp_init = 40.0;
            }

            vector<double> wp0 = getXY(car_s + wp_init, 2 + 4 * lane, map_waypoints_s, 
                      map_waypoints_x, map_waypoints_y);
            vector<double> wp1 = getXY(car_s + wp_init + 30, 2 + 4 * lane, map_waypoints_s, 
                      map_waypoints_x, map_waypoints_y);
            vector<double> wp2 = getXY(car_s + wp_init + 60, 2 + 4 * lane, map_waypoints_s, 
                      map_waypoints_x, map_waypoints_y);

            x_pts.push_back(wp0[0]);
            y_pts.push_back(wp0[1]);            
            x_pts.push_back(wp1[0]);
            y_pts.push_back(wp1[1]);
            x_pts.push_back(wp2[0]);
            y_pts.push_back(wp2[1]);
            

            // Transform to local co-ordinates
            for (int i = 0; i < x_pts.size(); ++i)
            {
              double x_transform = x_pts[i] - car_x_cur;
              double y_transform = y_pts[i] - car_y_cur;

              x_pts[i] = x_transform * cos(-car_yaw_cur) - y_transform * sin(-car_yaw_cur);
              y_pts[i] = x_transform * sin(-car_yaw_cur) + y_transform * cos(-car_yaw_cur);
              car_local_yaw = atan2(y_pts[2]-y_pts[0],x_pts[2]-x_pts[0]);
            }

            // Calculate curvature in ego co-ordinates
            tk::spline s;
            s.set_points(x_pts, y_pts);

            for (int i = 0; i < prev_path_size; ++i)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double x_prev = 0;
            for (int i = 1; i < 50 - prev_path_size; ++i)
            {
              //Increase speed gradually for the path
              ctrl_speed += speed_diff;
              if (ctrl_speed > MAX_SPEED)
              {
                ctrl_speed = MAX_SPEED;
              }
              else if (ctrl_speed < MAX_ACC)
              {
                ctrl_speed = MAX_ACC;
              }

              double x_point = x_prev + 0.02 * ctrl_speed * 16/36;
              double y_point = s(x_point);

              x_prev = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // Transform to world co-ordinates
              x_point = x_ref * cos(car_yaw_cur) - y_ref * sin(car_yaw_cur);
              y_point = x_ref * sin(car_yaw_cur) + y_ref * cos(car_yaw_cur);

              x_point += car_x_cur;
              y_point += car_y_cur;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
          }

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