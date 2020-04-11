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
  //string map_file_ = "../data/highway_map.csv";
  string map_file_ = "../../../data/highway_map.csv";
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

  if (map_waypoints_s.size() == 0) {
      std::cout << "Length of Map: " << map_waypoints_s.size() << " --> Exit now" << std::endl;
      return -1;
  }
  
  // start in lane 1 (Lanes: || 0 | 1 | 2 ||
  int lane = 1;     // add used variables as well in h.onMessage

  // reference velocity to target (in mph)
  double ref_vel = 0;           // start at 0.0, speed up is done with .224 beneath

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // static variable for timings
    static int timer = 0;    

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
                  
          bool debug = true;
          
          double smin_change_lane;       // minimum distance for lane change (sets "too_close")
          double smax_change_lane = 40;        // maximum distance for lane change (initiates lane change)

          if (timer > 0)
              timer--;                  // if timer was set, count down
          else
              smin_change_lane = 30;    // if timer reached 0 again, set smin_change_lane to 30

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // calculate car's lane for predicting collisions (if using lane-variable, target lane is used --> inaccurate)
          int car_lane = (int)(car_d / 4);
          std::cout << car_lane << std::endl;

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

          // set previous size to size of previous path from last iteration
          int prev_size = previous_path_x.size();

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // keep in lane example          
          /*double dist_inc = 0.5; 
          for (int i = 0; i < 50; ++i) {
              double next_s = car_s + dist_inc * (i+1);           // car moves forward by dist_inc
              double next_d = 6;                                // car starts in middle lane = 1,5 lanes from middle line = 1,5 * 4 m = 6 m              
              vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              next_x_vals.push_back(xy[0]);
              next_y_vals.push_back(xy[1]);
          } */

          // ===========================================================================================================
          // car-following part beneath --> set reference velocity (video until 48:56 minutes)
          if (prev_size > 0) {
              car_s = end_path_s;
          }

          bool too_close = false;               // is car ahead too close? --> lane change motivated
          bool left_free = true;                // is left lane free? 
          bool right_free = true;               // is right lane free?
          bool change_lane = false;             // motivation for change lane
          

          // variables for decision making for next left, right and ahead vehicle
          double v_init = 100;
          double v_veryLeft = v_init;
          double v_left = v_init; 
          double v_ahead = v_init;
          double v_right = v_init;
          double v_veryRight = v_init;
          double s_veryLeft = 300;
          double s_left = 300;
          double s_ahead = 300;
          double s_right = 300;
          double s_veryRight = 300;

          double max_det_dist = 80;            // maximum distance where a detection of vehicle is possible for considering lane changes          

          // find ref_v to use --> no car ahead: ref_vel; or car_velocity --> look for relevant vehicles in my lane
          for (int i = 0; i < sensor_fusion.size(); i++) {
              // sensor_fusion:
              // 0: car id  
              // 1: position in x
              // 2: position in y
              // 3: relative velocity in x-dir
              // 4: relative velocity in y-dir
              // 5: car's s-position (longitudinal)
              // 6: car's d-position (lateral)
              float d = sensor_fusion[i][6];        
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double speed = sqrt(vx * vx + vy * vy);
              double s = sensor_fusion[i][5];

              s += (double)prev_size * 0.02 * speed;    // if using previous points can project s value out

              // if car is in my lane (lane width = 4 m)              
              if (d < (4 * lane + 4) && d >(4 * lane)) {                    
                  if ((s > car_s) && (s - car_s) < smax_change_lane) {
                      if ((s > car_s) && (s - car_s) < smin_change_lane) 
                          too_close = true;       // boolean variable to decelerate  

                      v_ahead = speed;        
                      s_ahead = s - car_s;

                      change_lane = true;
                        
                      if(debug)
                          std::cout << "car ahead detected with " << v_ahead << " in " << s_ahead << " m." << std::endl;
                    }                    
              }

              // check if left lane is free              
              if (lane > 0) {                                                   // ego car is not yet on the very left lane
                  if ((car_lane * 4 - 4 < d) && (d < car_lane * 4)) {           // another car is in left lane
                      if (((abs(s - (car_s - 2)) < 8) && ((speed - car_speed) < 2))   // and in relevant safety distance +/- 10 m at low v_rel or +/- 20 m at higher v_rel
                      || ((abs(s - (car_s - 2)) < 15) && ((speed - car_speed) < 4))
                      || ((abs(s - (car_s - 2)) < 25) && ((speed - car_speed) >= 4))) // took s - car_s - 2 since car coordinate system is in front of vehicle --> shift to center
                      {
                          left_free = false;                                    // left lane is not possible            
                          if (debug)
                              std::cout << "car on left too close" << std::endl;
                      }
                      
                      if ((s - car_s < max_det_dist) && (s > car_s) && (s - car_s < s_left)) {   // check if car is in front of ego and the one on the left lane which is next to us
                          v_left = speed;                                      // store speed
                          s_left = s - car_s;
                          if(debug)
                            std::cout << "car left detected with " << v_left << " in " << s_left << " m." << std::endl;
                      }
                  }
              }

              // check if very left lane is free (in case of lane ahead and lane left is occupied, but very left lane is free
              if (lane > 1) {
                  if ((car_lane * 4 - 8 < d) && (d < car_lane * 4 - 4)) {           // another car is in very left lane
                      if ((s - car_s < max_det_dist) && (s > car_s) && (s - car_s < s_veryLeft)) {   // check if car is in front of ego and the one on the left lane which is next to us
                          v_veryLeft = speed;
                          s_veryLeft = s - car_s;
                      }
                  }
              }

              // check if right lane is free              
              if (lane < 2) {                                                   // ego car is not yet on the very left lane
                  if ((car_lane * 4 + 4 < d) && (d < lane * 4 + 8)) {       // another car is in right lane
                      if (((abs(s - (car_s - 2)) < 8) && ((speed - car_speed) < 2))   // and in relevant safety distance +/- 10 m at low v_rel or +/- 20 m at higher v_rel
                          || ((abs(s - (car_s - 2)) < 15) && ((speed - car_speed) < 4))
                          || ((abs(s - (car_s - 2)) < 25) && ((speed - car_speed) >= 4))) // took s - car_s - 2 since car coordinate system is in front of vehicle --> shift to center
                      {
                          right_free = false;                                    // right lane is not possible                          
                          if (debug)
                              std::cout << "car on right too close" << std::endl;
                      }

                      if ((s - car_s < max_det_dist) && (s > car_s) && (s - car_s < s_right)) {   // check if car is in front of ego and the one on the left lane which is next to us
                          v_right = speed;                                      // store speed
                          s_right = s - car_s;                                  // and velocity
                          if (debug)
                              std::cout << "car right detected with " << v_left << " in " << s_right << " m." << std::endl;
                      }
                  }
              }

              // check if very right lane is free (in case of lane ahead and lane right is occupied, but very left lane is free
              if (lane < 1) {
                  if ((car_lane * 4 + 8 < d) && (d < car_lane * 4 + 12)) {           // another car is in very right lane
                      if ((s - car_s < max_det_dist) && (s > car_s) && (s - car_s < s_veryRight)) {   // check if car is in front of ego and the one on the left lane which is next to us
                          v_veryRight = speed;
                          s_veryRight = s - car_s;
                      }
                  }
              }
          }

          // if too close to object ahead, slow down; if far enough away, speed up
          double max_vel = 49.5;
          if (too_close) {
              ref_vel -= .224;        // 0.224 mph = 0.01 m/s
          }
          else if (ref_vel < max_vel) {
              ref_vel += .224;
          }

          // if we are on left or right lane, set velocities on lanes to 0, s.t. decision making does not consider those lanes
          if (lane == 0) {
              left_free = false;
              v_left = 0;
          }
          if (lane == 2) {
              right_free = false;
              v_right = 0;
          }

          // check if lane change is possible and no other lane-change occured during last second
          if ((change_lane) && (timer == 0)) {
              // taken out: (v_left >= v_right + 0.5) --> right lane better, if velocity is higher. But initialized to 100 if empty; 
              if (((lane > 0) && (v_left > v_ahead + 0.5) && ((v_left > v_right + 0.5) || (v_left == v_init))   // normal case for changing only one lane
                  || (lane == 2) && (v_veryLeft > v_ahead + 0.5) && (v_veryLeft > v_left + 0.5))
                  && (left_free)) {
                  lane--;       // change to left
                  timer = 200;  // timer set to 100 --> avoid new lane change for 200 cycles * 20 ms = 4.000 ms
                  smin_change_lane = 15;                // during lane change it is possible to driver closer to target object until timer counted down
                  if(debug)
                    std::cout << "Change to left lane " << lane << std::endl;
              }  
              else if (((lane < 2) && (v_right > v_ahead + 0.5) && (v_right > v_ahead + 0.5) && ((v_right > v_left + 0.5) || (v_right == v_init)) 
                  || (lane == 0) && (v_veryRight > v_ahead + 0.5) && (v_veryRight > v_right + 0.5))
                  && (right_free)) {
                  lane++;
                  timer = 200;  // timer set to 200 --> avoid new lane change for 200 cycles * 20 ms = 4.000 ms
                  smin_change_lane = 15;                // during lane change it is possible to driver closer to target object until timer counted down
                  if (debug)
                      std::cout << "Change to right lane " << lane << std::endl;
              }
          }          

          // working with splines: create a list of widely spaced x,y-coordinates, evenly spaced at 30m 
          // later we will interpolate these waypoints with a psline and fill it in with more points that control jerk
          vector<double> pts_x;
          vector<double> pts_y;          

          // reference x,y,yaw state == starting-point from car
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw; 

          // if previous point-size is almost empty, use the car as starting reference
          if (prev_size < 2) {
              // use 2 points that make path tangent to car (go backwards in time from recent car-measurement)
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              pts_x.push_back(prev_car_x);
              pts_x.push_back(car_x);

              pts_y.push_back(prev_car_y);
              pts_y.push_back(car_y);

          }
          else
          {
              // use previous path's end point as starting reference      
              // now reference is set to last and previous_ref to last before 
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // use 2 points that make path tangent to previous path's end point
              pts_x.push_back(ref_x_prev);
              pts_x.push_back(ref_x);

              pts_y.push_back(ref_y_prev);
              pts_y.push_back(ref_y);
          }

          // in Frenet add evenly 30 m spaced points ahead of starting reference
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);
          
          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);

          for (int i = 0; i < pts_x.size(); i++) {
              // shift car reference angle to 0 degrees --> last point of previous path == point of car == [0,0] --> set reference to be [0,0]
              double shift_x = pts_x[i] - ref_x;
              double shift_y = pts_y[i] - ref_y;

              pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // create a spline with created points pts_x / _y
          tk::spline s;
          s.set_points(pts_x, pts_y);

          // set new points from spline to path next_x(_y)_vals
          for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up spline points s.t. we travel at our desired reference velocity
          double target_x = 30.0; 
          double target_y = s(target_x);        // use y-position from spline
          double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

          double x_add_on = 0;
          
          // fill up rest of path planner after filling it with previous points (output: 50 points)
          for (int i = 0; i <= 50 - previous_path_x.size(); i++) {
              // get needed number of points to travel distance of "target_dist" with given speed "ref_vel"
              double N = target_dist / (0.02 * ref_vel / 2.24);     // 2.24 factor conversion from mph to m/s
              double x_point = x_add_on + (target_x) / N;
              double y_point = s(x_point);                          // when tangent is divided into N points, get corresponding x-coordinate

              x_add_on = x_point;

              double x_ref = x_point; 
              double y_ref = y_point; 

              // rotate back to normal aufter rotating it earlier
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
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