#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <vector>
#include "helper.h"
#include "spline.h"

using namespace std;

class Vehicle{
private:

  int prev_size;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  vector<vector<double>> sensor_fusion;

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  /*
  Compute waypoints given the next lane number.
  */
  vector<vector<double>> compute_waypoints(int _next_lane);
  
  /*
  Find the s distacne between ego car and the closest car in a particular lane,
  either front or rear direction.
  Input:
    lane: lane number
    direction: "front" or "rear"
  Output:
    s distance to the closest car
  */
  double distance_to_car_inlane(int _lane, string _direction);

  /*
  Find the front car speed in a particular lane.
  Input:
    lane: lane number
  Output:
    spped of front car in that lane
  */
  double lane_speed(int _lane);

public:

  double ref_vel; // current car speed
  int lane; // current lane
  double x, y, s, d, yaw, speed;
  double buffer;  // 15 m
  double max_val;  // 49.5 mph
  double max_acc; // 0.5mph
  double spline_target_x; // distance for spline
  int num_plan_waypoints; // 50 points
  string last_state;  // state of the last step ("KL", "LCL", "LCR", "PLCL", "PLCR")

  /*
  Constructor of Vehicle.
  Input:
    ref_vel: initial reference velocity (mph), which is 0.0 in this case.
    lane: initial lane number, which is 1 in this case.
    map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx,
    map_waypoints_dx, map_waypoints_dy: map information
  */
  Vehicle(double _ref_vel, int _lane,
          vector<double> _map_waypoints_x, vector<double> _map_waypoints_y, vector<double> _map_waypoints_s,
          vector<double> _map_waypoints_dx, vector<double> _map_waypoints_dy);

  /*
  Convert from one state to the next state.
  Input:
    last_state: state of the last step.
  Output:
    state of the next step based on sensor mesurement.
  */
  string finite_state_machine(string _last_state);

  /*
  Return feasible waypoints given a desired state.
  Input:
    state: desired state
  Output:
    vector of x and y waypoints
  */
  void perform_state(string _state, vector<double> &_next_x_vals, vector<double> &_next_y_vals);

  /*
  Update information of ego vehicle's position and hitory, and data of sensor_fusion.
  Input:
    car_x, car_y, car_s, car_d, car_yaw, car_speed: ego vehicle's condition
    previous_path_x, previous_path_y: waypoints went throught during the last 0.02s timestep
    sensor_fusion: a list of all other cars on the same side of the road
    end_path_s, end_path_d: previous path's end s and d values
  */
  void update_state(double _car_x, double _car_y, double _car_s, double _car_d, double _car_yaw, double _car_speed,
                    vector<double> &_previous_path_x, vector<double> &_previous_path_y,
                    vector<vector<double>> _sensor_fusion, double _end_path_s, double _end_path_d);

  /*
  Generate waypoints of "KL"(keep lane) state
  */
  void keep_lane_waypoints(vector<double> &_next_x_vals, vector<double> &_next_y_vals);

  /*
  Generate waypoint of "LCL"(lane change left) or "LCR"(lane change right)
  Input:
    next_state: either "LCL" or "LCR"
  Output:
    waypoints
  */
  void change_lane_waypoints(string _next_state, vector<double> &_next_x_vals, vector<double> &_next_y_vals);

  /*
  Judge if ready to change lane.
  Input:
    next_state: either "PLCL" or "PLCR"
  Output:
    true if ready to change, false if not ready
  */
  bool prep_change_lane(string _next_state);

};

#endif
