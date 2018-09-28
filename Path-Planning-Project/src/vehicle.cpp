#include "vehicle.h"

Vehicle::Vehicle(double _ref_vel, int _lane,
                 vector<double> _map_waypoints_x, vector<double> _map_waypoints_y, vector<double> _map_waypoints_s,
                 vector<double> _map_waypoints_dx, vector<double> _map_waypoints_dy){

  this->ref_vel = mph2mps(_ref_vel);
  this->lane = _lane;

  this->map_waypoints_x = _map_waypoints_x;
  this->map_waypoints_y = _map_waypoints_y;
  this->map_waypoints_s = _map_waypoints_s;
  this->map_waypoints_dx = _map_waypoints_dx;
  this->map_waypoints_dy = _map_waypoints_dy;

  this->num_plan_waypoints = 50;
  this->spline_target_x = 30.0;
  this->max_val = mph2mps(49.5);
  this->max_acc = mph2mps(0.4);
  this->buffer = 15.0;
  this->last_state = "KL";

}

string Vehicle::finite_state_machine(string _last_state){
  string next_state;

  if (_last_state == "KL"){
    // if enough distance, change the lane back to middle
    if (distance_to_car_inlane(this->lane, "front") > this->buffer){
      if (this->lane != 1 && distance_to_car_inlane(1, "front") > 3*this->buffer){
        if (this->lane == 0)
          return "PLCR";
        else if (this->lane == 2)
          return "PLCL";
      }
      return "KL";
    }
    // compare left and right
    int dir = 0;
    int max_dist = distance_to_car_inlane(this->lane, "front");
    for(int i=-1; i<2; i++){
      int check_lane = this->lane + i;
      if (check_lane < 0 || check_lane > 2)
        continue;
      double check_dist = distance_to_car_inlane(check_lane, "front");
      if (check_dist > max_dist){
        max_dist = check_dist;
        dir = i;
      }
    }

    if (dir == -1){
      next_state = "PLCL";
    }else if (dir == 1){
      next_state = "PLCR";
    }else{
      next_state = "KL";
    }
  // change lane
  }else if (_last_state == "PLCL" || _last_state == "PLCR"){
    bool ready = prep_change_lane(_last_state);
    if (ready){
      if (_last_state == "PLCL")
        next_state = "LCL";
      else if (_last_state == "PLCR")
        next_state = "LCR";
    }else{
      next_state = "KL";
    }
  }else{
    next_state = "KL";
  }
  return next_state;
}

void Vehicle::perform_state(string _state,
                            vector<double> &_next_x_vals, vector<double> &_next_y_vals){
  if(_state == "LCL" || _state == "LCR"){
    change_lane_waypoints(_state, _next_x_vals, _next_y_vals);
  }else{
    keep_lane_waypoints(_next_x_vals, _next_y_vals);
  }
}

void Vehicle::update_state(double _car_x, double _car_y, double _car_s, double _car_d, double _car_yaw, double _car_speed,
                           vector<double> &_previous_path_x, vector<double> &_previous_path_y,
                           vector<vector<double>> _sensor_fusion, double _end_path_s, double _end_path_d){

  this->prev_size = _previous_path_x.size();
  this->previous_path_x = _previous_path_x;
  this->previous_path_y = _previous_path_y;
  this->sensor_fusion = _sensor_fusion;

  this->speed = mph2mps(_car_speed);
  if (prev_size > 0){
    this->x = _previous_path_x[prev_size-1];
    this->y = _previous_path_y[prev_size-1];
    this->s = _end_path_s;
    this->d = _end_path_d;
  }else {
    this->x = _car_x;
    this->y = _car_y;
    this->s = _car_s;
    this->d = _car_d;
    this->yaw = deg2rad(_car_yaw);
  }

  if (prev_size < 2){
    this->yaw = deg2rad(_car_yaw);
  }else {
    double ref_x = _previous_path_x[prev_size-1];
    double ref_y = _previous_path_y[prev_size-1];
    double ref_x_prev = _previous_path_x[prev_size-2];
    double ref_y_prev = _previous_path_y[prev_size-2];
    this->yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
  }
}

void Vehicle::keep_lane_waypoints(vector<double> &_next_x_vals, vector<double> &_next_y_vals){

  bool too_close = false;
  this->spline_target_x = 30.0;

  double distance_to_front = distance_to_car_inlane(this->lane, "front");
  if(distance_to_front < this->buffer){
    too_close = true;
  }

  double front_vel = lane_speed(this->lane);
  if (too_close){
    this->ref_vel -= this->max_acc;
  }else if (distance_to_front<this->buffer+2.0 &&
            this->ref_vel>=front_vel-this->max_acc && this->ref_vel<=front_vel+this->max_acc){
    this->ref_vel = front_vel;
  }else if (this->ref_vel < this->max_val){
    this->ref_vel += this->max_acc;
  }

  auto next_xy_vals = compute_waypoints(this->lane);
  _next_x_vals = next_xy_vals[0];
  _next_y_vals = next_xy_vals[1];

}

void Vehicle::change_lane_waypoints(string _next_state,
                                    vector<double> &_next_x_vals, vector<double> &_next_y_vals){
  int curr_lane = this->lane;
  int next_lane;
  if (_next_state == "LCL"){
    next_lane = curr_lane - 1;
  }else if (_next_state == "LCR"){
    next_lane = curr_lane + 1;
  }
  if (next_lane < 0 || next_lane > 2){
    cout << next_lane << " is out of boundary." << endl;
    return;
  }

  double distance_to_front = distance_to_car_inlane(next_lane, "front");
  double distance_to_rear = distance_to_car_inlane(next_lane, "rear");

  bool too_close = false;
  if (distance_to_front<this->buffer || distance_to_rear<this->buffer/3.0){
    cout << "Too close for " << _next_state << ", keep current lane." << endl;
    too_close = true;
    next_lane = this->lane;
  } else{
    this->lane = next_lane;
  }

  double front_vel = lane_speed(this->lane);
  if (too_close){
    this->ref_vel -= this->max_acc;
  }else if (distance_to_front<this->buffer+2.0 &&
            this->ref_vel>=front_vel-this->max_acc && this->ref_vel<=front_vel+this->max_acc){
    this->ref_vel = front_vel;
  }else if (this->ref_vel < this->max_val){
    this->ref_vel += this->max_acc;
  }

  auto next_xy_vals = compute_waypoints(next_lane);
  _next_x_vals = next_xy_vals[0];
  _next_y_vals = next_xy_vals[1];
}

bool Vehicle::prep_change_lane(string _next_state){

  int curr_lane = this->lane;
  int intend_lane;
  if (_next_state == "PLCL"){
    intend_lane = curr_lane - 1;
  }else if (_next_state == "PLCR"){
    intend_lane = curr_lane + 1;
  }
  if (intend_lane < 0 || intend_lane > 2){
    cout << intend_lane << " is out of boundary." << endl;
    return false;
  }

  double distance_to_front = distance_to_car_inlane(intend_lane, "front");
  double distance_to_rear = distance_to_car_inlane(intend_lane, "rear");

  if (distance_to_front>this->buffer && distance_to_rear>this->buffer/3.0){
    return true;
  }
  if (distance_to_front<this->buffer){
    this->ref_vel -= this->max_acc;
    return false;
  } else if (distance_to_rear<this->buffer){
    return false;
  }
  return false;
}

vector<vector<double>> Vehicle::compute_waypoints(int _next_lane){

  vector<double> ptsx, ptsy;
  // push first 2 points
  double ref_x = this->x;
  double ref_y = this->y;
  double ref_x_prev, ref_y_prev;
  if (prev_size < 2){
    ref_x_prev = ref_x - cos(this->yaw);
    ref_y_prev = ref_y - sin(this->yaw);
  }else {
    ref_x_prev = previous_path_x[prev_size-2];
    ref_y_prev = previous_path_y[prev_size-2];
  }
  ptsx.push_back(ref_x_prev);   ptsy.push_back(ref_y_prev);
  ptsx.push_back(ref_x);        ptsy.push_back(ref_y);

  // push last 3 points
  double target_x = this->spline_target_x;
  auto next_wp0 = getXY(this->s+1*target_x, 2+4*_next_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  auto next_wp1 = getXY(this->s+2*target_x, 2+4*_next_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  auto next_wp2 = getXY(this->s+3*target_x, 2+4*_next_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]); ptsx.push_back(next_wp1[0]); ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp0[1]); ptsy.push_back(next_wp1[1]); ptsy.push_back(next_wp2[1]);

  // rotate to car coordinate
  for(int i=0; i<ptsx.size(); i++){
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    auto xy_car = map2carCoord(shift_x, shift_y, this->yaw);
    ptsx[i] = xy_car[0];  ptsy[i] = xy_car[1];
  }

  // polynomial fitting
  tk::spline s;
  s.set_points(ptsx, ptsy);

  vector<double> next_x_waypoints;
  vector<double> next_y_waypoints;
  // push prev waypoints
  for (int i=0; i<prev_size; i++){
    next_x_waypoints.push_back(previous_path_x[i]);
    next_y_waypoints.push_back(previous_path_y[i]);
  }

  // push future waypoints
  double target_y = s(target_x);
  double x_add_on = 0;
  double target_dist = sqrt(target_x*target_x+target_y*target_y);
  double N = target_dist/(0.02*this->ref_vel);

  for (int i=0; i<this->num_plan_waypoints-prev_size; i++){
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);
    x_add_on = x_point;

    auto xy_map = car2mapCoord(x_point, y_point, this->yaw);
    x_point = xy_map[0] + ref_x;
    y_point = xy_map[1] + ref_y;

    next_x_waypoints.push_back(x_point);
    next_y_waypoints.push_back(y_point);
  }

  return {next_x_waypoints, next_y_waypoints};
}

double Vehicle::distance_to_car_inlane(int _lane, string _direction){
  double closest_distance = 1000000000.0;

  for(int i=0; i<this->sensor_fusion.size(); i++){
    double check_d = this->sensor_fusion[i][6];
    if(check_d>2+_lane*4-2 && check_d<2+_lane*4+2){
      double check_vx = this->sensor_fusion[i][3];
      double check_vy = this->sensor_fusion[i][4];
      double check_s = this->sensor_fusion[i][5];
      double check_speed = sqrt(check_vx*check_vx+check_vy*check_vy);
      check_s += ((double)0.02*prev_size*check_speed);
      if (_direction == "front" && check_s > this->s){
        closest_distance = min(closest_distance, check_s-this->s);
      } else if (_direction == "rear" && check_s < this->s){
        closest_distance = min(closest_distance, this->s-check_s);
      }
    }
  }
  return closest_distance;
}

double Vehicle::lane_speed(int _lane){
  double closest_distance = 1000000000.0;
  double front_speed = -1.0;

  for(int i=0; i<this->sensor_fusion.size(); i++){
    double check_d = this->sensor_fusion[i][6];
    if(check_d>2+_lane*4-2 && check_d<2+_lane*4+2){
      double check_vx = this->sensor_fusion[i][3];
      double check_vy = this->sensor_fusion[i][4];
      double check_s = this->sensor_fusion[i][5];
      double check_speed = sqrt(check_vx*check_vx+check_vy*check_vy);
      check_s += ((double)0.02*prev_size*check_speed);
      if (check_s>this->s && closest_distance>(check_s-this->s)){
        closest_distance = check_s-this->s;
        front_speed = check_speed;
      }
    }
  }
  return front_speed;
}
