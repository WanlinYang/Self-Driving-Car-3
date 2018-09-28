#ifndef HELPER_H
#define HELPER_H

#include <vector>
#include <math.h>
#include <string>

using namespace std;

double deg2rad(double x);
double rad2deg(double x);

double mph2mps(double vel);
double mps2mph(double vel);

double distance(double x1, double y1, double x2, double y2);

vector<double> map2carCoord(double x, double y, double theta);
vector<double> car2mapCoord(double x, double y, double theta);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                      const vector<double> &maps_x, const vector<double> &maps_y);

#endif /* HELPER_H */
