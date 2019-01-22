/* From Udacity Lesson 4: Behavior Planning
 * Modified for use in path planning project
 * Richard Swanson
 */

#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

struct points{
  vector<double> x;
  vector<double> y;
};

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;

  float a;

  float mTargetSpeed;

  int lanes_available;

  string state;

  /**
  * Constructor
  */
  void Vehicle(float targetSpeed, float goalLane, float minFollowDistance, float maxLane = 12, float minLane 0, float maxAccel = 10), float timeStep = 0.02);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void InitializePosition(double X, double Y, double S, double D, double Yaw, double Velocity);

  void updatePosition(double X, double Y, double S, double D, double Yaw, double Velocity);

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  vector<Vehicle> generate_predictions(int horizon=2);

  void realize_next_state(vector<Vehicle> trajectory);


private:
  float mTargetSpeed_mph;
  float mMaxLane;
  float mMinLane;
  float mDesiredLane;
  float mMaxAcceleration_mpss;
  float mMinFollowDistance_m;
  float mTimeStep;

  // Vehicle's localization Data
  double mX;
  double mY;
  double mS;
  double mD;
  double mYaw;
  double mVelocity;
  double mA;

};

#endif
