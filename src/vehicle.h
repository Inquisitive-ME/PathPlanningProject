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

#include "Eigen-3.3/Eigen/Dense"


using namespace std;

constexpr double pi() { return M_PI; }

struct points{
  vector<double> x;
  vector<double> y;
};

class Vehicle {
public:

  /**
  * Constructor
  */

  Vehicle();

  Vehicle(float targetSpeed, float goalLane, float minFollowDistance,
               vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
               float laneWidth = 4, float maxLane = 12, float minLane = 0, float maxAccel = 7,
               float timeStep = 0.02, int numTimeStepsToPredict = 25);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void UpdateFromPath(vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d,
                               double car_x, double car_y, double car_s, double car_d, double car_yaw, double Velocity);

  double getLocationPrediction(double predictionTime);

 bool getLeftGap(vector<vector<double>> sensor_fusion, double& leftGapCenter, double& leftLaneGapVelocity);

  void getSideLaneGaps(vector<vector<double>> sensor_fusion);

  points getPredictedPath(vector<vector<double>> sensor_fusion);


  void InitializePosition(double X, double Y, double S, double D, double Yaw, double Velocity);

  void updatePosition(double X, double Y, double S, double D, double Yaw, double Velocity);

  vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);


  bool get_vehicle_ahead_inLane(vector<vector<double>> sensor_fusion, double lane, Vehicle & rVehicle);
  bool get_vehicle_behind_inLane(vector<vector<double>> sensor_fusion, double lane, Vehicle & rVehicle);


private:
  float mGoalSpeed_mps;
  float mTargetSpeed_mps;
  //TODO Use these
  float mMaxLane;
  float mMinLane;
  float mLaneWidth_m;
  float mDesiredLane;
  float mMaxAcceleration_mpss;
  float mMinFollowDistance_m;
  float mTimeStep;
  int mNumTimeStepsToPredict;

  vector<double> mPredictedPath_x;
  vector<double> mPredictedPath_y;

  // Vehicle's localization Data
  double mX;
  double mY;
  double mS;
  double mD;
  double mYaw;
  double mVelocity;
  double mAcceleration;

  // Vehicles location at the end of the current path
  double mEndOfCurrentPathX;
  double mEndOfCurrentPathY;
  double mEndOfCurrentPathS;
  double mEndOfCurrentPathD;
  double mEndOfCurrentPathYaw;
  double mEndOfCurrentPathVelocity;
  double mEndOfCurrentPathAcceleration;

  // Vehicles location at
  int mPrevPathLength;

  points mPathPoints;
  vector<double> mmap_waypoints_x;
  vector<double> mmap_waypoints_y;
  vector<double> mmap_waypoints_s;


};


vector<double> JMT(vector< double> start, vector <double> end, double T);
double getJMTValue(vector<double> jmt, double t);

#endif
