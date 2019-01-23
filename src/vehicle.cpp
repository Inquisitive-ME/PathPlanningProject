/* From Udacity Lesson 4: Behavior Planning
 * Modified for use in path planning project
 * Richard Swanson
 */

#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "spline.h"

/**
 * Initializes Vehicle
 */

using namespace std;

//TODO add to it's own file
// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

using namespace std;

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

Vehicle::Vehicle(){
  /*
  Called by simulator before simulation begins. Sets various
  parameters which will impact the ego vehicle.
  */

  mGoalSpeed_mps = 0;
  mTargetSpeed_mps = 0;
  mDesiredLane = 0;
  mMinFollowDistance_m = 0;

  mMaxLane = 0;
  mMinLane = 0;
  mMaxAcceleration_mpss = 0;
  mTimeStep = 0;
  mNumTimeStepsToPredict = 0;

}
Vehicle::Vehicle(float targetSpeed, float goalLane, float minFollowDistance, vector<double> &map_waypoints_s,
                 vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, float laneWidth, float maxLane,
                 float minLane, float maxAccel, float timeStep, int numTimeStepsToPredict)
{

  mGoalSpeed_mps = targetSpeed;
  mTargetSpeed_mps = targetSpeed;
  mDesiredLane = goalLane;
  mMinFollowDistance_m = minFollowDistance;
  mmap_waypoints_s = map_waypoints_s;
  mmap_waypoints_x = map_waypoints_x;
  mmap_waypoints_y = map_waypoints_y;
  mMaxLane = maxLane;
  mLaneWidth = laneWidth;
  mMinLane = minLane;
  mMaxAcceleration_mpss = maxAccel;
  mTimeStep = timeStep;
  mNumTimeStepsToPredict = numTimeStepsToPredict;
}


void Vehicle::UpdateFromPath(vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s,
                             double car_x, double car_y, double car_s, double car_d, double car_yaw, double Velocity)
{
  static unsigned int counter = 0;
  cout << " Update from Path number " << counter << endl;
  counter++;

  // clear
  mPredictedPath_x.clear();
  mPredictedPath_y.clear();
  mPathPoints.x.clear();
  mPathPoints.y.clear();

  double prevPathLength = previous_path_x.size();
  mFuturePredictionTimeSteps = (mNumTimeStepsToPredict - prevPathLength);
  // cout << "PrevPathLength = " << prevPathLength << endl;
  if(prevPathLength < 2)
  {
    // use car as reference
    mX = car_x;
    mY = car_y;
    mYaw = deg2rad(car_yaw);
    if(Velocity == 0)
    {
      mVelocity = mMaxAcceleration_mpss * mTimeStep;
    }
    else
    {
      mVelocity = Velocity;
    }
    mA = Velocity/mTimeStep;
    mS = car_s;
    mD = car_d;

    mPredictionX = car_x + cos(mYaw) * mVelocity * mTimeStep;
    mPredictionY = car_y + sin(mYaw) * mVelocity * mTimeStep;
    mPredictionYaw = atan2(mPredictionY - mY, mPredictionX - mX);
    mPredictionVelocity = mVelocity + mA * mTimeStep;
    mPredictionS = mVelocity * mTimeStep + mS;

    // use cars position and get two previous points tangent to the car
    double prev_car_x = car_x - cos(mYaw) * mVelocity * mTimeStep;
    double prev_car_y = car_y - sin(mYaw) * mVelocity * mTimeStep;

    // TODO since car start not moving then this will probably fail because it's sending the same points
    mPredictedPath_x.push_back(prev_car_x);
    mPredictedPath_x.push_back(car_x);

    mPredictedPath_y.push_back(prev_car_y);
    mPredictedPath_y.push_back(car_y);
  }
  else
  {
    // set state of car
    mX = car_x;
    mY = car_y;
    mS = car_s;
    mYaw = deg2rad(car_yaw);
    mA = (mVelocity - Velocity)/mTimeStep;
    mVelocity = Velocity;

    // set state for where we want to predict (end of path)

    // use previous path as reference
    mPredictionX = previous_path_x[prevPathLength - 1];
    mPredictionY = previous_path_y[prevPathLength - 1];

    double prev_x = previous_path_x[prevPathLength - 2];
    double prev_y = previous_path_y[prevPathLength - 2];

    mPredictionYaw = atan2(mPredictionY - prev_y, mPredictionX - prev_x);

    mPredictedPath_x.push_back(prev_x);
    mPredictedPath_x.push_back(mPredictionX);

    mPredictedPath_y.push_back(prev_y);
    mPredictedPath_y.push_back(mPredictionY);

    double prev_velocity = sqrt(pow(previous_path_x[prevPathLength - 2] - previous_path_x[prevPathLength - 3], 2) +
                                pow(previous_path_y[prevPathLength - 2] - previous_path_y[prevPathLength - 3], 2)) / mTimeStep;

    mPredictionVelocity = sqrt(pow(mPredictionX - prev_x, 2) + pow(mPredictionY - prev_y, 2)) / mTimeStep;

    mPredictionAcceleration = sqrt(pow(mPredictionVelocity - prev_velocity, 2)) / mTimeStep;

    // set car_s because we are adding previous path so want to start prediction after these values
    mPredictionS = end_path_s;
  }

  for(int i =0; i< prevPathLength; i++)
  {
    mPathPoints.x.push_back(previous_path_x[i]);
    mPathPoints.y.push_back(previous_path_y[i]);
  }

}


Vehicle::~Vehicle() {}

double Vehicle::getLocationPrediction(double predictionTime)
{
  return (mPredictionS + mPredictionVelocity * predictionTime + 0.5 * mPredictionAcceleration * predictionTime * predictionTime);
}

points Vehicle::getPredictedPath(vector<vector<double>> sensor_fusion)
{
  // Check if we can stay in current lane
  Vehicle inFrontVehicle;

  if(get_vehicle_ahead(sensor_fusion, inFrontVehicle))
  {
    // Check if we are at unsafe distance
    // Simulator doesn't seem to give correct velocity for other cars so if we get within the unsafe driving distance
    // reduce to 90% of the other vehicles speed until we are outside of unsafe driving distance.
    if(inFrontVehicle.mS <= mMinFollowDistance_m + mS)
    {
      cout << endl << "Left Lane gap of " << getLeftGap(sensor_fusion)[0] << "m" << endl << endl;
      mTargetSpeed_mps = inFrontVehicle.mVelocity * 0.9;
    }
    // Check if we will collide
    else if(inFrontVehicle.getLocationPrediction(mFuturePredictionTimeSteps * mTimeStep)
        <= getLocationPrediction(mFuturePredictionTimeSteps*mTimeStep) + mMinFollowDistance_m)
    {
      cout << endl << "Left Lane gap of " << getLeftGap(sensor_fusion)[0] << "m" << endl << endl;
      // Will get within unsafe following distance
      mTargetSpeed_mps = inFrontVehicle.mVelocity;
      // cout << "Slowing Down TargetSpeed = " << mTargetSpeed_mps << " mps" << endl;

      if(getLocationPrediction(mFuturePredictionTimeSteps*mTimeStep) >
      inFrontVehicle.getLocationPrediction(mFuturePredictionTimeSteps*mTimeStep))
      {
        cout <<  endl << "Predicting crash" << endl << endl;
      }
    }
    else
    {
      // found vehicle but not too close
      if (mTargetSpeed_mps < mGoalSpeed_mps)
      {
        mTargetSpeed_mps = max(mPredictionVelocity +mMaxAcceleration_mpss * mFuturePredictionTimeSteps * mTimeStep,(double) mGoalSpeed_mps);
      }
    }
  }

  // Stay Straight
  //This would only be for straight paths
  // in Frenet add evenly 30m spaced points ahead of the starting referecne this smooths out predicted spline

  for(int i = 1; i <= 5; i++)
  {
    vector<double> next_wp = getXY(mPredictionS+20*i, (2+4*mDesiredLane),mmap_waypoints_s, mmap_waypoints_x, mmap_waypoints_y);
    mPredictedPath_x.push_back(next_wp[0]);
    mPredictedPath_y.push_back(next_wp[1]);
  }

  for (int i = 0; i < mPredictedPath_x.size(); i++)
  {
    double shift_x = mPredictedPath_x[i] - mPredictionX;
    double shift_y = mPredictedPath_y[i] - mPredictionY;

    mPredictedPath_x[i] = (shift_x * cos(0-mPredictionYaw) - shift_y*sin(0-mPredictionYaw));
    mPredictedPath_y[i] = (shift_x * sin(0-mPredictionYaw) + shift_y*cos(0-mPredictionYaw));
  }

  tk::spline s;

  s.set_points(mPredictedPath_x, mPredictedPath_y);


  //predict out .5 s
  double x_add_on = 0;
  for(int i = 0; i < mFuturePredictionTimeSteps; i++)
  {
    if(mPredictionVelocity < mTargetSpeed_mps)
    {
      // cout << "Increase velocity current = " << mPredictionVelocity << "m/s";
      mPredictionVelocity = min((double)mTargetSpeed_mps, (mPredictionVelocity + (mMaxAcceleration_mpss * mTimeStep)));
      // cout << "Next Desired velocity = " << mPredictionVelocity << "m/s" << endl;
    }
    else if(mPredictionVelocity > mTargetSpeed_mps)
    {
      // cout << "Decrease velocity current = " << mPredictionVelocity << "m/s" << endl;
      mPredictionVelocity = max((double)mTargetSpeed_mps, mPredictionVelocity - (mMaxAcceleration_mpss * mTimeStep));
    }

    double x_point = x_add_on + mPredictionVelocity * mTimeStep;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal
    x_point = (x_ref * cos(mPredictionYaw) - y_ref * sin(mPredictionYaw));
    y_point = (x_ref * sin(mPredictionYaw) + y_ref * cos(mPredictionYaw));

    x_point += mPredictionX;
    y_point += mPredictionY;

    mPathPoints.x.push_back(x_point);
    mPathPoints.y.push_back(y_point);
  }

  return mPathPoints;
}

vector<double> Vehicle::getLeftGap(vector<vector<double>> sensor_fusion)
{

  double predictionTime = (mNumTimeStepsToPredict - mFuturePredictionTimeSteps) * mTimeStep; // This is the time to the front of the last predicted path

  double distanceClosestCarInFront = 99999;
  double predictedDistanceCarInFront = 0;

  double distanceClosestCarBehind = -99999;
  double predictedDistanceCarBehind = 0;
  // Loop through all Vehicles in sensor_fusion
  // because Udacity doesn't feel this is needed in the code
  // sensor_fusion is
  // [0] car's unique ID,
  // [1] car's x position in map coordinates,
  // [2] car's y position in map coordinates,
  // [3] car's x velocity in m/s,
  // [4] car's y velocity in m/s,
  // [5] car's s position in frenet coordinates,
  // [6] car's d position in frenet coordinates.

  for (int i =0; i < sensor_fusion.size(); i++)
  {
    float d = sensor_fusion[i][6];
    float s = sensor_fusion[i][5];
    if( (d < (mD + 4 + mLaneWidth/2)) && (d> (mD +4 - mLaneWidth/2)) && (s < mS +200) && (s > mS - 200))
    {
      double velocity = sqrt(pow(sensor_fusion[i][3], 2) + pow(sensor_fusion[i][4], 2));
      double carDistance = s-mS;
      if (carDistance > 0)
      {
        if(distanceClosestCarInFront > carDistance)
        {
          distanceClosestCarInFront = carDistance;
          predictedDistanceCarInFront = velocity * predictionTime + s;
        }
      }
      else if (carDistance < 0)
      {
        if(distanceClosestCarBehind < carDistance)
        {
          distanceClosestCarBehind = carDistance;
          predictedDistanceCarBehind = velocity * predictionTime + s;
        }
      }
      }
    }

  //TODO handle only getting one car in front or one car behind
  //TODO really need to optimize based on distance of car in front as well
  return {distanceClosestCarInFront - distanceClosestCarBehind, predictedDistanceCarInFront - predictedDistanceCarBehind};



}
bool Vehicle::get_vehicle_ahead(vector<vector<double>> sensor_fusion, Vehicle & rVehicle) {
  /*
  Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
  rVehicle is updated if a vehicle is found.
  */

  double predictionTime = (mNumTimeStepsToPredict - mFuturePredictionTimeSteps) * mTimeStep; // This is the time to the front of the last predicted path

  bool found_vehicle = false;

  // Loop through all Vehicles in sensor_fusion
  // because Udacity doesn't feel this is needed in the code
  // sensor_fusion is
  // [0] car's unique ID,
  // [1] car's x position in map coordinates,
  // [2] car's y position in map coordinates,
  // [3] car's x velocity in m/s,
  // [4] car's y velocity in m/s,
  // [5] car's s position in frenet coordinates,
  // [6] car's d position in frenet coordinates.

  for (int i =0; i < sensor_fusion.size(); i++)
  {
    float d = sensor_fusion[i][6];
    float s = sensor_fusion[i][5];
    if( (d < (mD + mLaneWidth/2)) && (d> (mD - mLaneWidth/2)) && (mS < s))
    {
      double velocity = sqrt(pow(sensor_fusion[i][3], 2) + pow(sensor_fusion[i][4], 2));
      if (found_vehicle)
      {
        if( (rVehicle.mPredictionS - mPredictionS) < ((s + velocity * predictionTime) -mPredictionS) )
        {
          continue;
        }
      }

      cout << "Found Vehicle " << sensor_fusion[i][0] << " " << s-mS << "m ahead" << endl;
      cout << endl;
      found_vehicle = true;
      rVehicle.mX = sensor_fusion[i][1];
      rVehicle.mY = sensor_fusion[i][2];

      rVehicle.mVelocity = velocity;
      rVehicle.mS = s;
      rVehicle.mD = d;

      // X and Y prediction assumes vehicles are moving in same direction
      rVehicle.mPredictionX = sensor_fusion[i][1] + sensor_fusion[i][3] * predictionTime;
      rVehicle.mPredictionY = sensor_fusion[i][2] + sensor_fusion[i][4] * predictionTime;
      rVehicle.mYaw = atan2(rVehicle.mPredictionX - rVehicle.mX, rVehicle.mPredictionY - rVehicle.mY); // Don't have but want to put  value
      rVehicle.mPredictionYaw = rVehicle.mYaw;

      rVehicle.mPredictionVelocity = mVelocity; // Don't have acceleration could technically get it by keeping track of cars
      rVehicle.mPredictionS = velocity * predictionTime + s;
    }
  }
  return found_vehicle;
}

//TODO create a calculate kinematics for both the target vehicle and the in front vehicle

//TODO change implementation
void Vehicle::getSideLaneGaps(vector<vector<double>> sensor_fusion)
{

}

//
//vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
//  /*
//
//  ***Here you can implement the transition_function code from the Behavior Planning Pseudocode
//  classroom concept.***
//
//  INPUT: A predictions map. This is a map using vehicle id as keys with predicted
//      vehicle trajectories as values. A trajectory is a vector of Vehicle objects. The first
//      item in the trajectory represents the vehicle at the current timestep. The second item in
//      the trajectory represents the vehicle one timestep in the future.
//  OUTPUT: The the best (lowest cost) trajectory for the ego vehicle corresponding to the next ego vehicle state.
//
//  Functions that will be useful:
//  1. successor_states() - Uses the current state to return a vector of possible successor states for the finite
//     state machine.
//  2. generate_trajectory(string state, map<int, vector<Vehicle>> predictions) - Returns a vector of Vehicle objects
//     representing a vehicle trajectory, given a state and predictions. Note that trajectory vectors
//     might have size 0 if no possible trajectory exists for the state.
//  3. calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory) - Included from
//     cost.cpp, computes the cost for a trajectory.
//  */
//
//  // Get possible next states
//  float minCost = numeric_limits<float>::max();
//  string best_next_state = "KL";
//
//
//  vector<string> possibleStates = successor_states();
//
//  for (std::vector<string>::iterator it = possibleStates.begin(); it != possibleStates.end(); ++it)
//  {
//    float cost;
//    vector<Vehicle> traj = generate_trajectory(*it, predictions);
//    if (traj.size() > 0)
//    {
//      cost = calculate_cost(*this, predictions, traj);
//    }
//    if (cost < minCost)
//    {
//      minCost = cost;
//      best_next_state = *it;
//    }
//  }
//
//  //TODO: Change return value here:
//  return generate_trajectory(best_next_state, predictions);
//}
//
//vector<string> Vehicle::successor_states() {
//  /*
//  Provides the possible next states given the current state for the FSM
//  discussed in the course, with the exception that lane changes happen
//  instantaneously, so LCL and LCR can only transition back to KL.
//  */
//  vector<string> states;
//  states.push_back("KL");
//  string state = this->state;
//  if(state.compare("KL") == 0) {
//    states.push_back("PLCL");
//    states.push_back("PLCR");
//  } else if (state.compare("PLCL") == 0) {
//    if (lane != lanes_available - 1) {
//      states.push_back("PLCL");
//      states.push_back("LCL");
//    }
//  } else if (state.compare("PLCR") == 0) {
//    if (lane != 0) {
//      states.push_back("PLCR");
//      states.push_back("LCR");
//    }
//  }
//  //If state is "LCL" or "LCR", then just return "KL"
//  return states;
//}
//
//vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
//  /*
//  Given a possible next state, generate the appropriate trajectory to realize the next state.
//  */
//  vector<Vehicle> trajectory;
//  if (state.compare("CS") == 0) {
//    trajectory = constant_speed_trajectory();
//  } else if (state.compare("KL") == 0) {
//    trajectory = keep_lane_trajectory(predictions);
//  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
//    trajectory = lane_change_trajectory(state, predictions);
//  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
//    trajectory = prep_lane_change_trajectory(state, predictions);
//  }
//  return trajectory;
//}
//
//vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
//  /*
//  Gets next timestep kinematics (position, velocity, acceleration)
//  for a given lane. Tries to choose the maximum velocity and acceleration,
//  given other vehicle positions and accel/velocity constraints.
//  */
//  float max_velocity_accel_limit = this->max_acceleration + this->v;
//  float new_position;
//  float new_velocity;
//  float new_accel;
//  Vehicle vehicle_ahead;
//  Vehicle vehicle_behind;
//
//  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
//
//    if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
//      new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
//    } else {
//      float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
//      new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
//    }
//  } else {
//    new_velocity = min(max_velocity_accel_limit, this->target_speed);
//  }
//
//  new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
//  new_position = this->s + new_velocity + new_accel / 2.0;
//  return{new_position, new_velocity, new_accel};
//
//}
//
//points Vehicle::keep_lane(vector<double> prevPathX, vector<double> prevPathY, )
//
//vector<Vehicle> Vehicle::constant_speed_trajectory() {
//  /*
//  Generate a constant speed trajectory.
//  */
//  float next_pos = position_at(1);
//  vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state),
//                                Vehicle(this->lane, next_pos, this->v, 0, this->state)};
//  return trajectory;
//}
//
//vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
//  /*
//  Generate a keep lane trajectory.
//  */
//  vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
//  vector<float> kinematics = get_kinematics(predictions, this->lane);
//  float new_s = kinematics[0];
//  float new_v = kinematics[1];
//  float new_a = kinematics[2];
//  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
//  return trajectory;
//}
//
//vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
//  /*
//  Generate a trajectory preparing for a lane change.
//  */
//  float new_s;
//  float new_v;
//  float new_a;
//  Vehicle vehicle_behind;
//  int new_lane = this->lane + lane_direction[state];
//  vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
//  vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);
//
//  if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
//    //Keep speed of current lane so as not to collide with car behind.
//    new_s = curr_lane_new_kinematics[0];
//    new_v = curr_lane_new_kinematics[1];
//    new_a = curr_lane_new_kinematics[2];
//
//  } else {
//    vector<float> best_kinematics;
//    vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
//    //Choose kinematics with lowest velocity.
//    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
//      best_kinematics = next_lane_new_kinematics;
//    } else {
//      best_kinematics =  ;
//    }
//    new_s = best_kinematics[0];
//    new_v = best_kinematics[1];
//    new_a = best_kinematics[2];
//  }
//
//  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
//  return trajectory;
//}
//
//vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
//  /*
//  Generate a lane change trajectory.
//  */
//  int new_lane = this->lane + lane_direction[state];
//  vector<Vehicle> trajectory;
//  Vehicle next_lane_vehicle;
//  //Check if a lane change is possible (check if another vehicle occupies that spot).
//  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
//    next_lane_vehicle = it->second[0];
//    if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
//      //If lane change is not possible, return empty trajectory.
//      return trajectory;
//    }
//  }
//  trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
//  vector<float> kinematics = get_kinematics(predictions, new_lane);
//  trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
//  return trajectory;
//}
//
//void Vehicle::increment(int dt = 1) {
//  this->s = position_at(dt);
//}
//
//float Vehicle::position_at(int t) {
//  return this->mS + this->mVelocity*t + this->a*t*t/2.0;
//}
//
//bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
//  /*
//  Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
//  rVehicle is updated if a vehicle is found.
//  */
//  int max_s = -1;
//  bool found_vehicle = false;
//  Vehicle temp_vehicle;
//  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
//    temp_vehicle = it->second[0];
//    if (temp_vehicle.lane == this->lane && temp_vehicle.mS < this->mS && temp_vehicle.mS > max_s) {
//      max_s = temp_vehicle.mS;
//      rVehicle = temp_vehicle;
//      found_vehicle = true;
//    }
//  }
//  return found_vehicle;
//}
//
//
//vector<Vehicle> Vehicle::generate_predictions(int horizon) {
//  /*
//  Generates predictions for non-ego vehicles to be used
//  in trajectory generation for the ego vehicle.
//  */
//  vector<Vehicle> predictions;
//  for(int i = 0; i < horizon; i++) {
//    float next_s = position_at(i);
//    float next_v = 0;
//    if (i < horizon-1) {
//      next_v = position_at(i+1) - s;
//    }
//    //predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
//  }
//  return predictions;
//
//}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
  /*
  Sets state and kinematics for ego vehicle using the last state of the trajectory.
  */
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->mS = next_state.mS;
  this->mVelocity = next_state.mVelocity;
  this->a = next_state.a;
}

void Vehicle::InitializePosition(double X, double Y, double S, double D, double Yaw, double Velocity)
{
  mX = X;
  mY = Y;
  mS = S;
  mD = D;
  mYaw = Yaw;
  mVelocity = Velocity;
}

void Vehicle::updatePosition(double X, double Y, double S, double D, double Yaw, double Velocity)
{
  //Calculate Acceleration
  mA = (mVelocity - Velocity)/mTimeStep;

  mX = X;
  mY = Y;
  mS = S;
  mD = D;
  mYaw = Yaw;
  mVelocity = Velocity;
}
