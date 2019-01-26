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


void Vehicle::UpdateFromPath(vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d,
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
  //mNumPathPointsRemaining = prevPathLength;
  mFuturePredictionTimeSteps = (mNumTimeStepsToPredict - prevPathLength);
  // cout << "PrevPathLength = " << prevPathLength << endl;

  mX = car_x;
  mY = car_y;
  mYaw = deg2rad(car_yaw);
  mS = car_s;
  mD = car_d;

  if(prevPathLength < 2)
  {
    if(Velocity == 0)
    {
      mVelocity = mMaxAcceleration_mpss * mTimeStep;
    }
    else
    {
      mVelocity = Velocity;
    }
    mAcceleration = Velocity/mTimeStep;

    mEndOfCurrentPathX = car_x + cos(mYaw) * mVelocity * mTimeStep;
    mEndOfCurrentPathY = car_y + sin(mYaw) * mVelocity * mTimeStep;
    mEndOfCurrentPathYaw = atan2(mEndOfCurrentPathY - mY, mEndOfCurrentPathX - mX);
    mEndOfCurrentPathVelocity = mVelocity + mAcceleration * mTimeStep;
    mEndOfCurrentPathS = mVelocity * mTimeStep + mS;
    mEndOfCurrentPathD = mD;

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
    mAcceleration = (mVelocity - Velocity)/mTimeStep;
    mVelocity = Velocity;

    // set state for where we want to predict (end of path)

    // use previous path as reference
    mEndOfCurrentPathX = previous_path_x[prevPathLength - 1];
    mEndOfCurrentPathY = previous_path_y[prevPathLength - 1];

    double prev_x = previous_path_x[prevPathLength - 2];
    double prev_y = previous_path_y[prevPathLength - 2];

    mEndOfCurrentPathYaw = atan2(mEndOfCurrentPathY - prev_y, mEndOfCurrentPathX - prev_x);

    mPredictedPath_x.push_back(car_x);
    mPredictedPath_x.push_back(prev_x);
    mPredictedPath_x.push_back(mEndOfCurrentPathX);

    mPredictedPath_y.push_back(car_y);
    mPredictedPath_y.push_back(prev_y);
    mPredictedPath_y.push_back(mEndOfCurrentPathY);

    double prev_velocity = sqrt(pow(previous_path_x[prevPathLength - 2] - previous_path_x[prevPathLength - 3], 2) +
                                pow(previous_path_y[prevPathLength - 2] - previous_path_y[prevPathLength - 3], 2)) / mTimeStep;

    mEndOfCurrentPathVelocity = sqrt(pow(mEndOfCurrentPathX - prev_x, 2) + pow(mEndOfCurrentPathY - prev_y, 2)) / mTimeStep;

    mEndOfCurrentPathAcceleration = sqrt(pow(mEndOfCurrentPathVelocity - prev_velocity, 2)) / mTimeStep;

    // set car_s because we are adding previous path so want to start prediction after these values
    mEndOfCurrentPathS = end_path_s;
    mEndOfCurrentPathD = end_path_d;
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
  return (mEndOfCurrentPathS + mEndOfCurrentPathVelocity * predictionTime + 0.5 * mEndOfCurrentPathAcceleration * predictionTime * predictionTime);
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
      mTargetSpeed_mps = inFrontVehicle.mVelocity * 0.9;
    }
    // Check if we will collide
    else if(inFrontVehicle.getLocationPrediction(mFuturePredictionTimeSteps * mTimeStep)
        <= getLocationPrediction(mFuturePredictionTimeSteps*mTimeStep) + mMinFollowDistance_m)
    {
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
        mTargetSpeed_mps = max(mEndOfCurrentPathVelocity +mMaxAcceleration_mpss * mFuturePredictionTimeSteps * mTimeStep,(double) mGoalSpeed_mps);
      }
    }
  }
  double leftLaneGapCenter = 0;
  double leftLaneGapCenterVelocity = 0;
  if(!getLeftGap(sensor_fusion, leftLaneGapCenter, leftLaneGapCenterVelocity) && mEndOfCurrentPathD > 5.5 && (mTargetSpeed_mps < mGoalSpeed_mps || mVelocity > 30))
  {
    cout << endl << "CHANGING LANE" << endl << endl;
    double laneChangeTime_s = 2.5;
    double distance = mEndOfCurrentPathVelocity * laneChangeTime_s;
    mNumTimeStepsToPredict = laneChangeTime_s / mTimeStep;
    mDesiredLane = 0;

    vector<double> startS{mEndOfCurrentPathS, mEndOfCurrentPathVelocity, mEndOfCurrentPathAcceleration};
    vector<double> startD{mEndOfCurrentPathD, 0, 0};
    //vector<double> endS {leftLaneGapCenter + mEndOfCurrentPathVelocity * mNumTimeStepsToPredict * mTimeStep, mGoalSpeed_mps, 0};

    //vector<double> endS{mEndOfCurrentPathS + distance,  min(mEndOfCurrentPathVelocity +mMaxAcceleration_mpss * laneChangeTime_s,(double) mGoalSpeed_mps), 0};
    vector<double> endS{mEndOfCurrentPathS + distance, mEndOfCurrentPathVelocity + mMaxAcceleration_mpss * mTimeStep, 0};
    vector<double> endD{2 + mDesiredLane, 0, 0};

    vector<double> SJMT = JMT(startS, endS, laneChangeTime_s);
    vector<double> DJMT = JMT(startD, endD, laneChangeTime_s);

    for (double i = mTimeStep; i <= laneChangeTime_s; i = i + mTimeStep*5)
    {
      cout << "i = " << i << " S = " << getJMTValue(SJMT, i) << " D = " << getJMTValue(DJMT, i) << endl;
      vector<double> next_wp = getXY(getJMTValue(SJMT, i), getJMTValue(DJMT, i), mmap_waypoints_s, mmap_waypoints_x,
                                     mmap_waypoints_y);

      mPredictedPath_x.push_back(next_wp[0]);
      mPredictedPath_y.push_back(next_wp[1]);
      //cout << next_wp[0] << ", " << next_wp[1] << endl;

      //mTargetSpeed_mps = mGoalSpeed_mps;
    }

    /*
    cout << endl;
    for (int i = 0; i < mPathPoints.x.size(); i++)
    {
      cout << mPathPoints.x[i] << ", " << mPathPoints.y[i] << endl;
    }
    cout << endl;

   for (double i = 1; i <= 5; i++)
   {
     cout << "mD = " << mD << endl;
     cout << "((2+4*mDesiredLane) = " << ((2+4*mDesiredLane)) << endl;
     cout <<  mEndOfCurrentPathD + ((2+4*mDesiredLane) - mEndOfCurrentPathD) * i/5 << endl;

     vector<double> next_wp = getXY(mEndOfCurrentPathS + 5 * i, mEndOfCurrentPathD + ((2+4*mDesiredLane) - mEndOfCurrentPathD) * i/5, mmap_waypoints_s, mmap_waypoints_x,
                                    mmap_waypoints_y);
     mPredictedPath_x.push_back(next_wp[0]);
     mPredictedPath_y.push_back(next_wp[1]);
   }

     for (int i = 0; i < mPredictedPath_x.size(); i++)
     {
       double shift_x = mPredictedPath_x[i] - mEndOfCurrentPathX;
       double shift_y = mPredictedPath_y[i] - mEndOfCurrentPathY;

       mPredictedPath_x[i] = (shift_x * cos(0 - mEndOfCurrentPathYaw) - shift_y * sin(0 - mEndOfCurrentPathYaw));
       mPredictedPath_y[i] = (shift_x * sin(0 - mEndOfCurrentPathYaw) + shift_y * cos(0 - mEndOfCurrentPathYaw));

       cout << "Point " << i << " = (" << mPredictedPath_x[i] << ", " << mPredictedPath_y[i] << ") " << endl;
     }

*/
    tk::spline s;

    s.set_points(mPredictedPath_x, mPredictedPath_y);

    mPathPoints.x.push_back(mEndOfCurrentPathX);
    mPathPoints.y.push_back(s(mEndOfCurrentPathX));

    double x_add_on = 0;
    //predict out .5 s
    for (int i = 0; i < mNumTimeStepsToPredict-1; i++)
    {
/*
      if (mEndOfCurrentPathVelocity < mTargetSpeed_mps)
      {
        // cout << "Increase velocity current = " << mEndOfCurrentPathVelocity << "m/s";
        mEndOfCurrentPathVelocity = min((double) mTargetSpeed_mps,
                                  (mEndOfCurrentPathVelocity + (mMaxAcceleration_mpss * mTimeStep)));
        // cout << "Next Desired velocity = " << mEndOfCurrentPathVelocity << "m/s" << endl;
      }
      else if (mEndOfCurrentPathVelocity > mTargetSpeed_mps)
      {
        // cout << "Decrease velocity current = " << mEndOfCurrentPathVelocity << "m/s" << endl;
        mEndOfCurrentPathVelocity = max((double) mTargetSpeed_mps, mEndOfCurrentPathVelocity - (mMaxAcceleration_mpss * mTimeStep));
      }
*/
      double x_point = x_add_on + mEndOfCurrentPathVelocity * mTimeStep;

      x_add_on = x_point;

      x_point += mEndOfCurrentPathX;

      double y_point = s(x_point);

      cout << x_point << ", " << y_point << endl;
      mPathPoints.x.push_back(x_point);
      mPathPoints.y.push_back(y_point);
    }

  }

  else
  {
    mNumTimeStepsToPredict = 20;


    // Stay Straight
    //This would only be for straight paths
    // in Frenet add evenly 30m spaced points ahead of the starting referecne this smooths out predicted spline

    for (int i = 1; i <= 5; i++)
    {
      vector<double> next_wp = getXY(mEndOfCurrentPathS + 20 * i, (2 + 4 * mDesiredLane), mmap_waypoints_s,
                                     mmap_waypoints_x,
                                     mmap_waypoints_y);
      mPredictedPath_x.push_back(next_wp[0]);
      mPredictedPath_y.push_back(next_wp[1]);
    }


    for (int i = 0; i < mPredictedPath_x.size(); i++)
    {
      double shift_x = mPredictedPath_x[i] - mEndOfCurrentPathX;
      double shift_y = mPredictedPath_y[i] - mEndOfCurrentPathY;

      mPredictedPath_x[i] = (shift_x * cos(0 - mEndOfCurrentPathYaw) - shift_y * sin(0 - mEndOfCurrentPathYaw));
      mPredictedPath_y[i] = (shift_x * sin(0 - mEndOfCurrentPathYaw) + shift_y * cos(0 - mEndOfCurrentPathYaw));

      //cout << "Point " << i << " = (" << mPredictedPath_x[i] << ", " << mPredictedPath_y[i] << ") " << endl;
    }

    tk::spline s;

    s.set_points(mPredictedPath_x, mPredictedPath_y);


    //predict out .5 s
    double x_add_on = 0;
    for (int i = 0; i < mFuturePredictionTimeSteps; i++)
    {
      if (mEndOfCurrentPathVelocity < mTargetSpeed_mps)
      {
        // cout << "Increase velocity current = " << mEndOfCurrentPathVelocity << "m/s";
        mEndOfCurrentPathVelocity = min((double) mTargetSpeed_mps,
                                        (mEndOfCurrentPathVelocity + (mMaxAcceleration_mpss * mTimeStep)));
        // cout << "Next Desired velocity = " << mEndOfCurrentPathVelocity << "m/s" << endl;
      }
      else if (mEndOfCurrentPathVelocity > mTargetSpeed_mps)
      {
        // cout << "Decrease velocity current = " << mEndOfCurrentPathVelocity << "m/s" << endl;
        mEndOfCurrentPathVelocity = max((double) mTargetSpeed_mps,
                                        mEndOfCurrentPathVelocity - (mMaxAcceleration_mpss * mTimeStep));
      }

      double x_point = x_add_on + mEndOfCurrentPathVelocity * mTimeStep;
      double y_point = s(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      // rotate back to normal
      x_point = (x_ref * cos(mEndOfCurrentPathYaw) - y_ref * sin(mEndOfCurrentPathYaw));
      y_point = (x_ref * sin(mEndOfCurrentPathYaw) + y_ref * cos(mEndOfCurrentPathYaw));

      x_point += mEndOfCurrentPathX;
      y_point += mEndOfCurrentPathY;

      cout << x_point << ", " << y_point << endl;
      mPathPoints.x.push_back(x_point);
      mPathPoints.y.push_back(y_point);

    }
  }
  cout << endl;

  mNumTimeStepsToPredict = 20;
  return mPathPoints;
}

bool Vehicle::getLeftGap(vector<vector<double>> sensor_fusion, double& leftGapCenter, double& leftLaneGapVelocity)
{

  double predictionTime = (mNumTimeStepsToPredict - mFuturePredictionTimeSteps) * mTimeStep; // This is the time to the front of the last predicted path

  double distanceClosestCarInFront = 99999;
  double predictedDistanceCarInFront = 0;

  double distanceClosestCarBehind = -99999;
  double predictedDistanceCarBehind = 0;

  bool carInPath = false;
  leftLaneGapVelocity = 0;
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
    if( (d < (mD - 4 + mLaneWidth/2)) && (d> (mD - 4 - mLaneWidth/2)) && (s < mS +200) && (s > mS - 200))
    {
      double velocity = sqrt(pow(sensor_fusion[i][3], 2) + pow(sensor_fusion[i][4], 2));
      double carDistance = s-mS;
      if (carDistance > 0)
      {
        if(distanceClosestCarInFront > carDistance)
        {
          distanceClosestCarInFront = carDistance;
          predictedDistanceCarInFront = velocity * predictionTime + s;
          if((s < mS + 15 && s > mS - 15) || (predictedDistanceCarInFront < mEndOfCurrentPathS +15 && predictedDistanceCarInFront > mEndOfCurrentPathS -15))
          {
            carInPath = true;
          }
        }
      }
      else if (carDistance < 0)
      {
        if(distanceClosestCarBehind < carDistance)
        {
          distanceClosestCarBehind = carDistance;
          predictedDistanceCarBehind = velocity * predictionTime + s;
          if((s < mS + 15 && s > mS - 15) || (predictedDistanceCarInFront < mEndOfCurrentPathS +15 && predictedDistanceCarInFront > mEndOfCurrentPathS -15))
          {
            carInPath = true;
          }
          leftLaneGapVelocity = velocity;
        }
      }
      }
    }

  //TODO handle only getting one car in front or one car behind
  //TODO really need to optimize based on distance of car in front as well
  leftGapCenter = predictedDistanceCarInFront - predictedDistanceCarBehind;
  cout << "Left Gap = " << leftGapCenter << "m " << endl;
  return {carInPath};



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
        if( (rVehicle.mEndOfCurrentPathS - mEndOfCurrentPathS) < ((s + velocity * predictionTime) -mEndOfCurrentPathS) )
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
      rVehicle.mEndOfCurrentPathX = sensor_fusion[i][1] + sensor_fusion[i][3] * predictionTime;
      rVehicle.mEndOfCurrentPathY = sensor_fusion[i][2] + sensor_fusion[i][4] * predictionTime;
      rVehicle.mYaw = atan2(rVehicle.mEndOfCurrentPathX - rVehicle.mX, rVehicle.mEndOfCurrentPathY - rVehicle.mY); // Don't have but want to put  value
      rVehicle.mEndOfCurrentPathYaw = rVehicle.mYaw;

      rVehicle.mEndOfCurrentPathVelocity = mVelocity; // Don't have acceleration could technically get it by keeping track of cars
      rVehicle.mEndOfCurrentPathS = velocity * predictionTime + s;
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
  mAcceleration = (mVelocity - Velocity)/mTimeStep;

  mX = X;
  mY = Y;
  mS = S;
  mD = D;
  mYaw = Yaw;
  mVelocity = Velocity;
}

double getJMTValue(vector<double> jmt, double t)
{
  return (jmt[0] + jmt[1] * t + jmt[2] * pow(t,2) + jmt[3] * pow(t,3) + jmt[4] * pow(t,4) + jmt[5] * pow(t,5));
}

using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS

  start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
      length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

  EXAMPLE

  > JMT( [0, 10, 0], [10, 10, 0], 1)
  [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */

  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
      3*T*T, 4*T*T*T,5*T*T*T*T,
      6*T, 12*T*T, 20*T*T*T;

  MatrixXd B = MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
      end[1]-(start[1]+start[2]*T),
      end[2]-start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai*B;

  vector <double> result = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++)
  {
    result.push_back(C.data()[i]);
  }

  return result;

}