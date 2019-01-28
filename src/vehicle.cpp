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
#include <limits>

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

int getLane(double D)
{
  int lane;
  if(D == 4)
  {
    lane = 0;
  }
  else if(D == 8)
  {
    lane = 1;
  }
  else if(D == 12)
  {
    lane = 2;
  }
  else
  {
    lane = D / 4;
  }
  return lane;
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

  mMaxLaneMergeTime_s = 0;
  mMinLaneMergeTime_s = 0;
  mMaxAcceleration_mpss = 0;
  mTimeStep = 0;
  mNumTimeStepsToPredict = 0;

}
Vehicle::Vehicle(float targetSpeed, int goalLane, float minFollowDistance, vector<double> &map_waypoints_s,
                 vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, float laneWidth, double maxLane,
                 float minLane, float maxAccel, float timeStep, int numTimeStepsToPredict)
{

  mGoalSpeed_mps = targetSpeed;
  mTargetSpeed_mps = targetSpeed;
  mDesiredLane = goalLane;
  mGoalLane = goalLane;
  mMinFollowDistance_m = minFollowDistance;
  mmap_waypoints_s = map_waypoints_s;
  mmap_waypoints_x = map_waypoints_x;
  mmap_waypoints_y = map_waypoints_y;
  mMaxLaneMergeTime_s = maxLane;
  mLaneWidth_m = laneWidth;
  mMinLaneMergeTime_s = minLane;
  mMaxAcceleration_mpss = maxAccel;
  mTimeStep = timeStep;
  mNumTimeStepsToPredict = numTimeStepsToPredict;
}


void Vehicle::UpdateFromPath(vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d,
                             double car_x, double car_y, double car_s, double car_d, double car_yaw, double Velocity)
{
  Velocity = Velocity * 0.44704; // Convert to mps
  static unsigned int counter = 0;
  cout << " Update from Path number " << counter << endl;
  counter++;

  // clear
  mPredictedPath_x.clear();
  mPredictedPath_y.clear();
  mPathPoints.x.clear();
  mPathPoints.y.clear();

  mPrevPathLength =  previous_path_x.size();
  // cout << "PrevPathLength = " << prevPathLength << endl;

  mX = car_x;
  mY = car_y;
  mYaw = deg2rad(car_yaw);
  mS = car_s;
  mD = car_d;

  if(mPrevPathLength < 2)
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
    mEndOfCurrentPathX = previous_path_x[mPrevPathLength - 1];
    mEndOfCurrentPathY = previous_path_y[mPrevPathLength - 1];

    double prev_x = previous_path_x[mPrevPathLength - 2];
    double prev_y = previous_path_y[mPrevPathLength - 2];

    mEndOfCurrentPathYaw = atan2(mEndOfCurrentPathY - prev_y, mEndOfCurrentPathX - prev_x);

    //mPredictedPath_x.push_back(car_x);
    mPredictedPath_x.push_back(prev_x);
    mPredictedPath_x.push_back(mEndOfCurrentPathX);

   // mPredictedPath_y.push_back(car_y);
    mPredictedPath_y.push_back(prev_y);
    mPredictedPath_y.push_back(mEndOfCurrentPathY);

    double prev_velocity = sqrt(pow(previous_path_x[mPrevPathLength - 2] - previous_path_x[mPrevPathLength - 3], 2) +
                                pow(previous_path_y[mPrevPathLength - 2] - previous_path_y[mPrevPathLength - 3], 2)) / mTimeStep;

    mEndOfCurrentPathVelocity = sqrt(pow(mEndOfCurrentPathX - prev_x, 2) + pow(mEndOfCurrentPathY - prev_y, 2)) / mTimeStep;

    mEndOfCurrentPathAcceleration = sqrt(pow(mEndOfCurrentPathVelocity - prev_velocity, 2)) / mTimeStep;

    // set car_s because we are adding previous path so want to start prediction after these values
    mEndOfCurrentPathS = end_path_s;
    // mEndOfCurrentPathD = end_path_d; //This doesn't work in the right lane?
  }

  for(int i =0; i< mPrevPathLength; i++)
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

points Vehicle::getNextAction(vector<vector<double>> sensor_fusion)
{
  //reset mNumTimeStepsToPredict to default
  // TODO don't do this should have default saved
  mNumTimeStepsToPredict = 25;
  int currentLane = getLane(mEndOfCurrentPathD);
  cout << "End of D" << mEndOfCurrentPathD << endl;
  cout << "In Lane " << currentLane << endl;

  double stayInLaneCost = 0;
  double LeftMergeCost = 0;
  double RightMergeCost = 0;


  Vehicle inFrontVehicle;
  double inFrontDistanceAhead;
  double inFrontTargetSpeed = mGoalSpeed_mps;

  double LeftMergeDistanceAhead;
  double LeftMergeTargetSpeed;
  int LeftMergeTimeSteps;

  double RightMergeDistanceAhead;
  double RightMergeTargetSpeed;
  int RightMergeTimeSteps;

  double futureTimeSteps = mNumTimeStepsToPredict - mPrevPathLength;

  if(get_vehicle_ahead_inLane(sensor_fusion, mD, inFrontVehicle))
  {
    inFrontDistanceAhead = min(inFrontVehicle.mEndOfCurrentPathS - mEndOfCurrentPathS,
                                mMaxLaneMergeTime_s * mEndOfCurrentPathVelocity);
    // Check if we are at unsafe distance
    // Simulator doesn't seem to give correct velocity for other cars so if we get within the unsafe driving distance
    // reduce to 90% of the other vehicles speed until we are outside of unsafe driving distance.
    if(inFrontVehicle.mS <= mMinFollowDistance_m + mS)
    {
      cout << "CHECK " << endl;
      inFrontTargetSpeed = inFrontVehicle.mVelocity * 0.9;
    }
      // Check if we will collide
    else if(inFrontVehicle.getLocationPrediction(futureTimeSteps * mTimeStep)
            <= getLocationPrediction(futureTimeSteps*mTimeStep) + mMinFollowDistance_m)
    {
      // Will get within unsafe following distance
      inFrontTargetSpeed = inFrontVehicle.mVelocity;
      cout << "Slowing Down TargetSpeed = " << mTargetSpeed_mps << " mps" << endl;

      if(getLocationPrediction(futureTimeSteps*mTimeStep) >
         inFrontVehicle.getLocationPrediction(futureTimeSteps*mTimeStep))
      {
        cout <<  endl << "Predicting crash" << endl << endl;
      }
    }
    else
    {
      // found vehicle but not too close
      if (inFrontTargetSpeed < mGoalSpeed_mps)
      {
        inFrontTargetSpeed = max(mEndOfCurrentPathVelocity +mMaxAcceleration_mpss * futureTimeSteps * mTimeStep,(double) mGoalSpeed_mps);
      }
    }
  }
  else
  {
    inFrontDistanceAhead = mMaxLaneMergeTime_s * mEndOfCurrentPathVelocity;
    inFrontTargetSpeed = mGoalSpeed_mps;

  }

  cout << "Calculate Stay Cost " << endl;
  stayInLaneCost = calculate_cost(mGoalLane, currentLane, inFrontTargetSpeed, mGoalSpeed_mps, inFrontDistanceAhead);

  // Check Left Lane Merge
  if(currentLane >=1 && mVelocity > 10)
  {
    if (getLaneInfo(sensor_fusion, LeftMergeDistanceAhead, LeftMergeTargetSpeed, LeftMergeTimeSteps, currentLane - 1))
    {
      cout << "Calculate Left Merge Cost " << endl;
      LeftMergeCost = calculate_cost(mGoalLane, currentLane - 1, LeftMergeTargetSpeed, mGoalSpeed_mps,
                                     LeftMergeDistanceAhead);
    }
    else
    {
      cout << "CANNOT MERGE LEFT " << endl;
      LeftMergeCost = numeric_limits<double>::max();
    }
  }
  else
  {
    cout << "IN LEFT LANE " << endl;
    LeftMergeCost = numeric_limits<double>::max();
  }
  // Check Right Lane Merge
  if(currentLane < 2 && mVelocity > 10)
  {
    if (getLaneInfo(sensor_fusion, RightMergeDistanceAhead, RightMergeTargetSpeed, RightMergeTimeSteps,
                    currentLane + 1))
    {
      cout << "Calculate Right Merge Cost " << endl;
      RightMergeCost = calculate_cost(mGoalLane, currentLane + 1, RightMergeTargetSpeed, mGoalSpeed_mps,
                                      RightMergeDistanceAhead);
    }
    else
    {
      cout << "CANNOT MERGE RIGHT" << endl;
      RightMergeCost = numeric_limits<double>::max();
    }
  }
  else
  {
    cout << "In RIGHT LANE" << endl;
    RightMergeCost = numeric_limits<double>::max();
  }
  cout << "Stay In Lane Cost = " << stayInLaneCost << endl;
  cout << "Right Merge Cost = " << RightMergeCost << endl;
  cout << "Left Merge Cost = " << LeftMergeCost << endl;

  double minCost = min(min(LeftMergeCost, RightMergeCost), stayInLaneCost);
  if(stayInLaneCost == minCost)
  {
    cout << "Stay in Lane" << endl;
    cout << "Setting mTargetSpeed to " << inFrontTargetSpeed << endl;
    mTargetSpeed_mps = inFrontTargetSpeed;
    return stayStraight();


  }
  else if(RightMergeCost == minCost)
  {
    mDesiredLane+=1;
    mTargetSpeed_mps = RightMergeTargetSpeed;
    cout << "Merge Right " << mDesiredLane << endl;
    return merge(RightMergeDistanceAhead, RightMergeTimeSteps);
  }
  else if(LeftMergeCost == minCost)
  {
    mDesiredLane-=1;
    mTargetSpeed_mps = LeftMergeTargetSpeed;
    cout << "MergeLeft " << mDesiredLane << endl;
    return merge(LeftMergeDistanceAhead, LeftMergeTimeSteps);
  }

}

points Vehicle::stayStraight()
{
  tk::spline s;
  mNumTimeStepsToPredict = 25;
  double futureTimeSteps = mNumTimeStepsToPredict - mPrevPathLength;

  // Stay Straight
  //This would only be for straight paths
  // in Frenet add evenly 20m spaced points ahead of the starting referecne this smooths out predicted spline
  for (int i = 1; i <= 5; i++)
  {
    vector<double> next_wp = getXY(mEndOfCurrentPathS + 20 * i, (2 + 4 * mDesiredLane),
                                   mmap_waypoints_s,
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

  s.set_points(mPredictedPath_x, mPredictedPath_y);


  //predict out .5 s
  double x_add_on = 0;
  cout << "mEndofCurrentPath velocity = " << mEndOfCurrentPathVelocity << endl;
  for (int i = 0; i < futureTimeSteps; i++)
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

    // cout << x_point << ", " << y_point << endl;
    mPathPoints.x.push_back(x_point);
    mPathPoints.y.push_back(y_point);
  }
  return mPathPoints;
}

points Vehicle::merge(double distanceAhead, int TimeSteps)
{
  tk::spline s;
  cout << endl << "CHANGING LANE" << endl << endl;
  double laneChangeTime_s = TimeSteps * mTimeStep;
  cout << "Lane change  time = " << laneChangeTime_s << " s Distance = " << distanceAhead << " m" << endl;

  mEndOfCurrentPathD = 2 + 4 * mDesiredLane;
  for (int i = 0; i < 2; i++)
  {
    vector<double> next_wp = getXY(mEndOfCurrentPathS + distanceAhead + i * 20, 2 + 4 * mDesiredLane, mmap_waypoints_s,
                                   mmap_waypoints_x,
                                   mmap_waypoints_y);
    mPredictedPath_x.push_back(next_wp[0]);
    mPredictedPath_y.push_back(next_wp[1]);
    // cout << "SD " << mEndOfCurrentPathS + distanceAhead + i * 20 << ", " << 2 + 4 * mDesiredLane << endl;
  }

  for (int i = 0; i < mPredictedPath_x.size(); i++)
  {
    double shift_x = mPredictedPath_x[i] - mEndOfCurrentPathX;
    double shift_y = mPredictedPath_y[i] - mEndOfCurrentPathY;

    mPredictedPath_x[i] = (shift_x * cos(0 - mEndOfCurrentPathYaw) - shift_y * sin(0 - mEndOfCurrentPathYaw));
    mPredictedPath_y[i] = (shift_x * sin(0 - mEndOfCurrentPathYaw) + shift_y * cos(0 - mEndOfCurrentPathYaw));

    cout << "Point " << i << " = (" << mPredictedPath_x[i] << ", " << mPredictedPath_y[i] << ") " << endl;
  }

  s.set_points(mPredictedPath_x, mPredictedPath_y);

  //predict out .5 s
  double x_add_on = 0;
  double xLimit = mEndOfCurrentPathVelocity * laneChangeTime_s;
  // Go to lane change distance, since we are not taking into account the acceleration we are adding when calculating
  // the distance we just need to stop the path once we reach the distance.
  while (x_add_on < xLimit)
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

    // cout << x_point << ", " << y_point << endl;
    mPathPoints.x.push_back(x_point);
    mPathPoints.y.push_back(y_point);
  }
  return mPathPoints;
}

points Vehicle::getPredictedPath(vector<vector<double>> sensor_fusion)
{
  tk::spline s;
  double futureTimeSteps = mNumTimeStepsToPredict - mPrevPathLength;
  // Check if we can stay in current lane
  Vehicle inFrontVehicle;

  if(get_vehicle_ahead_inLane(sensor_fusion, mD, inFrontVehicle))
  {
    // Check if we are at unsafe distance
    // Simulator doesn't seem to give correct velocity for other cars so if we get within the unsafe driving distance
    // reduce to 90% of the other vehicles speed until we are outside of unsafe driving distance.
    if(inFrontVehicle.mS <= mMinFollowDistance_m + mS)
    {
      mTargetSpeed_mps = inFrontVehicle.mVelocity * 0.9;
    }
    // Check if we will collide
    else if(inFrontVehicle.getLocationPrediction(futureTimeSteps * mTimeStep)
        <= getLocationPrediction(futureTimeSteps*mTimeStep) + mMinFollowDistance_m)
    {
      // Will get within unsafe following distance
      mTargetSpeed_mps = inFrontVehicle.mVelocity;
      // cout << "Slowing Down TargetSpeed = " << mTargetSpeed_mps << " mps" << endl;

      if(getLocationPrediction(futureTimeSteps*mTimeStep) >
      inFrontVehicle.getLocationPrediction(futureTimeSteps*mTimeStep))
      {
        cout <<  endl << "Predicting crash" << endl << endl;
      }
    }
    else
    {
      // found vehicle but not too close
      if (mTargetSpeed_mps < mGoalSpeed_mps)
      {
        mTargetSpeed_mps = max(mEndOfCurrentPathVelocity +mMaxAcceleration_mpss * futureTimeSteps * mTimeStep,(double) mGoalSpeed_mps);
      }
    }
  }

  double leftLaneGapCenter = 0;
  double leftLaneGapCenterVelocity = 0;
  cout << "mNumTimeSteps to predict = " << mNumTimeStepsToPredict << endl;
  cout << "end of current path D = " << mEndOfCurrentPathD << endl;

  if(getLaneInfo(sensor_fusion, leftLaneGapCenter, leftLaneGapCenterVelocity, mNumTimeStepsToPredict, 2) && mEndOfCurrentPathD < 8 && mVelocity > 19)
  {
    cout << endl << "CHANGING LANE" << endl << endl;
    double laneChangeTime_s = mNumTimeStepsToPredict * mTimeStep;
    cout << "Lane change  time = " << laneChangeTime_s << " s Distance = " << leftLaneGapCenter << " m" << endl;
    double distance = leftLaneGapCenter;
    //mNumTimeStepsToPredict = laneChangeTime_s / mTimeStep;

    mDesiredLane = 2;
    return merge(distance, mNumTimeStepsToPredict);
/*
    mDesiredLane = 0;

    for (int i = 0; i < 2; i++)
    {
      vector<double> next_wp = getXY(mEndOfCurrentPathS + distance + i * 20, 2 + 4 * mDesiredLane, mmap_waypoints_s,
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

      cout << "Point " << i << " = (" << mPredictedPath_x[i] << ", " << mPredictedPath_y[i] << ") " << endl;
    }

    s.set_points(mPredictedPath_x, mPredictedPath_y);

    futureTimeSteps = mNumTimeStepsToPredict;

    //predict out .5 s
    double x_add_on = 0;
    double xLimit = mEndOfCurrentPathVelocity * mTimeStep * futureTimeSteps;
    // Go to lane change distance, since we are not taking into account the acceleration we are adding when calculating
    // the distance we just need to stop the path once we reach the distance.
    while (x_add_on < xLimit)
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
    */
  }
  else
  {
   mNumTimeStepsToPredict = 25;


    // Stay Straight
    //This would only be for straight paths
    // in Frenet add evenly 30m spaced points ahead of the starting referecne this smooths out predicted spline

    for (int i = 1; i <= 5; i++)
    {
      vector<double> next_wp = getXY(mEndOfCurrentPathS + 20 * i, (2 + 4 * mDesiredLane),
                                     mmap_waypoints_s,
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

    s.set_points(mPredictedPath_x, mPredictedPath_y);


    //predict out .5 s
    double x_add_on = 0;
    for (int i = 0; i < futureTimeSteps; i++)
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

      //cout << x_point << ", " << y_point << endl;
      mPathPoints.x.push_back(x_point);
      mPathPoints.y.push_back(y_point);
    }
  }
  cout << endl;

  mNumTimeStepsToPredict = 25;
  return mPathPoints;
}

bool Vehicle::getLaneInfo(vector<vector<double>> sensor_fusion, double& distanceAhead, double& laneVelocity, int& timeStepsToDistance, int Lane)
{

  double EndOfPathDistanceCarInFront = 0;

  laneVelocity = 0;
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

  Vehicle laneVehicleAhead;
  Vehicle laneVehicleBehind;

  if (get_vehicle_ahead_inLane(sensor_fusion, mEndOfCurrentPathD + 4 * (Lane - getLane(mEndOfCurrentPathD)), laneVehicleAhead))
  {
    cout << "Vehicle Ahead in Lane " << Lane << " " << laneVehicleAhead.mEndOfCurrentPathS - mEndOfCurrentPathS << " m" << endl;
    cout << "Vehicle speed " << laneVehicleAhead.mEndOfCurrentPathVelocity << endl;

    // How far in front at end of path
    EndOfPathDistanceCarInFront = min(laneVehicleAhead.mEndOfCurrentPathS - mEndOfCurrentPathS,
                                      mMaxLaneMergeTime_s * mEndOfCurrentPathVelocity);
    laneVelocity = laneVehicleAhead.mEndOfCurrentPathVelocity;

    if (EndOfPathDistanceCarInFront <= mMinFollowDistance_m)
    {
      cout << "Lane " << Lane << " " <<  "Vehicle in Front Too Close " << endl;
      return false;
    }
    else
    {
      timeStepsToDistance = ((EndOfPathDistanceCarInFront / (mEndOfCurrentPathVelocity)) / mTimeStep);
      if (timeStepsToDistance * mTimeStep * laneVehicleAhead.mEndOfCurrentPathVelocity +
              laneVehicleAhead.mEndOfCurrentPathS - mMinLaneMergeTime_s * mEndOfCurrentPathVelocity <=
          mEndOfCurrentPathS + mEndOfCurrentPathVelocity * timeStepsToDistance * mTimeStep)
      {
        cout << "Predicting would hit vehicle Ahead" << endl;
        return false;
      }
    }
  }
  else
  {
    cout << "No Vehicle Ahead in lane " << Lane << endl;
    laneVelocity = mGoalSpeed_mps;
    EndOfPathDistanceCarInFront = mMaxLaneMergeTime_s * mEndOfCurrentPathVelocity;
    timeStepsToDistance = ((EndOfPathDistanceCarInFront / (mEndOfCurrentPathVelocity)) / mTimeStep);
  }


  // Car is within Minimum Following Distance
  if (get_vehicle_behind_inLane(sensor_fusion,  mEndOfCurrentPathD + 4 * (Lane - getLane(mEndOfCurrentPathD)), laneVehicleBehind))
  {
    cout << "Vehicle Behind in lane " << Lane << laneVehicleBehind.mEndOfCurrentPathS - mEndOfCurrentPathS << " m" << endl;
    if (laneVehicleBehind.mEndOfCurrentPathS + mMinFollowDistance_m/2 >= mEndOfCurrentPathS)
    {
      cout << " Vehicle Behind TOO Close in lane" << Lane << endl;
      return false; // can't merge
    }
    else
    {
      if (laneVehicleBehind.mEndOfCurrentPathS + mMinFollowDistance_m/2 +
              timeStepsToDistance * mTimeStep * laneVehicleBehind.mEndOfCurrentPathVelocity >=
          mEndOfCurrentPathS + mEndOfCurrentPathVelocity * timeStepsToDistance * mTimeStep)
      {
        cout << "Vehicle Behind prediction " << (laneVehicleBehind.mEndOfCurrentPathS + mMinFollowDistance_m/2 +
            timeStepsToDistance * mTimeStep *
                                                     laneVehicleBehind.mEndOfCurrentPathVelocity) -
                                                (mEndOfCurrentPathS +
                                                 mEndOfCurrentPathVelocity * timeStepsToDistance * mTimeStep) << " m"
             << endl;
        cout << "Predicting would hit vehicle behind " << endl;
        return false;
      }
      else
      {
        cout << "Returning true vehicle behind" << endl;
        distanceAhead = EndOfPathDistanceCarInFront;
        return true;
      }
    }
  }
  else
  {
    cout << "No Vehicle Behind" << endl;
    distanceAhead = EndOfPathDistanceCarInFront;
    return true;
  }
}

bool Vehicle::getLeftGap(vector<vector<double>> sensor_fusion, double& leftGapCenter, double& leftLaneGapVelocity)
{

  // TODO refactor for lane as input
  // TODO rename prediction it really means end of current path
  double timeStepstoPredict = mNumTimeStepsToPredict;

  double EndOfPathDistanceCarInFront = 0;

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

  Vehicle leftVehicleAhead;
  Vehicle leftVehicleBehind;

  if (get_vehicle_ahead_inLane(sensor_fusion, mD - 4, leftVehicleAhead))
  {
    cout << "Vehicle Ahead in Left Lane " << leftVehicleAhead.mEndOfCurrentPathS - mEndOfCurrentPathS << " m" << endl;
    // How far in front at end of path
    EndOfPathDistanceCarInFront = min(leftVehicleAhead.mEndOfCurrentPathS - mEndOfCurrentPathS,
                                      mMaxLaneMergeTime_s * mEndOfCurrentPathVelocity);

    if (EndOfPathDistanceCarInFront <= mMinFollowDistance_m)
    {
      cout << " Left Lane Vehicle in Front Too Close " << endl;
      return false;
    }
    else
    {
      timeStepstoPredict = ((EndOfPathDistanceCarInFront / (mEndOfCurrentPathVelocity)) / mTimeStep);
      if (timeStepstoPredict * mTimeStep * leftVehicleAhead.mEndOfCurrentPathVelocity +
          leftVehicleAhead.mEndOfCurrentPathS - mMinLaneMergeTime_s * mEndOfCurrentPathVelocity <=
          mEndOfCurrentPathS + mEndOfCurrentPathVelocity * timeStepstoPredict * mTimeStep)
      {
        cout << "Predicting would hit vehicle Ahead" << endl;
        return false;
      }
    }
  }
  else
  {
    cout << "No Vehicle Ahead" << endl;
    EndOfPathDistanceCarInFront = mMaxLaneMergeTime_s * mEndOfCurrentPathVelocity;
    timeStepstoPredict = ((EndOfPathDistanceCarInFront / (mEndOfCurrentPathVelocity)) / mTimeStep);
  }


  // Car is within Minimum Following Distance
  if(get_vehicle_behind_inLane(sensor_fusion, mD - 4, leftVehicleBehind))
  {
    cout << "Vehicle Behind in left lane " << leftVehicleBehind.mEndOfCurrentPathS - mEndOfCurrentPathS << " m" << endl;
    if (leftVehicleBehind.mEndOfCurrentPathS + mMinFollowDistance_m >= mEndOfCurrentPathS)
    {
      cout << " Vehicle Behind TOO Close" << endl;
      return false; // can't merge
    }
    else
    {
      cout << " ELSE VEHICLE BEHIND" << endl;
      if (leftVehicleBehind.mEndOfCurrentPathS + mMinFollowDistance_m +
              timeStepstoPredict * mTimeStep * leftVehicleBehind.mEndOfCurrentPathVelocity >=
          mEndOfCurrentPathS + mEndOfCurrentPathVelocity * timeStepstoPredict * mTimeStep)
      {
        cout << "Vehicle Behind prediction " << (leftVehicleBehind.mEndOfCurrentPathS + mMinFollowDistance_m +
            timeStepstoPredict * mTimeStep *
                                                 leftVehicleBehind.mEndOfCurrentPathVelocity) -
                                                (mEndOfCurrentPathS +
                                                 mEndOfCurrentPathVelocity * timeStepstoPredict * mTimeStep) << " m"
             << endl;
        cout << "leftVehicleBehind.mEndOfCurrentPathS = " << leftVehicleBehind.mEndOfCurrentPathS << endl;
        cout << "mEndofCurrentPathS = " << mEndOfCurrentPathS << endl;
        cout << "mMinFollowDistance_m = " << mMinFollowDistance_m << endl;
        cout << "timeStepstoPredict = " << timeStepstoPredict << endl;
        cout << " mTimeStep = " << mTimeStep << endl;
        cout << " leftVehicleBehind.mEndOfCurrentPathVelocity " << leftVehicleBehind.mEndOfCurrentPathVelocity << endl;
        cout << " mEndOfCurrentPathVelocity = " << mEndOfCurrentPathVelocity << endl;
        cout << "Predicting would hit vehicle behind " << endl;
        return false;
      }
      else
      {
        cout << "Returning true vehicle behind" << endl;
        mNumTimeStepsToPredict = timeStepstoPredict;
        leftGapCenter = EndOfPathDistanceCarInFront;
        return true;
      }
    }
  }
  else
  {
    cout << "No Vehicle Behind" << endl;
    mNumTimeStepsToPredict = timeStepstoPredict;
    leftGapCenter = EndOfPathDistanceCarInFront;
    return true;
  }
/*

for (int i =0; i < sensor_fusion.size(); i++)
{
float d = sensor_fusion[i][6];
float s = sensor_fusion[i][5];
if( (d < (mD - 4 + mLaneWidth_m/2)) && (d> (mD - 4 - mLaneWidth_m/2)) && (s < mS +200) && (s > mS - 200))
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
*/



}
bool Vehicle::get_vehicle_ahead_inLane(vector<vector<double>> sensor_fusion, double lane, Vehicle & rVehicle) {
  /*
  Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
  rVehicle is updated if a vehicle is found.
  */

  // TODO rename prediction it really means end of current path
  double predictionTime = mPrevPathLength * mTimeStep; // This is the time to the front of the last predicted path

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

    if( (d < (lane + mLaneWidth_m/2)) && (d> (lane - mLaneWidth_m/2)) && (mS < s))
    {
      double velocity = sqrt(pow(sensor_fusion[i][3], 2) + pow(sensor_fusion[i][4], 2));
      if (found_vehicle)
      {
        if( (rVehicle.mEndOfCurrentPathS - mEndOfCurrentPathS) < ((s + velocity * predictionTime) -mEndOfCurrentPathS) )
        {
          continue;
        }
      }

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

// TODO one get Vehicle Call
bool Vehicle::get_vehicle_behind_inLane(vector<vector<double>> sensor_fusion, double lane, Vehicle & rVehicle) {
  /*
  Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
  rVehicle is updated if a vehicle is found.
  */

  // TODO rename prediction it really means end of current path
  double predictionTime = mPrevPathLength * mTimeStep; // This is the time to the front of the last predicted path

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

    if( (d < (lane + mLaneWidth_m/2)) && (d> (lane - mLaneWidth_m/2)) && (mS > s))
    {
      double velocity = sqrt(pow(sensor_fusion[i][3], 2) + pow(sensor_fusion[i][4], 2));
      if (found_vehicle)
      {
        if( (mEndOfCurrentPathS - rVehicle.mEndOfCurrentPathS ) < (mEndOfCurrentPathS - (s + velocity * predictionTime)) )
        {
          continue;
        }
      }

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
//  if (get_vehicle_ahead_inLane(predictions, lane, vehicle_ahead)) {
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