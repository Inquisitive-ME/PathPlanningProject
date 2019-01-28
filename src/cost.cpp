#include "cost.h"
#include <iostream>
#include <math.h>

using namespace std;
//TODO: change weights for cost functions.
const float EFFICIENCY = 5;
const float LANE_COST = 1;
const float OBSTACLE_COST = 1;

/*
The weighted cost over all cost functions is computed in calculate_cost.
*/

float lane_cost(const int desired_lane, const int lane)
{
  // Divide by max lanes away from desired lane to have max cost of 1
  return (abs(desired_lane - lane)) / 2.0;
}

float obstacle_cost(const double finalVehicleSpeed, const double distanceAhead)
{
  // Penalize for close distance and high speed
  //return (finalVehicleSpeed/distanceAhead);
  return (10 / distanceAhead);
}

float inefficiency_cost(const double GoalSpeed, const double finalVehicleSpeed) {
  /*
   * already going to calculate the target speed based on checking if there are vehicles in the lane, so pass lane speed
  */

  float cost = (GoalSpeed - finalVehicleSpeed) / GoalSpeed;

  return cost;
}

float calculate_cost(const int desired_lane, const int lane, const double finalVehicleSpeed, const double GoalSpeed, const double distanceAhead) {
  /*
  Sum weighted cost functions to get total cost for trajectory.
  */
  cout << "Lane Cost = " << LANE_COST * lane_cost(desired_lane, lane) << endl;
  cout << "Obstacle Cost = " << OBSTACLE_COST * obstacle_cost(finalVehicleSpeed, distanceAhead) << endl;
  cout << "Efficiency Cost = " <<  EFFICIENCY * inefficiency_cost(GoalSpeed, finalVehicleSpeed) << endl;
  float cost = LANE_COST * lane_cost(desired_lane, lane) + OBSTACLE_COST * obstacle_cost(finalVehicleSpeed, distanceAhead) +
                EFFICIENCY * inefficiency_cost(GoalSpeed, finalVehicleSpeed);

  return cost;

}