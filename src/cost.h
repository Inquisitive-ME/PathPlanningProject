//
// Created by richard on 1/21/19.
//

#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H

float calculate_cost(const int desired_lane, const int lane, const double finalVehicleSpeed, const double GoalSpeed, const double distanceAhead);

#endif //PATH_PLANNING_COST_H
