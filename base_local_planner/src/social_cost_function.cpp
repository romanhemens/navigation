/*
 * social_cost_function.cpp
 *
 *  Created on: Apr 20, 2016
 *      Author: Morgan Quigley
 */

#include <base_local_planner/social_cost_function.h>

#include <math.h>

namespace base_local_planner {

void SocialCostFunction::setParams(int head_dir, int speed) {
  head_dir_ = head_dir; // LEFT 0, STRAIGHT 1, RIGHT 2
  speed_ = speed;       // SLOW DOWN 0, SPEED UP 1, MAINTAIN 2, STOP 3
}

void SocialCostFunction::resetParams() {
  head_dir_ = -1; // LEFT 0, STRAIGHT 1, RIGHT 2
  speed_ = -1;       // SLOW DOWN 0, SPEED UP 1, MAINTAIN 2, STOP 3
}

double SocialCostFunction::scoreTrajectory(Trajectory &traj) {
    const double TURN_COST_WEIGHT = 10.0;
    const double SPEED_COST_WEIGHT = 5.0;
    double cost = 0.0;

    //  no social cost recieved
    if (head_dir_ < 0 || speed_ < 0)
        return 0;

    for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
        // Get the trajectory point (x, y, th)
        double px, py, pth, desired_theta, desired_speed;
        //////////////////////////////// lateral
        // preference on the left
        if (head_dir_ == 0) 
            desired_theta = 0.2;
        // preference on the straight
        else if (head_dir_ == 1) 
            desired_theta = 0;
        // preference on the right
        else if (head_dir_ == 2) 
            desired_theta = -0.2;

        cost += TURN_COST_WEIGHT * fabs(desired_theta - traj.thetav_);

        ////////////////////////////////// speed
        // slow down
        if (speed_ == 0) 
            desired_speed = 0.3;
        // speed up
        else if (speed_ == 1) 
            desired_speed = 0.6;
        // maintain
        else if (speed_ == 2)
            desired_speed = 0.4;// do nothing
        // stop
        else if (speed_ == 3) 
            return fabs(traj.xv_)*100;

        cost += SPEED_COST_WEIGHT * fabs(desired_speed - traj.xv_);
    }

    return cost;
}

} /* namespace base_local_planner */
