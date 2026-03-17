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
  head_dir_ = head_dir;
  speed_ = speed;
}

void SocialCostFunction::resetParams() {
  head_dir_ = -1;
  speed_ = -1;
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
        double desired_theta = 0.0;
        double desired_speed = 0.0;

        //////////////////////////////// lateral (head_dir 0..6)
        // 0 hard_left, 1 left, 2 slight_left, 3 straight, 4 slight_right, 5 right, 6 hard_right
        switch (head_dir_) {
          // Tuned for typical DWA params where theta_stopped_vel ~ 0.2 rad/s and max_vel_theta ~ 1.2 rad/s:
          // - slight_* should be clearly "turning" (> theta_stopped_vel)
          // - hard_* should be a strong turn but still below max_vel_theta
          case 0: desired_theta =  0.85; break;  // hard_left
          case 1: desired_theta =  0.55; break;  // left
          case 2: desired_theta =  0.25; break;  // slight_left
          case 3: desired_theta =  0.00; break;
          case 4: desired_theta = -0.25; break;  // slight_right
          case 5: desired_theta = -0.55; break;  // right
          case 6: desired_theta = -0.85; break;  // hard_right
          default:
            // Unknown head_dir: don't apply lateral preference.
            desired_theta = 0.0;
            break;
        }

        cost += TURN_COST_WEIGHT * fabs(desired_theta - traj.thetav_);

        ////////////////////////////////// speed (speed 0..5)
        // 0 stop, 1 slow, 2 maintain, 3 cruise, 4 speed_up, 5 fast_sprint
        switch (speed_) {
          case 0:
            // Strongly penalize any forward motion when we should stop.
            return fabs(traj.xv_) * 100.0;
          case 1: desired_speed = 0.40; break;
          case 2: desired_speed = 0.60; break;
          case 3: desired_speed = 0.9; break;
          case 4: desired_speed = 1.2; break;
          case 5: desired_speed = 1.6; break;
          default:
            // Unknown speed: don't apply speed preference.
            desired_speed = traj.xv_;
            break;
        }

        cost += SPEED_COST_WEIGHT * fabs(desired_speed - traj.xv_);
    }

    return cost;
}

} /* namespace base_local_planner */
