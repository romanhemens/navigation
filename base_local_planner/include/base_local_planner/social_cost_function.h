/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Morgan Quigley
 *********************************************************************/

#ifndef SOCIAL_COST_FUNCTION_H
#define SOCIAL_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>

namespace base_local_planner {

/**
 * This class provides a cost on how much a robot should behave
 * "socially" based on the result of our module. The cost is assigned
 * with the linear and the angular velocity. 
 */
class SocialCostFunction: public base_local_planner::TrajectoryCostFunction {
public:

  SocialCostFunction() {}
  ~SocialCostFunction() {}

  void setParams(int head_dir, int speed);
  void resetParams();
  double scoreTrajectory(Trajectory &traj);

  bool prepare() {return true;};

private:
  // Values published on /gemini_result (vlm_social_nav/SocialNavMsg).
  // New API (fine-grained):
  //   head_dir: 0..6  (hard_left, left, slight_left, straight, slight_right, right, hard_right)
  //   speed:    0..5  (stop, slow, maintain, cruise, speed_up, fast_sprint)
  //
  // -1 means "no social constraint".
  int head_dir_ = -1;
  int speed_ = -1;
};

} /* namespace base_local_planner */
#endif /* SOCIAL_COST_FUNCTION_H */
