#pragma once
#include "plan.h"

const double VMAX = 0.05;

double compute_lb_for_sequence(const TaskSequence &seq,
                               const RobotTaskPoseMap &rtpm,
                               const std::map<Robot, arr> &start_poses,
                               const uint start_index = 0,
                               const std::map<Robot, double> start_times = {}) {
  // the lower bound can be computed by finding the minimum time
  // of the current task, and using the precedence constraints as well.
  std::map<Robot, double> robot_time = start_times;
  std::map<Robot, arr> robot_pos = start_poses;

  for (uint i = start_index; i < seq.size(); ++i) {
    const auto task_tuple = seq[i];
    const auto robot = task_tuple.first;
    const auto task_index = task_tuple.second;

    // std::cout << robot << std::endl;

    if (!robot_time.count(robot)) {
      robot_time[robot] = 0.;
    }

    std::cout << robot << " " << task_index << std::endl;

    const arr start_pose = robot_pos[robot];
    const arr goal_pose = rtpm.at(robot)[task_index][0];

    // const arr dist = goal_pose - start_pose;
    // std::cout << goal_pose - start_pose << std::endl;
    const double max_dist = absMax(goal_pose - start_pose);
    const double max_acc = 0.1;
    const double time_to_accelerate = VMAX / max_acc;
    const double acc_dist = 0.5 * VMAX * VMAX / max_acc * 2;

    double dt = 0;
    if (acc_dist > max_dist) {
      // this is wrong
      std::cout << "using acc. timing only" << std::endl;
      dt = 2 * time_to_accelerate;
    } else {
      std::cout << "using acc. timing and max-vel" << std::endl;
      dt = (max_dist - acc_dist) / VMAX + 2 * time_to_accelerate;
    }

    robot_time[robot] += dt;

    for (const auto &rt : robot_time) {
      if (robot_time[robot] < rt.second) {
        robot_time[robot] = rt.second;
      }
    }

    robot_pos[robot] = goal_pose;
  }

  double max = 0;
  for (const auto rt : robot_time) {
    if (max < rt.second) {
      max = rt.second;
    }
  }

  return max;
}
