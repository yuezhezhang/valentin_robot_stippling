#pragma once

#include "spdlog/spdlog.h"

#include <string>
#include <vector>
#include <map>
#include <PlanningSubroutines/Animation.h>

#include <Core/array.h>

#include "util.h"
#include "env_util.h"

typedef std::vector<arr> TaskPoses;
typedef std::map<std::string, std::vector<TaskPoses>> RobotTaskPoseMap;

typedef std::string Robot;
typedef std::pair<Robot, int> robot_task_pair;
typedef std::vector<robot_task_pair> TaskSequence;

const double VMAX = 0.05;

// this is the solution of one task
struct TaskPart {
  bool has_solution = false;

  TaskPart(const arr &_t, const arr &_path)
      : has_solution(true), t(_t), path(_path){};
  TaskPart(){};

  rai::Animation::AnimationPart anim;

  arr t;
  arr path;

  Robot r;
  uint task_index;

  std::string name;
  std::string algorithm;

  bool is_exit = false;
};

typedef std::map<Robot, std::vector<TaskPart>> Plan;

enum class PlanStatus { failed, aborted, success, unplanned };

struct PlanResult {
  PlanResult() : status(PlanStatus::unplanned) {}
  PlanResult(PlanStatus _status) : status(_status){};
  PlanResult(PlanStatus _status, const Plan &_plan)
      : status(_status), plan(_plan){};

  PlanStatus status;
  Plan plan;
};

double
get_makespan_from_plan(const std::map<Robot, std::vector<TaskPart>> &plan) {
  double max_time = 0.;
  for (const auto &robot_plan : plan) {
    const auto last_subpath = robot_plan.second.back();
    max_time = std::max({last_subpath.t(-1), max_time});
  }

  return max_time;
}

// TODO
std::map<Robot, std::vector<TaskPart>> reoptimize_plan(const std::map<Robot, std::vector<TaskPart>> &unscaled_plan){
  std::map<Robot, std::vector<TaskPart>> optimized_plan;
  return optimized_plan;
}

// TODO
/*std::map<Robot, std::vector<TaskPart>> rescale_plan(const std::map<Robot, std::vector<TaskPart>> &unscaled_plan){

  std::vector<double> scaling_factors;

  // go over all the robots, by assembling the complete animation, extract the position at times,
  // and make up a timing schedule that respects the velocity and acceleration limits.
  rai::Animation A;
  for (const auto &p : paths) {
    for (const auto path : p.second) {
      A.A.append(path.anim);
    }
  }

  const uint makespan = 0;
  for (uint t=0; t<makespan; ++t){
    const double per_robot_v = 0;
    const double per_robot_a = 0;

    const double max_v = 0;
    const double max_a = 0;

    const double scaling_factor_at_t = 0;


    scaling_factors.push_back(scaling_factor_at_t);
  }

  std::map<Robot, std::vector<TaskPart>> scaled_plan;
  for (const auto &robot_plan : unscaled_plan) {
    const auto robot = robot_plan.first;
    const auto task_parts = robot_plan.second;

    // resample t
    arr t;
    arr scaled_t;

    // rescale path
    arr path = timed_path.resample(scaled_t);
    
    // rebuild animation
    //rai::Animation::AnimationPart anim;
  }


  return scaled_plan;
}*/