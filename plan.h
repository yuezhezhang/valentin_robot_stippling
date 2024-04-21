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