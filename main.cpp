#include <KOMO/komo.h>
#include <Kin/F_operators.h>
#include <Kin/F_pose.h>
#include <Kin/F_qFeatures.h>
#include <Kin/featureSymbols.h>
#include <Kin/kin.h>

#include <Kin/kinViewer.h>
#include <Kin/viewer.h>

#include <Manip/rrt-time.h>
#include <PlanningSubroutines/ConfigurationProblem.h>

#include <iomanip>
#include <numeric>

#include <algorithm>
#include <chrono>
#include <random>

#include <math.h>

#include <GL/gl.h>
#include <Gui/opengl.h>

#include <PlanningSubroutines/Animation.h>
#include <PlanningSubroutines/ConfigurationProblem.h>

#include "planners/prioritized_planner.h"
#include "searchers/sequencing.h"
#include "line.h"
#include "util.h"
#include "env_util.h"
#include "stippling_util.h"
#include "path_util.h"

// TODO:
// - fix loading and visualization of previously computed paths
// - time-rescale path
// - split main planning subroutine
// - squeaky wheel planner
// - enable things that are not only 'go to point', e.g. drawing a line
// - enable multi-arm cooperation
// - enable search over sequences with precendence constraints
// - look into more complex motion planning:
// -- joint optimization
// -- constrained sampling based planning

arr sampleConfigurationForRobot(KOMO &komo, const arr &point,
                                const std::string &prefix, bool rnd = false,
                                bool ineq = false) {
  OptOptions options;
  options.stopIters = 100;
  options.damping = 1e-3;

  komo.setDiscreteOpt(1);

  // komo.world.stepSwift();

  komo.add_collision(true, .01, 1e1);
  komo.add_jointLimits(true, 0., 1e1);

  komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")}, OT_eq,
                    {1e2}, point);

  if (!ineq) {
    komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
                      {1e1}, {0., 0., -1.});
  } else {
    komo.addObjective({1.}, FS_scalarProductZZ,
                      {STRING(prefix << "pen"), "world"}, OT_ineq, {1e1},
                      {-cos(15 * 3.1415 / 180.)});
  }
  // komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
  // {1e1}, {0., 0., -1.}); komo.addObjective({1.}, FS_vectorZDiff,
  // {STRING(prefix << "pen"), "world"}, OT_ineq, {1e1}, {0., 0., -0.9});
  ConfigurationProblem cp(komo.world);
  setActive(cp.C, prefix);

  for (uint i = 0; i < 10; ++i) {
    if (rnd) {
      komo.run_prepare(0.1, false);
    } else {
      komo.run_prepare(0.0, true);
    }
    komo.run(options);

    const arr q = komo.getPath()[0]();
    // komo.pathConfig.watch(true);

    // ensure via sampling as well
    const bool res = cp.query(q)->isFeasible;

    if (res && komo.getReport(false).get<double>("ineq") < 1. &&
        // komo.getReport(false).get<double>("sos") < 0.5 &&
        komo.getReport(false).get<double>("eq") < 1.) {
      // komo.pathConfig.watch(true);
      return q;
    }
  }

  std::cout << "failed for pt " << point << std::endl;
  return {};
}

arr sampleConfigurationForRobot(rai::Configuration &C, const arr &point,
                                const std::string &prefix) {
  // activate agents
  setActive(C, prefix);

  OptOptions options;
  options.stopIters = 100;
  options.damping = 1e-3;

  KOMO komo;

  komo.verbose = 0;

  // set up komo problem
  komo.setModel(C, true);

  return sampleConfigurationForRobot(komo, point, prefix);
}

std::vector<arr> computeConfigurationsForPoints(const arr &pts,
                                                rai::Configuration &C,
                                                const std::string prefix) {
  std::vector<arr> configurations;

  const auto start = std::chrono::high_resolution_clock::now();

  // activate agents
  setActive(C, prefix);

  const auto home = C.getJointState();

  KOMO komo;
  komo.verbose = 0;
  komo.setModel(C, true);

  for (uint i = 0; i < pts.d0; ++i) {
    const arr pt = {pts[i](0), pts[i](1), 0.075};
    const arr q = sampleConfigurationForRobot(
        komo, C["table"]->getPosition() + pt, prefix);
    // const arr q = sampleConfigurationForRobot(C, C["table"]->getPosition() +
    // pt, prefix);
    configurations.push_back(q);

    // C.setJointState(q);
    //
    komo.clearObjectives();
    komo.world.setJointState(home);
  }
  C.setJointState(home);

  const auto stop = std::chrono::high_resolution_clock::now();
  const auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Duration of IK pose computation: " << duration.count() / 1000.
            << " ms (per point: " << duration.count() / 1000. / pts.d0 << "ms )"
            << std::endl;

  return configurations;
}

void drawPts(rai::Configuration C, arr pts, arr color = {0., 0., 0., 1.}) {
  for (uint i = 0; i < pts.d0; ++i) {
    const arr pt = {pts[i](0), pts[i](1), 0.075};

    auto *dot = C.addFrame("goal", "table");
    dot->setShape(rai::ST_sphere, {0.005});
    dot->setRelativePosition({pts[i](0), pts[i](1), 0.05});
    dot->setContact(0.);
    dot->setColor(color);
  }

  C.gl()->displayCamera().setPosition(0, 0., 3);
  C.gl()->displayCamera().focusOrigin();

  C.watch(true);
}

// overloaded for both
void drawPts(rai::Configuration C, std::map<uint, arr> tmp,
             arr color = {0., 0., 0., 1.}) {
  for (auto element : tmp) {
    uint j = element.first;
    arr pts = element.second;

    for (uint i = 0; i < pts.d0; ++i) {
      const arr pt = {pts[i](0), pts[i](1), 0.075};

      auto *dot = C.addFrame("goal", "table");
      dot->setShape(rai::ST_sphere, {0.01});
      dot->setRelativePosition({pts[i](0), pts[i](1), 0.05});
      dot->setContact(0.);
      dot->setColor({1. * j, 1. * j, 1. * j, 1.});
    }
  }

  C.watch(true);
}

Plan plan_multiple_arms_unsynchronized(rai::Configuration &C,
                                       const RobotTaskPoseMap &rtpm,
                                       const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();

  const auto seq = generate_random_sequence(robots, num_tasks);

  // plan for it
  const auto plan_result =
      plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);
  return plan_result.plan;
}

arr get_robot_pose_at_time(const uint t, const Robot r,
                           const std::map<Robot, arr> &home_poses,
                           const std::map<Robot, std::vector<TaskPart>> &plan) {
  if (plan.count(r) > 0) {
    for (const auto &part : plan.at(r)) {
      // std::cout <<part.t(0) << " " << part.t(-1) << std::endl;
      if (part.t(0) > t || part.t(-1) < t) {
        continue;
      }

      for (uint i = 0; i < part.t.N; ++i) {
        if (part.t(i) == t) {
          return part.path[i];
          // std::cout <<part.path[i] << std::endl;
        }
      }
    }
  }

  return home_poses.at(r);
}

void visualize_plan(rai::Configuration C, const Plan &plan,
                    const bool save = false, 
                    const char* save_video_path = "video/") {
  rai::Animation A;
  for (const auto &p : plan) {
    for (const auto path : p.second) {
      A.A.append(path.anim);
    }
  }

  rai::ConfigurationViewer Vf;
  // Vf.setConfiguration(C, "\"Real World\"", true);
  Vf.setConfiguration(C, "\"Real World\"", false);

  const double makespan = get_makespan_from_plan(plan);

  if (save) {
      rai::system(STRING("mkdir -p " <<save_video_path));
  }

  for (uint t = 0; t < makespan; ++t) {
    // A.setToTime(C, t);

    // std::cout << t << std::endl;
    for (const auto tp : plan) {
      const auto r = tp.first;
      const auto parts = tp.second;

      bool done = false;
      for (auto part : parts) {
        // std::cout <<part.t(0) << " " << part.t(-1) << std::endl;
        if (part.t(0) > t || part.t(-1) < t) {
          continue;
        }

        for (uint i = 0; i < part.t.N - 1; ++i) {
          if (part.t(i) <= t && part.t(i + 1) > t) {
            setActive(C, r);
            C.setJointState(part.path[i]);
            // std::cout <<part.path[i] << std::endl;
            done = true;

            // set bin picking things
            const auto task_index = part.task_index;
            const auto obj_name = STRING("obj" << task_index + 1);

            if (part.anim.frameNames.contains(obj_name)) {
              const auto pose =
                  part.anim.X[uint(std::floor(t - part.anim.start))];
              arr tmp(1, 7);
              tmp[0] = pose[-1];
              C.setFrameState(tmp, {C[obj_name]});
            }
            break;
          }
        }

        if (done) {
          break;
        }
      }
    }

    // C.watch(false);
    Vf.setConfiguration(C, ".", false);
    rai::wait(0.01);

    if (save) {
      Vf.savePng(save_video_path);
    }
  }
}

void export_plan(const std::vector<Robot> &robots,
                 const std::map<Robot, arr> &home_poses,
                 const std::map<Robot, std::vector<TaskPart>> &plan,
                 const TaskSequence &seq, const std::string base_folder,
                 const uint iteration, const uint computation_time) {
  std::cout << "exporting plan" << std::endl;
  // make folder
  const std::string folder =
      "./out/" + base_folder + "/" + std::to_string(iteration) + "/";
  const int res = system(STRING("mkdir -p " << folder).p);
  (void)res;

  rai::Animation A;
  for (const auto &p : plan) {
    for (const auto path : p.second) {
      A.A.append(path.anim);
    }
  }

  // - add info
  // -- comp. time
  {
    std::ofstream f;
    f.open(folder + "comptime.txt", std::ios_base::trunc);
    f << computation_time;
  }

  // -- makespan
  {
    std::ofstream f;
    f.open(folder + "makespan.txt", std::ios_base::trunc);
    f << A.getT();
  }

  // -- sequence
  {
    std::ofstream f;
    f.open(folder + "sequence.txt", std::ios_base::trunc);
    for (const auto &s : seq) {
      f << "(" << s.first << " " << s.second << ")";
    }
  }

  // -- plan
  {
    std::ofstream f;
    f.open(folder + "plan.txt", std::ios_base::trunc);
    for (const auto per_robot_plan : plan) {
      const auto robot = per_robot_plan.first;
      const auto tasks = per_robot_plan.second;

      f << robot << ": ";
      for (const auto task : tasks) {
        f << task.name << "(" << task.algorithm << ")"
          << " " << task.task_index << ", " << task.t(0) << ", " << task.t(-1)
          << "; ";
      }
      f << std::endl;
    }
  }

  // -- actual path
  {
    std::ofstream f;
    f.open(folder + "robot_controls.txt", std::ios_base::trunc);
    arr path(A.getT(), home_poses.at(robots[0]).d0 * robots.size());
    for (uint i = 0; i < A.getT(); ++i) {
      uint offset = 0;
      for (uint j = 0; j < robots.size(); ++j) {
        const arr pose = get_robot_pose_at_time(i, robots[j], home_poses, plan);
        for (uint k = 0; k < pose.N; ++k) {
          path[i](k + offset) = pose(k);
        }
        offset += pose.N;
      }
    }

    f << path;
  }
}

Plan plan_multiple_arms_random_search(rai::Configuration &C,
                                      const RobotTaskPoseMap &rtpm,
                                      const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();

  TaskSequence best_seq;
  Plan best_plan;
  double best_makespan = 1e6;

  for (uint i = 0; i < 100; ++i) {
    const auto seq = generate_random_sequence(robots, num_tasks);

    const double lb = compute_lb_for_sequence(seq, rtpm, home_poses);
    std::cout << "LB for sequence " << lb << std::endl;
    for (auto s : seq) {
      std::cout << "(" << s.first << " " << s.second << ")";
    }
    std::cout << std::endl;

    if (lb > best_makespan) {
      continue;
    }

    // plan for it
    const auto plan_result = plan_multiple_arms_given_sequence(
        C, rtpm, seq, home_poses, best_makespan);
    if (plan_result.status == PlanStatus::success) {
      const Plan plan = plan_result.plan;
      const double makespan = get_makespan_from_plan(plan);

      std::cout << "\n\n\nMAKESPAN " << makespan << " best so far "
                << best_makespan << std::endl;
      for (auto s : seq) {
        std::cout << "(" << s.first << " " << s.second << ")";
      }
      std::cout << "\n\n\n" << std::endl;

      if (makespan < best_makespan) {
        best_makespan = makespan;
        best_plan = plan;
        break;
      }
    }
  }
  return best_plan;
}

Plan plan_multiple_arms_squeaky_wheel(
    rai::Configuration &C, const RobotTaskPoseMap &rtpm,
    const std::map<Robot, arr> &home_poses) {
}

Plan plan_multiple_arms_greedy_random_search(
    rai::Configuration &C, const RobotTaskPoseMap &rtpm,
    const std::map<Robot, arr> &home_poses) {
  // make foldername for current run
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  std::stringstream buffer;
  buffer << "greedy_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();

  TaskSequence best_seq;
  Plan best_plan;
  double best_makespan = 1e6;

  auto start_time = std::chrono::high_resolution_clock::now();

  std::vector<std::pair<TaskSequence, Plan>> cache;

  uint iter = 0;
  bool early_stop = false;
  for (uint i = 0; i < 20000; ++i) {
    if (early_stop) {
      break;
    }
    std::cout << "Generating completely new seq. " << i << std::endl;
    TaskSequence seq;
    // seq = generate_alternating_random_sequence(robots, num_tasks, rtpm);
    // seq = generate_single_arm_sequence(robots, num_tasks);
    seq = generate_alternating_greedy_sequence(robots, num_tasks, rtpm,
                                               home_poses);
    /*if (true || i == 0) {
      // seq = generate_single_arm_sequence(robots, num_tasks);
      //seq = generate_random_sequence(robots, num_tasks);
      // seq = generate_alternating_greedy_sequence(robots, num_tasks, rtpm,
    home_poses); } else if (i == 1) { seq =
    generate_alternating_random_sequence(robots, num_tasks, rtpm); } else if (i
    == 2) { seq = generate_single_arm_sequence(robots, num_tasks); } else { seq
    = generate_random_sequence(robots, num_tasks);
    }*/

    if (!sequence_is_feasible(seq, rtpm)) {
      std::cout << "Generated sequence no feasible" << std::endl;
      continue;
    }

    Plan plan;
    double prev_makespan = 1e6;
    for (uint j = 0; j < 30; ++j) {
      ++iter;
      TaskSequence new_seq = seq;
      while (true) {
        if (j > 0) {
          new_seq = neighbour(seq, robots);
        }

        // ensure that sequence is actually feasible, i.e. robots can do the
        // assigned tasks
        if (sequence_is_feasible(new_seq, rtpm)) {
          break;
        }
      }

      const double lb = compute_lb_for_sequence(new_seq, rtpm, home_poses);
      std::cout << "LB for sequence " << lb << std::endl;
      for (const auto &s : new_seq) {
        std::cout << "(" << s.first << " " << s.second << ")";
      }
      std::cout << std::endl;

      if (lb > best_makespan) {
        std::cout << "skipping planning, since lb is larger than best plan"
                  << std::endl;
        continue;
      }

      // plan for it
      PlanResult new_plan_result;
      if (plan.empty()) {
        new_plan_result = plan_multiple_arms_given_sequence(
            C, rtpm, new_seq, home_poses, prev_makespan);
      } else {
        // compute index where the new sequence starts
        uint change_in_sequence = 0;
        for (uint k = 0; k < seq.size(); ++k) {
          if (seq[k].first != new_seq[k].first ||
              seq[k].second != new_seq[k].second) {
            change_in_sequence = k;
            break;
          }
        }
        std::cout << "planning only subsequence " << change_in_sequence
                  << std::endl;
        new_plan_result = plan_multiple_arms_given_subsequence_and_prev_plan(
            C, rtpm, new_seq, change_in_sequence, plan, home_poses,
            prev_makespan);
      }

      if (new_plan_result.status == PlanStatus::success) {
        const Plan new_plan = new_plan_result.plan;
        const double makespan = get_makespan_from_plan(new_plan);

        const auto end_time = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                                  end_time - start_time)
                                  .count();

        // cache.push_back(std::make_pair(new_seq, new_plan));

        export_plan(robots, home_poses, new_plan, new_seq, buffer.str(), iter,
                    duration);

        std::cout << "\n\n\nMAKESPAN " << makespan << " best so far "
                  << best_makespan << " (" << prev_makespan << ")" << std::endl;
        for (const auto &s : new_seq) {
          std::cout << "(" << s.first << " " << s.second << ")";
        }
        std::cout << "\n\n\n" << std::endl;

        if (makespan < prev_makespan) {
          seq = new_seq;
          plan = new_plan;
          prev_makespan = makespan;

          // visualize_plan(C, plan, save_video, save_video_path); //!
          // early_stop = true;
          // break;
        }

        if (makespan < best_makespan) {
          best_makespan = makespan;
          best_plan = plan;
          best_seq = new_seq;
          early_stop = true;
          break;

          // visualize_plan(C, best_plan);
        }
      } else {
        const std::string folder =
            "./out/" + buffer.str() + "/" + std::to_string(iter) + "/";
        const int res = system(STRING("mkdir -p " << folder).p);
        (void)res;

        {
          std::ofstream f;
          f.open(folder + "comptime.txt", std::ios_base::trunc);
          const auto end_time = std::chrono::high_resolution_clock::now();
          const auto duration =
              std::chrono::duration_cast<std::chrono::seconds>(end_time -
                                                               start_time)
                  .count();
          f << duration;
        }
        {
          std::ofstream f;
          if (new_plan_result.status == PlanStatus::failed) {
            f.open(folder + "failed.txt", std::ios_base::trunc);
          } else if (new_plan_result.status == PlanStatus::aborted) {
            f.open(folder + "aborted.txt", std::ios_base::trunc);
          }
        }
      }
      if (new_plan_result.status == PlanStatus::failed) {
        break;
      }
    }
  }
  return best_plan;
}

Plan plan_multiple_arms_simulated_annealing(
    rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (const auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();
  const auto seq = generate_random_sequence(robots, num_tasks);

  // plan for it
  const auto plan_result =
      plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);

  auto best_plan = plan_result.plan;
  uint best_makespan = get_makespan_from_plan(plan_result.plan);

  uint curr_makespan = best_makespan;

  auto p = [](const double e, const double eprime, const double temperature) {
    if (eprime < e) {
      return 1.;
    }

    return exp(-(eprime - e) / temperature);
  };

  const uint max_iter = 1000;
  const double T0 = 1e6;

  double T = T0;
  double cooling_factor = 0.999;

  std::vector<uint> best_makespan_at_iteration;
  std::vector<double> computation_time_at_iteration;

  for (uint i = 0; i < max_iter; ++i) {
    // T = T0 * (1 - (i+1.)/nmax);
    T = T * cooling_factor; // temp(i);

    // modify sequence
    const TaskSequence seq_new = neighbour(seq, robots);

    // compute lower bound
    double lb_makespan = compute_lb_for_sequence(seq_new, rtpm, home_poses);

    arr rnd(1);
    rndUniform(rnd);

    if (p(curr_makespan, lb_makespan, T) > rnd(0)) {
      const auto new_plan =
          plan_multiple_arms_given_sequence(C, rtpm, seq_new, home_poses).plan;

      uint makespan = get_makespan_from_plan(new_plan);

      if (p(curr_makespan, lb_makespan, T) > rnd(0)) {
        curr_makespan = makespan;
      }

      if (makespan < best_makespan) {
        best_makespan = makespan;
        best_plan = new_plan;
        break;
      }
    }

    best_makespan_at_iteration.push_back(best_makespan);
    computation_time_at_iteration.push_back(i);
  }
  return best_plan;
}

RobotTaskPoseMap
compute_pick_and_place_positions(rai::Configuration &C,
                                 const std::vector<std::string> &robots,
                                 const uint n = 6) {
  RobotTaskPoseMap rtpm;

  for (const auto prefix : robots) {
    for (uint i = 0; i < n; ++i) {
      setActive(C, prefix);

      const auto home = C.getJointState();

      OptOptions options;
      options.stopIters = 100;
      options.damping = 1e-3;

      KOMO komo;
      komo.verbose = 0;
      komo.setModel(C, true);

      komo.setDiscreteOpt(2);

      // komo.world.stepSwift();

      komo.add_collision(true, .01, 1e1);
      komo.add_jointLimits(true, 0., 1e1);

      auto pen_tip = STRING(prefix << "pen_tip");
      auto obj = STRING("obj" << i + 1);
      auto goal = STRING("goal" << i + 1);

      Skeleton S = {
          {1., 1., SY_touch, {pen_tip, obj}},
          {1., 2., SY_stable, {pen_tip, obj}},
          {2., 2., SY_poseEq, {obj, goal}},
      };

      komo.setSkeleton(S);

      // komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")},
      // OT_eq,
      //                  {1e2}, point);
      // komo.addObjective({1., 1.}, FS_distance, {STRING(prefix << "pen_tip"),
      // STRING(obj << i + 1)}, OT_eq, {1e1});

      komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
                        {1e1}, {0., 0., -1.});
      // komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")},
      // OT_sos, {1e0}, C[obj]->getPosition());

      // komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
      // {1e1}, {0., 0., -1.}); komo.addObjective({1.}, FS_vectorZDiff,
      // {STRING(prefix << "pen"), "world"}, OT_ineq, {1e1}, {0., 0., -0.9});
      ConfigurationProblem cp(komo.world);
      setActive(cp.C, prefix);

      for (uint i = 0; i < 10; ++i) {
        komo.run_prepare(0.0, true);
        komo.run(options);

        const arr q0 = komo.getPath()[0]();
        const arr q1 = komo.getPath()[1]();
        // komo.pathConfig.watch(true);

        // ensure via sampling as well
        const bool res1 = cp.query(q0)->isFeasible;
        const bool res2 = cp.query(q1)->isFeasible;

        if (res1 && res2 && komo.getReport(false).get<double>("ineq") < 1. &&
            komo.getReport(false).get<double>("eq") < 1.) {
          rtpm[prefix].push_back({q0, q1});
          break;
        } else {
          std::cout << "failed for a bit" << std::endl;
        }
      }
    }
  }

  return rtpm;
}

RobotTaskPoseMap
compute_stippling_poses_for_arms(rai::Configuration &C, const arr &pts,
                                 const std::vector<Robot> &robots) {
  RobotTaskPoseMap rtpm;
  for (const Robot &r : robots) {
    const TaskPoses poses = computeConfigurationsForPoints(pts, C, r);
    std::vector<TaskPoses> tp;
    for (const arr &p : poses) {
      if (p.N > 0) {
        tp.push_back({p});
      } else {
        tp.push_back({});
      }
    }
    rtpm[r] = tp;
  }

  return rtpm;
}

std::map<Robot, arr> get_robot_home_poses(rai::Configuration &C,
                                          const std::vector<Robot> &robots) {
  std::map<Robot, arr> poses;
  for (auto r : robots) {
    setActive(C, r);
    poses[r] = C.getJointState();

    // std::cout << poses[r] << std::endl;
  }

  return poses;
}

void load_and_viz(rai::Configuration C, const bool pick_and_place) {
  arr path;
  // const std::string filepath =
  // "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid/greedy_20230328_000824/139/robot_controls.txt";
  // const std::string filepath =
  // "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lis/greedy_20230329_000242/86/robot_controls.txt";
  // const std::string filepath =
  // "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lis/greedy_20230329_103317/1/robot_controls.txt";

  // for bin picking:
  // robot, obj, start, end
  // for points
  // robot end
  // const std::string filepath =
  // "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/bin/greedy_20230328_211219/29/robot_controls.txt";
  /*std::vector<std::vector<uint>> timings;
  timings.push_back({0, 0, 22, 53});
  timings.push_back({0, 5, 92, 137});
  timings.push_back({0, 3, 151, 214});
  timings.push_back({0, 2, 233, 263});

  timings.push_back({1, 1, 110, 179});
  timings.push_back({1, 4, 218, 277});*/

  /*const std::string filepath =
  "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/bin/greedy_20230330_005636/19/robot_controls.txt";
  std::vector<std::vector<uint>> timings;
  timings.push_back({0, 5, 24, 69});
  timings.push_back({0, 0, 107, 138});
  timings.push_back({0, 3, 159, 220});
  timings.push_back({0, 2, 239, 269});

  timings.push_back({1, 1, 111, 170});
  timings.push_back({1, 4, 221, 269});*/

  /*
  const std::string filepath =
  "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/opt/greedy_20230331_020353/3/robot_controls.txt";
  std::vector<std::vector<uint>> timings;
  timings.push_back({0, 39});
  timings.push_back({0, 101});
  timings.push_back({0, 184});
  timings.push_back({0, 252});
  timings.push_back({0, 295});

  timings.push_back({1, 56});
  timings.push_back({1, 143});
  timings.push_back({1, 207});
  timings.push_back({1, 260});

  timings.push_back({2, 76});
  timings.push_back({2, 157});
  timings.push_back({2, 221});

  timings.push_back({3, 32});
  timings.push_back({3, 89});
  timings.push_back({3, 172});
  timings.push_back({3, 221});
  */

  // greedy
  /*const std::string filepath =
  "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/opt/greedy_20230331_165530/1/robot_controls.txt";
  std::vector<std::vector<uint>> timings;
  timings.push_back({0, 25});
  timings.push_back({0, 64});
  timings.push_back({0, 123});
  timings.push_back({0, 214});

  timings.push_back({1, 46});
  timings.push_back({1, 68});
  timings.push_back({1, 142});
  timings.push_back({1, 225});

  timings.push_back({2, 48});
  timings.push_back({2, 97});
  timings.push_back({2, 156});

  timings.push_back({3, 26});
  timings.push_back({3, 65});
  timings.push_back({3, 96});
  timings.push_back({3, 182});
  timings.push_back({3, 235});*/

  // opt
  const std::string filepath =
      "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/"
      "greedy_20230401_010325/17/robot_controls.txt";
  std::vector<std::vector<uint>> timings;
  timings.push_back({0, 25});
  timings.push_back({0, 52});
  timings.push_back({0, 66});
  timings.push_back({0, 94});
  timings.push_back({0, 105});
  timings.push_back({0, 172});

  timings.push_back({1, 116});
  timings.push_back({1, 171});

  timings.push_back({2, 34});
  timings.push_back({2, 60});
  timings.push_back({2, 141});

  timings.push_back({3, 26});
  timings.push_back({3, 35});
  timings.push_back({3, 53});
  timings.push_back({3, 59});
  timings.push_back({3, 153});
  timings.push_back({3, 194});

  FILE(filepath.c_str()) >> path;

  const std::vector<Robot> robots{"a0_", "a1_", "a2_", "a3_"};
  // const std::vector<Robot> robots{"a0_", "a1_"};

  setActive(C, robots);

  rai::ConfigurationViewer Vf;
  Vf.setConfiguration(C, "\"Real World\"", true);

  for (uint i = 0; i < path.d0; ++i) {
    C.setJointState(path[i]);
    // C.watch(false);
    // rai::wait(0.01);

    if (pick_and_place) {
      for (uint j = 0; j < timings.size(); ++j) {
        const auto pen_tip = STRING("a" << timings[j][0] << "_pen_tip");
        const auto obj = STRING("obj" << timings[j][1] + 1);

        if (i == timings[j][2]) {
          auto from = C[pen_tip];
          auto to = C[obj];

          to->unLink();

          // create a new joint
          to->linkFrom(from, true);
          // to->joint->makeRigid();
        }

        if (i == timings[j][3]) {
          auto to = C[obj];
          auto from = C["table_base"];

          to->unLink();

          // create a new joint
          to->linkFrom(from, true);
          // to->joint->makeRigid();
        }
      }
    } else {
      // draw dots
      for (uint j = 0; j < timings.size(); ++j) {
        const auto pen_tip = STRING("a" << timings[j][0] << "_pen_tip");
        const arr pos = C[pen_tip]->getPosition();

        if (timings[j][1] == i) {
          // add small sphere
          auto *dot = C.addFrame("goal", "table");
          dot->setShape(rai::ST_sphere, {0.01});
          dot->setPosition(pos);
          dot->setContact(0.);

          if (timings[j][0] == 0) {
            dot->setColor({0, 0., 0., 0.5});
          } else if (timings[j][0] == 1) {
            dot->setColor({1., 0., 0., 0.5});
          } else if (timings[j][0] == 2) {
            dot->setColor({1., 0., 1., 0.5});
          } else if (timings[j][0] == 3) {
            dot->setColor({1., 1., 0., 0.5});
          }
        }
      }
    }
    Vf.setConfiguration(C, ".", false);
    rai::wait(0.01);
    Vf.savePng();
  }
}

arr get_scenario(const rai::String &str) {
  // const arr pts = grid(2, 2, 0.4, 0.1);
  // const arr pts = grid(2, 3, 0.4, 0.1);

  arr pts;
  if (str == "default_grid") {
    pts = grid();
  } else if (str == "four_by_four_grid") {
    pts = grid(4, 4);
  } else if (str == "three_by_three_grid") {
    pts = grid(3, 3);
  } else if (str == "three_by_two_grid") {
    pts = grid(3, 2);
  } else if (str == "spiral") {
    pts = spiral();
  } else if (str == "random") {
    pts = randomPts();
  } else if (str == "cube") {
    pts = cube(200);
  } else if (str == "circles") {
    pts = circles(0.3, 7);
  } else if (str == "lis_default") {
    pts = LISlogo(false);
  } else if (str == "lis_large") {
    pts = LISlogo(true);
  } else if (str == "greedy_counterexample") {
    pts = greedy_counterexample();
  } else if (str == "four_robot_test_1") {
    pts = grid(2, 2, 0.05, 0.05);
  } else if (str == "four_robot_test_2") {
    pts = grid(2, 2, 0.7, 0.05);
  } else {
    std::cout << "Scenario not found" << std::endl;
  }

  return pts;
}

int main(int argc, char **argv) {
  rai::initCmdLine(argc, argv);
  const uint seed = rai::getParameter<double>("seed", 42); // seed
  rnd.seed(seed);

  const uint verbosity = rai::getParameter<double>(
      "verbosity", 1); // verbosity, does not do anything atm

  const bool plan_pick_and_place =
      rai::getParameter<bool>("pnp", true); // pick and place yes/no

  // possible modes:
  // - test
  // - optimize
  // - show scenario
  // - show saved path
  const rai::String mode =
      rai::getParameter<rai::String>("mode", "test"); // scenario greedy_random_search
  const rai::String stippling_scenario =
      rai::getParameter<rai::String>("stippling_pts", "lis_default"); // scenario lis_default four_by_four_grid

  const rai::String env =
      rai::getParameter<rai::String>("env", "lab"); // environment

  std::vector<std::string> robots; // string-prefix for robots

  rai::Configuration C;
  if (plan_pick_and_place) {
    pick_and_place(C);
    robots = {"a0_", "a1_"};
  } else {
    if (env == "lab") {
      labSetting(C);
      robots = {"a0_", "a1_"};
    } else {
      more_robots(C, 4);
      robots = {"a0_", "a1_", "a2_", "a3_"};
    }
  }

  // maps [robot] to home_pose
  const std::map<Robot, arr> home_poses = get_robot_home_poses(C, robots);

  // show prev path
  if (mode == "show_plan") {
    load_and_viz(C, plan_pick_and_place);
    return 0;
  }

  // stippling
  RobotTaskPoseMap robot_task_pose_mapping;
  if (!plan_pick_and_place) {
    const arr pts = get_scenario(stippling_scenario);
    if (pts.N == 0) {
      return 0;
    }

    if (verbosity > 0) {
      drawPts(C, pts);
    }

    // maps [robot] to [index, pose]
    std::cout << "Computing stippling poses" << std::endl;
    robot_task_pose_mapping = compute_stippling_poses_for_arms(C, pts, robots);
  } else {
    // bin picking
    std::cout << "Computing pick and place poses" << std::endl;
    robot_task_pose_mapping = compute_pick_and_place_positions(C, robots);
  }

  // initial test
  bool save_video = false;
  if (mode == "test") {
    const auto plan = plan_multiple_arms_unsynchronized(
        C, robot_task_pose_mapping, home_poses);
    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan(C, plan, save_video, "video/bin_picking/unsync");
  } else if (mode == "random_search") {
    const auto plan = plan_multiple_arms_random_search(
        C, robot_task_pose_mapping, home_poses);
    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan(C, plan, save_video, "video/bin_picking/random_search");
  } else if (mode == "greedy_random_search") {
    const auto plan = plan_multiple_arms_greedy_random_search(
        C, robot_task_pose_mapping, home_poses);
    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan(C, plan, save_video, "video/bin_picking/greedy_search");
  } else if (mode == "simulated_annealing") {
    const auto plan = plan_multiple_arms_simulated_annealing(   
        C, robot_task_pose_mapping, home_poses);
    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;
    visualize_plan(C, plan, save_video, "video/bin_picking/annealing");
  }

  return 0;
}
