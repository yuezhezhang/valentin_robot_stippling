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
#include "searchers/annealing_searcher.h"
#include "searchers/random_searcher.h"
#include "searchers/greedy_random_searcher.h"
#include "searchers/unsync_searcher.h"

#include "plan.h"
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

int main(int argc, char **argv) {
  rai::initCmdLine(argc, argv);
  const uint seed = rai::getParameter<double>("seed", 42); // seed
  rnd.seed(seed);

  const uint verbosity = rai::getParameter<double>(
      "verbosity", 1); // verbosity, does not do anything atm

  const bool plan_pick_and_place =
      rai::getParameter<bool>("pnp", true); // pick and place yes/no

  const rai::String mode =
      rai::getParameter<rai::String>("mode", "test"); // test, greedy_random_search, show_plan
  const rai::String stippling_scenario =
      rai::getParameter<rai::String>("stippling_pts", "lis_default"); // lis_default, four_by_four_grid, default_grid

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