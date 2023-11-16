#include <KOMO/komo.h>
#include <Kin/F_operators.h>
#include <Kin/F_pose.h>
#include <Kin/F_qFeatures.h>
#include <Kin/featureSymbols.h>
#include <Kin/kin.h>

#include <Kin/kinViewer.h>
#include <Kin/viewer.h>

#include <PlanningSubroutines/ConfigurationProblem.h>

#include <Geo/fclInterface.h>

#include <Core/util.h>

#include "problem.h"
#include "solver.h"
#include <Manip/rrt.h>

#include "postprocessing.h"

#include <numeric>
#include <iomanip>
#include <unordered_set>
#include <fstream>

#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

std::shared_ptr<Problem> getProblem(const rai::String &scenario){
  std::shared_ptr<Problem> s;
  if (scenario == "empty"){
    s = std::make_shared<Empty>(); // 96% planning
  }
  else if (scenario == "empty2"){
    s = std::make_shared<Empty2>(); // 96% planning
  }
  else if (scenario == "empty3"){
    s = std::make_shared<Empty3>(); // 96% planning
  }
  else if (scenario == "move_only"){
    s = std::make_shared<MoveOnly>(); // 96% planning
  }
  else if (scenario == "move_only_2"){
    s = std::make_shared<MoveOnlyNoObs>(); // 96% planning
  }
  else if (scenario == "move_only_3"){
    s = std::make_shared<MoveOnly2>(); // 96% planning
  }
  else if (scenario == "move_only_4"){
    s = std::make_shared<MoveOnly3>(); // 96% planning
  }
  else if (scenario == "four_rooms"){
    s = std::make_shared<FourRooms>();
  }
  else if (scenario == "svetlana"){
    s = std::make_shared<Svetlana>(); // 96% planning
  }
  else if (scenario == "svetlana_easy"){
    s = std::make_shared<SvetlanaEasy>(); // 96% planning
  }
  else if (scenario == "multi_svetlana"){
    s = std::make_shared<MultipleSvetlana>(); // 96% planning
  }
  else if (scenario == "opt_svetlana"){
    s = std::make_shared<OptimalSvetlana>(); // 96% planning
  }
  else if (scenario == "sofa"){
    s = std::make_shared<Sofa>(); // 
  }
  else if (scenario == "multi_sofa"){
    s = std::make_shared<MultipleSofa>(); // 
  }
  else if (scenario == "insertion_2d"){
    s = std::make_shared<Insertion2d>(); // 50% both
  }
  else if (scenario == "puzzle"){
    s = std::make_shared<Puzzle>(); // sampling heavy (50%)
  }
  else if (scenario == "quim"){
    s = std::make_shared<Quim>();
  }
  else if (scenario == "obstacle_2d"){
    s = std::make_shared<Obstacle2d>();
  }
  else if (scenario == "cage_2d"){
    s = std::make_shared<Cage2d>(); //  planning heavy problem - up to 96%
  }
  else if (scenario == "move"){
    s = std::make_shared<MoveTable>();
  }
  else if (scenario == "move_restricted"){
    s = std::make_shared<MoveTableRestricted>();
  }
  else if (scenario == "cage"){
    s = std::make_shared<Cage3d>();
  }
  else if (scenario == "dog"){
    s = std::make_shared<Dog>();
  }
  else if (scenario == "bottle"){
    s = std::make_shared<Wine>();
  }
  else if (scenario == "rotate_cube"){
    s = std::make_shared<RotateCube>();
  }
  else if (scenario == "physics"){
    s = std::make_shared<ForceRearrange>();
  }
  else if (scenario == "arm_move"){
    s = std::make_shared<ArmMove>();
  }
  else if (scenario == "arm_move_2"){
    s = std::make_shared<ArmMove2>();
  }
  else if (scenario == "arm_stacking"){
    s = std::make_shared<ArmStacking>();
  }
  else if (scenario == "arm_stacking_2"){
    s = std::make_shared<ArmStacking2>();
  }
  else if (scenario == "arm_stacking_2_restricted"){
    s = std::make_shared<ArmStacking2Restricted>();
  }
  else if (scenario == "arm_stacking_3"){
    s = std::make_shared<ArmStacking3>();
  }
  else if (scenario == "arm_bin"){
    s = std::make_shared<ArmBinPicking>();
  }
  else if (scenario == "arm_bin_2"){
    s = std::make_shared<ArmBinPicking2>();
  }
  else if (scenario == "arm_bin_3"){
    s = std::make_shared<ArmBinPicking3>();
  }
  else if (scenario == "arm_bin_4"){
    s = std::make_shared<ArmBinPicking4>();
  }
  else if (scenario == "arm_bin_5"){
    s = std::make_shared<ArmBinPicking5>();
  }
  else if (scenario == "amazon"){
    s = std::make_shared<Amazon>();
  }
  else if (scenario == "amazon_single"){
    s = std::make_shared<AmazonSingle>();
  }
  else if (scenario == "stack_mobile"){
    s = std::make_shared<StackMobile>();
  }
  else if (scenario == "wall"){
    s = std::make_shared<Wall>();
  }
  else if (scenario == "wall_move"){
    s = std::make_shared<WallMoveOnly>();
  }
  else if (scenario == "brickwall"){
    s = std::make_shared<BrickWall>();
  }
  else if (scenario == "brickwall2"){
    s = std::make_shared<BrickWall2>();
  }
  else if (scenario == "brickwall3"){
    s = std::make_shared<BrickWall3>();
  }
  else if (scenario == "brickwall4"){
    s = std::make_shared<BrickWall4>();
  }
  else if (scenario == "brickwall5"){
    s = std::make_shared<BrickWall5>();
  }
  else if (scenario == "brickwall6"){
    s = std::make_shared<BrickWallLongObs>();
  }
  else if (scenario == "brickwall_super_long"){
    s = std::make_shared<BrickWallSuperLong>();
  }
  else if (scenario == "brickwall_long"){
    s = std::make_shared<BrickWallLong>();
  }
  else if (scenario == "brickwall_move"){
    s = std::make_shared<BrickWallMoveOnly>();
  }
  else if (scenario == "handover"){
    s = std::make_shared<Handover>();
  }
  else if (scenario == "handover_perturbed"){
    s = std::make_shared<HandoverPerturbed>();
  }
  else if (scenario == "handover_perturbed_box"){
    s = std::make_shared<HandoverPerturbedBox>();
  }
  else if (scenario == "handover_perturbed_box_2"){
    s = std::make_shared<HandoverPerturbedBox2>();
  }
  else if (scenario == "handover2"){
    s = std::make_shared<Handover2>();
  }
  else if (scenario == "handover_stacking"){
    s = std::make_shared<HandoverStacking>();
  }
  else{
    throw "Error";
  }

  //std::cout << "Scenario \t" << s->name_ << std::endl;

  return s;
}

bool verifySolution(std::shared_ptr<Problem> p, const Solution& sol, const uint res=0.001, bool display=true){
  p->initial_.calc_indexedActiveJoints();

  StringA joints;
  for (auto a: p->initial_.activeJoints){
    joints.append(a->frame->name);
  }

  rai::Configuration C(p->initial_);
  ConfigurationProblem cp(C);

  for (uint i=0; i<sol.segments_.size(); ++i){
    const auto &s = sol.segments_[i];
    cp.C.setJointState(s[0]);
    
    p->setConfigurationToMode(cp.C, i);
    // set the right frames to be active
    cp.C.selectJointsByName(joints);

    for (uint j=0; j<s.d0-1; ++j){
      uint N = length(s[j+1] - s[j]) / res;
      N = std::max({N, 10u});
      // interpolate between poinst of segment
      for (uint k=0; k<N; ++k){
        const arr q = s[j] + 1.0 * k / N * (s[j+1] - s[j]);
        const auto res = cp.query(q);
        if (!res->isFeasible){
          std::cout << "segment " << i << " not valid " << std::endl;
          if (display){
            cp.C.watch(true);
          }
          return false;
        }
      }
    }
  }

  return true;
}

std::vector<arr> find_feasible_keyframe(std::shared_ptr<Problem> p, const uint seed){
  auto s = std::make_shared<JointPTR>(p, PlannerType::RRTConnect);
  //auto s = std::make_shared<CommittingSequentialSolver>(p);
  s->seed_ = seed;
  s->verbose_ = 10;

  const auto sol = s->solve(100.);
  std::cout << "found solution" << std::endl;

  const auto valid = verifySolution(p, sol);
  if(!valid){
    std::cout << "Solution not valid!!!" << std::endl;
    sol.show(p, true);
    return {};
  }

  // get keyframes form the solution
  std::vector<arr> keyframes;
  for (const auto segment: sol.segments_){
    keyframes.push_back(segment[0] * 1.0);
    std::cout << segment[0] << std::endl;
  }

  keyframes.push_back(sol.segments_.back()[-1] * 1.0);

  return keyframes;
}

std::vector<std::vector<arr>> load_keyframes(const std::string& name){
  std::ifstream file;
  file.open ("./in/keyframes/" + name + ".txt");

  if (!file.good()){
    return {};
  }

  std::vector<std::vector<arr>> out;
  std::vector<arr> keyframes;

  std::string line;
  while (std::getline(file, line)) {
    if (line == "--"){
      if (keyframes.size() != 0){
        out.push_back(keyframes);
      }
      keyframes.clear();
    }
    else{
      std::istringstream iss(line);
      arr tmp;
      iss >> tmp;
      keyframes.push_back(tmp * 1.0);
    }
  }

  out.push_back(keyframes);

  return out;
}

void export_keyframes(const std::string& name, const std::vector<std::vector<arr>> &set){
  std::ofstream file;
  file.open ("./in/keyframes/" + name + ".txt");

  for (const auto& keyframes: set){
    file << "--" << std::endl;
    for (const auto& keyframe: keyframes){
      file << std::setprecision(16) << keyframe << '\n';
    }
  }
}

std::vector<arr> load_or_compute_keyframes(std::shared_ptr<Problem> p){
  auto set_of_keyframes = load_keyframes(p->name_);
  if (set_of_keyframes.size() == 0){
    uint cnt = 0;
    while(set_of_keyframes.size() == 0){
      const auto keyframes = find_feasible_keyframe(p, cnt);
      if (keyframes.size() == 0){
        continue;
      }
      set_of_keyframes.push_back(keyframes);
      cnt += 1;
    }
    export_keyframes(p->name_, set_of_keyframes);
  }
  return set_of_keyframes[0];
}

void show(std::shared_ptr<Problem> p){
  p->initial_.watch(true);
}

void show_from_top(std::shared_ptr<Problem> p){
  // load keyframes
  std::vector<arr> all_keyframes;

  auto set_of_keyframes = load_or_compute_keyframes(p);
  all_keyframes = set_of_keyframes;

  rai::Configuration C(p->initial_);

  rai::ConfigurationViewer Vf;
  Vf.setConfiguration(C, "\"Real World\"");

  Vf.displayCamera().setPosition(0, 10, 15);
  Vf.displayCamera().focusOrigin();
  Vf.displayCamera().upright();
  Vf.displayCamera().setPosition(0, -0.001, 15);
  Vf.displayCamera().focusOrigin();
  Vf.displayCamera().setFocalLength(5);

  StringA joints_;
  p->initial_.calc_indexedActiveJoints();
  for (const auto &a: p->initial_.activeJoints){
    joints_.append(a->frame->name);
  }

  for (uint i = 0; i<all_keyframes.size(); ++i){

    C.setJointState(all_keyframes[i]);

    Vf.setConfiguration(C, "\"Real World\"");
    Vf.watch();
    p->setConfigurationToMode(C, i);
    C.selectJointsByName(joints_);
  }
}

void display_path(rai::Configuration C, const arr &path, const bool pause=false){
  for (uint i=0; i<path.d0; ++i){
    C.setJointState(path[i]);
    C.watch(pause);
    rai::wait(0.01);
  }
}

arr run_planner(std::shared_ptr<Problem> p, bool disp = false){
  auto keyframes = load_or_compute_keyframes(p);
  rai::Configuration C(p->initial_);

  // solve rrt-things
  C.setJointState(keyframes[0]);
  ConfigurationProblem cp(C, true);

  std::cout << "starting rrt" << std::endl;
  PathFinder_RRT rrt(cp, keyframes[1]);
  const arr path = rrt.planConnect(keyframes[1]);

  // display path
  // interpolate path for displaying
  if (disp){
    uint N = 5;
    arr interp_path((path.d0-1)*N+1, path.d1);
    for (uint k=0; k<path.d0-1; ++k){
      for (uint j=0; j<N; ++j){
        interp_path[k*N+j] = path[k] + 1. * j / N * (path[k+1]-path[k]);
      }
    }
    interp_path[-1] = path[-1];

    display_path(C, interp_path);

  }
  return path;
}

struct Statistics{
  std::vector<double> lengths;
  std::vector<double> times_in_s;

  void export_to_file(const std::string &foldername, const std::string &method_name) const{
    // open file
    // make folder
    const std::string folder =
        "./out/" + foldername + "/";
    const int res = system(STRING("mkdir -p " << folder).p);
    (void)res;

    std::ofstream f;
    f.open(folder + method_name + ".txt", std::ios_base::app);
    
    for (uint i=0; i<times_in_s.size(); ++i){
      f << times_in_s[i];
      if (i<times_in_s.size()-1){f << ",";}
    }
    f << "\n";
    for (uint i=0; i<lengths.size(); ++i){
      f << lengths[i];
      if (i<lengths.size()-1){f << ",";}
    }
    f << "\n";
  
  };
};

struct ShortcuttingProblem{
  rai::Configuration C;
  arr path;
};

void run_experiment(std::shared_ptr<Problem> p, const uint num_runs = 1, const uint num_reps=10){
  const double resolution = 0.01;

  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  const auto keyframes = load_or_compute_keyframes(p);
  if (keyframes.size() == 0){
    std::cout << "no keyframes found" << std::endl;
  }

  p->initial_.calc_indexedActiveJoints();

  StringA joints;
  for (auto a: p->initial_.activeJoints){
    joints.append(a->frame->name);
  }

  for (uint k=0; k<2; ++k){
    std::cout << "Mode " << k << std::endl;

    std::vector<ShortcuttingProblem> problems;
    for (uint i=0; i<num_runs; ++i){
      // make config and set to mode
      rai::Configuration C = p->initial_;
      for (uint j=0; j<=k; ++j){
        C.setJointState(keyframes[j]);
        p->setConfigurationToMode(C, j);
        C.selectJointsByName(joints);
      }

      // set to initial state
      C.setJointState(keyframes[k]);
      ConfigurationProblem cp(C, true);

      // plan path
      PathFinder_RRT rrt(cp, keyframes[k+1]);
      const arr path = rrt.planConnect(keyframes[k+1]);
      if (path.d0 == 0){
        std::cout << "no path found" << std::endl;
      }

      ShortcuttingProblem sc{
        .C = C,
        .path = path
      };
      for (uint l=0; l<num_reps; ++l){
        problems.push_back(sc);
      }
    }

    std::map<std::string, std::vector<Statistics>> stats_per_approach;

    // run shorteners
    for (uint i=0; i<problems.size(); ++i){
      const auto problem = problems[i];
      std::cout << "\r" << i+1 << "/" << problems.size() << std::flush;

      rai::Configuration C = problem.C;
      const arr path = problem.path;
      //display_path(C, path);

      // pruning
      {
        const auto res = prune(path, C, resolution);
        stats_per_approach["pruner"].push_back(
            Statistics{.lengths = res.costs, .times_in_s = res.times}
            );

        //display_path(C, res.path, true);
      }

      // shortcutter
      {
        const auto res = shortcut(path, C, resolution);
        stats_per_approach["shortcutter"].push_back(
            Statistics{.lengths = res.costs, .times_in_s = res.times}
            );
      }

      // shortcutter (single dim)
      {
        const auto res = shortcut_single_dim(path, C, resolution);
        stats_per_approach["shortcutter_single_dim"].push_back(
            Statistics{.lengths = res.costs, .times_in_s = res.times}
            );
      }

      // shortcutter(subset)
      {
        //const arr discretized_path = interpolate_path(path, path.d0*50);
        //display_path(C, discretized_path, false);
        const auto res = partial_shortcut(C, path, resolution);
        stats_per_approach["shortcutter_subset"].push_back(
            Statistics{.lengths = res.costs, .times_in_s = res.times}
            );
        //display_path(C, res.path, false);
      }
    }
    std::cout << std::endl;
    std::cout << "Saving results" << std::endl;

    // create foldername
    std::stringstream buffer;
    buffer << p->name_ << "_" << k << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

    // export statistics
    for (const auto stats: stats_per_approach){
      const std::string filename = stats.first;
      for (const auto s: stats.second){
        s.export_to_file(buffer.str(), filename);
      }
    }
  }
}

void discretization_ablation(std::shared_ptr<Problem> p, const uint num_runs = 1, const uint num_reps=10){
  const double resolution = 0.01;

  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  const auto keyframes = load_or_compute_keyframes(p);
  if (keyframes.size() == 0){
    std::cout << "no keyframes found" << std::endl;
  }

  p->initial_.calc_indexedActiveJoints();

  StringA joints;
  for (auto a: p->initial_.activeJoints){
    joints.append(a->frame->name);
  }

  for (uint k=0; k<2; ++k){
    std::cout << "Mode " << k << std::endl;

    std::vector<ShortcuttingProblem> problems;
    for (uint i=0; i<num_runs; ++i){
      // make config and set to mode
      rai::Configuration C = p->initial_;
      for (uint j=0; j<=k; ++j){
        C.setJointState(keyframes[j]);
        p->setConfigurationToMode(C, j);
        C.selectJointsByName(joints);
      }

      // set to initial state
      C.setJointState(keyframes[k]);
      ConfigurationProblem cp(C, true);

      // plan path
      PathFinder_RRT rrt(cp, keyframes[k+1]);
      const arr path = rrt.planConnect(keyframes[k+1]);
      if (path.d0 == 0){
        std::cout << "no path found" << std::endl;
      }

      ShortcuttingProblem sc{
        .C = C,
        .path = path
      };
      for (uint l=0; l<num_reps; ++l){
        problems.push_back(sc);
      }
    }

    std::map<std::string, std::vector<Statistics>> stats_per_approach;

    // run shorteners
    for (uint i=0; i<problems.size(); ++i){
      const auto problem = problems[i];
      std::cout << "\r" << i + 1 << "/" << problems.size() << std::flush;

      rai::Configuration C = problem.C;
      const arr path = problem.path;
      //display_path(C, path);

      // shortcutter(subset)
      {
        //const arr discretized_path = interpolate_path(path, path.d0*50);
        //display_path(C, discretized_path, false);
        const auto res = partial_shortcut(C, path, resolution, 1);
        stats_per_approach["shortcutter_subset_coarse"].push_back(
            Statistics{.lengths = res.costs, .times_in_s = res.times}
            );
        //display_path(C, res.path, false);
      }

      // shortcutter(subset)
      {
        //const arr discretized_path = interpolate_path(path, path.d0*50);
        //display_path(C, discretized_path, false);
        const auto res = partial_shortcut(C, path, resolution);
        stats_per_approach["shortcutter_subset"].push_back(
            Statistics{.lengths = res.costs, .times_in_s = res.times}
            );
        //display_path(C, res.path, false);
      }

      // shortcutter(subset)
      {
        //const arr discretized_path = interpolate_path(path, path.d0*50);
        //display_path(C, discretized_path, false);
        const auto res = partial_shortcut(C, path, resolution, resolution);
        stats_per_approach["shortcutter_subset_finer"].push_back(
            Statistics{.lengths = res.costs, .times_in_s = res.times}
            );
        //display_path(C, res.path, false);
      }
    }
    std::cout << std::endl;
    std::cout << "Saving results" << std::endl;

    // create foldername
    std::stringstream buffer;
    buffer << p->name_ << "_" << k << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

    // export statistics
    for (const auto stats: stats_per_approach){
      const std::string filename = stats.first;
      for (const auto s: stats.second){
        s.export_to_file(buffer.str(), filename);
      }
    }
  }
}

void sampling_ablation(std::shared_ptr<Problem> p, const uint num_runs = 1, const uint num_reps=10){
  const double resolution = 0.01;

  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  const auto keyframes = load_or_compute_keyframes(p);
  if (keyframes.size() == 0){
    std::cout << "no keyframes found" << std::endl;
  }

  p->initial_.calc_indexedActiveJoints();

  StringA joints;
  for (auto a: p->initial_.activeJoints){
    joints.append(a->frame->name);
  }

  for (uint k=0; k<2; ++k){
    std::cout << "Mode " << k << std::endl;

    std::vector<ShortcuttingProblem> problems;
    for (uint i=0; i<num_runs; ++i){
      // make config and set to mode
      rai::Configuration C = p->initial_;
      for (uint j=0; j<=k; ++j){
        C.setJointState(keyframes[j]);
        p->setConfigurationToMode(C, j);
        C.selectJointsByName(joints);
      }

      // set to initial state
      C.setJointState(keyframes[k]);
      ConfigurationProblem cp(C, true);

      // plan path
      PathFinder_RRT rrt(cp, keyframes[k+1]);
      const arr path = rrt.planConnect(keyframes[k+1]);
      if (path.d0 == 0){
        std::cout << "no path found" << std::endl;
      }

      ShortcuttingProblem sc{
        .C = C,
        .path = path
      };
      for (uint l=0; l<num_reps; ++l){
        problems.push_back(sc);
      }
    }

    std::map<std::string, std::vector<Statistics>> stats_per_approach;

    // run shorteners
    for (uint i=0; i<problems.size(); ++i){
      const auto problem = problems[i];
      std::cout << "\r" << i + 1 << "/" << problems.size() << std::flush;

      rai::Configuration C = problem.C;
      const arr path = problem.path;
      //display_path(C, path);

      // shortcutter(subset)
      {
        //const arr discretized_path = interpolate_path(path, path.d0*50);
        //display_path(C, discretized_path, false);
        const auto res = partial_shortcut(C, path, resolution, resolution*10, 400, PartialShortcutType::Subset, SubsetSampling::Uniform);
        stats_per_approach["shortcutter_subset"].push_back(
            Statistics{.lengths = res.costs, .times_in_s = res.times}
            );
        //display_path(C, res.path, false);
      }

      // shortcutter(subset)
      {
        //const arr discretized_path = interpolate_path(path, path.d0*50);
        //display_path(C, discretized_path, false);
        const auto res = partial_shortcut(C, path, resolution, resolution*10, 400, PartialShortcutType::Subset, SubsetSampling::Other);
        //const auto res = partial_shortcut(C, path, resolution, resolution/10);
        stats_per_approach["shortcutter_subset_probability"].push_back(
            Statistics{.lengths = res.costs, .times_in_s = res.times}
            );
        //display_path(C, res.path, false);
      }
    }
    std::cout << std::endl;
    std::cout << "Saving results" << std::endl;

    // create foldername
    std::stringstream buffer;
    buffer << p->name_ << "_" << k << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

    // export statistics
    for (const auto stats: stats_per_approach){
      const std::string filename = stats.first;
      for (const auto s: stats.second){
        s.export_to_file(buffer.str(), filename);
      }
    }
  }
}

int main(int argc, char **argv) {
  rai::initCmdLine(argc, argv);
  const uint seed = rai::getParameter<double>("seed", 42); // seed
  const rai::String scenario = rai::getParameter<rai::String>("scenario", ""); // scenario

  const rai::String mode = rai::getParameter<rai::String>("mode", ""); // mode

  rnd.seed(seed);
  rnd_.seed(seed);

  if (mode == "show_from_top"){
    show_from_top(getProblem(scenario));
  }
  else if (mode == "run_planner"){
    run_planner(getProblem(scenario), true);
  }
  else if (mode == "run_experiment"){
    run_experiment(getProblem(scenario));
  }
  else if (mode == "run_discretization_ablation"){
    discretization_ablation(getProblem(scenario), 1, 50);
  }
  else if (mode == "run_sampling_ablation"){
    sampling_ablation(getProblem(scenario), 1, 50);
  }
  else{
    show(getProblem(scenario));
  }
}

