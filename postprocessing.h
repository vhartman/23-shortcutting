#include <KOMO/komo.h>
#include <Kin/kin.h>

#include <Kin/kinViewer.h>
#include <Kin/viewer.h>

#include <PlanningSubroutines/ConfigurationProblem.h>

#ifndef _POSTPROCESSING_H
#define _POSTPROCESSING_H

const double pathLength(const arr& path){
  double cost = 0.;
  for (uint i=0; i<path.d0-1; ++i){
    cost += euclideanDistance(path[i], path[i+1]);
  }

  return cost;
}

arr interpolate_path(const arr &path, const double discretization_resolution, const double min_dist_between_pts){
  std::vector<arr> pts;

  for (uint i=0; i<path.d0-1; ++i){
    const arr q0 = path[i];
    const arr q1 = path[i+1];

    const arr delta = q1 - q0;
    const double dist = length(delta);

    const uint num_pts = std::max(
        1u, 
        std::min(uint(dist / min_dist_between_pts), uint(dist / discretization_resolution))
        );
    
    for (uint j=0; j<num_pts; ++j){
      const arr pt = q0 + 1. * j / num_pts * (q1 - q0);
      pts.push_back(pt);
    }
  }

  // add final pt at the end
  pts.push_back(path[-1]);

  arr interpolated_path(pts.size(), path.d1);
  for (uint i=0; i<pts.size(); ++i){
    interpolated_path[i] = pts[i];
  }
  return interpolated_path;
}

arr interpolate_path(const arr in, const uint steps){
  arr out(steps, in.d1);
  const double scaling = 1. * (in.d0-1) / (steps-1);

  for (uint i=0; i<steps; ++i){
    const uint ind = i * scaling;

    const double remainder = i*scaling - ind;
    arr pt = in[ind]();

    if (ind+1 < in.d0){
      const arr dir = in[ind+1] - in[ind];
      pt = 1.* pt + 1.*dir*remainder;
    }

    out[i] = pt;
  }

  return out;
}


struct PathAndStatistics{
  arr path;

  std::vector<double> times;
  std::vector<double> costs;
};

/*class Pruner{
  public:
    arr process();

    std::vector<double> times;
    std::vector<double> costs;

  private:
    void iter();
};

class PartialSetShortcutter{
  public:
    arr process();

    std::vector<double> times;
    std::vector<double> costs;

  private:
    void iter();
};

class Shortcutter{
  public:
    arr process();

    std::vector<double> times;
    std::vector<double> costs;

  private:
    void iter();
};

class PartialShortcutter{
  public:
    arr process();

    std::vector<double> times;
    std::vector<double> costs;

  private:
    void iter();
};*/

bool check_edge(ConfigurationProblem &cp, const arr &q0, const arr &q1, const double resolution){
  const arr delta = q1 - q0;
  const double dist = length(delta);
  // check the pts on the new edge for collisions
  const uint num_pts = uint(std::max(dist / resolution, 1.));
  arr pt(q0.N);
  for (uint i=0; i<num_pts; ++i){
    double interp = 1. * (1. + i) / (num_pts + 1);
    for (uint m=0; m<delta.N; ++m){
      pt(m) = q0(m) + delta(m) * interp;
    }
    //const arr pt = q0 + interp * (q1 - q0);
    const auto qr = cp.query(pt);
    //cp.C.watch(true);
    if(!qr->isFeasible){
      return false;
    }
  }

  return true;
}

PathAndStatistics prune(const arr &path, rai::Configuration &C, const double resolution){
  ConfigurationProblem cp(C);
  arr pruned_path = path;

  std::vector<double> times;
  std::vector<double> lengths;

  times.push_back(0.);
  lengths.push_back(pathLength(path));

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

  while(true){
    bool removed_something = false;

    for (uint i=1; i<pruned_path.d0-1; ++i){
      // try removing the edge
      const arr q0 = pruned_path[i-1];
      const arr q1 = pruned_path[i+1];

      // check shortcut validity
      const bool edge_validity = check_edge(cp, q0, q1, resolution);
      if (edge_validity){
        // construct new path
        const uint prev_num_pts = pruned_path.d0;
        arr tmp(prev_num_pts - 1, path.d1);

        uint offset=0;
        for (uint j=0; j<prev_num_pts-1; ++j){
          if (j == i){
            offset = 1;
          }

          tmp[j] = pruned_path[j+offset];
        }
        pruned_path = tmp;

        removed_something = true;
        break;
      }
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    const double delta =(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) /1.e6;

    times.push_back(delta);
    lengths.push_back(pathLength(pruned_path));

    if (!removed_something){
      break;
    }
  }

  return PathAndStatistics{
    .path = pruned_path,
    .times = times,
    .costs = lengths
  };
}

arr constructShortcutPath(const rai::Configuration &C, const arr& path, 
    const uint i, const uint j, const std::vector<uint> short_ind){
  auto periodicDimensions = std::vector<bool>(C.getJointState().N, false);

  for (auto *j: C.activeJoints){
    if (j->type == rai::JT_hingeX || j->type == rai::JT_hingeY|| j->type == rai::JT_hingeZ){
      periodicDimensions[j->qIndex] = true;
    }
  }

  const arr p1 = path[i]();
  const arr p2 = path[j]();
  arr delta = p2 - p1;
  for (uint l=0; l<delta.N; ++l){
    if (periodicDimensions[l]){
      // this is an angular joint -> we need to check the other direction
      const double start = p2(l);
      const double end = p1(l);
      delta(l) = std::fmod(start - end + 3.*RAI_PI, 2*RAI_PI) - RAI_PI;
    }
  }

  arr p(j-i, path.d1);

  for(uint l=0; l<j-i; ++l){
    for(uint k=0; k<path.d1; ++k){
      if(std::find(short_ind.begin(), short_ind.end(), k) != short_ind.end()){
        const double a = 1. * l / (j-i);
        //p(l, k) = path(i, k) + a * (path(j, k) - path(i, k));
        p(l, k) = path(i, k) + a * (delta(k));
      }
      else{
        p(l, k) = path(l+i, k);
      }
    }
  }

  return p;
}

enum class PartialShortcutType {Single, Subset, Full};
enum class SubsetSampling {Uniform, Other};

struct IterationData{
  bool success;
  uint idx1;
  uint idx2;
  double length;
  std::vector<uint> indices;
  double time;
};

PathAndStatistics partial_shortcut(rai::Configuration C, const arr &path, const double resolution = 0.01, const double discretization_resolution = 0.1, const uint max_iter = 400, PartialShortcutType type=PartialShortcutType::Subset, SubsetSampling sampling_type = SubsetSampling::Uniform){
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

  const arr discretized_path = interpolate_path(path, discretization_resolution, resolution);
  arr smoothedPath = discretized_path;

  std::vector<double> times;
  std::vector<double> costs;

  std::vector<IterationData> data;

  times.push_back(0.);
  costs.push_back(pathLength(path));

  ConfigurationProblem cp(C);

  std::random_device rd;
  std::mt19937 generator(rd());

  for(uint k=0; k<max_iter; ++k){
    // choose random indices
    int i, j;
    do{
      i = rand() % discretized_path.d0;
      j = rand() % discretized_path.d0;
    } while(abs(j - i) <= 1);

    if(i > j){
      std::swap(i, j);
    }

    // choose which indices to shortcut
    std::vector<uint> ind;
    if (type == PartialShortcutType::Subset){
      if (sampling_type == SubsetSampling::Uniform){
        const uint set_size = rand() % (discretized_path.d1-1) + 1;

        std::vector<uint> tmp;
        for(uint q=0; q<smoothedPath.d1; ++q){
          tmp.push_back(q);
        }

        // Shuffle the vector
        std::shuffle(tmp.begin(), tmp.end(), generator);

        for (uint q = 0; q < set_size; ++q) {
          ind.push_back(tmp[q]);
        }
      }
      else{
        for(uint q=0; q<smoothedPath.d1; ++q){
          const double r = 1. * rand() / RAND_MAX;
          if(r > 1./smoothedPath.d1){
            ind.push_back(q);
          }
        }
      }
    }
    else if(type == PartialShortcutType::Full){
      for(uint q=0; q<smoothedPath.d1; ++q){
        ind.push_back(q);
      }
    }
    else{
      ind.push_back(rand() % discretized_path.d1);
    }

    if (ind.size() == 0){
      continue;
    }
    
    // construct the new path
    const auto original_path = constructShortcutPath(cp.C, smoothedPath, i, j, {});
    //const auto original_path = smoothedPath({i,j});
    const auto shortcut_path = constructShortcutPath(cp.C, smoothedPath, i, j, ind);

    const double shortcut_length = pathLength(shortcut_path);
    if (pathLength(original_path) <= shortcut_length){
      continue;
    }

    // check if the new path is feasible (interpolate)
    uintA q; q.setStraightPerm(j-i-1);
    q.permuteRandomly();
    bool shortcutFeasible = true;
    for(const uint &n: q){
      const bool valid = check_edge(cp, shortcut_path[n], shortcut_path[n+1], resolution);
      if (!valid){
        shortcutFeasible = false;
        break;
      }
    }
   
    // check if the new path is shorter
    if(shortcutFeasible){
      for(uint n=1; n<j-i; ++n){
        smoothedPath[i+n] = shortcut_path[n];
      }
    }
    
    const auto c = pathLength(smoothedPath);
    costs.push_back(c);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    const double delta =(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) /1.e6;

    times.push_back(delta);
  }

  //std::cout << '\t' << "Change in path costs: " << costs[0] << " " << costs.back() << std::endl;
  return PathAndStatistics{
    .path = smoothedPath,
    .times = times,
    .costs = costs
  };
}

PathAndStatistics shortcut(const arr &path, rai::Configuration &C, const double resolution, const double discretization_resolution = 0.1, const uint max_iter=400){
  return partial_shortcut(C, path, resolution, discretization_resolution, max_iter, PartialShortcutType::Full);
}

PathAndStatistics shortcut_single_dim(const arr &path, rai::Configuration &C, const double resolution, const double discretization_resolution = 0.1, const uint max_iter=400){
  return partial_shortcut(C, path, resolution, discretization_resolution, max_iter, PartialShortcutType::Single);
}


#endif
