#ifndef EDGE_CHECKER_H
#define EDGE_CHECKER_H

constexpr double RESOLUTION = 0.005;

class Vertex{
  public:
    static int nextID;

    Vertex(const arr &q, const int id=-1): q_(q){
      if(id == -1){
        id_ = nextID++;
      }
      else{
        id_ = id;
      }
    };

    uint id_;

    const arr q_;

    double dist_to_goal{-1};
    double checks_to_goal{-1};

    std::vector<std::shared_ptr<Vertex>> neighbors_;
    std::vector<std::shared_ptr<Vertex>> whitelisted_;
    uint nnTag_ = 0u;
};

int Vertex::nextID = 0;

enum EdgeStatus{
  valid=0,
  invalid=1,
  unknown=2
};

class Edge{
  public:
    Edge(std::shared_ptr<Vertex> n1, std::shared_ptr<Vertex> n2) :n1_(n1), n2_(n2){};

    std::shared_ptr<Vertex> n1_;
    std::shared_ptr<Vertex> n2_;

    bool isValid() const{
      if (valid_ == EdgeStatus::valid){
        return true;
      }

      return false;
    };
    bool isInvalid() const{
      if (valid_ == EdgeStatus::invalid){
        return true;
      }

      return false;
    }
    
    EdgeStatus valid_{EdgeStatus::unknown};
    uint collCheckLevel_{0u};
};

// Basically a container that serves as an interface to the hierarchical edge
// but has all the parameterized thing in it already
struct ParametrizedHierarchicalEdge{
  public:
    ParametrizedHierarchicalEdge(const uint parameter,
        std::shared_ptr<Vertex> n1,
        std::shared_ptr<Vertex> n2, 
        uint &ccRobotEnv, 
        uint &ccRobotObs, 
        uint &ccObsEnv, 
        EdgeStatus &vfRobotEnv, 
        EdgeStatus &vfRobotObs, 
        EdgeStatus &vfObsEnv,
        uint &annotatedValidAt,
        uint &annotatedInvalidAt)
    :parameter_(parameter),
     n1_(n1), n2_(n2),
     collCheckLevelRobotEnv_(ccRobotEnv),
     collCheckLevelRobotObs_(ccRobotObs),
     collCheckLevelObsEnv_(ccObsEnv),
     validForRobotEnv_(vfRobotEnv),
     validForRobotObs_(vfRobotObs),
     validForObsEnv_(vfObsEnv),
     annotatedValidAt_(annotatedValidAt),
     annotatedInvalidAt_(annotatedInvalidAt){};

    bool isValid() const{
      if (validForRobotEnv_ != EdgeStatus::valid){
        return false;
      }

      if (validForRobotObs_ != EdgeStatus::valid){
        return false;
      }

      if (validForObsEnv_ != EdgeStatus::valid){
        return false;
      }

      return true;
    }

    bool isInvalid() const{
      if (validForRobotEnv_ == EdgeStatus::invalid){
        return true;
      }

      if (validForRobotObs_ == EdgeStatus::invalid){
        return true;
      }

      if (validForObsEnv_ == EdgeStatus::invalid){
        return true;
      }

      return false;
    };

    uint parameter_;

    std::shared_ptr<Vertex> n1_;
    std::shared_ptr<Vertex> n2_;

    uint &collCheckLevelRobotEnv_;
    uint &collCheckLevelRobotObs_;
    uint &collCheckLevelObsEnv_;

    EdgeStatus &validForRobotEnv_;
    EdgeStatus &validForRobotObs_;
    EdgeStatus &validForObsEnv_;

    uint &annotatedValidAt_;
    uint &annotatedInvalidAt_;
};

class HierarchicalEdge{
  public:
    HierarchicalEdge(std::shared_ptr<Vertex> n1, std::shared_ptr<Vertex> n2) :n1_(n1), n2_(n2){};

    std::shared_ptr<Vertex> n1_;
    std::shared_ptr<Vertex> n2_;

    bool isInvalid(const uint param) const{
      if (validForRobotEnv_ == EdgeStatus::invalid){
        if (param != annotatedInvalidAt){
          ++reusedInvalid;
        }
        return true;
      }

      if (validForRobotObs_.count(param) > 0 && 
          validForRobotObs_.at(param) == EdgeStatus::invalid){
        return true;
      }

      if (validForObsEnv_.count(param) > 0 && 
          validForObsEnv_.at(param) == EdgeStatus::invalid){
        return true;
      }

      return false;
    };
    bool isValid(const uint param) const{
      if (validForRobotEnv_ != EdgeStatus::valid){
        if (param != annotatedValidAt){
          ++reusedValid;
        }
        return false;
      }

      if (validForRobotObs_.count(param) == 0 || 
          validForRobotObs_.at(param) != EdgeStatus::valid){
        return false;
      }

      if (validForObsEnv_.count(param) == 0 || 
          validForObsEnv_.at(param) != EdgeStatus::valid){
        return false;
      }

      return true;
    };

    ParametrizedHierarchicalEdge getParametrizedEdge(const uint param){
      // first ensure that all the things exist
      if (validForRobotObs_.count(param) == 0){
        validForRobotObs_[param] = EdgeStatus::unknown;
      }
      if (collCheckLevelRobotObs_.count(param) == 0){
        collCheckLevelRobotObs_[param] = 0;
      }

      if (validForObsEnv_.count(param) == 0){
        validForObsEnv_[param] = EdgeStatus::unknown;
      }
      if (collCheckLevelObsEnv_.count(param) == 0){
        collCheckLevelObsEnv_[param] = 0;
      }

      auto pe = ParametrizedHierarchicalEdge(param, n1_, n2_,
          collCheckLevelRobotEnv_, 
          collCheckLevelRobotObs_[param], 
          collCheckLevelObsEnv_[param], 
          validForRobotEnv_, 
          validForRobotObs_[param], 
          validForObsEnv_[param],
          annotatedValidAt,
          annotatedInvalidAt);

      return pe;
    }

    EdgeStatus validForRobotEnv_{EdgeStatus::unknown}; 
    uint collCheckLevelRobotEnv_{0u};

    // transformation specific collision information
#if 0
    std::unordered_map<uint, EdgeStatus> validForRobotObs_; 
    std::unordered_map<uint, uint> collCheckLevelRobotObs_;

    std::unordered_map<uint, EdgeStatus> validForObsEnv_; 
    std::unordered_map<uint, uint> collCheckLevelObsEnv_;
#else
    std::map<uint, EdgeStatus> validForRobotObs_; 
    std::map<uint, uint> collCheckLevelRobotObs_;

    std::map<uint, EdgeStatus> validForObsEnv_; 
    std::map<uint, uint> collCheckLevelObsEnv_;
#endif

    // Statistics
    uint annotatedValidAt;
    uint annotatedInvalidAt;

    mutable uint reusedValid{0u};
    mutable uint reusedInvalid{0u};
};

class EdgeChecker{
  public:
    EdgeChecker(std::shared_ptr<BaseCollisionChecker> cc): cc_(cc){};
    double resolution_{RESOLUTION};

    bool isValid(
        const arr &start,
        const arr &end){
      const uint disc = getNumberOfCollisionChecks(start, end);
      //std::cout << disc << std::endl;

      for (uint i=0; i<disc; ++i){
        const bool res = isValidAtResolution(start, end, i);

        if(!res){
          //std::cout << disc - i << std::endl;
          return false;
        }
      }

      return true;
    }

    bool couldBeValid(
        const arr &start,
        const arr &end,
        const uint i){
      const bool res = isValidAtResolution(start, end, i);

      if (!res){
        return false;
      }

      return true;
    }

    uint getNumberOfCollisionChecks(const arr &start, const arr &end) const{
      const uint disc = uint(euclideanDistance(start, end) / resolution_);
      return std::max(2u, disc);
      /*if (disc < 2){
        disc = 2;
      }

      return disc;*/
    }

    uint calls_{0u};
    uint robot_calls_{0u};
    uint obs_calls_{0u};
    uint env_calls_{0u};

    std::shared_ptr<BaseCollisionChecker> cc_;

  private:
    bool isValidAtResolution(const arr &start, const arr &goal, const uint i){
      calls_++;
      robot_calls_++;
      obs_calls_++;
      env_calls_++;
      const double ind = corput.get(i+1);
      const arr p = start + ind * (goal-start);

      return cc_->isValid(p);
    }

};


#endif
