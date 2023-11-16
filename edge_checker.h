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

#if 0
class HierarchicalEdgeCheckerRetired{
  public:
    HierarchicalEdgeCheckerRetired(std::shared_ptr<Problem> p, const rai::Configuration C) :hcc(p, C){};
    const double resolution{RESOLUTION};
    uint calls_{0u};
    
    bool isValid(HierarchicalEdge &e, const uint param) {
      //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      auto pe = e.getParametrizedEdge(param);

      // Both these checks here should not have to happen 
      // TODO: rethink design
      if (hcc.isAlwaysValidForRobotObs()){
        pe.validForRobotObs_ = EdgeStatus::valid;
        pe.collCheckLevelRobotObs_ = 1e6;
      }
      if (hcc.isAlwaysValidForObsEnv()){// check if this edge is valid
        pe.validForObsEnv_ = EdgeStatus::valid;
        pe.collCheckLevelObsEnv_ = 1e6;
      }

      if(pe.isValid()){
        //std::cout << "B" << std::endl;
        return true;
      }

      if(pe.isInvalid()){
        //std::cout << "A" << std::endl;
        return false;
      }

      // doing the compete collision check here
      const uint necessaryCollisionChecks = getNumberOfCollisionChecks(e.n1_->q_, e.n2_->q_);

      for (uint i=0; i<necessaryCollisionChecks; ++i){
        //const bool res = isValidAtResolution(e, param);
        const bool res = isValidAtResolution(pe, necessaryCollisionChecks);

        if (!res){
          // std::cout << "A " << necessaryCollisionChecks - i << std::endl;
          // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
          // std::cout << "A " << i << " " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / (1.*1e6) << std::endl;
          return false;
        }

        /*std::cout << "collchecks: " << e.collCheckLevelRobotEnv_ << " " 
          << e.collCheckLevelObsEnv_[param] << " "
          << e.collCheckLevelRobotObs_[param] << std::endl;*/

        if (pe.validForRobotEnv_ == EdgeStatus::valid && 
            pe.validForObsEnv_ == EdgeStatus::valid && 
            pe.validForRobotObs_ == EdgeStatus::valid){

          //e.n1_->whitelisted_.push_back(e.n2_);
          //e.n2_->whitelisted_.push_back(e.n1_);

          // std::cout << "B" << necessaryCollisionChecks - i << std::endl;
          // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
          // std::cout << "B " << i << " "<< std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / (1.*1e6) << std::endl;
          return true;
        }
      }

      return true;
    };

    bool couldBeValid(HierarchicalEdge &e, const uint param) {
      // only checking a single resolution-step
      return isValidAtResolution(e, param);
    };

    uint getNumberOfCollisionChecks(const arr &start, const arr &end) const{
      uint disc = uint(length(start-end) / resolution);
      if (disc < 2){
        disc = 2;
      }

      return disc;
    }
    HierarchicalCollisionChecker hcc;

  private:
    arr config;
    bool isValidAtResolution(ParametrizedHierarchicalEdge &pe, const uint necessaryCollisionChecks) {
      /*
       * Before checking anything, we make sure that the edge is not already marked as valid/invalid
       *
       * We then check three things separately here:
       * - first check robot/static env
       *   . if we have a collision, we can return early, since the edge will be invalid
       * - we then check moving/static env
       *   . we can again stop early in case of a collision
       * - finally, we check robot/obs
       */

      if (pe.isInvalid()){
        return false;
      }

      if (pe.isValid()){
        return true;
      }
      calls_++;

      const arr &start = pe.n1_->q_;
      const arr &goal = pe.n2_->q_;

      //ccRobotEnv_->C.watch(true);

      int i = -1;
      double ind = 0.5;

      // check robot-static env
      if (pe.validForRobotEnv_ == EdgeStatus::unknown){
        // std::cout << "doing robot/env check" << std::endl;
        i = pe.collCheckLevelRobotEnv_;
        ind = corput.get(i+1);
        config = start + ind * (goal-start);

        const auto valid = hcc.isValidForRobotEnv(config);
        pe.collCheckLevelRobotEnv_ += 1;

        if (!valid){
          pe.validForRobotEnv_ = EdgeStatus::invalid;

          // std::cout << "robot en invalid" << std::endl;

          return false;
        }
        else if (valid && pe.collCheckLevelRobotEnv_ >= necessaryCollisionChecks){ // check if this edge is valid
          pe.validForRobotEnv_ = EdgeStatus::valid;

          //e.n1_->whitelisted_.push_back(e.n2_);
          //e.n2_->whitelisted_.push_back(e.n1_);
        }
      }

      //ccObsEnv_->C.watch(true);

      // check moving obs-env
      if (pe.validForObsEnv_ == EdgeStatus::unknown){
        // std::cout << "doing obs/env check" << std::endl;
        if (i < 0 || i != pe.collCheckLevelObsEnv_){
          i = pe.collCheckLevelObsEnv_;
          ind = corput.get(i+1);
          config = start + ind * (goal-start);
        }
        const auto valid = hcc.isValidForObsEnv(config);

        pe.collCheckLevelObsEnv_ += 1;

        if (!valid){
          pe.validForObsEnv_ = EdgeStatus::invalid;
          // std::cout << "obs env invalid" << std::endl;

          return false;
        }
        else if (pe.collCheckLevelObsEnv_ >= necessaryCollisionChecks || hcc.isAlwaysValidForObsEnv()){// check if this edge is valid
          pe.validForObsEnv_ = EdgeStatus::valid;
        }
      }

      //ccRobotObs_->C.watch(true);
      
      // check robot-moving obs
      if (pe.validForRobotObs_ == EdgeStatus::unknown){
        // std::cout << "doing robot/obs check" << std::endl;
        if (i < 0 || i != pe.collCheckLevelRobotObs_){
          i = pe.collCheckLevelRobotObs_;
          ind = corput.get(i+1);
          config = start + ind * (goal-start);
        }
        const auto valid = hcc.isValidForRobotObs(config);

        pe.collCheckLevelRobotObs_ += 1;

        if (!valid){
          pe.validForRobotObs_ = EdgeStatus::invalid;
          //std::cout << "robot obs" << std::endl;

          return false;
        }
        else if (pe.collCheckLevelRobotObs_ >= necessaryCollisionChecks){// check if this edge is valid
          pe.validForRobotObs_ = EdgeStatus::valid;
        }
      }
     
      return true;
    }

    bool isValidAtResolution(HierarchicalEdge &e, const uint param) {
      //std::cout << "Q" << std::endl;
      /*
       * Before checking anything, we make sure that the edge is not already marked as valid/invalid
       *
       * We then check three things separately here:
       * - first check robot/static env
       *   . if we have a collision, we can return early, since the edge will be invalid
       * - we then check moving/static env
       *   . we can again stop early in case of a collision
       * - finally, we check robot/obs
       */

      if (e.isInvalid(param)){
        return false;
      }

      if (e.isValid(param)){
        return true;
      }
      calls_++;

      const arr start = e.n1_->q_;
      const arr goal = e.n2_->q_;

      arr q;
      uint prev_i;

      //ccRobotEnv_->C.watch(true);
      const uint necessaryCollisionChecks = getNumberOfCollisionChecks(start, goal);

      // check robot-static env
      if (e.validForRobotEnv_ == EdgeStatus::unknown){
        // std::cout << "doing robot/env check" << std::endl;
        const uint i = e.collCheckLevelRobotEnv_;
        prev_i = i;
        const double ind = corput.get(i+1);
        q = start + ind * (goal-start);

        const auto valid = hcc.isValidForRobotEnv(q);
        e.collCheckLevelRobotEnv_ += 1;

        if (!valid){
          e.validForRobotEnv_ = EdgeStatus::invalid;
          e.annotatedInvalidAt = param;

          return false;
        }
        else if (valid && e.collCheckLevelRobotEnv_ >= necessaryCollisionChecks){ // check if this edge is valid
          e.validForRobotEnv_ = EdgeStatus::valid;

          //e.n1_->whitelisted_.push_back(e.n2_);
          //e.n2_->whitelisted_.push_back(e.n1_);

          e.annotatedValidAt = param;
        }
      }

      //ccObsEnv_->C.watch(true);

      // check moving obs-env
      if (e.validForObsEnv_.count(param) == 0 ||
          e.validForObsEnv_.at(param) == EdgeStatus::unknown){
        // std::cout << "doing obs/env check" << std::endl;
        uint i = 0;
        if (e.collCheckLevelObsEnv_.count(param) > 0){
          i = e.collCheckLevelObsEnv_[param];
        }

        if (i != prev_i){
          const double ind = corput.get(i+1);
          q = start + ind * (goal-start);
        }

        const auto valid = hcc.isValidForObsEnv(q);
        if (e.collCheckLevelObsEnv_.count(param) > 0){
          e.collCheckLevelObsEnv_[param] += 1;
        }
        else{
          e.collCheckLevelObsEnv_[param] = 1;
        }

        if (!valid){
          e.validForObsEnv_[param] = EdgeStatus::invalid;

          return false;
        }
        else if (e.collCheckLevelObsEnv_[param] >= necessaryCollisionChecks){// check if this edge is valid
          e.validForObsEnv_[param] = EdgeStatus::valid;
        }
      }

      //ccRobotObs_->C.watch(true);
      
      // check robot-moving obs
      if (e.validForRobotObs_.count(param) == 0 ||
          e.validForRobotObs_.at(param) == EdgeStatus::unknown){
        // std::cout << "doing robot/obs check" << std::endl;
        uint i = 0;
        if (e.collCheckLevelRobotObs_.count(param) > 0){
          i = e.collCheckLevelRobotObs_[param];
        }

        if (i != prev_i){
          const double ind = corput.get(i+1);
          q = start + ind * (goal-start);
        }

        const auto valid = hcc.isValidForRobotObs(q);
        if (e.collCheckLevelRobotObs_.count(param) > 0){
          e.collCheckLevelRobotObs_[param] += 1;
        }
        else{
          e.collCheckLevelRobotObs_[param] = 1;
        }

        if (!valid){
          e.validForRobotObs_[param] = EdgeStatus::invalid;

          return false;
        }
        else if (e.collCheckLevelRobotObs_[param] >= necessaryCollisionChecks){// check if this edge is valid
          e.validForRobotObs_[param] = EdgeStatus::valid;
        }
      }
     
      return true;
    }


    // stats
    uint numEdgesChecked_{0u};
};
#endif

class HierarchicalEdgeCheckerV2{
  public:
    HierarchicalEdgeCheckerV2(std::shared_ptr<Problem> p, const rai::Configuration C){
      hcc = std::make_shared<HierarchicalCollisionCheckerV4>(p, C);

      config.resize(C.getJointState().N);
    };
    double resolution_{RESOLUTION};
    uint calls_{0u};

    uint robot_calls_{0u};
    uint obs_calls_{0u};
    uint env_calls_{0u};
    
    bool isValid(HierarchicalEdge &e, const uint param) {
      //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      auto pe = e.getParametrizedEdge(param);

      // Both these checks here should not have to happen 
      // TODO: rethink design
      if (pe.validForRobotObs_ == EdgeStatus::unknown && hcc->isAlwaysValidForRobotObs()){
        pe.validForRobotObs_ = EdgeStatus::valid;
        pe.collCheckLevelRobotObs_ = 1e6;
      }
      if (pe.validForObsEnv_ == EdgeStatus::unknown && hcc->isAlwaysValidForObsEnv()){// check if this edge is valid
        pe.validForObsEnv_ = EdgeStatus::valid;
        pe.collCheckLevelObsEnv_ = 1e6;
      }

      if(pe.isValid()){
        //std::cout << "B" << std::endl;
        return true;
      }

      if(pe.isInvalid()){
        return false;
      }

      // doing the compete collision check here
      const uint necessaryCollisionChecks = getNumberOfCollisionChecks(e.n1_->q_, e.n2_->q_);

      const arr &start = pe.n1_->q_;
      const arr &goal = pe.n2_->q_;

      const arr dist = goal - start;

      for (uint i=0; i<necessaryCollisionChecks; ++i){
        //const bool res = isValidAtResolution(e, param);
        const bool res = isValidAtResolution(pe, necessaryCollisionChecks, dist);

        if (!res){
          // std::cout << "A " << necessaryCollisionChecks - i << std::endl;
          // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
          // std::cout << "A " << i << " " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / (1.*1e6) << std::endl;
          return false;
        }

        /*std::cout << "collchecks: " << e.collCheckLevelRobotEnv_ << " " 
          << e.collCheckLevelObsEnv_[param] << " "
          << e.collCheckLevelRobotObs_[param] << std::endl;*/

        if (pe.validForRobotEnv_ == EdgeStatus::valid && 
            pe.validForObsEnv_ == EdgeStatus::valid && 
            pe.validForRobotObs_ == EdgeStatus::valid){

          //e.n1_->whitelisted_.push_back(e.n2_);
          //e.n2_->whitelisted_.push_back(e.n1_);

          // std::cout << "B" << necessaryCollisionChecks - i << std::endl;
          // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
          // std::cout << "B " << i << " "<< std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / (1.*1e6) << std::endl;
          return true;
        }
      }

      return true;
    };

    bool couldBeValid(HierarchicalEdge &e, const uint param) {
      // only checking a single resolution-step
      //std::cout << "AAAAAAAAA" << std::endl;
      return isValidAtResolution(e, param);
    };

    uint getNumberOfCollisionChecks(const arr &start, const arr &end) const{
      uint disc = uint(euclideanDistance(start, end) / resolution_);
      if (disc < 2){
        disc = 2;
      }

      return disc;
    }
    std::shared_ptr<HierarchicalCollisionCheckerV4> hcc;

  private:
    arr config;
    bool isValidAtResolution(HierarchicalEdge &e, const uint param) {
      auto pe = e.getParametrizedEdge(param);

      // doing the compete collision check here
      const uint necessaryCollisionChecks = getNumberOfCollisionChecks(e.n1_->q_, e.n2_->q_);
      return isValidAtResolution(pe, necessaryCollisionChecks);
    }

    bool isValidAtResolution(ParametrizedHierarchicalEdge &pe, const uint necessaryCollisionChecks, const arr& dist = {}) {
      if (hcc->isAlwaysValidForRobotObs()){
        pe.validForRobotObs_ = EdgeStatus::valid;
        pe.collCheckLevelRobotObs_ = necessaryCollisionChecks;
      }
      if (hcc->isAlwaysValidForObsEnv()){// check if this edge is valid
        pe.validForObsEnv_ = EdgeStatus::valid;
        pe.collCheckLevelObsEnv_ = necessaryCollisionChecks;
      }
      /*
       * Before checking anything, we make sure that the edge is not already marked as valid/invalid
       *
       * We then check three things separately here:
       * - first check robot/static env
       *   . if we have a collision, we can return early, since the edge will be invalid
       * - we then check moving/static env
       *   . we can again stop early in case of a collision
       * - finally, we check robot/obs
       */

      if (pe.isInvalid()){
        std::cout << "invalid" << std::endl;
        return false;
      }

      if (pe.isValid()){
        //std::cout << "valid" << std::endl;
        return true;
      }
      calls_++;

      const int minCollCheckLevel = std::min({pe.collCheckLevelRobotEnv_, pe.collCheckLevelObsEnv_, pe.collCheckLevelRobotObs_});
      //std::cout << minCollCheckLevel << std::endl;

      if (minCollCheckLevel >= necessaryCollisionChecks){
        std::cout << minCollCheckLevel << " " << necessaryCollisionChecks << std::endl;
        std::cout << "should never be here!" << std::endl;
        return true;
      }

      // Get the things that need to be checked
      bool checkRobotEnv = false;
      bool checkObsEnv = false;
      bool checkRobotObs = false;

      // check robot-static env
      if (pe.collCheckLevelRobotEnv_ == minCollCheckLevel){
        checkRobotEnv = true;
        robot_calls_++;
      }
      //else{std::cout << "saved something" << std::endl;}
      // check moving obs-env
      if (pe.collCheckLevelObsEnv_ == minCollCheckLevel){
        checkObsEnv = true;
        env_calls_++;
      }
      // check robot-moving obs
      if (pe.collCheckLevelRobotObs_ == minCollCheckLevel){
        checkRobotObs = true;
        obs_calls_++;
      }

      // get the position
      const double ind = corput.get(minCollCheckLevel+1);
      const arr &start = pe.n1_->q_;
      const arr &goal = pe.n2_->q_;

      if (dist.N == 0){
        const arr dist_tmp = goal - start;
        for (uint i=0; i<start.d0; ++i){
          config(i) = start(i) + ind * dist_tmp(i);
        }
      }
      else{
        for (uint i=0; i<start.d0; ++i){
          config(i) = start(i) + ind * dist(i);
        }
        //config = start + ind * dist;
      }

      // set up the collision checking
      //virtual bool isValid(const arr &q, const bool checkRobotEnv, const bool checkRobotObs, const bool checkObsEnv){
      //std::cout << checkRobotEnv << checkRobotObs << checkObsEnv << std::endl;
      const auto res = hcc->isValid(config, checkRobotEnv, checkRobotObs, checkObsEnv);

      /*if (!res){
        hcc->cp.C.watch(true);
      }*/

      // proxy hashes
      std::vector<std::size_t> proxy_hashes;
      const auto splitfcl = hcc->cp.C.splitfcl();
      for (const auto& p: hcc->cp.C.proxies){
        proxy_hashes.push_back(splitfcl->key(p.a->ID, p.b->ID));
        /*if(!res){
          std::cout << p.a->ID << " " << p.b->ID << std::endl;
          std::cout << p.a->name << " " << p.b->name << std::endl;
        }*/
      }

      /*if (!res) {
        std::cout << checkRobotEnv << checkRobotObs << checkObsEnv << std::endl;
        std::cout << res << std::endl;
        std::cout << proxy_hashes.size() << std::endl;
      }*/

      if (checkRobotEnv){
        pe.collCheckLevelRobotEnv_ += 1;
        // check if a robot-env collision was found
        if (!res){
          for (const auto& hash: proxy_hashes){
            if (hcc->robot_env_deactivate_set_.count(hash) > 0){
              pe.validForRobotEnv_ = EdgeStatus::invalid;
              //std::cout << "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB" << std::endl;
              break;
            }
          }
        }
        if (pe.validForRobotEnv_ == EdgeStatus::unknown && pe.collCheckLevelRobotEnv_ >= necessaryCollisionChecks){
          pe.validForRobotEnv_ = EdgeStatus::valid;
          pe.annotatedValidAt_ = pe.parameter_;
        }
      }
      if (checkObsEnv){
        pe.collCheckLevelObsEnv_ += 1;
        // check if a obs-env collision was found
        if (!res){
          for (const auto& hash: proxy_hashes){
            if (hcc->obs_env_deactivate_set_.count(hash) > 0){
              pe.validForObsEnv_ = EdgeStatus::invalid;
              //std::cout << "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC" << std::endl;
              break;
            }
          }
        }
        if (pe.validForObsEnv_ == EdgeStatus::unknown && pe.collCheckLevelObsEnv_ >= necessaryCollisionChecks){
          pe.validForObsEnv_ = EdgeStatus::valid;
        }
      }
      if (checkRobotObs){
        pe.collCheckLevelRobotObs_ += 1;
        // check if a robot-obs collision was found
        if (!res){
          for (const auto& hash: proxy_hashes){
            if (hcc->robot_obs_deactivate_set_.count(hash) > 0){
              pe.validForRobotObs_ = EdgeStatus::invalid;
              //std::cout << "CDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD" << std::endl;
              break;
            }
          }
        }
        if (pe.validForRobotObs_ == EdgeStatus::unknown && pe.collCheckLevelRobotObs_ >= necessaryCollisionChecks){
          pe.validForRobotObs_ = EdgeStatus::valid;
        }
      }

      return res;
    }

    // stats
    uint numEdgesChecked_{0u};
};



#endif
