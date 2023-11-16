#include <Kin/kin.h>
#include "corput.h"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>

struct CollisionInformation{
  bool all;

  bool robotEnv;
  bool robotObs;
  bool obsEnv;
};

class BaseCollisionChecker{
  public: 
    BaseCollisionChecker(){};

    virtual bool isValid(const arr &q) = 0;

    double calls_{0.};
  protected:
    uintA getCantCollidePairs(const rai::Configuration &C) const{
      uintA cantCollidePairs;
      for (uint i=0; i<C.frames.d0; ++i){
        const auto a = C.frames(i);
        for (uint j=i+1; j<C.frames.d0; ++j){
          const auto b = C.frames(j);

          if (!a->getShape().canCollideWith(b)){
            cantCollidePairs.append(TUP(a->ID, b->ID));
          }

          if (b == a->parent || a == b->parent){
            cantCollidePairs.append(TUP(a->ID, b->ID));
          }
        }
      }
      cantCollidePairs.reshape(-1,2);

      return cantCollidePairs;
    }

    uintA getPairs(const rai::Configuration &C, const StringA &frames) const{
      uintA pairs;

      for (uint i=0; i<frames.d0; ++i){
        const auto& a = C.getFrame(frames(i), false);
        if (!a){continue;}
        for (uint j=0; j<frames.d0; ++j){
          const auto& b = C.getFrame(frames(j), false);
          if (!b){continue;}
          if (i != j && a->getShape().cont != 0 && b->getShape().cont != 0){
            pairs.append(TUP(a->ID, b->ID));
          }
        }
      }
      pairs.reshape(-1,2);

      return pairs;
    }

    uintA getPairs(const rai::Configuration &C, const StringA &frames, const StringA &frames2) const{
      uintA pairs;

      for (uint i=0; i<frames.d0; ++i){
        const auto& a = C.getFrame(frames(i), false);
        if (!a){continue;}
        for (uint j=0; j<frames2.d0; ++j){
          const auto& b = C.getFrame(frames2(j), false);
          if (!b){continue;}
          if (a->getShape().cont != 0 && b->getShape().cont != 0){
            pairs.append(TUP(a->ID, b->ID));
          }
        }
      }
      pairs.reshape(-1,2);

      return pairs;
    }

    void deleteUnnecessaryFrames(rai::Configuration &C){
      FrameL allCollidingFrames;
      for (auto f: C.frames){
        if (f->shape && f->getShape().cont != 0){
          allCollidingFrames.append(f);
        }
      }

      FrameL joints = C.getJoints();
      std::vector<uint> ids;
      for (auto f: allCollidingFrames){
        ids.push_back(f->ID);
        rai::Frame* upwards = f->parent;
        while(upwards){
          ids.push_back(upwards->ID);
          upwards = upwards->parent;
        }
      }

      FrameL remove;
      for(auto f: C.frames){
        if (f->getShape().cont == 0 && std::find(ids.begin(), ids.end(), f->ID) == ids.end()){
          //std::cout << f->name << std::endl;
          remove.append(f);
        }
      }

      for (auto f: remove){
        delete f;
      }
    }
};

class aCollisionChecker: public BaseCollisionChecker{
  public: 
    aCollisionChecker(const rai::Configuration &C) :cp(C){
      deleteUnnecessaryFrames(cp.C);

      auto pairs = getCantCollidePairs(cp.C);
      //std::cout <<  pairs << std::endl;
      //for (uint i=0; i<pairs.d0; ++i){std::cout << C.getFrames(uintA{pairs[i](0)})(0)->name << " " << C.getFrames(uintA{pairs[i](1)})(0)->name << std::endl;}
      //for (auto f: cp.C.getFrames(getCantCollidePairs(cp.C))) {std::cout << f->name << std::endl;}
      cp.C.fcl()->deactivatePairs(getCantCollidePairs(cp.C));
    };

    virtual bool isValid(const arr &q){
      calls_++;
      auto qr = cp.query(q);

      // create collision-information object
      const bool res = qr->isFeasible;
      return res;
    };

    ConfigurationProblem cp;
};

class aTorqueCollisionChecker: public aCollisionChecker{
  public:
    aTorqueCollisionChecker(const rai::Configuration &C, const std::string &ee, const std::string &obj)
      : aCollisionChecker(C), ee_(ee), obj_(obj){};

    virtual bool isValid(const arr &q){
      calls_++;
      const bool collFeasible = aCollisionChecker::isValid(q);

      // geometry collsion check
      if (!collFeasible){
        return false;
      }

      // torque in endeffector collision check
      if (cp.C[STRING(obj_)]->parent->name == STRING(ee_)){
        const arr objPos = cp.C[STRING(obj_)]->getPosition();
        const arr eePos = cp.C[STRING(ee_)]->getPosition();

        const arr diff = objPos - eePos;
        const double l = length(diff({0,1}));

        //std::cout << l << " " << diff({0,1}) << std::endl;

        if (l > limit_){
          return false;
        }
      }

      return true;
    }

    double limit_{.3};
    std::string ee_;
    std::string obj_;
};

// This collision checker maintains 3 different conficurations, and separately checksthe ones that are necessary
class HierarchicalCollisionChecker: public BaseCollisionChecker{
  public:
    HierarchicalCollisionChecker(std::shared_ptr<Problem> p, const rai::Configuration &C)
      : C_(C), objs_(p->movableFrames_), robot_(p->robotFrames_){

      deleteUnnecessaryFrames(C_);

      //C_.write(std::cout);

      //C_.watch(true);

      const auto staticFrames = p->getStaticFrames();

      ccRobotEnv_ = std::make_shared<ConfigurationProblem>(p->getRobotStaticEnv());
      ccRobotObs_ = std::make_shared<ConfigurationProblem>(p->getRobotMoving());
      ccObsEnv_ = std::make_shared<ConfigurationProblem>(p->getMovingStaticEnv());

      // set up robot/env collision checker
      {
        rai::Configuration &tmp = ccRobotEnv_->C;

        const uintA staticPairs = getPairs(tmp, staticFrames);
        tmp.fcl()->deactivatePairs(staticPairs);

        tmp.fcl()->deactivatePairs(getCantCollidePairs(tmp));
        tmp.fcl()->deactivatePairs(getCantCollidePairsFromReference(tmp, C_));
        tmp.fcl()->stopEarly = true;

        //tmp.watch(true);
      }

      // set up obs/env collision checker
      {
        rai::Configuration &tmp = ccObsEnv_->C;

        uintA staticPairs = getPairs(tmp, staticFrames);
        tmp.fcl()->deactivatePairs(staticPairs);

        tmp.fcl()->deactivatePairs(getCantCollidePairs(tmp));
        tmp.fcl()->deactivatePairs(getCantCollidePairsFromReference(tmp, C_));
        tmp.fcl()->stopEarly = true;

        //tmp.watch(true);
      }

      // set up robot/obs checker
      {
        rai::Configuration &tmp = ccRobotObs_->C;

        const uintA robotPairs = getPairs(tmp, robot_);
        tmp.fcl()->deactivatePairs(robotPairs);

        const uintA objPairs = getPairs(tmp, objs_);
        tmp.fcl()->deactivatePairs(objPairs);

        tmp.fcl()->deactivatePairs(getCantCollidePairs(tmp));
        tmp.fcl()->deactivatePairs(getCantCollidePairsFromReference(tmp, C_));
        tmp.fcl()->stopEarly = true;

        //tmp.watch(true);
      }

      reference_frame_state.resize(C_.frames.N, 7);
      reference_frame_state_updated.resize(C_.frames.N);

      for (const auto& frame: C_.frames){
        reference_frame_state[frame->ID] = frame->getPose();
        reference_frame_state_updated[frame->ID] = 0;
        reference_frames[std::string(frame->name.p)] = frame;
      }

      obs_robot_frame_state.resize(ccRobotObs_->C.frames.N, 7);
      for (const auto& frame: ccRobotObs_->C.frames){
        if (frame->getShape().cont!=0){
          obs_robot_frames[std::string(frame->name.p)] = frame;
        }
      }

      obs_env_frame_state.resize(ccObsEnv_->C.frames.N, 7);
      for (const auto& frame: ccObsEnv_->C.frames){
        if (frame->getShape().cont!=0){
          obs_env_frames[std::string(frame->name.p)] = frame;
        }
      }

      robot_env_frame_state.resize(ccRobotEnv_->C.frames.N, 7);
      for (const auto& frame: ccRobotEnv_->C.frames){
        robot_env_frame_state[frame->ID] = reference_frames[std::string(frame->name.p)]->getPose();
        if (frame->getShape().cont!=0){
          robot_env_frames[std::string(frame->name.p)] = frame;
        }
      }
    };

    std::unordered_map<std::string, rai::Frame*> reference_frames;

    std::unordered_map<std::string, rai::Frame*> obs_robot_frames;
    std::unordered_map<std::string, rai::Frame*> obs_env_frames;
    std::unordered_map<std::string, rai::Frame*> robot_env_frames;

    arr reference_frame_state;
    std::vector<uint> reference_frame_state_updated;
    uint cache_id = 0;

    arr obs_robot_frame_state;
    arr obs_env_frame_state;
    arr robot_env_frame_state;

    virtual bool isValid(const arr &q){
      if (isValidForRobotEnv(q) && isValidForRobotObs(q) && isValidForObsEnv(q)){
        return true;
      }

      //std::cout << isValidForRobotEnv(q) << " " << isValidForRobotObs(q) << " " << isValidForObsEnv(q) << std::endl;

      return false;
    };

    uint robotEnvCalls{0u};
    bool isValidForRobotEnv(const arr &q) {
      calls_ += 1./3;
      robotEnvCalls++;
      // check robot-static env
      if (lastState != q){
        C_.setJointState(q);
        lastState = q;
        cache_id++;
      }
      //setFramesToPose(ccRobotEnv_->C, C_, robot_);
      setFramesToPose(robot_env_frames);
      //setFramesToPose(robot_env_frames, robot_env_frame_state);

      //ccRobotEnv_->C.setFrameState(robot_env_frame_state);
      const auto X = ccRobotEnv_->C.getFrameState();
      //std::cout << X - robot_env_frame_state << std::endl;
      //std::cout << robot_env_frame_state << std::endl;
      //const auto res = ccRobotEnv_->queryUsingFramestate(robot_env_frame_state);
      const auto res = ccRobotEnv_->queryUsingFramestate(X);
      //auto res = ccRobotEnv_->query(q, false);
      if (!res->isFeasible){
        return false;
      }
      return true;
    }

    bool isAlwaysValidForObsEnv() const {
      if (objs_.N == 0){
        return true;
      }
      return false;
    }

    uint obsEnvCalls{0u};
    bool isValidForObsEnv(const arr &q){
      calls_ += 1./3;
      obsEnvCalls++;

      if (objs_.N == 0){
        return true;
      }

      if (lastState != q){
        C_.setJointState(q);
        lastState = q;
        cache_id++;
      }
      //setFramesToPose(ccObsEnv_->C, C_, objs_);
      setFramesToPose(obs_env_frames);
      //setFramesToPose(obs_env_frames, obs_env_frame_state);

      const auto X = ccObsEnv_->C.getFrameState();
      const auto res = ccObsEnv_->queryUsingFramestate(X);
      if (!res->isFeasible){
        return false;
      }
      return true;
    }

    bool isAlwaysValidForRobotObs() const{
      if (objs_.N == 0){
        return true;
      }
      return false;
    }

    uint robotObsCalls{0u};
    bool isValidForRobotObs(const arr &q){
      calls_ += 1./3;
      robotObsCalls ++;

      if (objs_.N == 0){
        return true;
      }

      // check robot-moving obs
      if (lastState != q){
        C_.setJointState(q);
        lastState = q;
        cache_id++;
      }
      //setFramesToPose(ccRobotObs_->C, C_, objs_);
      //setFramesToPose(ccRobotObs_->C, C_, robot_);
      setFramesToPose(obs_robot_frames);

      // query(ccRobotObs, q)
      const auto X = ccRobotObs_->C.getFrameState();
      const auto res = ccRobotObs_->queryUsingFramestate(X);
      //const auto res = ccRobotObs_->query({}, false);
      if (!res->isFeasible){
        return false;
      }
      return true;
    }

  private:
    //void setFramesToPose(rai::Configuration &C, const rai::Configuration &ref, const StringA &frames) {
    void setFramesToPose(std::unordered_map<std::string, rai::Frame*>& frames_to_update) {
      arr ref_pose(7);
      arr pose(7);

      uint cnt = 0;
      for (const auto& [name, frame]: frames_to_update){
        const auto& ref_frame = reference_frames[name];

        ref_frame->getPoseInplace(ref_pose);
        frame->getPoseInplace(pose);
        //frame->X.getArr7dInplace(pose);

        if (maxDiff(pose, ref_pose) > 1e-8){
          frame->setPose(ref_pose);
          //frame->X = ref_pose;
          //frame_list.elem(cnt)=C[f];
          //X[cnt] = pose;
          //cnt++;
        }
      }
    }

    void setFramesToPose(std::unordered_map<std::string, rai::Frame*>& frames_to_update, arr& frame_state) {
      arr ref_pose(7);
      arr pose(7);

      uint cnt = 0;
      for (const auto& [name, frame]: frames_to_update){
        const auto& ref_frame = reference_frames[name];

        ref_frame->getPoseInplace(ref_pose);
        //frame->getPoseInplace(pose);
        //frame->X.getArr7dInplace(pose);

        if (maxDiff(frame_state[frame->ID], ref_pose) > 1e-8){
          frame_state[frame->ID] = ref_pose;
          //frame->setPose(ref_pose);
          //frame->X = ref_pose;
          //frame_list.elem(cnt)=C[f];
          //X[cnt] = pose;
          //cnt++;
        }
      }
    }

    uintA getCantCollidePairsFromReference(const rai::Configuration &C, const rai::Configuration &ref){
      uintA cantCollidePairs;

      for (uint i=0; i<C.frames.d0; ++i){
        const auto a = C.frames(i);
        const auto ref_a = ref.getFrame(a->name, false);
        if (ref_a == 0){continue;}

        for (uint j=i+1; j<C.frames.d0; ++j){
          const auto b = C.frames(j);
          const auto ref_b = ref.getFrame(b->name, false);
          if (ref_b == 0){continue;}

          if (!ref_a->getShape().canCollideWith(ref_b)){
            cantCollidePairs.append(TUP(a->ID, b->ID));
          }

          //if (ref_b == ref_a->parent || ref_a == ref_b->parent){
          //  cantCollidePairs.append(TUP(a->ID, b->ID));
          //}
        }
      }
      cantCollidePairs.reshape(-1,2);

      return cantCollidePairs;
    
    }

    arr lastState;
    rai::Configuration C_;

    StringA objs_;
    StringA robot_;

  public:
    std::shared_ptr<ConfigurationProblem> ccRobotEnv_;
    std::shared_ptr<ConfigurationProblem> ccRobotObs_;
    std::shared_ptr<ConfigurationProblem> ccObsEnv_;
};

// this maintains one configuration and deactivates the pairs that can not collide
class HierarchicalCollisionCheckerV2: public BaseCollisionChecker{
  public:
    HierarchicalCollisionCheckerV2(std::shared_ptr<Problem> p, const rai::Configuration &C)
    : cp(C), C_(C), objs_(p->movableFrames_), robot_(p->robotFrames_), staticFrames_(p->getStaticFrames()){
      deleteUnnecessaryFrames(cp.C);

      staticPairs = getPairs(cp.C, staticFrames_);
      robotPairs = getPairs(cp.C, robot_);
      robotStaticPairs = getPairs(cp.C, robot_, staticFrames_);
      robotObsPairs = getPairs(cp.C, robot_, objs_);
      staticObsPairs = getPairs(cp.C, staticFrames_, objs_);
      objPairs = getPairs(cp.C, objs_);

      /*std::cout << robot_ << std::endl;
      std::cout << staticFrames_ << std::endl;

      std::cout << robotStaticPairs << std::endl;
      std::cout << robotObsPairs << std::endl;
      std::cout << "" << std::endl;*/

      cantCollidPairs = getCantCollidePairs(cp.C);

      cp.C.fcl()->deactivatedPairs.clear();
      cp.C.fcl()->deactivatePairs(cantCollidPairs);
      cp.C.fcl()->deactivatePairs(staticPairs);

      //cp.C.fcl()->stopEarly = true;
      
      cp.C.fcl()->temporaryDeactivatedPairs.clear();
      cp.C.fcl()->temporaryDeactivatePairs(robotPairs);
      cp.C.fcl()->temporaryDeactivatePairs(robotStaticPairs);
      robot_env_deactivate_set_ = cp.C.fcl()->temporaryDeactivatedPairs;

      cp.C.fcl()->temporaryDeactivatedPairs.clear();
      cp.C.fcl()->temporaryDeactivatePairs(robotObsPairs);
      robot_obs_deactivate_set_ = cp.C.fcl()->temporaryDeactivatedPairs;

      cp.C.fcl()->temporaryDeactivatedPairs.clear();
      cp.C.fcl()->temporaryDeactivatePairs(staticObsPairs);
      cp.C.fcl()->temporaryDeactivatePairs(objPairs);
      obs_env_deactivate_set_ = cp.C.fcl()->temporaryDeactivatedPairs;

      cp.C.fcl()->temporaryDeactivatedPairs.clear();
    };

    uintA staticPairs;
    uintA robotPairs;
    uintA robotStaticPairs;
    uintA robotObsPairs;
    uintA staticObsPairs;
    uintA objPairs;
    uintA cantCollidPairs;

    std::unordered_set<std::size_t> robot_env_deactivate_set_;
    std::unordered_set<std::size_t> robot_obs_deactivate_set_;
    std::unordered_set<std::size_t> obs_env_deactivate_set_;

    bool isAlwaysValidForObsEnv() const {
      if (objs_.N == 0){
        return true;
      }
      return false;
    }
    bool isAlwaysValidForRobotObs() const{
      if (objs_.N == 0){
        return true;
      }
      return false;
    }

    std::shared_ptr<QueryResult> qr;
    virtual bool isValid(const arr &q){
      calls_++;
      qr = cp.query(q);

      // create collision-information object
      const bool res = qr->isFeasible;
      return res;
    };

    std::vector<std::unordered_set<std::size_t>*> vec;
    virtual bool isValid(const arr &q, const bool checkRobotEnv, const bool checkRobotObs, const bool checkObsEnv){
      rai::Configuration &C = cp.C;

      //C.fcl()->temporaryDeactivatedPairs.clear();
      vec.clear();

      if (!checkRobotEnv){
        // deactivate self collisions
        //C.fcl()->temporaryDeactivatePairs(robotPairs);

        // deactivate collisions with the environment
        //C.fcl()->temporaryDeactivatePairs(robotStaticPairs);
        //C.fcl()->temporaryDeactivatePairs(robot_env_deactivate_set_);
        vec.push_back(&robot_env_deactivate_set_);
      }

      if (!checkRobotObs){
        // deactivate robot obs
        //C.fcl()->temporaryDeactivatePairs(robotObsPairs);
        //C.fcl()->temporaryDeactivatePairs(robot_obs_deactivate_set_);
        vec.push_back(&robot_obs_deactivate_set_);
      }

      if (!checkObsEnv){
        // deactivate objects environment
        //C.fcl()->temporaryDeactivatePairs(staticObsPairs);
        
        // deactivate obs obs
        //C.fcl()->temporaryDeactivatePairs(objPairs);
        //C.fcl()->temporaryDeactivatePairs(obs_env_deactivate_set_);
        vec.push_back(&obs_env_deactivate_set_);
      }

      C.fcl()->vec = &vec;

      const auto res = isValid(q);
      C.fcl()->vec = nullptr;
      return res;
    };

    arr lastState;
    rai::Configuration C_;

    StringA objs_;
    StringA robot_;
    StringA staticFrames_;

    ConfigurationProblem cp;
};

// this maintains one configuration, but uses a specific fcl instance
class HierarchicalCollisionCheckerV4: public BaseCollisionChecker{
  public:
    bool has_moving_obstacle{false};

    HierarchicalCollisionCheckerV4(std::shared_ptr<Problem> p, const rai::Configuration &C)
    : cp(C), C_(C), objs_(p->movableFrames_), robot_(p->robotFrames_), staticFrames_(p->getStaticFrames()){
      deleteUnnecessaryFrames(cp.C);

      for (auto m: objs_){
        if (robot_.contains(C[m]->parent->name)){
          //std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
          has_moving_obstacle = true;
        }
      }

      /*for (size_t i=0; i<cp.C.frames.d0; ++i){
        std::cout << cp.C.frames(i)->ID << " " << cp.C.frames(i)->name << std::endl;
      }*/

      robot_ids = GetIdsFromName(cp.C, robot_);
      obs_ids = GetIdsFromName(cp.C, objs_);
      env_ids = GetIdsFromName(cp.C, staticFrames_);

      cp.C.InitSplitFcl(robot_ids, obs_ids, env_ids);

      robotPairs = getPairs(cp.C, robot_);
      robotStaticPairs = getPairs(cp.C, robot_, staticFrames_);
      robotObsPairs = getPairs(cp.C, robot_, objs_);
      staticObsPairs = getPairs(cp.C, staticFrames_, objs_);
      objPairs = getPairs(cp.C, objs_);

      cp.C.splitfcl()->deactivatedPairs.clear();
      cp.C.splitfcl()->deactivatePairs(robotPairs);
      cp.C.splitfcl()->deactivatePairs(robotStaticPairs);
      robot_env_deactivate_set_ = cp.C.splitfcl()->deactivatedPairs;

      cp.C.splitfcl()->deactivatedPairs.clear();
      cp.C.splitfcl()->deactivatePairs(robotObsPairs);
      robot_obs_deactivate_set_ = cp.C.splitfcl()->deactivatedPairs;

      cp.C.splitfcl()->deactivatedPairs.clear();
      cp.C.splitfcl()->deactivatePairs(staticObsPairs);
      cp.C.splitfcl()->deactivatePairs(objPairs);
      obs_env_deactivate_set_ = cp.C.splitfcl()->deactivatedPairs;

      cantCollidePairs = getCantCollidePairs(cp.C);
      staticPairs = getPairs(cp.C, staticFrames_);

      cp.C.splitfcl()->deactivatedPairs.clear();
      cp.C.splitfcl()->deactivatePairs(cantCollidePairs);
      cp.C.splitfcl()->deactivatePairs(staticPairs);

      // we can not stop early bc. this gives us more information
      cp.C.splitfcl()->stopEarly = true;
    };

    std::vector<size_t> robot_ids;
    std::vector<size_t> obs_ids;
    std::vector<size_t> env_ids;

    uintA robotPairs;
    uintA robotStaticPairs;
    uintA robotObsPairs;
    uintA staticObsPairs;
    uintA objPairs;

    std::unordered_set<std::size_t> robot_env_deactivate_set_;
    std::unordered_set<std::size_t> robot_obs_deactivate_set_;
    std::unordered_set<std::size_t> obs_env_deactivate_set_;

    std::vector<size_t> GetIdsFromName(const rai::Configuration& C, const StringA& names){
      std::vector<std::size_t> res;
      for (uint i=0; i<names.d0; ++i){
        const auto a = C.getFrame(names(i), false);
        if (!a){continue;}
        res.push_back(a->ID);
      }
      return res;
    }

    uintA staticPairs;
    uintA cantCollidePairs;

    bool isAlwaysValidForObsEnv() const {
      if (objs_.N == 0 || !has_moving_obstacle){
        return true;
      }
      return false;
    }
    bool isAlwaysValidForRobotObs() const{
      if (objs_.N == 0){
        return true;
      }
      return false;
    }

    bool isValidForRobotEnv(const arr &q) {
      return isValid(q, true, false, false);
    }

    bool isValidForObsEnv(const arr &q){
      if (objs_.N == 0){
        return true;
      }
      return isValid(q, false, true, false);
    }

    bool isValidForRobotObs(const arr &q){
      if (objs_.N == 0){
        return true;
      }
      return isValid(q, false, false, true);

    }

    std::shared_ptr<QueryResult> qr;
    virtual bool isValid(const arr &q){
      calls_++;
      return isValid(q, true, true, true);
    };

    virtual bool isValid(const arr &q, const bool checkRobotEnv, const bool checkRobotObs, const bool checkObsEnv){
      //std::cout << checkRobotEnv << " " << checkRobotObs << " " << checkObsEnv << std::endl;
      const auto res = cp.queryUsingSplitFcl(q, checkRobotEnv, checkRobotObs, checkObsEnv);
      return res->isFeasible;
    };

    arr lastState;
    rai::Configuration C_;

    StringA objs_;
    StringA robot_;
    StringA staticFrames_;

    ConfigurationProblem cp;
};

struct OMPLValidityChecker : ompl::base::StateValidityChecker {
  mutable HierarchicalCollisionCheckerV4 hcc;
  uint state_dimension_{0u};
  //arr x;
  mutable uint numQueries = 0;

  OMPLValidityChecker(std::shared_ptr<Problem> p, const rai::Configuration &C, const ompl::base::SpaceInformationPtr& si)
    : ompl::base::StateValidityChecker(si),
      hcc(p, C),
      state_dimension_(C.getJointState().N) {}

  virtual ~OMPLValidityChecker() = default;

  virtual bool isValid(const ompl::base::State *state) const {
    const double* _state = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    this->numQueries++;
    //if(!(numQueries%100)) cout <<"#queries: " <<numQueries <<endl;

    const arr x(_state, state_dimension_, true);
    return hcc.isValid(x, true, true, true);
  }
};

