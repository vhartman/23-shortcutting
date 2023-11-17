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

