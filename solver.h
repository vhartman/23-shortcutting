#include <functional>
#include <chrono>
#include <numeric> 
#include <algorithm>

#include <Manip/rrt.h>
#include <Manip/methods.h>
#include <PlanningSubroutines/ConfigurationProblem.h>

#include "solution.h"
#include "problem.h"
#include "sampler.h"
#include "planners.h"

#ifndef _SOLVER_H
#define _SOLVER_H

using namespace std::placeholders;

enum class PlannerType{
  RRTConnect=0,
  LPRM,
  EffortOrderedLPRM,

  HLPRM,
  EffortOrderedHLPRM,

  EffortOrderedHLPRMNoSparse,

  EIRM,
  EIRMv2,

  HLPRMUnmanagedGraph,
  EffortOrderedHLPRMUnmanagedGraph,
  EIRMUnmanagedGraph,

  OMPLRRTConnect,
  OMPLEITStar,
};

class Solver{
  public:
    Solver(std::shared_ptr<Problem> P) :P_(P){
      P_->initial_.calc_indexedActiveJoints();
      for (auto a: P_->initial_.activeJoints){
        joints_.append(a->frame->name);
      }
    };
    virtual Solution solve(const double maxRuntime, const bool optimize_solution=false) = 0;

    //virtual bool isOptimizingSolver() const = 0;

    uint verbose_{0u};
    uint seed_{0u};
      
  protected:
    const std::shared_ptr<Problem> P_;
    StringA joints_;
};

// this is very similar to the solver above, except that it keeps the planners in memory,
// and comes back to them. This makes it probabilistically complete
class JointPTR: public Solver{
  public:
    JointPTR(std::shared_ptr<Problem> P, PlannerType plannerType=PlannerType::RRTConnect)
      :Solver(P), plannerType_(plannerType){
        base_ = std::make_shared<SharedGraphInformation>();
      };

    virtual Solution solve(const double maxRuntime, const bool optimize_solution=false) override{
      auto fp = std::bind(&Problem::getKomoInstance, P_, _1, _2);
      JointKeyframeSampler sampler({}, fp);
      sampler.komoInitializer_.rnd_.seed(seed_);

      uint iterations = 0;

      std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
      while (true){
        iterations++;

        /*const double rnd = 1. * std::rand() / RAND_MAX;
        constexpr double pNewSolAttempt = 0.9;
        if (partialSols_.size() == 0 || rnd > pNewSolAttempt){// generate new solution attempt
          auto attempt = setupNewAttempt(sampler);   
          attempt.id = partialSols_.size();
          partialSols_.push_back(attempt);
        }*/

        if (partialSols_.size() == 0 || partialSols_.size()*partialSols_.size() <= iterations){
          auto attempt = setupNewAttempt(sampler);   
          attempt.id = partialSols_.size();
          partialSols_.push_back(attempt);
        }

        // pick one of the previous attempts
        PartialSolution &partialSol = pickFromAttempts();
        partialSol.attempts += 1;
        bool res = continueSolvingPreviousAttempt(partialSol);

        // if we found a solution, return it
        if (res){
          if (verbose_) std::cout << "found the valid solution in keyframe with id " << partialSol.id << std::endl;
  
          std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
          const double duration = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / (1.*1e6);
          
          return Solution(partialSol.path_);
        }
      };

      return Solution({});
    };

  private:
    struct PartialSolution{
      std::shared_ptr<ContinuingPlanner> planner;

      std::vector<arr> keyframes_;
      std::vector<arr> path_;

      uint id;
      uint attempts{0u};
    };

    PlannerType plannerType_;

    std::vector<PartialSolution> partialSols_;

    double duration_sampling{0.};
    double duration_planning{0.};

    double duration_planner_sampling{0.};
    double duration_planner_search{0.};
    double duration_planner_nn{0.};
    double duration_planner_verifying{0.};

    double calls{0.};

    PartialSolution& pickFromAttempts(){
      if(verbose_ > 3) std::cout << "Picking a solution attempt" << std::endl;

      if (false){
        const uint ind = std::rand() % partialSols_.size();
        if(verbose_ > 3) std::cout << '\t' << ind << std::endl;
        return partialSols_[ind];
      }
      else if(true){
        // pick the partial solution with the least attempts so far
        uint ind{0u};
        uint minAttempts = 1e6;
        uint cnt = 0;
        for (const auto &s: partialSols_){
          if (s.attempts < minAttempts){
            ind = cnt;
            minAttempts = s.attempts;
          }
          cnt ++;
        }
        if(verbose_ > 3) std::cout << "picked attempt " << ind << " with " << minAttempts << " attempts " << std::endl;
        return partialSols_[ind];
      }
      else{
        // this should probably be a UCT approach here
      }
    }

    bool checkIfKeyframesAreFeasible(const std::vector<arr>& keyframes){
      rai::Configuration C(P_->initial_);
      ConfigurationProblem cp(C);

      for (uint i=0; i<keyframes.size(); ++i){
        if (i>=1){
          const auto q = keyframes[i-1];
          cp.C.setJointState(q);
        }
        
        P_->setConfigurationToMode(cp.C, i);
        // set the right frames to be active
        cp.C.selectJointsByName(joints_);

        const auto q = keyframes[i];
        std::cout << q << std::endl;

        const auto res = cp.query(q);
        //cp.C.watch(true);
        if (!res->isFeasible){
          std::cout << "segment " << i << " not valid " << std::endl;
          //cp.C.watch(true);
          return false;
        }
      }
      return true;
    }

    PartialSolution setupNewAttempt(JointKeyframeSampler &sampler){
      if(verbose_ > 3) std::cout << "Setting up new solution attempt" << std::endl;
      PartialSolution p;

      // sample keyframes
      while (true){
        std::chrono::steady_clock::time_point begin_sampling = std::chrono::steady_clock::now();
        const auto keyframes = sampler.sampleAll();

        std::chrono::steady_clock::time_point end_sampling = std::chrono::steady_clock::now();
        duration_sampling += std::chrono::duration_cast<std::chrono::microseconds>(end_sampling - begin_sampling).count() / (1.*1e6);

        bool feasible = checkIfKeyframesAreFeasible(keyframes);
        if (feasible && keyframes.size() > 0){
          p.keyframes_ = keyframes;
          break;
        }
      }

      return p;
    }

    bool continueSolvingPreviousAttempt(PartialSolution &s){
      if(verbose_ > 3) {
        std::cout << "continuing solving attempt " << s.id << " (mode " << s.path_.size() << ")" << std::endl;
      }
      //while(true){
        if (!s.planner){
          // figure out which mode we are in
          const uint mode = s.path_.size();

          // setup planner for that mode
          rai::Configuration C(P_->initial_);
          const arr homePos = C.getJointState();

          // apply the right mode switches
          if(verbose_ > 3) std::cout << "Setting configuration to mode " << mode << std::endl;
          for (uint i=0; i<=mode; ++i){
            P_->setConfigurationToMode(C, i);

            C.selectJointsByName(joints_);
            C.setJointState(s.keyframes_[i]);

            std::cout <<s.keyframes_[i] << std::endl;
            //std::cout << C.cp.query(s.keyframes_[i])->isFeasible << std::endl;
          }

          // set the right frames to be active
          C.selectJointsByName(joints_);

          const arr q0 = (mode>0)?s.keyframes_[mode-1]:homePos; 
          const arr qT = s.keyframes_[mode]; 

          ConfigurationProblem cp(C);
          if (plannerType_ == PlannerType::RRTConnect){
            s.planner = std::make_shared<RRTConnect>(cp);
          }
          else if (plannerType_ == PlannerType::LPRM){
            s.planner = std::make_shared<LPRM>(cp, P_);
          }

          s.planner->setup(q0, qT);
        }

        if(verbose_ > 3) std::cout << "solving" << std::endl;
        std::chrono::steady_clock::time_point begin_planning = std::chrono::steady_clock::now();

        //const auto path = s.planner->planMore(10000);
        const auto path = s.planner->planMore(0.5);

        std::chrono::steady_clock::time_point end_planning = std::chrono::steady_clock::now();
        duration_planning += std::chrono::duration_cast<std::chrono::microseconds>(end_planning - begin_planning).count() / (1.*1e6);

        duration_planner_sampling += s.planner->duration_sampling;
        duration_planner_search += s.planner->duration_search;
        duration_planner_nn += s.planner->duration_nn;
        duration_planner_verifying += s.planner->duration_verifying;
        calls += s.planner->getCollCalls();

        s.planner->duration_sampling = 0.;
        s.planner->duration_search = 0.;
        s.planner->duration_nn = 0.;
        s.planner->duration_verifying = 0.;
        s.planner->resetCollCalls();

        if (path.d0 != 0){
          s.path_.push_back(path);
      
          if (s.path_.size() == s.keyframes_.size()){
            uint sumValid{0u};
            uint sumInvalid{0u};
            for (auto &e: base_->edges_){
              sumValid += e.second->reusedValid;
              sumInvalid += e.second->reusedInvalid;
            }

            std::cout << sumInvalid << " " << sumValid << std::endl;

            return true;
          }

          s.planner = nullptr;
        }

      return false;
    }

    std::shared_ptr<SharedGraphInformation> base_;

    uint cnt{0};
};

#endif
