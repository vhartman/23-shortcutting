#include <KOMO/komo.h>
#include <Kin/F_operators.h>
#include <Kin/F_pose.h>
#include <Kin/F_qFeatures.h>
#include <Kin/featureSymbols.h>
#include <Kin/kin.h>

#ifndef _SAMPLER_H
#define _SAMPLER_H

// we need this separate class with its own rnd object, as otherwise we have issues with
// repeatability
class Initializer{
  public: 
    Initializer(const uint seed=0){
      rnd_.seed(seed);
    };

    virtual void init(KOMO &komo){
      komo.run_prepare(0.);

      const arr limits = komo.pathConfig.getLimits();
      const uint dim = limits.dim(0);
      arr sample(dim);

      // sample uniformly between 0,1
      localRndUniform(sample,0,1,false);

      // scale sample
      for (uint i=0; i<sample.d0; ++i){
        if(limits(i,1) > limits(i,0)){
          sample(i) = limits(i, 0) + sample(i) * (limits(i, 1) - limits(i, 0));
        }
        else {
          // default: [-5, 5]
          sample(i) = sample(i) * 10 - 5;
        }
      }

      komo.x = sample();
    }

    void localRndUniform(rai::Array<double>& a, double low=0., double high=1., bool add=false){
      if(!add) for(uint i=0; i<a.N; i++) a.p[i] =(double)rnd_.uni(low, high);
      else     for(uint i=0; i<a.N; i++) a.p[i]+=(double)rnd_.uni(low, high);
    }

    rai::Rnd rnd_;
};

class NoFreeJointsInitializer: public Initializer{
  public:
    NoFreeJointsInitializer(const uint seed=0): Initializer(seed){};

    virtual void init(KOMO &komo){
      komo.run_prepare(0.);

      const arr limits = komo.pathConfig.getLimits();
      const uint dim = limits.dim(0);
      arr sample(dim);

      // sample uniformly between 0,1
      localRndUniform(sample,0,1,false);

      // scale sample
      for (uint i=0; i<sample.d0; ++i){
        // check if the index is a free joint
        auto joints = komo.pathConfig.activeJoints;
        if (getJointTypeFromIndex(joints, i) != rai::JointType::JT_free){
          if(limits(i,1) > limits(i,0)){
            sample(i) = limits(i, 0) + sample(i) * (limits(i, 1) - limits(i, 0));
          }
          else {
            // default: [-5, 5]
            sample(i) = sample(i) * 10 - 5;
          }
        }
        else{
          sample(i) = komo.x(i);
        }
      }

      komo.x = sample();
    }

    rai::JointType getJointTypeFromIndex(JointL joints, uint index){
      for (auto j: joints){
        if (j->qIndex <= index && j->qIndex + j->dim > index){
          return j->type;
        }
      }

      std::cout << "can never be here (getJointType)" << std::endl;
    }
};

class JointKeyframeSampler{
  public:
    JointKeyframeSampler(const std::vector<arr> prevKeyframes, std::function<void (KOMO &, int)> komoInit, int phases=-1) 
      : fixedKeyframes_(prevKeyframes) {
      komoInit(komo_, phases);

      // get maximum time from komo
      uint maxPhases = 0u;
      auto ks = komo_.switches;
      for (uint j=0; j<ks.d0; ++j){
        if (ks(j)->timeOfApplication + 1 > maxPhases){
          maxPhases = ks(j)->timeOfApplication;
        }
      }
      if (maxPhases < prevKeyframes.size()){
        if(verbose_) std::cout << "Something is off with the previous keyframes" << std::endl;
      }

      setFixedKeyframes(fixedKeyframes_);
    };

    std::vector<arr> sampleAll(){
      calls_ += 1;
      for (uint i=1; i<maxAttempts_; ++i){
        attempts_ += 1;
        //std::cout << "A" << std::endl;

        if(verbose_)std::cout << "Attempt nr. " << i << " for JointKeyframeSampler" << std::endl;
        komo_.reset();
        
        //if (i==0) {komo_.run_prepare(0.0, true);}
        //else {komo_.run_prepare(0.2, true);}
        komo_.run_prepare(0.2, true);
        
        //komoInitializer_.init(komo_);
        //komo_.run_prepare(0.01, false);
        //komo_.run_prepare(0.0001, false);
        //komo_.run_prepare(0.05, false);
        //komo_.animateOptimization = 5;
    
        // watch initialization
        komo_.pathConfig.setJointState(komo_.x);
        //komo_.pathConfig.watch(true);

        komo_.run();
        std::cout << "found keyframe with ineq-value " << komo_.getReport(false).get<double>("ineq") << " " << komo_.getReport(false).get<double>("eq") << std::endl;
        //komo_.pathConfig.watch(true);
        if (komo_.getReport(false).get<double>("ineq") < 1. && 
            komo_.getReport(false).get<double>("eq") < 1.){
          if(verbose_) std::cout << "found valid keyframe with ineq-value " <<komo_.getReport(false).get<double>("ineq") << std::endl;
          const arr sol = komo_.getPath();

          //std::cout << fixedKeyframes_.size() << std::endl;
          // komo_.pathConfig.watch(true);

          std::vector<arr> ret{};
          for (uint j=0; j<sol.d0; ++j){
            ret.push_back(sol[j]());
          }

          return ret;
        }
      }

      if(verbose_) std::cout << "Was not able to find a solution" << std::endl;
      return {};
    }

    arr sample(){
      auto res = sampleAll();

      if (res.size() == 0){
        return {};
      }

      const uint keyframe = fixedKeyframes_.size();
      return res[keyframe];
    };

    NoFreeJointsInitializer komoInitializer_;
    //Initializer komoInitializer_;

    KOMO komo_;
    uint maxAttempts_{20u};

    uint verbose_{0u};

    uint calls_{0u};
    uint attempts_{0u};

  protected:
    std::vector<arr> fixedKeyframes_;

    void setFixedKeyframes(const std::vector<arr> &fixedKeyframes){
      if(verbose_) std::cout << "setting fixed keyframes" << std::endl;
      // determine the keyframe we are sampling at atm
      const uint keyframe = fixedKeyframes.size();
      const uint k_order = komo_.k_order;

      // extract active configurations
      FrameL activeConfigurations;
      for (uint t=0; t<komo_.timeSlices.d0-k_order; ++t){
       // if the configuration is not part of what we optimize, add it to the active configurations
        if (t >= keyframe){
          for(auto* f:komo_.timeSlices[t + k_order]) {
            if(f->joint && f->joint->active &&
                f->joint->type != rai::JT_rigid && f->joint->type != rai::JT_free){
              activeConfigurations.append(f);
            }
          }
        }
        // otherwise, set the joint state
        // TODO: fix this - this needs to set the state of the moved objects as well!
        else{
          FrameL F;
          for(auto* f:komo_.timeSlices[t + k_order]) {
            if(f->joint && f->joint->active &&
                f->joint->type != rai::JT_rigid && f->joint->type != rai::JT_free){
              F.append(f);
            }
          }
          //std::cout << "f:" << fixedKeyframes(t) << " " << f(t) << " " << fixedKeyframes(t) - f(t)<< std::endl;
          komo_.pathConfig.setJointState(fixedKeyframes[t], F);
        }
      }

      // set configuratiosn to active
      komo_.pathConfig.selectJoints(activeConfigurations);
    }
};

#endif
