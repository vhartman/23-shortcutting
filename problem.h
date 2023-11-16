#include <Kin/kin.h>
#include <KOMO/komo.h>

#include <Kin/kinViewer.h>
#include <Kin/viewer.h>

#include <random>

#ifndef _PROBLEM_H
#define _PROBLEM_H


void addBoxConstraint(KOMO &komo,
    double time, rai::String obj, rai::String prefix="", unsigned int axis=0){
  rai::String finger1 = STRING(prefix << "finger1");
  rai::String finger2 = STRING(prefix << "finger2");
  rai::String gripper = STRING(prefix << "gripper");
  rai::String center = STRING(prefix << "grippercenter");

  // touch constraint
  komo.addObjective({time}, FS_pairCollision_negScalar, {obj, finger1}, OT_sos, {1e1});
  komo.addObjective({time}, FS_pairCollision_negScalar, {obj, finger2}, OT_sos, {1e1});
  //komo.addObjective({2.}, FS_pairCollision_negScalar, {"goal2", "gripper"}, OT_eq, {1e1});

  switch(axis) {
    case 0:
      komo.addObjective({time}, FS_scalarProductXY, {gripper, obj}, OT_eq, {1e1});
      komo.addObjective({time}, FS_scalarProductXZ, {gripper, obj}, OT_eq, {1e1});
      break;
    case 1:
      komo.addObjective({time}, FS_scalarProductXX, {gripper, obj}, OT_eq, {1e1});
      komo.addObjective({time}, FS_scalarProductXZ, {gripper, obj}, OT_eq, {1e1});
      break;
    case 2:
      komo.addObjective({time}, FS_scalarProductXX, {gripper, obj}, OT_eq, {1e1});
      komo.addObjective({time}, FS_scalarProductXY, {gripper, obj}, OT_eq, {1e1});
      break;
    default: HALT("axis " <<axis <<" needs to be in {0,1,2}");
  }

  // inequality constraint
  komo.addObjective({time}, FS_insideBox, {center, obj}, OT_ineq, {1e2});

  // but still try to get as close as possible to the obj-center
  komo.addObjective({time},
      FS_positionDiff, {center, obj}, OT_sos, {1e1});

  //komo.addObjective({1.}, 
  //    FS_positionDiff, {"grippercenter", "obj"}, OT_ineq, {1e2}, {0.03, 0.03, 0.03});
  //komo.addObjective({1.}, 
  //    FS_positionDiff, {"grippercenter", "obj"}, OT_ineq, {-1e2}, {0.1, 0.1, 0.1});

  //komo.addObjective({1.},
    //  std::make_shared<F_Norm>(make_shared<F_PositionDiff>()), {"grippercenter", "obj"}, OT_ineq, {1e1}, {0.1});
  //komo.addObjective({1.}, F_Norm, {"grippercenter", "obj"}, OT_ineq, {1e2}, {0.01});
  //komo.addObjective({1.}, FS_distance, {"grippercenter", "objCenter"}, OT_sos, {1e2}, {0.01});
}

Skeleton filterSkeleton(const Skeleton &S, const uint t){
  Skeleton S2;

  for (auto entry: S){
    if (entry.phase0 <= t) {S2.append(entry);}
  }

  return S2;

}

class Problem{
  public:
    Problem(const std::string &name, const double collision_resolution = 0.01) 
      :name_(name), 
      collision_resolution_(collision_resolution){};
    uint getPhases() {komo_.run_prepare(0.); return komo_.switches.d0;};

    void displayProblem(){
      initial_.watch(true);
    };

    rai::Configuration getRobotStaticEnv() const {
      rai::Configuration C(initial_);

      FrameL remove;
      for(auto f: C.frames){
        if (movableFrames_.contains(f->name) || f->getShape().cont == 0){
          remove.append(f);
        }
      }

      for (auto f: remove){
        delete f;
      }

      // unlink everything
      for (auto f:C.frames){
        if (f->parent){
          f->unLink();
        }
      }

      return C;
    };
    rai::Configuration getRobotMoving() const {
      rai::Configuration C(initial_);

      FrameL remove;
      for(auto f: C.frames){
        if ((!movableFrames_.contains(f->name) && !robotFrames_.contains(f->name) && f->name != "World" && f->name != "World_v2")
            || f->getShape().cont == 0){
          remove.append(f);
        }
      }

      for (auto f: remove){
        delete f;
      }
      
      // unlink everything
      for (auto f:C.frames){
        if (f->parent){
          f->unLink();
        }
      }

      return C;
    };
    rai::Configuration getMovingStaticEnv() const {
      rai::Configuration C(initial_);

      FrameL remove;
      for(auto f: C.frames){
        if (robotFrames_.contains(f->name) || f->getShape().cont == 0){
          remove.append(f);
        }
      }

      for (auto f: remove){
        delete f;
      }

      // unlink everything
      for (auto f:C.frames){
        if (f->parent){
        f->unLink();
        }
      }

      return C;
    };

    virtual void perturb(const uint seed){
      std::cout << "No perturbation specified!" << std::endl;
      throw "AAAAAAAAAAAA";
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1) = 0;
    virtual void getContKomoInstance(KOMO &komo, const int steps=30){};

    void setConfigurationToMode(rai::Configuration &C, const uint t){
      const auto ks = komo_.switches;

      // iterate over all switches, and check if it is an active switch, if yes, apply it
      for (uint j=0; j<ks.d0; ++j){
        if (ks(j)->timeOfApplication+1 <= (int)t){
          //if(verbose_)std::cout << "Time of application " << ks(j)->timeOfApplication << std::endl;
          //if(verbose_)std::cout << "Applying switch at time " << t << std::endl;
          ks(j)->apply(C.frames);
        }
      }
    }

    virtual bool customCollisionChecker(rai::Configuration &C, const arr &q){
      return true;
    }

    // the name of the problem
    const std::string name_;

    // the initial state of the problem
    rai::Configuration initial_;

    StringA robotFrames_;
    StringA movableFrames_;

    StringA getStaticFrames(){
      StringA names;
      
      for(auto f: initial_.frames){
        if(f->shape && f->shape->cont!=0 && 
            !robotFrames_.contains(f->name) &&
            !movableFrames_.contains(f->name)) {
          names.append(f->name);
        }
      }

      return names;
    }
    
    // the komo instance contains the switches and the constraints for the keyframes
    KOMO komo_;

    // collision tolerance
    const double collisionMargin_{0.01};
    const double collisionScale_{1e1};

    double collision_resolution_;
};

class Empty: public Problem{
  public:
    Empty(): Problem("empty", 0.0001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      while(true){
        {
          auto agent = initial_["agent"];
          arr pos = agent->getPosition();
          std::uniform_real_distribution<> x_distr(-1.1,0); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_distr(-1.1,1.1); // distribution in range [1, 6]

          pos(0) = x_distr(rng);
          pos(1) = y_distr(rng);
          agent->setPosition(pos);
        }

        {
          std::uniform_real_distribution<> shape_distr(0.5,1.5); // distribution in range [1, 6]
          std::uniform_real_distribution<> x_pos_distr(0,1); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_pos_distr(-1,1); // distribution in range [1, 6]

          auto obj = initial_["obj"];
          obj->setShape(rai::ST_ssBox, {0.3*shape_distr(rng), 0.3*shape_distr(rng), .1, .005});

          arr pos = obj->getPosition();

          pos(0) = x_pos_distr(rng);
          pos(1) = y_pos_distr(rng);
          obj->setPosition(pos);
        }

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

    virtual void getContKomoInstance(KOMO &komo, const int steps=30){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      komo.setTiming(2., steps, steps, 1);
      //komo.setTiming(2);

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },
      };

      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/empty.g");
    }
};

class Empty2: public Problem{
  public:
    Empty2(): Problem("empty2", 0.0001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");

      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(4);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj1"}},
        { 1., 2., SY_stable, {"agent", "obj1"} },
        { 2., 2., SY_poseEq, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"table", "obj1"} },

        {3., 3., SY_touch, {"agent", "obj2"}},
        { 3., 4., SY_stable, {"agent", "obj2"} },
        { 4., 4., SY_poseEq, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"table", "obj2"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

    virtual void getContKomoInstance(KOMO &komo, const int steps=30){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      komo.setTiming(4., steps, steps, 1);
      //komo.setTiming(2);

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj1"}},
        { 1., 2., SY_stable, {"agent", "obj1"} },
        { 2., 2., SY_poseEq, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"table", "obj1"} },

        {3., 3., SY_touch, {"agent", "obj2"}},
        { 3., 4., SY_stable, {"agent", "obj2"} },
        { 4., 4., SY_poseEq, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"table", "obj2"} },
      };

      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/empty2.g");
    }
};

class Empty3: public Problem{
  public:
    Empty3(): Problem("empty3", 0.0001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");

      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj1"}},
        { 1., 2., SY_stable, {"agent", "obj1"} },
        { 2., 2., SY_poseEq, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"table", "obj1"} },

        {3., 3., SY_touch, {"agent", "obj2"}},
        { 3., 4., SY_stable, {"agent", "obj2"} },
        { 4., 4., SY_poseEq, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"table", "obj2"} },

        {5., 5., SY_touch, {"agent", "obj3"}},
        { 5., 6., SY_stable, {"agent", "obj3"} },
        { 6., 6., SY_poseEq, {"obj3", "goal3"} },
        { 6., -1, SY_stable, {"table", "obj3"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

    virtual void getContKomoInstance(KOMO &komo, const int steps=30){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      komo.setTiming(6., steps, steps, 1);
      //komo.setTiming(2);

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj1"}},
        { 1., 2., SY_stable, {"agent", "obj1"} },
        { 2., 2., SY_poseEq, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"table", "obj1"} },

        {3., 3., SY_touch, {"agent", "obj2"}},
        { 3., 4., SY_stable, {"agent", "obj2"} },
        { 4., 4., SY_poseEq, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"table", "obj2"} },

        {5., 5., SY_touch, {"agent", "obj3"}},
        { 5., 6., SY_stable, {"agent", "obj3"} },
        { 6., 6., SY_poseEq, {"obj3", "goal3"} },
        { 6., -1, SY_stable, {"table", "obj3"} },
      };

      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/empty3.g");
    }
};

class MoveOnly: public Problem{
  public:
    MoveOnly(): Problem("MoveOnly", 0.0001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_poseEq, {"agent", "start"}},
        {2., 2., SY_poseEq, {"agent", "goal"}},
        {3., 3., SY_poseEq, {"agent", "start"}},
        {4., 4., SY_poseEq, {"agent", "goal"}},
        {5., 5., SY_poseEq, {"agent", "start"}},
        {6., 6., SY_poseEq, {"agent", "goal"}},
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

    virtual void getContKomoInstance(KOMO &komo, const int steps=30){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      komo.setTiming(6., steps, steps, 1);
      //komo.setTiming(2);

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_poseEq, {"agent", "start"}},
        {2., 2., SY_poseEq, {"agent", "goal"}},
        {3., 3., SY_poseEq, {"agent", "start"}},
        {4., 4., SY_poseEq, {"agent", "goal"}},
        {5., 5., SY_poseEq, {"agent", "start"}},
        {6., 6., SY_poseEq, {"agent", "goal"}},
      };

      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/move_only.g");
    }
};

class MoveOnly2: public Problem{
  public:
    MoveOnly2(): Problem("MoveOnly2", 0.0001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      while(true){
        {
          auto agent = initial_["agent"];
          arr pos = agent->getPosition();
          std::uniform_real_distribution<> x_distr(-1.3,-0.5); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_distr(-1.,1.); // distribution in range [1, 6]

          pos(0) = x_distr(rng);
          pos(1) = y_distr(rng);
          agent->setPosition(pos);
        }

        {
          std::uniform_real_distribution<> x_pos_distr(0.5, 1.1); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_pos_distr(-0.8,0.8); // distribution in range [1, 6]

          auto obj = initial_["goal"];

          arr pos = obj->getPosition();

          pos(0) = x_pos_distr(rng);
          pos(1) = y_pos_distr(rng);
          obj->setPosition(pos);
        }

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(1);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_poseEq, {"agent", "goal"}},
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

    virtual void getContKomoInstance(KOMO &komo, const int steps=30){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      komo.setTiming(1., steps, steps, 1);
      //komo.setTiming(2);

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_poseEq, {"agent", "goal"}},
      };

      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/move_only.g");
    }
};

class MoveOnly3: public Problem{
  public:
    MoveOnly3(): Problem("MoveOnly3", 0.0001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      while(true){
        {
          auto agent = initial_["agent"];
          arr pos = agent->getPosition();
          std::uniform_real_distribution<> x_distr(-1.3,-0.5); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_distr(-1.,1.); // distribution in range [1, 6]

          pos(0) = x_distr(rng);
          pos(1) = y_distr(rng);
          agent->setPosition(pos);
        }

        {
          std::uniform_real_distribution<> x_pos_distr(0.5, 1.1); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_pos_distr(-0.8,0.8); // distribution in range [1, 6]

          auto obj = initial_["goal"];

          arr pos = obj->getPosition();

          pos(0) = x_pos_distr(rng);
          pos(1) = y_pos_distr(rng);
          obj->setPosition(pos);
        }

        {
          std::uniform_real_distribution<> x_pos_distr(-1.1,-0.5); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_pos_distr(-0.8,0.8); // distribution in range [1, 6]

          auto obj = initial_["start"];

          arr pos = obj->getPosition();

          pos(0) = x_pos_distr(rng);
          pos(1) = y_pos_distr(rng);
          obj->setPosition(pos);
        }

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_poseEq, {"agent", "goal"}},
        {2., 2., SY_poseEq, {"agent", "start"}},
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

    virtual void getContKomoInstance(KOMO &komo, const int steps=30){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      komo.setTiming(1., steps, steps, 1);
      //komo.setTiming(2);

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_poseEq, {"agent", "goal"}},
      };

      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/move_only.g");
    }
};

class MoveOnlyNoObs: public Problem{
  public:
    MoveOnlyNoObs(): Problem("MoveOnlyNoObs", 0.0001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_poseEq, {"agent", "start"}},
        {2., 2., SY_poseEq, {"agent", "goal"}},
        {3., 3., SY_poseEq, {"agent", "start"}},
        {4., 4., SY_poseEq, {"agent", "goal"}},
        {5., 5., SY_poseEq, {"agent", "start"}},
        {6., 6., SY_poseEq, {"agent", "goal"}},
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

    virtual void getContKomoInstance(KOMO &komo, const int steps=30){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      komo.setTiming(6., steps, steps, 1);
      //komo.setTiming(2);

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_poseEq, {"agent", "start"}},
        {2., 2., SY_poseEq, {"agent", "goal"}},
        {3., 3., SY_poseEq, {"agent", "start"}},
        {4., 4., SY_poseEq, {"agent", "goal"}},
        {5., 5., SY_poseEq, {"agent", "start"}},
        {6., 6., SY_poseEq, {"agent", "goal"}},
      };

      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/move_only_2.g");
    }
};

class FourRooms: public Problem{
  public:
    FourRooms(): Problem("FourRooms", 0.0001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(5);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_poseEq, {"agent", "start"}},
        {2., 2., SY_poseEq, {"agent", "goal1"}},
        {3., 3., SY_poseEq, {"agent", "goal2"}},
        {4., 4., SY_poseEq, {"agent", "goal3"}},
        {5., 5., SY_poseEq, {"agent", "start"}},
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

    virtual void getContKomoInstance(KOMO &komo, const int steps=30){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      komo.setTiming(6., steps, steps, 1);
      //komo.setTiming(2);

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_poseEq, {"agent", "start"}},
        {2., 2., SY_poseEq, {"agent", "goal1"}},
        {3., 3., SY_poseEq, {"agent", "goal2"}},
        {4., 4., SY_poseEq, {"agent", "goal3"}},
        {5., 5., SY_poseEq, {"agent", "start"}},
      };

      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/four_rooms.g");
    }
};

class Svetlana: public Problem{
  public:
    Svetlana(): Problem("svetlana", 0.0001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      while(true){
        {
          auto agent = initial_["agent"];
          arr pos = agent->getPosition();
          std::uniform_real_distribution<> x_distr(-1.1,-0.3); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_distr(-1.1,1.1); // distribution in range [1, 6]

          pos(0) = x_distr(rng);
          pos(1) = y_distr(rng);
          agent->setPosition(pos);
        }

        {
          std::uniform_real_distribution<> shape_distr(0.5,1.5); // distribution in range [1, 6]
          std::uniform_real_distribution<> x_pos_distr(0.3,1); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_pos_distr(-1,1); // distribution in range [1, 6]

          auto obj = initial_["obj"];
          obj->setShape(rai::ST_ssBox, {0.3*shape_distr(rng), 0.3, .1, .005});

          arr pos = obj->getPosition();

          pos(0) = x_pos_distr(rng);
          pos(1) = y_pos_distr(rng);
          obj->setPosition(pos);
        }

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

    virtual void getContKomoInstance(KOMO &komo, const int steps=30){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      komo.setTiming(2., steps, steps, 1);
      //komo.setTiming(2);

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },
      };

      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/svetlana.g");
    }
};

class SvetlanaEasy: public Problem{
  public:
    SvetlanaEasy(): Problem("svetlana_easy", 0.0001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      while(true){
        {
          auto agent = initial_["agent"];
          arr pos = agent->getPosition();
          std::uniform_real_distribution<> x_distr(-1.1,-0.3); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_distr(-1.1,0); // distribution in range [1, 6]

          pos(0) = x_distr(rng);
          pos(1) = y_distr(rng);
          agent->setPosition(pos);
        }

        {
          std::uniform_real_distribution<> shape_distr(0.5,1.5); // distribution in range [1, 6]
          std::uniform_real_distribution<> x_pos_distr(0.3,1); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_pos_distr(-1,0); // distribution in range [1, 6]

          auto obj = initial_["obj"];
          obj->setShape(rai::ST_ssBox, {0.3*shape_distr(rng), 0.3, .1, .005});

          arr pos = obj->getPosition();

          pos(0) = x_pos_distr(rng);
          pos(1) = y_pos_distr(rng);
          obj->setPosition(pos);
        }

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

    virtual void getContKomoInstance(KOMO &komo, const int steps=30){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      komo.setTiming(2., steps, steps, 1);
      //komo.setTiming(2);

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },
      };

      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/svetlana_easy.g");
    }
};

class MultipleSvetlana: public Problem{
  public:
    MultipleSvetlana(): Problem("MultipleSvetlana", 0.0001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj");
      movableFrames_.append("obj2");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(4);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },

        {3., 3., SY_touch, {"agent", "obj2"}},
        { 3., 4., SY_stable, {"agent", "obj2"} },
        { 4., 4., SY_poseEq, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"table", "obj2"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

    virtual void getContKomoInstance(KOMO &komo, const int steps=30){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      komo.setTiming(4., steps, steps, 1);
      //komo.setTiming(2);

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },

        {3., 3., SY_touch, {"agent", "obj2"}},
        { 3., 4., SY_stable, {"agent", "obj2"} },
        { 4., 4., SY_poseEq, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"table", "obj2"} },
      };

      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/multiple_svetlana.g");
    }
};

class OptimalSvetlana: public Problem{
  public:
    OptimalSvetlana(): Problem("opt_svetlana"){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/opt_svetlana.g");
    }

};

class Sofa: public Problem{
  public:
    Sofa(): Problem("sofa", 0.001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      while(true){
        {
          auto agent = initial_["agent"];
          arr pos = agent->getPosition();
          std::uniform_real_distribution<> x_distr(-1.1,1.1); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_distr(-1.1,-0.5); // distribution in range [1, 6]

          pos(0) = x_distr(rng);
          pos(1) = y_distr(rng);
          agent->setPosition(pos);
        }

        {
          std::uniform_real_distribution<> shape_distr(0.5,1); // distribution in range [1, 6]
          std::uniform_real_distribution<> x_pos_distr(0.4,1.5); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_pos_distr(0.2,1.5); // distribution in range [1, 6]

          auto obj = initial_["obj"];
          obj->setShape(rai::ST_ssBox, {0.3*shape_distr(rng), 0.6*shape_distr(rng), .1, .005});

          arr pos = obj->getPosition();

          pos(0) = x_pos_distr(rng);
          pos(1) = y_pos_distr(rng);
          obj->setPosition(pos);
        }

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/sofa.g");
    }

};

class MultipleSofa: public Problem{
  public:
    MultipleSofa(): Problem("MultipleSofa", 0.001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(4);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj1"}},
        { 1., 2., SY_stable, {"agent", "obj1"} },
        { 2., 2., SY_poseEq, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"table", "obj1"} },

        {3., 3., SY_touch, {"agent", "obj2"}},
        { 3., 4., SY_stable, {"agent", "obj2"} },
        { 4., 4., SY_poseEq, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"table", "obj2"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/multiple_sofa.g");
    }

};

class Quim: public Problem{
  public:
    Quim(): Problem("quim", 0.001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      while(true){
        {
          auto agent = initial_["agent"];
          arr pos = agent->getPosition();
          std::uniform_real_distribution<> x_distr(-0.6,1.1); // distribution in range [1, 6]
          std::uniform_real_distribution<> y_distr(-1.1,1.1); // distribution in range [1, 6]

          pos(0) = x_distr(rng);
          pos(1) = y_distr(rng);
          agent->setPosition(pos);
        }

        std::uniform_real_distribution<> x_pos_distr(0.3,1); // distribution in range [1, 6]
        std::uniform_real_distribution<> y_pos_distr(-0.5,1); // distribution in range [1, 6]

        auto obj1 = initial_["obj1"];
        arr pos1 = obj1->getPosition();

        pos1(0) = x_pos_distr(rng);
        pos1(1) = y_pos_distr(rng);
        obj1->setPosition(pos1);

        auto obj2 = initial_["obj2"];
        arr pos2 = obj2->getPosition();

        pos2(0) = x_pos_distr(rng);
        pos2(1) = y_pos_distr(rng);
        obj2->setPosition(pos2);

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible && length(pos1 - pos2) > 0.5 && 
            length(initial_["agent"]->getPosition() - pos1) > 0.4 &&
            length(initial_["agent"]->getPosition() - pos2) > 0.4){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(4);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj1"}},
        { 1., 2., SY_stable, {"agent", "obj1"} },
        { 2., 2., SY_inside, {"obj1", "goal"} },
        { 2., -1, SY_stable, {"table", "obj1"} },

        {3., 3., SY_touch, {"agent", "obj2"}},
        { 3., 4., SY_stable, {"agent", "obj2"} },
        { 4., 4., SY_inside, {"obj2", "goal"} },
        { 4., -1, SY_stable, {"table", "obj2"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/quim.g");
    }
};

class Puzzle: public Problem{
  public:
    Puzzle(): Problem("puzzle", 0.001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(4);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 3., SY_stable, {"table", "obj"} },

        {3., 3., SY_touch, {"agent", "obj"}},
        { 3., 4., SY_stable, {"agent", "obj"} },
        { 4., 4., SY_inside, {"obj", "goal"} },
        { 4., -1, SY_stable, {"table", "obj"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/puzzle.g");
    }
};

class Cage2d: public Problem{
  public:
    Cage2d(): Problem("cage_2d", 0.001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/2d_cage.g");
    }

};

class Obstacle2d: public Problem{
  public:
    Obstacle2d(): Problem("obstacle_2d", 0.001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(3);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., -1, SY_stable, {"table", "obj"} },
        { 3., 3., SY_poseEq, {"agent", "goal"} },
        { 3., -1, SY_stable, {"agent", "goal"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/obstacle.g");
    }

};

class Insertion2d: public Problem{
  public:
    Insertion2d(): Problem("insertion_2d", 0.001){
      setup();
      getKomoInstance(komo_);

      robotFrames_.append("agent");
      movableFrames_.append("obj");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

    virtual void getContKomoInstance(KOMO &komo, const int steps=30){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      komo.setTiming(2., steps, steps, 1);

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"agent", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"table", "obj"} },
      };

      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/2d_insertion.g");
    }
};

/////////////////
// 3D Problems //
/////////////////

class Wine: public Problem{
  public:
    Wine() :Problem("bottle", 0.01){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("bottle");
      //movableFrames_.append("bottle_base");
      //movableFrames_.append("bottle_neck");
      //movableFrames_.append("bottle_cylinder");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        { 1., 1., SY_touch, {"a0_finger1", "bottle"}},
        { 1., 1., SY_touch, {"a0_finger2", "bottle"}},
        { 1., 2., SY_stable, {"a0_gripper", "bottle"} },
        { 2., 2., SY_poseEq, {"goalPose", "bottle"} },
        //{ 2., 2., SY_touch, {"goal", "bottle"} },
        { 2., 3., SY_stable, {"table", "bottle"} },

        /*{ 3., 3., SY_touch, {"a0_finger1", "bottle"}},
        { 3., 3., SY_touch, {"a0_finger2", "bottle"}},
        { 3., 4., SY_stable, {"bottle", "a0_finger2"} },
        {4., 4., SY_touch, {"bottle", "goal"}},
        { 4., -1, SY_stableOn, {"table", "bottle"} },*/
      };

      //if (maxPhase == -1 || maxPhase >= 2) komo.addObjective({2.}, FS_scalarProductZZ, {"bottle", "table"}, OT_eq, {1e1}, {1});
      //if (maxPhase == -1 || maxPhase >= 4) komo.addObjective({4.}, FS_scalarProductZZ, {"bottle", "table"}, OT_eq, {1e1}, {1});

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/bottle.g");
    }
};

class StackMobile: public Problem{
  public:
    StackMobile(): Problem("StackMobile", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("obj");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "obj"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj"} },
        //{ 2., 2., SY_poseEq, {"stick", "goal"} },
        {2., 2., SY_positionEq, {"goal", "obj"}},
        { 2., -1, SY_stable, {"goal", "obj"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/stack_mobile.g");
    }
};

class Wall: public Problem{
  public:
    Wall(): Problem("Wall", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
      movableFrames_.append("obj4");
      movableFrames_.append("obj5");
      movableFrames_.append("obj6");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 12){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },
        {2., 2., SY_poseEq, {"goal1", "obj1"}},
        { 2., -1, SY_stable, {"goal1", "obj1"} },
        
        {3., 3., SY_touch, {"a0_gripper", "obj2"}},
        { 3., 4., SY_stable, {"a0_gripper", "obj2"} },
        {4., 4., SY_poseEq, {"goal2", "obj2"}},
        { 4., -1, SY_stable, {"goal2", "obj2"} },
        
        {5., 5., SY_touch, {"a0_gripper", "obj3"}},
        { 5., 6., SY_stable, {"a0_gripper", "obj3"} },
        {6., 6., SY_poseEq, {"goal3", "obj3"}},
        { 6., -1, SY_stable, {"goal3", "obj3"} },

        /*{7., 7., SY_touch, {"a0_gripper", "obj4"}},
        { 7., 8., SY_stable, {"a0_gripper", "obj4"} },
        {8., 8., SY_poseEq, {"goal4", "obj4"}},
        { 8., -1, SY_stable, {"goal4", "obj4"} },
        
        {9., 9., SY_touch, {"a0_gripper", "obj5"}},
        { 9., 10., SY_stable, {"a0_gripper", "obj5"} },
        {10., 10., SY_poseEq, {"goal5", "obj5"}},
        {10., -1, SY_stable, {"goal5", "obj5"} },

        {11., 11., SY_touch, {"a0_gripper", "obj6"}},
        {11., 12., SY_stable, {"a0_gripper", "obj6"} },
        {12., 12., SY_poseEq, {"goal6", "obj6"}},
        {12., -1, SY_stable, {"goal6", "obj6"} },*/
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/wall.g");
    }
};

class WallMoveOnly: public Problem{
  public:
    WallMoveOnly(): Problem("WallMoveOnly", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      //movableFrames_.append("obj1");
      //movableFrames_.append("obj2");
      //movableFrames_.append("obj3");
      //movableFrames_.append("obj4");
      //movableFrames_.append("obj5");
      //movableFrames_.append("obj6");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 12){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "obj1"}},
        {2., 2., SY_touch, {"a0_gripper", "goal2"}},
        
        {3., 3., SY_touch, {"a0_gripper", "obj1"}},
        {4., 4., SY_touch, {"a0_gripper", "goal1"}},
        
        {5., 5., SY_touch, {"a0_gripper", "obj3"}},
        {6., 6., SY_touch, {"a0_gripper", "goal3"}},

        /*{7., 7., SY_touch, {"a0_gripper", "obj4"}},
        { 7., 8., SY_stable, {"a0_gripper", "obj4"} },
        {8., 8., SY_poseEq, {"goal4", "obj4"}},
        { 8., -1, SY_stable, {"goal4", "obj4"} },
        
        {9., 9., SY_touch, {"a0_gripper", "obj5"}},
        { 9., 10., SY_stable, {"a0_gripper", "obj5"} },
        {10., 10., SY_poseEq, {"goal5", "obj5"}},
        {10., -1, SY_stable, {"goal5", "obj5"} },

        {11., 11., SY_touch, {"a0_gripper", "obj6"}},
        {11., 12., SY_stable, {"a0_gripper", "obj6"} },
        {12., 12., SY_poseEq, {"goal6", "obj6"}},
        {12., -1, SY_stable, {"goal6", "obj6"} },*/
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/wall.g");
    }
};

class BrickWall: public Problem{
  public:
    BrickWall(): Problem("BrickWall", 0.01){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
      //movableFrames_.append("obj4");
      //movableFrames_.append("obj5");
      //movableFrames_.append("obj6");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 12){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "obj1"}},
        {1., 2., SY_stable, {"a0_gripper", "obj1"} },
        {2., 2., SY_poseEq, {"goal1", "obj1"}},
        {2., -1, SY_stable, {"goal1", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper", "obj2"}},
        {3., 4., SY_stable, {"a0_gripper", "obj2"} },
        {4., 4., SY_poseEq, {"goal2", "obj2"}},
        {4., -1, SY_stable, {"goal2", "obj2"} },

        {5., 5., SY_touch, {"a0_gripper", "obj3"}},
        {5., 6., SY_stable, {"a0_gripper", "obj3"} },
        {6., 6., SY_poseEq, {"goal3", "obj3"}},
        {6., -1, SY_stable, {"goal3", "obj3"} },

        /*{7., 7., SY_touch, {"a0_gripper", "obj4"}},
        { 7., 8., SY_stable, {"a0_gripper", "obj4"} },
        {8., 8., SY_poseEq, {"goal4", "obj4"}},
        { 8., -1, SY_stable, {"goal4", "obj4"} },
        
        {9., 9., SY_touch, {"a0_gripper", "obj5"}},
        { 9., 10., SY_stable, {"a0_gripper", "obj5"} },
        {10., 10., SY_poseEq, {"goal5", "obj5"}},
        {10., -1, SY_stable, {"goal5", "obj5"} },

        {11., 11., SY_touch, {"a0_gripper", "obj6"}},
        {11., 12., SY_stable, {"a0_gripper", "obj6"} },
        {12., 12., SY_poseEq, {"goal6", "obj6"}},
        {12., -1, SY_stable, {"goal6", "obj6"} },*/
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/wall_2.g");
    }
};

class BrickWall2: public Problem{
  public:
    BrickWall2(): Problem("BrickWall2", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
      //movableFrames_.append("obj4");
      //movableFrames_.append("obj5");
      //movableFrames_.append("obj6");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 12){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "obj1"}},
        {1., 2., SY_stable, {"a0_gripper", "obj1"} },
        {2., 2., SY_poseEq, {"goal1", "obj1"}},
        {2., -1, SY_stable, {"goal1", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper", "obj2"}},
        {3., 4., SY_stable, {"a0_gripper", "obj2"} },
        {4., 4., SY_poseEq, {"goal2", "obj2"}},
        {4., -1, SY_stable, {"goal2", "obj2"} },

        {5., 5., SY_touch, {"a0_gripper", "obj3"}},
        {5., 6., SY_stable, {"a0_gripper", "obj3"} },
        {6., 6., SY_poseEq, {"goal3", "obj3"}},
        {6., -1, SY_stable, {"goal3", "obj3"} },

        /*{7., 7., SY_touch, {"a0_gripper", "obj4"}},
        { 7., 8., SY_stable, {"a0_gripper", "obj4"} },
        {8., 8., SY_poseEq, {"goal4", "obj4"}},
        { 8., -1, SY_stable, {"goal4", "obj4"} },
        
        {9., 9., SY_touch, {"a0_gripper", "obj5"}},
        { 9., 10., SY_stable, {"a0_gripper", "obj5"} },
        {10., 10., SY_poseEq, {"goal5", "obj5"}},
        {10., -1, SY_stable, {"goal5", "obj5"} },

        {11., 11., SY_touch, {"a0_gripper", "obj6"}},
        {11., 12., SY_stable, {"a0_gripper", "obj6"} },
        {12., 12., SY_poseEq, {"goal6", "obj6"}},
        {12., -1, SY_stable, {"goal6", "obj6"} },*/
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/wall_2.g");
    }
};

class BrickWall3: public Problem{
  public:
    BrickWall3(): Problem("BrickWall3", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("obj1");
      //movableFrames_.append("obj2");
      //movableFrames_.append("obj3");
      //movableFrames_.append("obj4");
      //movableFrames_.append("obj5");
      //movableFrames_.append("obj6");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      auto obj = initial_["goal1"];

      arr pos = obj->getPosition();

      pos(1) = -1 + 0.5 * (seed % 5);
      pos(2) = 0.25 + 0.2 * int(seed / 5.);
      obj->setPosition(pos*1.);

      // TODO: add additional object to the brickwall
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "obj1"}},
        {1., 2., SY_stable, {"a0_gripper", "obj1"} },
        {2., 2., SY_poseEq, {"goal1", "obj1"}},
        {2., -1, SY_stable, {"goal1", "obj1"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/wall_3.g");
    }
};

class BrickWall4: public Problem{
  public:
    BrickWall4(): Problem("BrickWall4", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("obj1");
      //movableFrames_.append("obj2");
      //movableFrames_.append("obj3");
      //movableFrames_.append("obj4");
      //movableFrames_.append("obj5");
      //movableFrames_.append("obj6");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      auto obj = initial_["goal1"];

      arr pos = obj->getPosition();

      pos(1) = -1 + 0.5 * (seed % 5);
      pos(2) = 0.45 + 0.2 * int(seed / 5.);
      obj->setPosition(pos*1.);

      // TODO: add additional object to the brickwall
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "obj1"}},
        {1., 2., SY_stable, {"a0_gripper", "obj1"} },
        {2., 2., SY_poseEq, {"goal1", "obj1"}},
        {2., -1, SY_stable, {"goal1", "obj1"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/wall4.g");
    }
};

class BrickWall5: public Problem{
  public:
    BrickWall5(): Problem("BrickWall5", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
      //movableFrames_.append("obj4");
      //movableFrames_.append("obj5");
      //movableFrames_.append("obj6");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      auto obj = initial_["goal1"];

      arr pos = obj->getPosition();

      pos(1) = -1 + 0.5 * (seed % 5);
      pos(2) = 0.45 + 0.2 * int(seed / 5.);
      obj->setPosition(pos*1.);

      // TODO: add additional object to the brickwall
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "obj1"}},
        {1., 2., SY_stable, {"a0_gripper", "obj1"} },
        {2., 2., SY_poseEq, {"goal1", "obj1"}},
        {2., -1, SY_stable, {"goal1", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper", "obj2"}},
        {3., 4., SY_stable, {"a0_gripper", "obj2"} },
        {4., 4., SY_poseEq, {"goal2", "obj2"}},
        {4., -1, SY_stable, {"goal2", "obj2"} },

        {5., 5., SY_touch, {"a0_gripper", "obj3"}},
        {5., 6., SY_stable, {"a0_gripper", "obj3"} },
        {6., 6., SY_poseEq, {"goal3", "obj3"}},
        {6., -1, SY_stable, {"goal3", "obj3"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/wall4.g");
    }
};

class BrickWallLong: public Problem{
  public:
    BrickWallLong(): Problem("BrickWallLong", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
      movableFrames_.append("obj4");
      movableFrames_.append("obj5");
      movableFrames_.append("obj6");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 12){
        komo.setDiscreteOpt(12);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "obj1"}},
        {1., 2., SY_stable, {"a0_gripper", "obj1"} },
        {2., 2., SY_poseEq, {"goal1", "obj1"}},
        {2., -1, SY_stable, {"goal1", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper", "obj2"}},
        {3., 4., SY_stable, {"a0_gripper", "obj2"} },
        {4., 4., SY_poseEq, {"goal2", "obj2"}},
        {4., -1, SY_stable, {"goal2", "obj2"} },

        {5., 5., SY_touch, {"a0_gripper", "obj3"}},
        {5., 6., SY_stable, {"a0_gripper", "obj3"} },
        {6., 6., SY_poseEq, {"goal3", "obj3"}},
        {6., -1, SY_stable, {"goal3", "obj3"} },

        {7., 7., SY_touch, {"a0_gripper", "obj4"}},
        { 7., 8., SY_stable, {"a0_gripper", "obj4"} },
        {8., 8., SY_poseEq, {"goal4", "obj4"}},
        { 8., -1, SY_stable, {"goal4", "obj4"} },
        
        {9., 9., SY_touch, {"a0_gripper", "obj5"}},
        { 9., 10., SY_stable, {"a0_gripper", "obj5"} },
        {10., 10., SY_poseEq, {"goal5", "obj5"}},
        {10., -1, SY_stable, {"goal5", "obj5"} },

        {11., 11., SY_touch, {"a0_gripper", "obj6"}},
        {11., 12., SY_stable, {"a0_gripper", "obj6"} },
        {12., 12., SY_poseEq, {"goal6", "obj6"}},
        {12., -1, SY_stable, {"goal6", "obj6"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/wall_2.g");
    }
};

class BrickWallLongObs: public Problem{
  public:
    BrickWallLongObs(): Problem("BrickWallLongObs", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
      movableFrames_.append("obj4");
      movableFrames_.append("obj5");
      movableFrames_.append("obj6");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 12){
        komo.setDiscreteOpt(12);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "obj1"}},
        {1., 2., SY_stable, {"a0_gripper", "obj1"} },
        {2., 2., SY_poseEq, {"goal1", "obj1"}},
        {2., -1, SY_stable, {"goal1", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper", "obj2"}},
        {3., 4., SY_stable, {"a0_gripper", "obj2"} },
        {4., 4., SY_poseEq, {"goal2", "obj2"}},
        {4., -1, SY_stable, {"goal2", "obj2"} },

        {5., 5., SY_touch, {"a0_gripper", "obj3"}},
        {5., 6., SY_stable, {"a0_gripper", "obj3"} },
        {6., 6., SY_poseEq, {"goal3", "obj3"}},
        {6., -1, SY_stable, {"goal3", "obj3"} },

        {7., 7., SY_touch, {"a0_gripper", "obj4"}},
        { 7., 8., SY_stable, {"a0_gripper", "obj4"} },
        {8., 8., SY_poseEq, {"goal4", "obj4"}},
        { 8., -1, SY_stable, {"goal4", "obj4"} },
        
        {9., 9., SY_touch, {"a0_gripper", "obj5"}},
        { 9., 10., SY_stable, {"a0_gripper", "obj5"} },
        {10., 10., SY_poseEq, {"goal5", "obj5"}},
        {10., -1, SY_stable, {"goal5", "obj5"} },

        {11., 11., SY_touch, {"a0_gripper", "obj6"}},
        {11., 12., SY_stable, {"a0_gripper", "obj6"} },
        {12., 12., SY_poseEq, {"goal6", "obj6"}},
        {12., -1, SY_stable, {"goal6", "obj6"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/wall4.g");
    }
};

class BrickWallSuperLong: public Problem{
  public:
    BrickWallSuperLong(): Problem("BrickWallSuperLong", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      for (uint i=0; i<15; ++i){
        movableFrames_.append(STRING("obj"<<i+1));
      }
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      auto obj = initial_["goal1"];

      arr pos = obj->getPosition();

      pos(1) = -1 + 0.5 * (seed % 5);
      pos(2) = 0.45 + 0.2 * int(seed / 5.);
      obj->setPosition(pos*1.);

      // TODO: add additional object to the brickwall
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(10);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S;

      for (uint i=0; i<5; ++i){
        const uint pick = 1 + i*2;
        const uint place = pick + 1;
        
        rai::String obj = STRING("obj"<<i+1);
        rai::String goal = STRING("goal" << i+1);

        S.append({pick, pick, SY_touch, {"a0_gripper", obj}});
        S.append({pick, place, SY_stable, {"a0_gripper", obj} });
        S.append({place, place, SY_poseEq, {goal, obj}});
        S.append({place, -1, SY_stable, {goal, obj} });
      }

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/wall5.g");
    }
};

class BrickWallMoveOnly: public Problem{
  public:
    BrickWallMoveOnly(): Problem("BrickWallMoveOnly", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      //movableFrames_.append("obj1");
      //movableFrames_.append("obj2");
      //movableFrames_.append("obj3");
      //movableFrames_.append("obj4");
      //movableFrames_.append("obj5");
      //movableFrames_.append("obj6");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 12){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "obj1"}},
        {2., 2., SY_touch, {"a0_gripper", "goal1"}},
        
        {3., 3., SY_touch, {"a0_gripper", "obj2"}},
        {4., 4., SY_touch, {"a0_gripper", "goal2"}},
        
        {5., 5., SY_touch, {"a0_gripper", "obj3"}},
        {6., 6., SY_touch, {"a0_gripper", "goal3"}},

        /*{7., 7., SY_touch, {"a0_gripper", "obj4"}},
        { 7., 8., SY_stable, {"a0_gripper", "obj4"} },
        {8., 8., SY_poseEq, {"goal4", "obj4"}},
        { 8., -1, SY_stable, {"goal4", "obj4"} },
        
        {9., 9., SY_touch, {"a0_gripper", "obj5"}},
        { 9., 10., SY_stable, {"a0_gripper", "obj5"} },
        {10., 10., SY_poseEq, {"goal5", "obj5"}},
        {10., -1, SY_stable, {"goal5", "obj5"} },

        {11., 11., SY_touch, {"a0_gripper", "obj6"}},
        {11., 12., SY_stable, {"a0_gripper", "obj6"} },
        {12., 12., SY_poseEq, {"goal6", "obj6"}},
        {12., -1, SY_stable, {"goal6", "obj6"} },*/
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/wall_2.g");
    }
};

class Dog: public Problem{
  public:
    Dog(): Problem("dog"){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("stick");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "stick"}},
        { 1., 2., SY_stable, {"a0_gripper", "stick"} },
        //{ 2., 2., SY_poseEq, {"stick", "goal"} },
        {2., 2., SY_positionEq, {"goal", "stick"}},
        { 2., -1, SY_stable, {"goal", "stick"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      if (maxPhase >= 2) komo.addObjective({2.}, FS_scalarProductZZ, {"stick", "goal"}, OT_eq, {1e1}, {0});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/dog.g");
    }
};

class Cage3d: public Problem{
  public:
    Cage3d(): Problem("cage_3d"){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("obj");
    };

    void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        { 1., 1., SY_touch, {"a0_finger1", "obj"}},
        { 1., 1., SY_touch, {"a0_finger2", "obj"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj"} },
        { 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1, SY_stable, {"goal", "obj"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/cage.g");
    }
};

class RotateCube: public Problem{
  public:
    RotateCube(): Problem("rotate_cube"){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("obj");
    };

    void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      // setup of the skeleton
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        //{1., 1., SY_touch, {"agent", "obj"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj"} },
        //{ 2., 2., SY_poseEq, {"obj", "goal"} },
        { 2., -1., SY_touch, {"obj", "table"}},
        { 2., -1, SY_stable, {"table", "obj"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      if (maxPhase == -1 || maxPhase >= 1) {addBoxConstraint(komo, 1., "obj", "a0_", 1);}
      if (maxPhase == -1 || maxPhase >= 2) {komo.addObjective({2.}, FS_scalarProductZZ, {STRING("obj"), "world"}, OT_eq, {1e1}, {-1.});}
    }

  private:

    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/rotate_cube.g");
    }
};

class ForceRearrange: public Problem{
  public:
    ForceRearrange(): Problem("force_rearrange"){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("stick");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "stick"}},
        { 1., 2., SY_stable, {"a0_gripper", "stick"} },
        { 2., 2., SY_poseEq, {"stick", "goal"} },
        { 2., -1, SY_stable, {"goal", "stick"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/force_rearrange.g");
    }
};

class MoveTable: public Problem{
  public:
    MoveTable(): Problem("MoveTable"){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("table");
      movableFrames_.append("leg_1");
      movableFrames_.append("leg_2");
      movableFrames_.append("leg_3");
      movableFrames_.append("leg_4");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.1, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "table"}},
        { 1., 2., SY_stable, {"a0_gripper", "table"} },
        { 2., 2., SY_poseEq, {"table", "goal"} },
        { 2., -1, SY_stable, {"goal", "table"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/move_table.g");
    }
};

class ArmStacking: public Problem{
  public:
    ArmStacking(): Problem("ArmStacking", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 4){
        komo.setDiscreteOpt(4);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },
        { 2., 2., SY_poseEq, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"goal1", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper_p1", "obj2"}},
        {3., 3., SY_touch, {"a0_gripper_p2", "obj2"}},
        {3., 3., SY_touch, {"a0_gripper_p3", "obj2"}},
        { 3., 4., SY_stable, {"a0_gripper", "obj2"} },
        { 4., 4., SY_poseEq, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"goal2", "obj2"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      if (maxPhase >= 1 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj1", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      if (maxPhase >= 3 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj2", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/arm_stacking.g");
    }
};

class ArmStacking2: public Problem{
  public:
    ArmStacking2(): Problem("ArmStacking2", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 4){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },
        { 2., 2., SY_poseEq, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"goal1", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper_p1", "obj2"}},
        {3., 3., SY_touch, {"a0_gripper_p2", "obj2"}},
        {3., 3., SY_touch, {"a0_gripper_p3", "obj2"}},
        { 3., 4., SY_stable, {"a0_gripper", "obj2"} },
        { 4., 4., SY_poseEq, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"goal2", "obj2"} },

        {5., 5., SY_touch, {"a0_gripper_p1", "obj3"}},
        {5., 5., SY_touch, {"a0_gripper_p2", "obj3"}},
        {5., 5., SY_touch, {"a0_gripper_p3", "obj3"}},
        { 5., 6., SY_stable, {"a0_gripper", "obj3"} },
        { 6., 6., SY_poseEq, {"obj3", "goal3"} },
        { 6., -1, SY_stable, {"goal3", "obj3"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      if (maxPhase >= 1 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj1", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      if (maxPhase >= 3 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj2", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      if (maxPhase >= 5 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj3", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/arm_stacking_2.g");
    }
};

class ArmStacking2Restricted: public Problem{
  public:
    ArmStacking2Restricted(): Problem("ArmStacking2Restricted", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 4){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },
        { 2., 2., SY_poseEq, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"goal1", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper_p1", "obj2"}},
        {3., 3., SY_touch, {"a0_gripper_p2", "obj2"}},
        {3., 3., SY_touch, {"a0_gripper_p3", "obj2"}},
        { 3., 4., SY_stable, {"a0_gripper", "obj2"} },
        { 4., 4., SY_poseEq, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"goal2", "obj2"} },

        {5., 5., SY_touch, {"a0_gripper_p1", "obj3"}},
        {5., 5., SY_touch, {"a0_gripper_p2", "obj3"}},
        {5., 5., SY_touch, {"a0_gripper_p3", "obj3"}},
        { 5., 6., SY_stable, {"a0_gripper", "obj3"} },
        { 6., 6., SY_poseEq, {"obj3", "goal3"} },
        { 6., -1, SY_stable, {"goal3", "obj3"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      if (maxPhase >= 1 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj1", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      if (maxPhase >= 3 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj2", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      if (maxPhase >= 5 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj3", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/arm_stacking_restricted.g");
    }
};

class ArmStacking3: public Problem{
  public:
    ArmStacking3(): Problem("ArmStacking3", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
      movableFrames_.append("obj4");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 4){
        komo.setDiscreteOpt(8);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },
        { 2., 2., SY_poseEq, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"goal1", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper_p1", "obj2"}},
        {3., 3., SY_touch, {"a0_gripper_p2", "obj2"}},
        {3., 3., SY_touch, {"a0_gripper_p3", "obj2"}},
        { 3., 4., SY_stable, {"a0_gripper", "obj2"} },
        { 4., 4., SY_poseEq, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"goal2", "obj2"} },

        {5., 5., SY_touch, {"a0_gripper_p1", "obj3"}},
        {5., 5., SY_touch, {"a0_gripper_p2", "obj3"}},
        {5., 5., SY_touch, {"a0_gripper_p3", "obj3"}},
        { 5., 6., SY_stable, {"a0_gripper", "obj3"} },
        { 6., 6., SY_poseEq, {"obj3", "goal3"} },
        { 6., -1, SY_stable, {"goal3", "obj3"} },

        {7., 7., SY_touch, {"a0_gripper_p1", "obj4"}},
        {7., 7., SY_touch, {"a0_gripper_p2", "obj4"}},
        {7., 7., SY_touch, {"a0_gripper_p3", "obj4"}},
        { 7., 8., SY_stable, {"a0_gripper", "obj4"} },
        { 8., 8., SY_poseEq, {"obj4", "goal4"} },
        { 8., -1, SY_stable, {"goal4", "obj4"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      if (maxPhase >= 1 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj1", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      if (maxPhase >= 3 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj2", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      if (maxPhase >= 5 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj3", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      if (maxPhase >= 7 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj4", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/arm_stacking_restricted_2.g");
    }
};

class ArmMove: public Problem{
  public:
    ArmMove(): Problem("ArmMove", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      //movableFrames_.append("obj1");
      //movableFrames_.append("obj2");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 4){
        komo.setDiscreteOpt(4);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "goal1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "goal1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "goal1"}},

        {2., 2., SY_touch, {"a0_gripper_p1", "goal2"}},
        {2., 2., SY_touch, {"a0_gripper_p2", "goal2"}},
        {2., 2., SY_touch, {"a0_gripper_p3", "goal2"}},

        {3., 3., SY_touch, {"a0_gripper_p1", "goal3"}},
        {3., 3., SY_touch, {"a0_gripper_p2", "goal3"}},
        {3., 3., SY_touch, {"a0_gripper_p3", "goal3"}},

        {4., 4., SY_touch, {"a0_gripper_p1", "goal1"}},
        {4., 4., SY_touch, {"a0_gripper_p2", "goal1"}},
        {4., 4., SY_touch, {"a0_gripper_p3", "goal1"}},
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/arm_move_only.g");
    }
};

class ArmMove2: public Problem{
  public:
    ArmMove2(): Problem("ArmMove2", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      //movableFrames_.append("obj1");
      //movableFrames_.append("obj2");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 5){
        komo.setDiscreteOpt(5);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "goal1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "goal1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "goal1"}},

        {2., 2., SY_touch, {"a0_gripper_p1", "goal2"}},
        {2., 2., SY_touch, {"a0_gripper_p2", "goal2"}},
        {2., 2., SY_touch, {"a0_gripper_p3", "goal2"}},

        {3., 3., SY_touch, {"a0_gripper_p1", "goal1"}},
        {3., 3., SY_touch, {"a0_gripper_p2", "goal1"}},
        {3., 3., SY_touch, {"a0_gripper_p3", "goal1"}},

        {4., 4., SY_touch, {"a0_gripper_p1", "goal3"}},
        {4., 4., SY_touch, {"a0_gripper_p2", "goal3"}},
        {4., 4., SY_touch, {"a0_gripper_p3", "goal3"}},

        {5., 5., SY_touch, {"a0_gripper_p1", "goal1"}},
        {5., 5., SY_touch, {"a0_gripper_p2", "goal1"}},
        {5., 5., SY_touch, {"a0_gripper_p3", "goal1"}},
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/arm_move_only.g");
    }
};

class ArmBinPicking: public Problem{
  public:
    ArmBinPicking(): Problem("ArmBinPicking", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      /*std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      std::uniform_real_distribution<> shape_distr(0.5, 1.5); // distribution in range [1, 6]
      std::uniform_real_distribution<> x_pos_distr(-0.95,-0.65); // distribution in range [1, 6]
      std::uniform_real_distribution<> y_pos_distr(-0.15,0.15); // distribution in range [1, 6]

      while(true){
        for (const int i: {1, 2, 3}){
          auto obj = initial_[STRING("obj" << std::to_string(i))];
          double width = 0.06*shape_distr(rng);
          double height = 0.06*shape_distr(rng);

          obj->setShape(rai::ST_ssBox, {width, height, .06, .005});
          obj->getShape();

          arr pos = obj->getPosition();

          pos(0) = x_pos_distr(rng);
          pos(1) = y_pos_distr(rng);
          obj->setPosition(pos*1.);
        }

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }*/
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 4){
        komo.setDiscreteOpt(4);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        { 1.,  1., SY_touch, {"a0_gripper", "obj1"}},
        { 1.,  2., SY_stable, {"a0_gripper", "obj1"} },
        { 2.,  2., SY_touch, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"goal1", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper", "obj2"}},
        { 3.,  4., SY_stable, {"a0_gripper", "obj2"} },
        { 4.,  4., SY_touch, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"goal2", "obj2"} },

        /*{5., 5., SY_touch, {"a0_gripper", "obj3"}},
        { 5.,  6., SY_stable, {"a0_gripper", "obj3"} },
        { 6.,  6., SY_touch, {"obj3", "goal1"} },
        { 6., -1, SY_stable, {"goal1", "obj3"} },*/
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/arm_bin_picking.g");
    }
};

class ArmBinPicking2: public Problem{
  public:
    ArmBinPicking2(): Problem("ArmBinPicking2", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 6){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        { 1.,  1., SY_touch, {"a0_gripper", "obj1"}},
        { 1.,  2., SY_stable, {"a0_gripper", "obj1"} },
        { 2.,  2., SY_touch, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"goal1", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper", "obj2"}},
        { 3.,  4., SY_stable, {"a0_gripper", "obj2"} },
        { 4.,  4., SY_touch, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"goal2", "obj2"} },

        {5., 5., SY_touch, {"a0_gripper", "obj3"}},
        { 5.,  6., SY_stable, {"a0_gripper", "obj3"} },
        { 6.,  6., SY_touch, {"obj3", "goal1"} },
        { 6., -1, SY_stable, {"goal1", "obj3"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/arm_bin_picking.g");
    }
};

class ArmBinPicking3: public Problem{
  public:
    ArmBinPicking3(): Problem("ArmBinPicking3", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      std::uniform_real_distribution<> shape_distr(0.5, 1.5); // distribution in range [1, 6]
      std::uniform_real_distribution<> x_pos_distr(-0.95,-0.65); // distribution in range [1, 6]
      std::uniform_real_distribution<> y_pos_distr(-0.15,0.15); // distribution in range [1, 6]

      while(true){
        for (const int i: {1, 2, 3}){
          auto obj = initial_[STRING("obj" << std::to_string(i))];
          double width = 0.06*shape_distr(rng);
          double height = 0.06*shape_distr(rng);

          obj->setShape(rai::ST_ssBox, {width, height, .06, .005});
          obj->getShape();

          arr pos = obj->getPosition();

          pos(0) = x_pos_distr(rng);
          pos(1) = y_pos_distr(rng);
          obj->setPosition(pos*1.);
        }

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 6){
        komo.setDiscreteOpt(6);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        { 1.,  1., SY_touch, {"a0_gripper", "obj1"}},
        { 1.,  2., SY_stable, {"a0_gripper", "obj1"} },
        { 2.,  2., SY_touch, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"goal1", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper", "obj2"}},
        { 3.,  4., SY_stable, {"a0_gripper", "obj2"} },
        { 4.,  4., SY_touch, {"obj2", "goal2"} },
        { 4., -1, SY_stable, {"goal2", "obj2"} },

        {5., 5., SY_touch, {"a0_gripper", "obj3"}},
        { 5.,  6., SY_stable, {"a0_gripper", "obj3"} },
        { 6.,  6., SY_touch, {"obj3", "goal1"} },
        { 6., -1, SY_stable, {"goal1", "obj3"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/arm_bin_picking.g");
    }
};

class ArmBinPicking4: public Problem{
  public:
    ArmBinPicking4(): Problem("ArmBinPicking4", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      std::uniform_real_distribution<> shape_distr(0.5, 1.5); // distribution in range [1, 6]
      std::uniform_real_distribution<> x_pos_distr(-0.95,-0.65); // distribution in range [1, 6]
      std::uniform_real_distribution<> y_pos_distr(-0.15,0.15); // distribution in range [1, 6]

      while(true){
        for (const int i: {1, 2, 3}){
          auto obj = initial_[STRING("obj" << std::to_string(i))];
          double width = 0.06*shape_distr(rng);
          double height = 0.06*shape_distr(rng);

          obj->setShape(rai::ST_ssBox, {width, height, .06, .005});
          obj->getShape();

          arr pos = obj->getPosition();

          pos(0) = x_pos_distr(rng);
          pos(1) = y_pos_distr(rng);
          obj->setPosition(pos*1.);
        }

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        { 1.,  1., SY_touch, {"a0_gripper", "obj1"}},
        { 1.,  2., SY_stable, {"a0_gripper", "obj1"} },
        { 2.,  2., SY_touch, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"goal1", "obj1"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/arm_bin_picking.g");
    }
};

class ArmBinPicking5: public Problem{
  public:
    ArmBinPicking5(): Problem("ArmBinPicking5", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
    };

    virtual void perturb(const uint seed){
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      std::uniform_real_distribution<> shape_distr(0.5, 1.1); // distribution in range [1, 6]
      std::uniform_real_distribution<> x_pos_distr(-0.95,-0.75); // distribution in range [1, 6]
      std::uniform_real_distribution<> y_pos_distr(-0.2,0.05); // distribution in range [1, 6]

      while(true){
        for (const int i: {1}){
          auto obj = initial_[STRING("obj" << std::to_string(i))];
          double width = 0.06*shape_distr(rng);
          double height = 0.06*shape_distr(rng);

          obj->setShape(rai::ST_ssBox, {width, height, .06, .005});
          obj->getShape();

          arr pos = obj->getPosition();

          pos(0) = x_pos_distr(rng);
          pos(1) = y_pos_distr(rng);
          obj->setPosition(pos*1.);
        }

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible && 
            euclideanDistance(initial_["obj1"]->getPosition(), initial_["obj2"]->getPosition()) > 0.1 &&
            euclideanDistance(initial_["obj1"]->getPosition(), initial_["obj3"]->getPosition()) > 0.1){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        //{ 1.,  1., SY_touch, {"a0_gripper", "obj1"}},
        { 1.,  2., SY_stable, {"a0_gripper", "obj1"} },
        { 2.,  2., SY_touch, {"obj1", "goal1"} },
        { 2., -1, SY_stable, {"goal1", "obj1"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      if (maxPhase == -1 || maxPhase >= 1) {addBoxConstraint(komo, 1., "obj1", "a0_", 1);}
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/arm_bin_picking_2.g");
    }
};

class AmazonSingle: public Problem{
  public:
    AmazonSingle(): Problem("amazon", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },
        { 2., 2., SY_touch, {"obj1", "goal"} },
        { 2., -1, SY_stable, {"goal", "obj1"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      if (maxPhase >= 1 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj1", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      if (maxPhase >= 2 || maxPhase == -1) komo.addObjective({2.}, FS_scalarProductZZ, {"obj1", "goal"}, OT_eq, {1e1}, {1});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/amazon.g");
    }
};

class Amazon: public Problem{
  public:
    Amazon(): Problem("amazon_2", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 4){
        komo.setDiscreteOpt(4);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },
        { 2., 2., SY_touch, {"obj1", "goal"} },
        { 2., -1, SY_stable, {"goal", "obj1"} },

        {3., 3., SY_touch, {"a0_gripper_p1", "obj2"}},
        {3., 3., SY_touch, {"a0_gripper_p2", "obj2"}},
        {3., 3., SY_touch, {"a0_gripper_p3", "obj2"}},
        { 3., 4., SY_stable, {"a0_gripper", "obj2"} },
        { 4., 4., SY_touch, {"obj2", "goal"} },
        { 4., -1, SY_stable, {"goal", "obj2"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      if (maxPhase >= 1 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj1", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      if (maxPhase >= 2 || maxPhase == -1) komo.addObjective({2.}, FS_scalarProductZZ, {"obj1", "goal"}, OT_eq, {1e1}, {1});

      if (maxPhase >= 3 || maxPhase == -1) komo.addObjective({3.}, FS_scalarProductZZ, {"obj2", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      if (maxPhase >= 4 || maxPhase == -1) komo.addObjective({4.}, FS_scalarProductZZ, {"obj2", "goal"}, OT_eq, {1e1}, {1});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/amazon.g");
    }
};

class MoveTableRestricted: public Problem{
  public:
    MoveTableRestricted(): Problem("move_table_walls", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
      }

      movableFrames_.append("table");
      movableFrames_.append("leg_1");
      movableFrames_.append("leg_2");
      movableFrames_.append("leg_3");
      movableFrames_.append("leg_4");
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 2){
        komo.setDiscreteOpt(2);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper", "table"}},
        { 1., 2., SY_stable, {"a0_gripper", "table"} },
        { 2., 2., SY_poseEq, {"table", "goal"} },
        { 2., -1, SY_stable, {"goal", "table"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/move_table_bend.g");
    }
};

class Handover: public Problem{
  public:
    Handover(): Problem("Handover", 0.005){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
        if (f->name.contains("a1_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      //movableFrames_.append("obj2");
    };

    virtual void perturb(const uint seed){
      //std::cout << "No perturbation specified!" << std::endl;
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 3){
        komo.setDiscreteOpt(3);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },

        {2., 2., SY_touch, {"a1_gripper_p1", "obj1"}},
        {2., 2., SY_touch, {"a1_gripper_p2", "obj1"}},
        {2., 2., SY_touch, {"a1_gripper_p3", "obj1"}},
        { 2., 3, SY_stable, {"a1_gripper", "obj1"} },

        { 3., 3., SY_poseEq, {"obj1", "goal1"} },
        { 3., -1, SY_stable, {"goal1", "obj1"} },

        //{3., 3., SY_touch, {"a0_gripper_p1", "obj2"}},
        //{3., 3., SY_touch, {"a0_gripper_p2", "obj2"}},
        //{3., 3., SY_touch, {"a0_gripper_p3", "obj2"}},
        //{ 3., 4., SY_stable, {"a0_gripper", "obj2"} },
        //{ 4., 4., SY_poseEq, {"obj2", "goal2"} },
        //{ 4., -1, SY_stable, {"goal2", "obj2"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      //if (maxPhase >= 1 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj1", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      //if (maxPhase >= 3 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj2", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/handover.g");
    }
};

class HandoverPerturbed: public Problem{
  public:
    HandoverPerturbed(): Problem("HandoverPerturbed", 0.01){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
        if (f->name.contains("a1_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      //movableFrames_.append("obj2");
    };

    virtual void perturb(const uint seed){
      //std::cout << "No perturbation specified!" << std::endl;
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      std::uniform_real_distribution<> shape_distr(0.5, 1.5); // distribution in range [1, 6]
      std::uniform_real_distribution<> x_pos_distr(-0.1,0.1); // distribution in range [1, 6]
      std::uniform_real_distribution<> y_pos_distr(-0.1,0.1); // distribution in range [1, 6]

      while(true){
        auto obj = initial_["obj1"];
        double width = 0.1*shape_distr(rng);
        double height = 0.1*shape_distr(rng);

        obj->setShape(rai::ST_ssBox, {width, height, .06, .005});
        obj->getShape();

        arr pos = obj->getPosition();

        pos(0) = pos(0) + x_pos_distr(rng);
        pos(1) = pos(1) + y_pos_distr(rng);
        obj->setPosition(pos*1.);

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 3){
        komo.setDiscreteOpt(3);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },

        {2., 2., SY_touch, {"a1_gripper_p1", "obj1"}},
        {2., 2., SY_touch, {"a1_gripper_p2", "obj1"}},
        {2., 2., SY_touch, {"a1_gripper_p3", "obj1"}},
        { 2., 3, SY_stable, {"a1_gripper", "obj1"} },

        { 3., 3., SY_poseEq, {"obj1", "goal1"} },
        { 3., -1, SY_stable, {"goal1", "obj1"} },

        //{3., 3., SY_touch, {"a0_gripper_p1", "obj2"}},
        //{3., 3., SY_touch, {"a0_gripper_p2", "obj2"}},
        //{3., 3., SY_touch, {"a0_gripper_p3", "obj2"}},
        //{ 3., 4., SY_stable, {"a0_gripper", "obj2"} },
        //{ 4., 4., SY_poseEq, {"obj2", "goal2"} },
        //{ 4., -1, SY_stable, {"goal2", "obj2"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      //if (maxPhase >= 1 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj1", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      //if (maxPhase >= 3 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj2", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/handover.g");
    }
};

class HandoverPerturbedBox: public Problem{
  public:
    HandoverPerturbedBox(): Problem("HandoverPerturbedBox", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
        if (f->name.contains("a1_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      //movableFrames_.append("obj2");
    };

    virtual void perturb(const uint seed){
      //std::cout << "No perturbation specified!" << std::endl;
      initial_.clear();
      setup();

      /*std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      std::uniform_real_distribution<> shape_distr(0.999, 1.); // distribution in range [1, 6]
      std::uniform_real_distribution<> x_pos_distr(-0.001,0.001); // distribution in range [1, 6]
      std::uniform_real_distribution<> y_pos_distr(-0.001,0.001); // distribution in range [1, 6]

      auto obj = initial_["obj1"];
      arr initial_pos = obj->getPosition() * 1.;

      while(true){
        double width = 0.2*shape_distr(rng);
        double height = 0.2*shape_distr(rng);

        obj->setShape(rai::ST_ssBox, {width, height, .1, .03});

        arr pos = 1. * initial_pos;

        pos(0) = pos(0) + x_pos_distr(rng);
        pos(1) = pos(1) + y_pos_distr(rng);
        obj->setPosition(pos*1.);

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }*/
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 3){
        komo.setDiscreteOpt(3);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },

        {2., 2., SY_touch, {"a1_gripper_p1", "obj1"}},
        {2., 2., SY_touch, {"a1_gripper_p2", "obj1"}},
        {2., 2., SY_touch, {"a1_gripper_p3", "obj1"}},
        { 2., 3, SY_stable, {"a1_gripper", "obj1"} },

        { 3., 3., SY_poseEq, {"obj1", "goal1"} },
        { 3., -1, SY_stable, {"goal1", "obj1"} },

        //{3., 3., SY_touch, {"a0_gripper_p1", "obj2"}},
        //{3., 3., SY_touch, {"a0_gripper_p2", "obj2"}},
        //{3., 3., SY_touch, {"a0_gripper_p3", "obj2"}},
        //{ 3., 4., SY_stable, {"a0_gripper", "obj2"} },
        //{ 4., 4., SY_poseEq, {"obj2", "goal2"} },
        //{ 4., -1, SY_stable, {"goal2", "obj2"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      //if (maxPhase >= 1 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj1", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      //if (maxPhase >= 3 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj2", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/handover3.g");
    }
};

class HandoverPerturbedBox2: public Problem{
  public:
    HandoverPerturbedBox2(): Problem("HandoverPerturbedBox2", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
        if (f->name.contains("a1_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      //movableFrames_.append("obj2");
    };

    virtual void perturb(const uint seed){
      //std::cout << "No perturbation specified!" << std::endl;
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      std::uniform_real_distribution<> shape_distr(0.5, 1.2); // distribution in range [1, 6]
      std::uniform_real_distribution<> x_pos_distr(-0.1,0.15); // distribution in range [1, 6]
      std::uniform_real_distribution<> y_pos_distr(-0.1,0.1); // distribution in range [1, 6]

      auto obj = initial_["obj1"];
      arr initial_pos = obj->getPosition() * 1.;

      while(true){
        double width = 0.2*shape_distr(rng);
        double height = 0.2*shape_distr(rng);

        obj->setShape(rai::ST_ssBox, {width, height, .1, .03});

        arr pos = 1. * initial_pos;

        pos(0) = pos(0) + x_pos_distr(rng);
        pos(1) = pos(1) + y_pos_distr(rng);
        obj->setPosition(pos*1.);

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 3){
        komo.setDiscreteOpt(3);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },

        {2., 2., SY_touch, {"a1_gripper_p1", "obj1"}},
        {2., 2., SY_touch, {"a1_gripper_p2", "obj1"}},
        {2., 2., SY_touch, {"a1_gripper_p3", "obj1"}},
        { 2., 3, SY_stable, {"a1_gripper", "obj1"} },

        { 3., 3., SY_poseEq, {"obj1", "goal1"} },
        { 3., -1, SY_stable, {"goal1", "obj1"} },

        //{3., 3., SY_touch, {"a0_gripper_p1", "obj2"}},
        //{3., 3., SY_touch, {"a0_gripper_p2", "obj2"}},
        //{3., 3., SY_touch, {"a0_gripper_p3", "obj2"}},
        //{ 3., 4., SY_stable, {"a0_gripper", "obj2"} },
        //{ 4., 4., SY_poseEq, {"obj2", "goal2"} },
        //{ 4., -1, SY_stable, {"goal2", "obj2"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      //if (maxPhase >= 1 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj1", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      //if (maxPhase >= 3 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj2", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/handover4.g");
    }
};

class Handover2: public Problem{
  public:
    Handover2(): Problem("Handover2", 0.005){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
        if (f->name.contains("a1_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      //movableFrames_.append("obj2");
    };

    virtual void perturb(const uint seed){
      //std::cout << "No perturbation specified!" << std::endl;
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 3){
        komo.setDiscreteOpt(3);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },

        {2., 2., SY_touch, {"a1_gripper_p1", "obj1"}},
        {2., 2., SY_touch, {"a1_gripper_p2", "obj1"}},
        {2., 2., SY_touch, {"a1_gripper_p3", "obj1"}},
        { 2., 3, SY_stable, {"a1_gripper", "obj1"} },

        { 3., 3., SY_poseEq, {"obj1", "goal1"} },
        { 3., -1, SY_stable, {"goal1", "obj1"} },

        //{3., 3., SY_touch, {"a0_gripper_p1", "obj2"}},
        //{3., 3., SY_touch, {"a0_gripper_p2", "obj2"}},
        //{3., 3., SY_touch, {"a0_gripper_p3", "obj2"}},
        //{ 3., 4., SY_stable, {"a0_gripper", "obj2"} },
        //{ 4., 4., SY_poseEq, {"obj2", "goal2"} },
        //{ 4., -1, SY_stable, {"goal2", "obj2"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      //if (maxPhase >= 1 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj1", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      //if (maxPhase >= 3 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj2", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/handover2.g");
    }
};

class HandoverStacking: public Problem{
  public:
    HandoverStacking(): Problem("HandoverStacking", 0.001){
      setup();
      getKomoInstance(komo_);

      for (const auto &f: initial_.frames){
        if (f->name.contains("a0_")){
          robotFrames_.append(f->name);
        }
        if (f->name.contains("a1_")){
          robotFrames_.append(f->name);
        }
      }
      movableFrames_.append("obj1");
      movableFrames_.append("obj2");
      movableFrames_.append("obj3");
    };

    virtual void perturb(const uint seed){
      //std::cout << "No perturbation specified!" << std::endl;
      initial_.clear();
      setup();

      std::random_device dev;
      std::mt19937 rng(dev());
      rng.seed(seed);

      std::uniform_real_distribution<> shape_distr(0.5, 1.2); // distribution in range [1, 6]
      std::uniform_real_distribution<> x_pos_distr(-0.1,0.15); // distribution in range [1, 6]
      std::uniform_real_distribution<> y_pos_distr(-0.1,0.1); // distribution in range [1, 6]

      auto obj = initial_["obj1"];
      arr initial_pos = obj->getPosition() * 1.;

      while(true){
        double width = 0.2*shape_distr(rng);
        double height = 0.2*shape_distr(rng);

        obj->setShape(rai::ST_ssBox, {width, height, .1, .03});

        arr pos = 1. * initial_pos;

        pos(0) = pos(0) + x_pos_distr(rng);
        pos(1) = pos(1) + y_pos_distr(rng);
        obj->setPosition(pos*1.);

        ConfigurationProblem cp(initial_);
        if (cp.query({}, false)->isFeasible){
          break;
        }
      }
    };

    virtual void getKomoInstance(KOMO &komo, const int maxPhase=-1){
      komo.verbose = 0;
      komo.setModel(initial_, true);
      if (maxPhase == -1 || maxPhase > 3){
        komo.setDiscreteOpt(9);
      }
      else{
        komo.setDiscreteOpt(maxPhase);
      }

      komo.world.stepSwift();

      komo.add_collision(true, collisionMargin_, collisionScale_);
      komo.add_jointLimits(true, 0.01, 1e1);

      Skeleton S = {
        {1., 1., SY_touch, {"a0_gripper_p1", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p2", "obj1"}},
        {1., 1., SY_touch, {"a0_gripper_p3", "obj1"}},
        { 1., 2., SY_stable, {"a0_gripper", "obj1"} },

        {2., 2., SY_touch, {"a1_gripper_p1", "obj1"}},
        {2., 2., SY_touch, {"a1_gripper_p2", "obj1"}},
        {2., 2., SY_touch, {"a1_gripper_p3", "obj1"}},
        { 2., 3, SY_stable, {"a1_gripper", "obj1"} },

        { 3., 3., SY_poseEq, {"obj1", "goal1"} },
        { 3., -1, SY_stable, {"goal1", "obj1"} },

        // obj2
        {4., 4., SY_touch, {"a0_gripper_p1", "obj2"}},
        {4., 4., SY_touch, {"a0_gripper_p2", "obj2"}},
        {4., 4., SY_touch, {"a0_gripper_p3", "obj2"}},
        { 4., 5., SY_stable, {"a0_gripper", "obj2"} },

        {5., 5., SY_touch, {"a1_gripper_p1", "obj2"}},
        {5., 5., SY_touch, {"a1_gripper_p2", "obj2"}},
        {5., 5., SY_touch, {"a1_gripper_p3", "obj2"}},
        { 5., 6, SY_stable, {"a1_gripper", "obj2"} },

        { 6., 6., SY_poseEq, {"obj2", "goal2"} },
        { 6., -1, SY_stable, {"goal2", "obj2"} },
        
        // obj3
        {7., 7., SY_touch, {"a0_gripper_p1", "obj3"}},
        {7., 7., SY_touch, {"a0_gripper_p2", "obj3"}},
        {7., 7., SY_touch, {"a0_gripper_p3", "obj3"}},
        { 7., 8., SY_stable, {"a0_gripper", "obj3"} },

        {8., 8., SY_touch, {"a1_gripper_p1", "obj3"}},
        {8., 8., SY_touch, {"a1_gripper_p2", "obj3"}},
        {8., 8., SY_touch, {"a1_gripper_p3", "obj3"}},
        { 8., 9, SY_stable, {"a1_gripper", "obj3"} },

        { 9., 9., SY_poseEq, {"obj3", "goal3"} },
        { 9., -1, SY_stable, {"goal3", "obj3"} },
      };

      if (maxPhase > 0) {S = filterSkeleton(S, maxPhase);}
      komo.setSkeleton(S);

      //if (maxPhase >= 1 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj1", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
      //if (maxPhase >= 3 || maxPhase == -1) komo.addObjective({1.}, FS_scalarProductZZ, {"obj2", "a0_gripper_vis"}, OT_eq, {1e1}, {-1});
    }

  private:
    void setup(){
      auto *base = initial_.addFrame("world", "");
      base->setShape(rai::ST_marker, {0.001});
      base->setPosition({0., 0., .5});
      base->setContact(0.);

      initial_.addFile("./in/handover4.g");
    }
};

#endif
