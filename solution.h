#ifndef SOLUTION_H
#define SOLUTION_H

#include <fstream>

class Solution{
  public:
    Solution(const std::vector<arr> &segments) :segments_(segments){};

    double cost() const{
      double sum{0.};
      for (const auto &s: segments_){
        for (uint i=1; i<s.d0; ++i){
          sum += length(s[i] - s[i-1]);
        }
      }
      return sum;
    };

    void show(std::shared_ptr<Problem> P, const bool save=false) const{
      P->initial_.calc_indexedActiveJoints();

      StringA joints;
      for (auto a: P->initial_.activeJoints){
        joints.append(a->frame->name);
      }

      rai::Configuration C(P->initial_);

      rai::ConfigurationViewer Vf;
      Vf.setConfiguration(C, "\"Real World\"");

      Vf.displayCamera().setPosition(0, 10, 15);
      Vf.displayCamera().focusOrigin();
      Vf.displayCamera().upright();
      Vf.displayCamera().setPosition(0, -0.001, 15);
      Vf.displayCamera().focusOrigin();
      Vf.displayCamera().setFocalLength(5);

      Vf.watch();
      for (uint i=0; i<segments_.size(); ++i){
        P->setConfigurationToMode(C, i);

        // set the right frames to be active
        C.selectJointsByName(joints);

        const auto &s = segments_[i];
        for (uint j=0; j<s.d0; ++j){
          C.setJointState(s[j]);

          Vf.setConfiguration(C, "\"Real World\"");
          rai::wait(0.05);

          if (save) {
            Vf.savePng();
          }
        }
      }
    };

    void exportSolution(const std::string& filename) const{
      ofstream myfile;
      myfile.open (filename);
      myfile << "Writing solution segments";
    }

    const std::vector<arr> segments_;
};

#endif
