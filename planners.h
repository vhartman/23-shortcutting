#include <iostream>
#include <vector>
#include <set>
#include <random>

#include <future>

#include <Kin/kin.h>
#include <Algo/ann.h>

#include <GL/gl.h>
#include <Gui/opengl.h>

#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsLinear.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/BinaryHeap.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>

#include "validity.h"
#include "edge_checker.h"
#include "corput.h"

#ifndef _PLANNERS_H
#define _PLANNERS_H

class Planner{
  public:
    Planner(ConfigurationProblem cp){
      //cc_ = std::make_shared<aCollisionChecker>(cp.C);
    };

    virtual arr plan(const arr& q0, const arr& q1, double time=-1) = 0;

    std::shared_ptr<aCollisionChecker> cc_;
};

const double get_path_length(const arr& path){
  double cost = 0.;
  for (uint i=0; i<path.d0-1; ++i){
    cost += euclideanDistance(path[i], path[i+1]);
  }

  return cost;
}

class ContinuingPlanner: public Planner{
  public:
    ContinuingPlanner(const ConfigurationProblem cp): Planner(cp){};

    virtual void setup(const arr &q0, const arr &q1) = 0;

    virtual void set_resolution(const double resolution) = 0;

    virtual arr plan(const arr &q0, const arr &q1, double time_in_s=-1) = 0;
    virtual arr planMore(double time_in_s = -1) = 0;

    virtual double getCollCalls(){return 0.;};
    virtual void resetCollCalls(){};

    double duration_sampling{0.};
    double duration_search{0.};
    double duration_nn{0.};
    double duration_verifying{0.};

    double resolution_{0.01};

    // planner cost at time
    std::vector<std::pair<double, double>> costs;

    uint verbose_ = 0;

    bool stop_at_first_solution{true};
  private:
};

class ReusingPlanner: public ContinuingPlanner{
  public:
  //ReusingPlanner(){};
  
  // shared data
};

class Node{
  public:
    Node(const arr &q) :q_(q){};

    arr q_;
    std::shared_ptr<Node> parent_;

    uint id_;
};

class Tree{
  public:
    Tree(){};

    std::vector<std::shared_ptr<Node>> nodes;
    ANN ann;

    std::shared_ptr<Node> getNear(const arr &q) {
      /*
         // linear search
      std::shared_ptr<Node> nearest;
      double min_dist = 0.;

      for (const auto &n: nodes){
        const double d = length(q-n->q_);
        if (d < min_dist || nearest == nullptr){
          min_dist = d;
          nearest = n;
        }
      }
      */

      const uint id = ann.getNN(q);
      const auto nearest = nodes[id];

      //std::cout << "rrt-size: " <<  nodes.size() << std::endl;

      return nearest;
    };
    void addNode(std::shared_ptr<Node> n){
      nodes.push_back(n);
      ann.append(n->q_);
    };
};


class RRTConnect: public ContinuingPlanner{
  public:
    RRTConnect(ConfigurationProblem cp): ContinuingPlanner(cp){
      cc_ = std::make_shared<aCollisionChecker>(cp.C);
      ec_ = std::make_shared<EdgeChecker>(cc_);

      if (true){
        const arr limits = cp.C.getLimits();
        double d = 0.;
        for (uint i=0; i<limits.d0; ++i){
          double diff = 10;
          if(limits(i,1) > limits(i,0)){
            diff = limits(i,1) - limits(i,0);
          }
          d += diff * diff;
        }
        maxDelta_ = std::sqrt(d) * 0.2;
      }
    };

    virtual void set_resolution(const double resolution){
      ec_->resolution_ = resolution;
    }

    std::shared_ptr<EdgeChecker> ec_;

    virtual void setup(const arr &q0, const arr &q1){
      start_ = std::make_shared<Node>(q0);
      goal_ = std::make_shared<Node>(q1);

      tstart_.addNode(start_);
      tgoal_.addNode(goal_);
    };

    virtual arr plan(const arr &q0, const arr &q1, double time_in_s=-1){
      setup(q0, q1);
      return planMore(time_in_s);
    };
    virtual arr planMore(double time_in_s){
      Tree *ta = &tstart_;
      Tree *tb = &tgoal_;

      uint cnt = 0;
      std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      while(true){
        const auto diff = std::chrono::duration_cast<
          std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();

        //if (time < 0 || cnt < time)
        if (time_in_s > 0 && diff > time_in_s * 1000){
          return {};
        }

        arr q_rnd = cc_->cp.sample();
        auto n_near_ta = ta->getNear(q_rnd);
        auto q_new = extend(n_near_ta->q_, q_rnd);

        // check node
        bool newPoseValid = cc_->isValid(q_new);

        // check path
        bool edgeValid = false;
        if (newPoseValid){
          std::chrono::steady_clock::time_point begin_feas = std::chrono::steady_clock::now();

          edgeValid = ec_->isValid(n_near_ta->q_, q_new);

          std::chrono::steady_clock::time_point end_feas = std::chrono::steady_clock::now();
          duration_verifying += std::chrono::duration_cast<std::chrono::microseconds>(end_feas - begin_feas).count() / (1.*1e6);
        }

        if (newPoseValid && edgeValid){
          // make and add node
          auto n_new = std::make_shared<Node>(q_new);
          n_new->parent_ = n_near_ta;
          ta->addNode(n_new);

          auto n_near_tb = tb->getNear(q_new);
          //auto q_connect = connect(n_near_tb->q_, q_new);
          auto qs = connect(n_near_tb->q_, q_new);

          arr finalValidQ;
          std::shared_ptr<Node> finalValidNode;

          arr prevQ = n_near_tb->q_;
          auto prevNode = n_near_tb;
          for (uint i=0; i<qs.size(); ++i){
            arr q = qs[i];

            // check if pose valid
            bool connectPoseValid = cc_->isValid(q);
            if (!connectPoseValid){
              //continue;
              break;
            }

            // check if edge valid
            std::chrono::steady_clock::time_point begin_feas = std::chrono::steady_clock::now();
            bool connectEdgeValid = ec_->isValid(prevQ, q);
            std::chrono::steady_clock::time_point end_feas = std::chrono::steady_clock::now();
            duration_verifying += std::chrono::duration_cast<std::chrono::microseconds>(end_feas - begin_feas).count() / (1.*1e6);
            if (!connectEdgeValid){
              //continue;
              break;
            }

            auto n = std::make_shared<Node>(q);
            n->parent_ = prevNode;
            tb->addNode(n);

            finalValidQ = q;
            finalValidNode = n;

            prevNode = n;
            prevQ = q;
          }

          if (finalValidQ.d0 > 0 && length(finalValidQ - q_new) < 0.01){
            // trees connected
            std::vector<arr> p1;
            p1.push_back(n_new->q_);

            {
              auto parent = n_new->parent_;
              while (parent){
                p1.push_back(parent->q_);
                parent = parent->parent_;
              }
            }

            std::vector<arr> p2;
            {
              auto parent = finalValidNode->parent_;
              while (parent){
                p2.push_back(parent->q_);
                parent = parent->parent_;
              }
            }

            // check which path needs to be reversed
            std::vector<arr> p;
            for (uint i=0; i<p1.size(); ++i){
              p.push_back(p1[p1.size()-i-1]);
            }
            for (uint i=0; i<p2.size(); ++i){
              p.push_back(p2[i]);
            }

            arr path;
            path.resize(p1.size() + p2.size(), p1[0].d0);
            if (cnt%2 == 0){
              for (uint i=0; i<p.size(); ++i){
                path[i] = p[i];
              }
            }
            else {
              for (uint i=0; i<p.size(); ++i){
                path[i] = p[p.size()-1-i];
              }
            }

            if (verbose_ > 1) std::cout << ec_->calls_ << " " << ec_->robot_calls_ << " " << ec_->obs_calls_ << " " << ec_->env_calls_ << std::endl;

            const auto diff = std::chrono::duration_cast<
              std::chrono::microseconds>(std::chrono::steady_clock::now() - begin).count() / (1.*1e6);
            costs.push_back(std::make_pair(diff, get_path_length(path)));
            return path;
          }
        }

        std::swap(ta, tb);
        cnt += 1;

        //if (cnt > 100) return {};
      }
      return {};
    };

    double maxDelta_{1};

  protected:
    arr extend(const arr qStart, const arr &qGoal){
      const arr dir = qGoal - qStart;

      double delta = length(dir);
      const arr e = dir / delta;

      if (delta > maxDelta_){
        delta = maxDelta_;
      }

      const arr qNew = qStart + delta * e;
      return qNew;

    };
    std::vector<arr> connect(const arr &qStart, const arr &qGoal){
      std::vector<arr> qs;
      arr q = qStart;
      while (length(q - qGoal) > 0.01){
        q = extend(q, qGoal);
        qs.push_back(q);
      }
      return qs;
    };

    uint getNumNodes() const{
      return tstart_.nodes.size() + tgoal_.nodes.size();
    };

  private:
    std::shared_ptr<Node> start_;
    std::shared_ptr<Node> goal_;

    Tree tstart_;
    Tree tgoal_;
};

class Graph{
  public:
    typedef std::shared_ptr<Vertex> vPtr;

    Graph(){};

    void addVertices(const std::vector<vPtr> &v){
      for (auto &a: v){
        vertices_.push_back(a);
        ann.append(a->q_);
      }
    };

    std::pair<uint, uint> key(const vPtr &l, const vPtr &r){
      if (l->id_ < r->id_){
        return std::make_pair(l->id_, r->id_);
      }
      return std::make_pair(r->id_, l->id_);
    }

    bool couldEdgeBeValid(const vPtr &l, const vPtr &r){
      const auto k = key(l, r);
      if (edges_.count(k) > 0){
        const auto e = getEdge(l, r);
        return couldEdgeBeValid(e);
      }

      return true;
    }

    bool couldEdgeBeValid(const std::shared_ptr<Edge> e){
      if (e->isInvalid()){
        return false;
      }
      return true;
    }

    bool isEdgeKnownValid(const vPtr &l, const vPtr &r){
      const auto k = key(l, r);
      if (edges_.count(k) > 0){
        auto e = getEdge(l, r);

        if (e->isValid()){
          return true;
        }
      }

      return false;
    }

    std::vector<vPtr> getNeighbors(const vPtr &v) {
      std::vector<vPtr> unfiltered;
      std::chrono::steady_clock::time_point begin_nn = std::chrono::steady_clock::now();

      // knn
      if (vertices_.size() != v->nnTag_){
        if (true){ // using nn-structure
          const uint k = 
            std::ceil(1.001 * (2.718 + (2.718 / v->q_.d0)) * std::log(1.*vertices_.size()));

          //std::cout << k << std::endl;

          intA idx;
          if (vertices_.size() > k){
            ann.getkNN(idx, v->q_, k);
          }
          else{
            ann.getkNN(idx, v->q_, vertices_.size());
          }

          for (uint i=0; i<idx.d0; ++i){
            auto n = vertices_[idx(i)];

            if (n == v) {continue;}

            unfiltered.push_back(n);
          }
        }
        else{ // radius based
          for (auto n: vertices_){
            if (n == v) {continue;}

            //std::cout << length(n->q_ - v->q_) << std::endl;
            if (length(n->q_ - v->q_) > 2){
              continue;
            }

            unfiltered.push_back(n);
          }
        }

        v->neighbors_ = unfiltered;
        v->nnTag_ = vertices_.size();
      }
      else{
        unfiltered = v->neighbors_;
      }

      const bool keepWhitelistedNeighbors{false};
      if (keepWhitelistedNeighbors){
        unfiltered.insert(unfiltered.end(), v->whitelisted_.begin(), v->whitelisted_.end());
        // remove the elements that are duplicates
        std::set<vPtr> s( unfiltered.begin(), unfiltered.end() );
        unfiltered.assign( s.begin(), s.end() );
      }

      std::vector<vPtr> neighbors = unfiltered;
      /*std::vector<vPtr> neighbors;
      for (const auto &n: unfiltered){
        if (couldEdgeBeValid(v, n)){
          neighbors.push_back(n);
        }
      }*/
      /*
      std::cout << v->q_ << std::endl;
      for (auto n: neighbors){
        std::cout << n->q_ << std::endl;
      }
      std::cout << std::endl;
      */

      std::chrono::steady_clock::time_point end_nn = std::chrono::steady_clock::now();
      duration_nn += std::chrono::duration_cast<std::chrono::microseconds>(end_nn - begin_nn).count() / (1.*1e6);

      return neighbors;
    };

    std::vector<vPtr> open;
    std::unordered_map<vPtr, double> g;
    std::unordered_map<vPtr, vPtr> parents;

    //std::map<vPtr, double> g;
    //std::map<vPtr, vPtr> parents;

    bool search_running{false};

    uint calls{0u};

    std::vector<vPtr> search(const vPtr &start, const vPtr &goal, std::shared_ptr<EdgeChecker> ec=nullptr){
      ++calls;
      if(!search_running){
        parents.clear();
        open.clear();
        g.clear();

        open.push_back(start);

        g[start] = 0;

        search_running = true;
      }

      auto d = [](const vPtr &l, const vPtr &r){return length(l->q_ - r->q_);};
      auto h = [&goal, &d](const vPtr &r){return d(r, goal);};

      // pop top from open
      vPtr v = open.front();
      double f = g[v] + h(v);
      for (const auto &o: open){
        const double tmp = g[o] + h(o);
        //std::cout << tmp << std::endl;
        if (tmp < f){
          v = o;
          f = tmp;
        }
      }
      //std::cout << v->id_ << std::endl;
      open.erase(std::remove(open.begin(), open.end(), v), open.end());

      if (v == goal){
      //if (length(v->q_ - goal->q_) < 0.01){
        // extract path, return   
        std::vector<vPtr> path;
        auto p = v;
        while(true){
          path.push_back(p);
          if (parents.count(p) == 0){
            std::reverse(path.begin(), path.end());
            search_running = false;
            return path;
          }
          p = parents[p];
        }
      }

      const auto neighbors = getNeighbors(v);
      for (const auto &n: neighbors){
        auto e = getEdge(n, v);

        if (ec != nullptr && e->collCheckLevel_ == 0){
          const auto ci = ec->couldBeValid(n->q_, v->q_, 0u);
          e->collCheckLevel_ = 1;

          if (!ci){
            markInvalid(n, v);
            continue;
          }
        }


        const double c = g[v] + d(v, n);
        if (g.count(n) == 0 || g[n] > c){
          g[n] = c;
          parents[n] = v;

          if (n != v && std::find(open.begin(), open.end(), n) != open.end()) {continue;}
          open.push_back(n);
        }
      }

      //std::cout << "no path found in graph" << std::endl;
      if (open.size() == 0){
        search_running = false;
      }
      return {};
    };

    void markInvalid(const vPtr &l, const vPtr &r){
      auto edge = getEdge(l, r);
      edge->valid_ = EdgeStatus::invalid;
    }

    void markValid(const vPtr &l, const vPtr &r){
      auto edge = getEdge(l, r);
      edge->valid_ = EdgeStatus::valid;

      // if something is valid for the robot, we add it to the whitelist for a vertex
      l->whitelisted_.push_back(r);
      r->whitelisted_.push_back(l);
    }

    void clearTotalEdgeValidity(){
      for (auto e: edges_){
        e.second->valid_ = EdgeStatus::unknown;
      }
    }

    void resetCollChecks(){
      for (auto e: edges_){
        e.second->collCheckLevel_ = 0u;
      }
    }

    void clear(){
      vertices_.clear();
      edges_.clear();
      ann.clear();
    }

  //private:
    std::shared_ptr<Edge> getEdge(const vPtr &l, const vPtr &r){
      //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
      const auto k = key(l, r);
      std::shared_ptr<Edge> edge;

      if (edges_.count(k) == 0){
        edge = std::make_shared<Edge>(l, r);
        edges_[k] = edge;
      }
      else{
        edge = edges_[k];
      }
      /*const auto diff = std::chrono::duration_cast<
        std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begin).count();
      std::cout << "\t" << diff << std::endl;*/
      return edge;
    }
    
    double duration_nn{0.};

    ANN ann;

    std::vector<vPtr> vertices_;
    std::map<std::pair<uint, uint>, std::shared_ptr<Edge>> edges_;
};

struct Lines : GLDrawer {
  std::vector<std::pair<arr, arr>> pairs;
  arr color{0., 1., 1.};
  Lines(){};

  void glDraw(OpenGL &gl){
    for(auto &p: pairs){
      glColor(color);
      glLineWidth(2.f);
      glBegin(GL_LINES);

      glVertex3dv(&(p.first(0)));
      glVertex3dv(&(p.second(0)));

      glEnd();
      glLineWidth(2.f);
    }
  }
};

struct Points : GLDrawer {
  std::vector<arr> points;
  arr color{0., 0., 1.};

  Points(){};

  void glDraw(OpenGL &gl){
    glPointSize(3.);
    glBegin(GL_POINTS);
    glColor(color);
    for(auto &p: points){
      glVertex3dv(&(p(0)));
    }
    glEnd();
  }
};

class LPRM: public ContinuingPlanner{
  public:
    LPRM(ConfigurationProblem cp, std::shared_ptr<Problem> p): ContinuingPlanner(cp), batchSize_(100u){
      Vertex::nextID = 0;
      
      cc_ = std::make_shared<aCollisionChecker>(cp.C);

      auto hcc = std::make_shared<HierarchicalCollisionCheckerV4>(p, cp.C);
      ec_ = std::make_shared<EdgeChecker>(hcc);
      //ec_ = std::make_shared<EdgeChecker>(cc_);
    };

    virtual void set_resolution(const double resolution){
      ec_->resolution_ = resolution;
    }

    virtual double getCollCalls(){return ec_->cc_->calls_;};
    virtual void resetCollCalls(){
      ec_->cc_->calls_ = 0;
    };

    virtual void setup(const arr &q0, const arr &q1){
      start_ = nullptr;
      goal_ = nullptr;
      for (auto &v: G_.vertices_){
        if (length(v->q_ - q0) < 1e-3){
          start_ = v;
        }
        if (length(v->q_ - q1) < 1e-3){
          goal_ = v;
        }
      }
      if (start_ == nullptr){
        start_ = std::make_shared<Vertex>(q0);
      }
      if (goal_ == nullptr){
        goal_ = std::make_shared<Vertex>(q1);
      }

      G_.addVertices({start_, goal_});
      G_.addVertices(addSamples(batchSize_));
    }

    virtual arr plan(const arr &q0, const arr &q1, double time_in_s=-1){
      setup(q0, q1);
      return planMore(time_in_s);
    };
    virtual arr planMore(double time_in_s=-1){
      //std::cout << "LPRM" << std::endl;
      uint cnt = 0;

      if (duration_nn == 0){
        G_.duration_nn = 0.;
      }

      std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      while (true){
        //std::cout << G_.vertices_.size() << std::endl;
        const auto diff = std::chrono::duration_cast<
          std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();

        if (time_in_s > 0 && diff > time_in_s * 1000){
          return {};
        }

        if (false){
          std::cout << G_.getNeighbors(start_).size() << std::endl;
          std::cout << G_.getNeighbors(goal_).size() << std::endl;

          rai::ConfigurationViewer Vf;
          Vf.setConfiguration(cc_->cp.C, "\"Real World\"");

          Lines line;

          for (auto &v: G_.vertices_){
            const auto nei = G_.getNeighbors(v);
            for (auto n: nei){
              //if (!G_.isEdgeKnownValid(v, n)){
              //if (G_.couldEdgeBeValid(v, n)){
              if(G_.getEdge(v, n)->valid_ == EdgeStatus::invalid){
                arr p1{n->q_(0), n->q_(1), 1.};
                arr p2{v->q_(0), v->q_(1), 1.};
                line.pairs.push_back(std::make_pair(p1, p2));
              }
            }
          }

          Vf.add(line);
          Vf.watch();
        }

        // graph search
        while(true){
          //std::cout << "start search" << std::endl;
          const auto diff_inner_loop = std::chrono::duration_cast<
            std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();

          if (time_in_s > 0 && diff_inner_loop > time_in_s * 1000){
            return {};
          }

          std::chrono::steady_clock::time_point begin_search = std::chrono::steady_clock::now();

          auto vertexPath = G_.search(start_, goal_, ec_);
          while(G_.search_running){
            const auto diff_search_loop = std::chrono::duration_cast<
              std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();

            if (time_in_s > 0 && diff_search_loop > time_in_s * 1000){
              std::chrono::steady_clock::time_point end_search = std::chrono::steady_clock::now();
              duration_search += std::chrono::duration_cast<std::chrono::microseconds>(end_search - begin_search).count() / (1.*1e6);
              return {};
            }
            vertexPath = G_.search(start_, goal_, ec_);
          }

          std::chrono::steady_clock::time_point end_search = std::chrono::steady_clock::now();
          duration_search += std::chrono::duration_cast<std::chrono::microseconds>(end_search - begin_search).count() / (1.*1e6);
          //auto vertexPath = G_.search(start_, goal_);
          //std::cout << "done search" << std::endl;
          //const auto diff_inner_loop_2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();
          //std::cout << diff_inner_loop_2 << std::endl;
          if (vertexPath.size() != 0){
            // verify path
            //std::cout << "start feasibility" << std::endl;
            std::chrono::steady_clock::time_point begin_feas = std::chrono::steady_clock::now();

            const bool feasible = verifyPath(vertexPath);

            std::chrono::steady_clock::time_point end_feas = std::chrono::steady_clock::now();
            duration_verifying += std::chrono::duration_cast<std::chrono::microseconds>(end_feas - begin_feas).count() / (1.*1e6);
            //const bool feasible = verifyPathIncrementally(vertexPath);
            //std::cout << feasible << std::endl;

            if (feasible){
              arr path(vertexPath.size(), vertexPath[0]->q_.d0);
              for (uint i=0; i<vertexPath.size(); ++i){
                path[i] = vertexPath[i]->q_;
              }
              return path;
            }
          }
          else{break;}
        }

        duration_nn = G_.duration_nn;

        // add batch of samples
        G_.addVertices(addSamples(batchSize_));
        //std::cout << G_.vertices_.size() << std::endl;
        cnt++;
        //if (cnt == 1){return{};}
      }
    };

    void clearSpecificInformation(){
      G_.clearTotalEdgeValidity();
      //G_.clearAllEdgeValidity();
      G_.resetCollChecks();
    };

    void clearQuery(){
      G_.vertices_.clear();
      G_.ann.clear();
    }

    void clear(){
      G_.clear();
    }

  //private:
    bool verifyPath(const std::vector<std::shared_ptr<Vertex>> &vertexPath) {
      std::vector<uint> order(vertexPath.size()-1);
      std::iota(std::begin(order), std::end(order), 1);

      // check the path in a random order
      auto rng = std::default_random_engine {};
      std::shuffle(std::begin(order), std::end(order), rng);

      for (auto i: order){
        const auto l = vertexPath[i-1];
        const auto r = vertexPath[i];

        if(!G_.isEdgeKnownValid(l, r)){ // only check if we didnt mark yet
          if (ec_->isValid(l->q_, r->q_)){
            G_.markValid(l, r);
          }
          else{
            G_.markInvalid(l, r);
            return false;
          }
        }
      }

      return true;
    }

    bool verifyPathIncrementally(const std::vector<std::shared_ptr<Vertex>> &vertexPath){
      std::vector<uint> order(vertexPath.size()-1);
      std::iota(std::begin(order), std::end(order), 1);

      // check the path in a random order
      auto rng = std::default_random_engine {};
      std::shuffle(std::begin(order), std::end(order), rng);

      while(order.size() > 0){
        std::vector<uint> remove;

        for (auto i: order){
          const auto l = vertexPath[i-1];
          const auto r = vertexPath[i];

          if(!G_.isEdgeKnownValid(l, r)){ // only check if we didnt mark yet
            const uint numberOfRequiredChecks = ec_->getNumberOfCollisionChecks(l->q_, r->q_);
            const uint level = G_.getEdge(l, r)->collCheckLevel_;
            
            if (level >= numberOfRequiredChecks){
              // remove from order
              remove.push_back(i);

              G_.markValid(l, r);
              continue;
            }

            auto ci = ec_->couldBeValid(l->q_, r->q_, level);
            G_.getEdge(l, r)->collCheckLevel_ += 1;
            if (!ci){
              G_.markInvalid(l, r);
              return false;
            }
          }
          else{
            remove.push_back(i);
          }
        }
        for (auto i: remove){
          order.erase(std::remove(order.begin(), order.end(), i), order.end());
        }
      }

      return true;
    }

    std::vector<std::shared_ptr<Vertex>> addSamples(const uint n){
      std::chrono::steady_clock::time_point begin_sampling = std::chrono::steady_clock::now();

      std::vector<std::shared_ptr<Vertex>> vertices;
      while (vertices.size() < n){
        const arr q = cc_->cp.sample();
        cc_->cp.C.setJointState(q);
        //if (cc_->isValid(q).robot){
        if (cc_->isValid(q)){
          auto v = std::make_shared<Vertex>(q);
          vertices.push_back(v);
        }
      }

      std::chrono::steady_clock::time_point end_sampling = std::chrono::steady_clock::now();
      duration_sampling += std::chrono::duration_cast<std::chrono::microseconds>(end_sampling - begin_sampling).count() / (1.*1e6);
      return vertices;
    };

    const uint batchSize_{100u};

    std::shared_ptr<Vertex> start_;
    std::shared_ptr<Vertex> goal_;

    Graph G_;

    std::shared_ptr<EdgeChecker> ec_;
};

class SharedGraphInformation{
  typedef std::shared_ptr<Vertex> vPtr;

  struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        return h1 + 0x9e3779b9 + (h2<<6) + (h2>>2);;
    }
  };

  public:
    SharedGraphInformation(){
      Vertex::nextID = 0;
    };

    std::vector<vPtr> startGoalVertices_;

    std::vector<vPtr> vertices_;
    std::unordered_map<std::pair<uint, uint>, std::shared_ptr<HierarchicalEdge>, pair_hash> edges_;
    //std::map<std::pair<uint, uint>, std::shared_ptr<HierarchicalEdge>> edges_;

    uint cumulative_collision_checks_{0};
};

#endif
