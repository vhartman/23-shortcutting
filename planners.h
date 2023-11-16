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

#define USE_ANN
#define USE_HEAP

class HierarchicalGraph{
  public:
    typedef std::shared_ptr<Vertex> vPtr;
    typedef std::shared_ptr<HierarchicalEdge> ePtr;

    bool sparse_checking_enabled_{true};

    HierarchicalGraph(std::shared_ptr<SharedGraphInformation> base, const bool effort_ordered=false)
      :base_(base), effort_ordered_(effort_ordered)
#ifdef USE_HEAP
      ,open(effort_ordered ? getEffortComparisonOperator(): getCostComparisonOperator())
#endif
  {
#ifndef USE_ANN
        gnat.setDistanceFunction([](const vPtr l, const vPtr r){return euclideanDistance(l->q_, r->q_);});
#endif
      };
    bool effort_ordered_ = false;

    void addVertices(const std::vector<vPtr> &v){
      for (const auto &a: v){
        //localVertices_.push_back(a);
        localVertices_.push_back(std::make_shared<Vertex>(a->q_, a->id_));
#ifdef USE_ANN
        ann.append(a->q_);
#else
        gnat.add(localVertices_.back());
#endif
      }

    };

    static std::pair<uint, uint> key(const vPtr &l, const vPtr &r){
      if (l->id_ < r->id_){
        return std::make_pair(l->id_, r->id_);
      }
      return std::make_pair(r->id_, l->id_);
    }

    bool couldEdgeBeValid(const vPtr &l, const vPtr &r, const uint param){
      const auto k = key(l, r);
      if (blacklist_.count(k) > 0){
        return false;
      }/*

      if (whitelist_.count(k) > 0){
        return true;
      }*/

      const auto e = getEdgeIfExists(l, r);
      //if (base_->edges_.count(k) > 0){
        //const auto e = getEdge(l, r);
        return couldEdgeBeValid(e, param);
      //}

      //return true;
    }

    bool couldEdgeBeValid(const std::shared_ptr<HierarchicalEdge> &e, const uint param){
      if (e && e->isInvalid(param)){
        blacklist_.insert(key(e->n1_, e->n2_));
        return false;
      }
      return true;
    }

    bool isEdgeKnownValid(const std::shared_ptr<HierarchicalEdge> &e, const uint param){
      if (!e){
        return false;
      }

      if (e->isValid(param)){
        //whitelist_.insert(k);
        return true;
      }

      return false;
    }

    bool isEdgeKnownValid(const vPtr &l, const vPtr &r, const uint param){
      const auto k = key(l, r);
      if (blacklist_.count(k) > 0){
        return false;
      }/*

      if (whitelist_.count(k) > 0){
        return true;
      }*/

      const auto e = getEdgeIfExists(l, r);
      return isEdgeKnownValid(e, param);
    }

    std::vector<vPtr> getNeighbors(const vPtr &v, const uint param) {
      std::vector<vPtr> unfiltered;
      std::chrono::steady_clock::time_point begin_nn = std::chrono::steady_clock::now();

      if (v->nnTag_ != localVertices_.size()){
        // knn
        if (true){ // using nn-structure
          const uint k =
            std::ceil(1.001 * (2.718 + (2.718 / v->q_.d0)) * std::log(1.*localVertices_.size()));

          intA idx;
          if (localVertices_.size() > k){
#ifdef USE_ANN
            ann.getkNN(idx, v->q_, k);
#else
            gnat.nearestK(v, k, unfiltered);
#endif
          }
          else{
#ifdef USE_ANN
            ann.getkNN(idx, v->q_, localVertices_.size());
#else
            gnat.nearestK(v, localVertices_.size(), unfiltered);
#endif
          }

#ifdef USE_ANN
          unfiltered.reserve(idx.d0);
          for (uint i=0; i<idx.d0; ++i){
            const auto n = localVertices_[idx(i)];

            if (n == v) {continue;}

            unfiltered.push_back(n);
          }
#endif
        }
        else{ // radius based
          for (auto n: localVertices_){
            if (n == v) {continue;}

            //std::cout << length(n->q_ - v->q_) << std::endl;
            if (euclideanDistance(n->q_, v->q_) > 2){
              continue;
            }

            unfiltered.push_back(n);
          }
        }

        /*for (const auto n: unfiltered){
          const auto e = getEdgeIfExists(n, v);
          if (e && !couldEdgeBeValid(e, param)) {continue;}
          //if (couldEdgeBeValid(v, n, param)){
            neighbors.push_back(n);
          //}
        }*/
        /*std::vector<vPtr> filtered;
        for (const auto n: unfiltered){
          const auto e = getEdgeIfExists(n, v);
          if (e && !couldEdgeBeValid(e, param)) {continue;}
          //if (couldEdgeBeValid(v, n, param)){
            filtered.push_back(n);
          //}
        }

        unfiltered = filtered;*/
        v->neighbors_ = unfiltered;
        v->nnTag_ = localVertices_.size();
        //std::cout << "not reusing" << std::endl;
      }
      else{
        //std::cout << "reusing" << std::endl;
        unfiltered = v->neighbors_;
      }

      const bool keepWhitelistedNeighbors{false};
      if (keepWhitelistedNeighbors){
        unfiltered.insert(unfiltered.end(), v->whitelisted_.begin(), v->whitelisted_.end());
        // remove the elements that are duplicates
        std::set<vPtr> s( unfiltered.begin(), unfiltered.end() );
        unfiltered.assign( s.begin(), s.end() );
      }

      //std::vector<vPtr> neighbors = unfiltered;
      /*for (const auto n: unfiltered){
        const auto e = getEdgeIfExists(n, v);
        if (e && !couldEdgeBeValid(e, param)) {continue;}
        //if (couldEdgeBeValid(v, n, param)){
          neighbors.push_back(n);
        //}
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

      return v->neighbors_;
    };

    virtual double d_dist(const std::shared_ptr<HierarchicalEdge> &edge, const uint param, const uint numCollChecks) const {
      return d_dist(edge->n1_, edge->n2_);
    }
    virtual double d_dist(const vPtr &l, const vPtr &r) const {
      return euclideanDistance(l->q_, r->q_);
    }
    virtual double h_dist(const vPtr &r, const vPtr &goal) const{
      if (r->dist_to_goal > 0.){
        return r->dist_to_goal;
      }
      r->dist_to_goal = d_dist(r, goal);
      return r->dist_to_goal;
    }

    virtual double d_effort(const std::shared_ptr<HierarchicalEdge> &edge, const uint param, const uint totalCollChecks) const {
      //if (edge){
      if (edge && edge->collCheckLevelRobotEnv_ > 2){
        const uint requiredCollChecksRobotEnv = totalCollChecks - edge->collCheckLevelRobotEnv_;
        const uint requiredCollChecksRobotObs = totalCollChecks - edge->collCheckLevelObsEnv_[param];
        const uint requiredCollChecksObsEnv = totalCollChecks - edge->collCheckLevelRobotObs_[param];

        //std::cout << edge->collCheckLevelRobotEnv_ << " " << edge->collCheckLevelObsEnv_[param] << " " << edge->collCheckLevelRobotObs_[param] << std::endl;
        //std::cout << "A" << edge->collCheckLevelRobotEnv_ <<  std::endl;
        //std::cout << totalCollChecks << " " << requiredCollChecksRobotEnv << " " << requiredCollChecksRobotObs << " " << requiredCollChecksObsEnv << std::endl;

        //constexpr double weightRobotEnv{12./20.};
        //constexpr double weightRobotObs{3/20.};
        //constexpr double weightObsEnv{3./20};

        constexpr double weightRobotEnv{1./3.};
        constexpr double weightRobotObs{1/3.};
        constexpr double weightObsEnv{1./3.};

        //std::cout << "B" << weightRobotEnv * requiredCollChecksRobotEnv << std::endl;

        return weightRobotEnv * requiredCollChecksRobotEnv + 
          weightRobotObs * requiredCollChecksRobotObs + 
          weightObsEnv * requiredCollChecksObsEnv;
      }

      return totalCollChecks;
    }
    virtual double h_effort(const vPtr &r, const vPtr &goal) const{
      /*if (totalCollChecks < hec->robot_calls_){
        return 0u;
      }
      else{
      //const uint totalCollChecks - hec->robot_calls_ + claimedCollChecks;
      }*/
      return 0;
    }

    struct costs{
      double dist;
      double effort;
    };

    virtual bool cmp(const costs& l, const costs& r) const {
      if (effort_ordered_){
        if (l.effort != r.effort){
          return l.effort < r.effort;
        }
        return l.dist < r.dist;
      }
      return l.dist < r.dist;
    }

    //std::unordered_map<uint, costs> g;
    //std::map<vPtr, vPtr> parents;
    std::unordered_map<uint, costs> g;
    std::unordered_map<vPtr, vPtr> parents;

    struct Element{
      vPtr source;
      vPtr target;
      costs f;
      costs edge_cost;
      //costs g;
    };

    std::function<bool(const Element &, const Element &)> getEffortComparisonOperator() const{
      return [](const Element &lhs, const Element &rhs){
        if (lhs.f.effort != rhs.f.effort){
          return lhs.f.effort < rhs.f.effort;
        }
        return lhs.f.dist < rhs.f.dist;
      };
    }

    std::function<bool(const Element &, const Element &)> getCostComparisonOperator() const{
      return [](const Element &lhs, const Element &rhs){
        return lhs.f.dist < rhs.f.dist;
      };
    }


    const std::vector<std::pair<vPtr, vPtr>> expand(const vPtr& v){
      const auto& neighbors = getNeighbors(v, 0);
      std::vector<std::pair<vPtr, vPtr>> expand_buffer;
      expand_buffer.reserve(neighbors.size());
      for (const auto &n: neighbors){
        expand_buffer.emplace_back(v, n);
      }

      return expand_buffer;
    }

    void add_to_queue(const std::vector<std::pair<vPtr, vPtr>>& edges, const uint& param, const vPtr& goal, std::shared_ptr<HierarchicalEdgeCheckerV2> hec=nullptr, const double max_cost = std::numeric_limits<double>::max()){
      for (const auto& edge: edges){
        const auto tmp = getEdgeIfExists(edge.first, edge.second);

        const uint necessaryCollisionChecks = hec->getNumberOfCollisionChecks(edge.first->q_, edge.second->q_);
        if (tmp){
          claimed_effort += (necessaryCollisionChecks - tmp->collCheckLevelRobotEnv_);
        }

        if (tmp && !couldEdgeBeValid(tmp, param)){
          continue;
        }

        if (g.count(edge.second->id_) > 0){
          continue;
        }

        const costs& cost_to_get_to_source = g[edge.first->id_];

        const costs edge_cost_{
          .dist = d_dist(edge.first, edge.second),
          .effort = d_effort(tmp, param, necessaryCollisionChecks),
        };

        //std::cout << base_->cumulative_collision_checks_ << " " << claimed_effort << std::endl;

        if (edge.second->checks_to_goal < 0){
          edge.second->checks_to_goal = hec->getNumberOfCollisionChecks(edge.second->q_, goal->q_);
        }
        const uint collChecksToGoal = edge.second->checks_to_goal;

        uint effort_to_goal = 0;
        const auto goal_edge = getEdgeIfExists(edge.second, goal);
        if (goal_edge){
          effort_to_goal = d_effort(goal_edge, param, collChecksToGoal);
        }
        else if (collChecksToGoal > base_->cumulative_collision_checks_ - claimed_effort){
          if (claimed_effort > base_->cumulative_collision_checks_){
            std::cout << "AAAAAAAAAAAAAAAAAAA" << std::endl;
          }
          effort_to_goal = collChecksToGoal - (base_->cumulative_collision_checks_ - claimed_effort);
        }
        else{
          effort_to_goal = 0;
        }

        const costs target_to_goal = costs{
          .dist = h_dist(edge.second, goal),
          .effort = effort_to_goal,
        };

        const Element elem{
          .source = edge.first,
          .target = edge.second,
          .f = costs{
            .dist = cost_to_get_to_source.dist + edge_cost_.dist + target_to_goal.dist,
            .effort = cost_to_get_to_source.effort + edge_cost_.effort + target_to_goal.effort,
          },
          .edge_cost = edge_cost_,
        };

        if (elem.f.dist > max_cost){
          continue;
        }

#ifdef USE_HEAP
        open.insert(elem);
        //std::cout << open.size() << std::endl;
#else
        open.insert(std::upper_bound(open.begin(), open.end(), elem, [this](const Element& l, const Element& r){
                return cmp(r.f, l.f);
              }), elem);
#endif
      }
    }

#ifdef USE_HEAP
    using Heap =
        ompl::BinaryHeap<Element, std::function<bool(const Element &, const Element &)>>;
    Heap open;
#else
    std::vector<Element> open;
    //std::priority_queue<> open
    //std::set<uint> closed;
#endif

    costs goal_costs = costs{.dist=std::numeric_limits<double>::max(), .effort=std::numeric_limits<double>::max()};

    bool search_running{false};
    uint calls{0u};

    uint claimed_effort = 0;

    void set_queue_order(const bool effort_ordered){
      effort_ordered_ = effort_ordered;

      if (search_running){
        throw "cant change cost during running search";
      }

      open.clear();
      open.getComparisonOperator() = effort_ordered ? getEffortComparisonOperator(): getCostComparisonOperator();
    }

    std::vector<vPtr> search(const vPtr &start, const vPtr &goal, const uint param, std::shared_ptr<HierarchicalEdgeCheckerV2> hec=nullptr, const double max_cost = std::numeric_limits<double>::max()){
      ++calls;
      if(!search_running){
        parents.clear();
        open.clear();
        g.clear();
        //closed.clear();

        //std::cout << start << " " << goal << std::endl;
        //open.push_back(start);
#ifdef USE_HEAP
        //open.insert(Element{.v = start});
#else
        //open.push_back(Element{.v = start});
#endif
        add_to_queue(expand(start), param, goal, hec, max_cost);

        g[start->id_] = costs{.dist=0, .effort=0};

        goal_costs = costs{.dist=std::numeric_limits<double>::max(), .effort=std::numeric_limits<double>::max()};

        search_running = true;
        claimed_effort = 0;
      }

      // pop top from open
      /*for (const auto elem: open){
        std::cout << elem.f.dist << " " << elem.f.effort << std::endl;
      }
      std::cout << std::endl;
      */
      //std::cout << open.back().f.dist << " " << open.back().f.effort << std::endl;
      if (open.size() == 0){
        search_running = false;
        return {};
      }
#ifdef USE_HEAP
      //vPtr v = open.top();
      //open.pop();
      
      const Element element = open.top()->data;
      const vPtr source = element.source;
      const vPtr target = element.target;

      //std::cout << element.f.dist << " " << element.f.effort << std::endl;

      open.pop();
#else
      //vPtr v = open.back().v;
      //open.pop_back();

      Element element = open.back();
      vPtr source = open.back().source;
      vPtr target = open.back().target;

      open.pop_back();
#endif

      if (g.count(target->id_) > 0){
        return {};
      }

      if (element.f.dist > max_cost){
        search_running = false;
        return {};
      }

      /*vPtr v = open.front().v;
      costs f = open.front().f;
      for (const auto &o: open){
        //std::cout << tmp << std::endl;
        if (cmp(o.f, f)){
          v = o.v;
          
          f = o.f;
        }
      }
      open.erase(std::remove_if(open.begin(), open.end(), [v](const auto& l){return l.v == v;}), open.end());*/
      //std::cout << v->id_ << std::endl;
      //std::cout << open.size() << std::endl;
      //std::cout << v->id_ << std::endl;
      //std::cout << base_->vertices_.size() << std::endl;

      /*if (closed.count(v->id_) > 0){
        return {};
      }*/

      /*if (parents.count(v) > 0){
        const auto edge = getEdge(v, parents[v]);
        if (hec && edge->collCheckLevelRobotEnv_ == 0){
          sparselyValidatedEdge_ += 1;
          hec->couldBeValid(*edge, param);

          base_->cumulative_collision_checks_ += 1;
        }
        if (!couldEdgeBeValid(edge, param)) {
          g.erase(v->id_);
          return{};
        }
      }*/

      const auto edge = getEdge(source, target);
      if (sparse_checking_enabled_){
        if (hec && edge->collCheckLevelRobotEnv_ == 0){
          sparselyValidatedEdge_ += 1;
          const bool res = hec->couldBeValid(*edge, param);

          // this effort can not be claimed by something else
          if (res){
            base_->cumulative_collision_checks_ += 1;
            claimed_effort += 1;
          }
        }
      }
      if (!couldEdgeBeValid(edge, param)) {
        return{};
      }

      parents[target] = source;
      const costs g_source = g[source->id_]; 
      g[target->id_] = costs{
        .dist = g_source.dist + element.edge_cost.dist,
        .effort = g_source.effort + element.edge_cost.effort,
      };

      //closed.insert(v->id_);

      //std::cout << v->id_ << " " << goal->id_ << std::endl;

      //if (v == goal){
      if (target->id_ == goal->id_){
      //if (v->id_ ==  goal->id_){
      //if (length(v->q_ - goal->q_) < 0.01){
        // extract path, return   
        std::vector<vPtr> path;
        auto p = target;
        //auto p = v;
        while(true){
          //std::cout << p->q_ << std::endl;
          path.push_back(p);
          if (p == start){
            // reverse the path, since we started from the goal
            std::reverse(path.begin(), path.end());

            //std::cout << "path" << std::endl;
            search_running = false;
            return path;
          }
          p = parents[p];
        }
      }

      add_to_queue(expand(target), param, goal, hec, max_cost);

      if (open.size() == 0){
        search_running = false;
      }
      //std::cout << "no path found in graph" << std::endl;
      return {};
    };

  //private:
    std::shared_ptr<HierarchicalEdge> getEdgeIfExists(const vPtr &l, const vPtr &r){
      //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      const auto k = key(l, r);
      //std::shared_ptr<HierarchicalEdge> edge;

      const auto it = base_->edges_.find(k);
      if (it != base_->edges_.end()){
        return it->second;
      }
      return nullptr;
      /*if (base_->edges_.count(k) > 0){
        edge = base_->edges_[k];
      }*/
      /*const auto diff = std::chrono::duration_cast<
        std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begin).count();
      std::cout << diff << std::endl;*/
      //return edge;
    }
    std::shared_ptr<HierarchicalEdge> getEdge(const vPtr &l, const vPtr &r){
      //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      const auto k = key(l, r);
      //std::shared_ptr<HierarchicalEdge> edge;

      const auto it = base_->edges_.find(k);
      if (it != base_->edges_.end()){
        return it->second;
      }
      else{
        auto edge = std::make_shared<HierarchicalEdge>(l, r);
        base_->edges_[k] = edge;
        return edge;
      }

      /*if (base_->edges_.count(k) == 0){
        edge = std::make_shared<HierarchicalEdge>(l, r);
        base_->edges_[k] = edge;
      }
      else{
        edge = base_->edges_[k];
      }*/
      /*const auto diff = std::chrono::duration_cast<
        std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begin).count();
      std::cout << diff << std::endl;*/
      //return edge;
    }
    
    double duration_nn{0.};

    ompl::NearestNeighborsGNAT<vPtr> gnat;
    //ompl::NearestNeighborsLinear<vPtr> gnat;
    //ompl::NearestNeighborsSqrtApprox<vPtr> gnat;

    ANN ann;
    std::vector<vPtr> localVertices_;

    struct pair_hash {
      template <class T1, class T2>
      std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        return h1 + 0x9e3779b9 + (h2<<6) + (h2>>2);
      }
    };

    std::unordered_set<std::pair<uint, uint>, pair_hash> whitelist_;
    std::unordered_set<std::pair<uint, uint>, pair_hash> blacklist_;

    //std::set<std::pair<uint, uint>> whitelist_;
    //std::set<std::pair<uint, uint>> blacklist_;

    std::shared_ptr<SharedGraphInformation> base_;

    uint sparselyValidatedEdge_ = 0;
};

class HierarchicalLPRM: public ContinuingPlanner{
  public:
    const bool manageGraphSize_;
    const bool sparseChecking_;

    HierarchicalLPRM(ConfigurationProblem cp, 
        std::shared_ptr<Problem> p, 
        const uint param, const bool effortOrdered, const bool manageGraphSize, const bool sparseChecking,
        std::shared_ptr<SharedGraphInformation> base)
      :ContinuingPlanner(cp),
      parametrization_(param), 
      base_(base), 
      G_(base_, effortOrdered), 
      batchSize_(500), 
      manageGraphSize_(manageGraphSize),
      sparseChecking_(sparseChecking){

      //hcc = std::make_shared<HierarchicalCollisionCheckerV4>(p, cp.C);
      hec = std::make_shared<HierarchicalEdgeCheckerV2>(p, cp.C);

      G_.sparse_checking_enabled_ = sparseChecking_;
    };

    virtual void set_resolution(const double resolution){
      hec->resolution_ = resolution;
    }

    /*virtual double getCollCalls(){return hec->hcc.calls_;};
    virtual void resetCollCalls(){
      hec->hcc.calls_ = 0;
    };*/

    virtual void setup(const arr &q0, const arr &q1){
      //std::cout << q0 << std::endl;
      //std::cout << q1 << std::endl;

      //hec->hcc->cp.C.setJointState(q0);
      //hec->hcc->cp.C.watch(true);

      //hec->hcc->cp.C.setJointState(q1);
      //hec->hcc->cp.C.watch(true);
      
      if (!hec->hcc->isValid(q0, true, true, true)){
        std::cout << "start not valid" << std::endl;
      }
      if (!hec->hcc->isValid(q1, true, true, true)){
        std::cout << "goal not valid" << std::endl;
      }

      G_.addVertices(base_->startGoalVertices_);
      if (manageGraphSize_){
        G_.addVertices(addSamples(batchSize_));
      }
      else{
        const uint current_num_vertices = base_->vertices_.size();
        G_.addVertices(addSamples(std::max({batchSize_, current_num_vertices})));
      }

      //std::cout << q0 << " " << q1 << std::endl;

      start_ = nullptr;
      goal_ = nullptr;
      for (const auto &v: G_.localVertices_){
        if (!start_ && euclideanDistance(v->q_, q0) < 1e-4){
          start_ = v;
          std::cout << "using start vertex, dist " << euclideanDistance(v->q_, q0) << std::endl;
        }
        if (!goal_ && euclideanDistance(v->q_, q1) < 1e-4){
          std::cout << "using goal vertex" << std::endl;
          goal_ = v;
        }

        if (start_ && goal_){
          break;
        }
      }

      // start and goal vertices are not added to the shared information
      if (start_ == nullptr){
        auto tmp = std::make_shared<Vertex>(q0);
        G_.addVertices({tmp});
        start_ = G_.localVertices_.back();

        base_->startGoalVertices_.push_back(tmp);
      }
      if (goal_ == nullptr){
        auto tmp = std::make_shared<Vertex>(q1);
        G_.addVertices({tmp});
        goal_ = G_.localVertices_.back();

        base_->startGoalVertices_.push_back(tmp);
      }
    }

    virtual arr plan(const arr &q0, const arr &q1, double time_in_s=-1){
      setup(q0, q1);
      return planMore(time_in_s);
    };
    virtual arr planMore(double time_in_s=-1){
      //std::cout << "LPRM" << std::endl;
      if (duration_nn == 0){
        G_.duration_nn = 0.;
      }

      uint cnt = 0;

      if (false){
        rai::ConfigurationViewer Vf;
        Vf.setConfiguration(hec->hcc->cp.C, "\"Real World\"");

        Points localPoints;

        for (auto &v: G_.localVertices_){
          arr p{v->q_(0), v->q_(1), 1.};
          localPoints.points.push_back(p);
        }

        Vf.add(localPoints);

        /*Points allPoints;
        allPoints.color = {1., 0., 0.};

        for (auto &v: base_->vertices_){
          arr p{v->q_(0), v->q_(1), 1.};
          allPoints.points.push_back(p);
        }

        Vf.add(allPoints);*/

        Vf.watch();
      }

      if (false){
        std::cout << G_.getNeighbors(start_, parametrization_).size() << std::endl;
        std::cout << G_.getNeighbors(goal_, parametrization_).size() << std::endl;

        rai::ConfigurationViewer Vf;
        Vf.setConfiguration(cc_->cp.C, "\"Real World\"");

        Lines line;

        for (auto &v: G_.localVertices_){
          auto nei = G_.getNeighbors(v, parametrization_);
          for (auto n: nei){
            //if (!G_.isEdgeKnownValid(v, n, parametrization_)){
            //if (G_.couldEdgeBeValid(v, n, parametrization_)){
            if(G_.getEdge(v, n)->validForRobotEnv_ == EdgeStatus::valid){
            //{
              arr p1{n->q_(0), n->q_(1), 1.};
              arr p2{v->q_(0), v->q_(1), 1.};
              line.pairs.push_back(std::make_pair(p1, p2));
            }
          }
        }

        Vf.add(line);
        Vf.watch();
      }

      std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      while (true){
        //std::cout << G_.localVertices_.size() << std::endl;
        //std::cout << base_->vertices_.size() << std::endl;
        const auto diff = std::chrono::duration_cast<
          std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();

        if (time_in_s > 0 && diff > time_in_s * 1000){
          return best_path;
        }

        if (false){
          std::cout << G_.getNeighbors(start_, parametrization_).size() << std::endl;
          std::cout << G_.getNeighbors(goal_, parametrization_).size() << std::endl;

          rai::ConfigurationViewer Vf;
          Vf.setConfiguration(hec->hcc->cp.C, "\"Real World\"");

          Lines line;

          for (auto &v: G_.localVertices_){
            auto nei = G_.getNeighbors(v, parametrization_);
            for (auto n: nei){
              //if (!G_.isEdgeKnownValid(v, n)){
              //if (G_.couldEdgeBeValid(v, n)){
              //if(G_.getEdge(v, n)->validForRobotEnv_ == EdgeStatus::valid){
              if (G_.getEdge(v, n)->collCheckLevelRobotEnv_ >= 1){
              //{
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
            // std::cout << "exiting before search (" << diff_inner_loop << ")" << std::endl;
            return best_path;
          }

          std::chrono::steady_clock::time_point begin_search = std::chrono::steady_clock::now();

          std::vector<std::shared_ptr<Vertex>> vertexPath;
          if (searchInReverseOrder_){
            vertexPath = G_.search(goal_, start_, parametrization_, hec, best_cost);
            if (vertexPath.size() != 0){
              std::reverse(vertexPath.begin(), vertexPath.end());
            }
          }
          else{
            //std::cout << start_ << " " << goal_ << std::endl;
            vertexPath = G_.search(start_, goal_, parametrization_, hec, best_cost);
          }

          while(G_.search_running){
            const auto diff_search_loop = std::chrono::duration_cast<
              std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();

            if (time_in_s > 0 && diff_search_loop > time_in_s*1000){
              // std::cout << "exiting at search (" << diff_search_loop << ")" << std::endl;
              std::chrono::steady_clock::time_point end_search = std::chrono::steady_clock::now();
              duration_search += std::chrono::duration_cast<std::chrono::microseconds>(end_search - begin_search).count() / (1.*1e6);
              return best_path;
            }
            if (searchInReverseOrder_){
              vertexPath = G_.search(goal_, start_, parametrization_, hec, best_cost);
              if (vertexPath.size() != 0){
                std::reverse(vertexPath.begin(), vertexPath.end());
              }
            }
            else{
              vertexPath = G_.search(start_, goal_, parametrization_, hec, best_cost);
            }

            if (false){
              //std::cout << G_.getNeighbors(start_, parametrization_).size() << std::endl;
              //std::cout << G_.getNeighbors(goal_, parametrization_).size() << std::endl;

              rai::ConfigurationViewer Vf;
              Vf.setConfiguration(hec->hcc->cp.C, "\"Real World\"");

              Lines line3;

              for (auto &v: G_.localVertices_){
                auto nei = G_.getNeighbors(v, parametrization_);
                for (auto n: nei){
                  //if (!G_.isEdgeKnownValid(v, n, parametrization_)){
                  //if (G_.couldEdgeBeValid(v, n, parametrization_)){
                  if(G_.getEdge(v, n)->validForRobotEnv_ == EdgeStatus::valid){
                  //{
                    arr p1{n->q_(0), n->q_(1), 1.1};
                    arr p2{v->q_(0), v->q_(1), 1.1};
                    line3.pairs.push_back(std::make_pair(p1, p2));
                    //std::cout << n->id_ << " " << v->id_ <<std::endl;
                  }
                }
              }

              Vf.add(line3);
              Lines line;
              line.color(0) = 0.5;
              line.color(1) = 0.5;
              line.color(2) = 0;

              for (auto &v: G_.localVertices_){
                auto nei = G_.getNeighbors(v, parametrization_);
                for (auto n: nei){
                  //if (!G_.isEdgeKnownValid(v, n, parametrization_)){
                  if (G_.getEdge(v, n)->collCheckLevelRobotEnv_ >= 1){
                  //if(G_.getEdge(v, n)->validForRobotEnv_ == EdgeStatus::valid){
                  //{
                    arr p1{n->q_(0), n->q_(1), 1.};
                    arr p2{v->q_(0), v->q_(1), 1.};
                    line.pairs.push_back(std::make_pair(p1, p2));
                    //std::cout << n->id_ << " " << v->id_ <<std::endl;
                  }
                }
              }

              Vf.add(line);

              Vf.watch();
            }
          }

          std::chrono::steady_clock::time_point end_search = std::chrono::steady_clock::now();
          duration_search += std::chrono::duration_cast<std::chrono::microseconds>(end_search - begin_search).count() / (1.*1e6);
          //auto vertexPath = G_.search(start_, goal_);
          //std::cout << "done search" << std::endl;
          if (vertexPath.size() != 0){
            // verify path
            //std::cout << "start feasibility" << std::endl;
            //std::cout << "\t verifying" << std::endl;
            std::chrono::steady_clock::time_point begin_feas = std::chrono::steady_clock::now();

            const bool feasible = verifyPath(vertexPath);

            std::chrono::steady_clock::time_point end_feas = std::chrono::steady_clock::now();
            duration_verifying += std::chrono::duration_cast<std::chrono::microseconds>(end_feas - begin_feas).count() / (1.*1e6);

            //const bool feasible = verifyPathIncrementally(vertexPath);
            //std::cout << "feasibility:" << feasible << std::endl;

            if (feasible){
              arr path(vertexPath.size(), vertexPath[0]->q_.d0);
              for (uint i=0; i<vertexPath.size(); ++i){
                path[i] = vertexPath[i]->q_;
                //std::cout << "P" << vertexPath[i]->id_ << std::endl;
              }
              //std::cout << path.d0 << " " << length(path[0] - path[1]) << " " << length(path[-1] - path[-2]) << std::endl;
              if (verbose_ > 1) std::cout << hec->calls_ << " " << hec->robot_calls_ << " " << hec->obs_calls_ << " " << hec->env_calls_ << std::endl;
              if (verbose_ > 1) std::cout << path.d0 << " " << free_edge_counter << std::endl;
              if (verbose_ > 1) std::cout << G_.sparselyValidatedEdge_ << std::endl;
              //std::cout << hec->calls_ << std::endl;

              const auto diff = std::chrono::duration_cast<
                std::chrono::microseconds>(std::chrono::steady_clock::now() - begin).count() / (1.*1e6);
              
              const double pathlength = get_path_length(path);
              if (pathlength < best_cost){
                best_path = path;
                best_cost = pathlength;

                costs.push_back(std::make_pair(diff, pathlength));
              }

              G_.set_queue_order(false);

              if (stop_at_first_solution){
                return path;
              }

              break;
            }
          }
          else{break;}
        }

        duration_nn = G_.duration_nn;

        // add batch of samples
        //std::cout << "\t\t\tadding samples in param " << parametrization_ << std::endl;
        G_.addVertices(addSamples(batchSize_));

        // std::cout << G_.localVertices_.size() << std::endl;
        // std::cout << G_.base_->vertices_.size() << " " << G_.base_->edges_.size() << std::endl;

        ++cnt;

        //if (cnt == 1){return{};}
      }
    };

    double best_cost{std::numeric_limits<double>::max()};
    arr best_path;

  //private:
    bool searchInReverseOrder_{false};
    bool verifyPathInReverseOrder_{false};

    uint free_edge_counter{0};

    bool verifyPath(const std::vector<std::shared_ptr<Vertex>> &vertexPath) {
      free_edge_counter = 0u;

      std::vector<uint> order(vertexPath.size()-1);
      std::iota(std::begin(order), std::end(order), 1);

      if (verifyPathInReverseOrder_){
        std::reverse(order.begin(), order.end());
      }

      // Path reuse is higher if we do not check in a completely random order
      //auto rng = std::default_random_engine {};
      //std::shuffle(std::begin(order), std::end(order), rng);

      //std::cout << "A" << start_->id_ << " " << goal_->id_ << std::endl;
      //std::cout << "A" << start_->q_ << " " << goal_->q_ << std::endl;
      for (const auto i: order){
        const auto l = vertexPath[i-1];
        const auto r = vertexPath[i];
        //std::cout << i << " " << l->id_ << " " << r->id_ << std::endl;
        if (false){
          //std::cout << G_.getNeighbors(start_, parametrization_).size() << std::endl;
          //std::cout << G_.getNeighbors(goal_, parametrization_).size() << std::endl;

          rai::ConfigurationViewer Vf;
          Vf.setConfiguration(hec->hcc->cp.C, "\"Real World\"");

          Lines line2;
          line2.color(0) = 1;

          for (auto &v: G_.localVertices_){
            auto nei = G_.getNeighbors(v, parametrization_);
            for (auto n: nei){
              //if (!G_.isEdgeKnownValid(v, n, parametrization_)){
              //if (G_.couldEdgeBeValid(v, n, parametrization_)){
              //if(G_.getEdge(v, n)->validForRobotEnv_ == EdgeStatus::valid){
              {
                arr p1{n->q_(0), n->q_(1), 0.9};
                arr p2{v->q_(0), v->q_(1), 0.9};
                line2.pairs.push_back(std::make_pair(p1, p2));
                //std::cout << n->id_ << " " << v->id_ <<std::endl;
              }
            }
          }

          Vf.add(line2);

          Lines line3;

          for (auto &v: G_.localVertices_){
            auto nei = G_.getNeighbors(v, parametrization_);
            for (auto n: nei){
              //if (!G_.isEdgeKnownValid(v, n, parametrization_)){
              //if (G_.couldEdgeBeValid(v, n, parametrization_)){
              if(G_.getEdge(v, n)->validForRobotEnv_ == EdgeStatus::valid){
              //{
                arr p1{n->q_(0), n->q_(1), 1.1};
                arr p2{v->q_(0), v->q_(1), 1.1};
                line3.pairs.push_back(std::make_pair(p1, p2));
                //std::cout << n->id_ << " " << v->id_ <<std::endl;
              }
            }
          }

          Vf.add(line3);
          Lines line;
          line.color(0) = 0.5;
          line.color(1) = 0.5;
          line.color(2) = 0;

          for (auto &v: G_.localVertices_){
            auto nei = G_.getNeighbors(v, parametrization_);
            for (auto n: nei){
              //if (!G_.isEdgeKnownValid(v, n, parametrization_)){
              if (G_.getEdge(v, n)->collCheckLevelRobotEnv_ >= 1){
              //if(G_.getEdge(v, n)->validForRobotEnv_ == EdgeStatus::valid){
              //{
                arr p1{n->q_(0), n->q_(1), 1.};
                arr p2{v->q_(0), v->q_(1), 1.};
                line.pairs.push_back(std::make_pair(p1, p2));
                //std::cout << n->id_ << " " << v->id_ <<std::endl;
              }
            }
          }

          Vf.add(line);

          Vf.watch();
        }

        //hec->hcc->cp.C.setJointState(l->q_);
        //hec->hcc->cp.C.watch(true);

        const uint prev_calls = hec->robot_calls_;
        const auto e = G_.getEdge(l, r);
        //std::cout << hec->calls_ << " " << hec->robot_calls_ << " " << hec->obs_calls_ << " " << hec->env_calls_ << std::endl;
        if(!G_.isEdgeKnownValid(e, parametrization_)){ // only check if we didnt mark yet
          //std::cout << e->collCheckLevelRobotEnv_ << std::endl;
          //std::cout << "QQQQQQQQQQQQQQQQQQQQQ" << std::endl;
          if (!hec->isValid(*e, parametrization_)){
            //base_->cumulative_collision_checks_ += hec->robot_calls_ - prev_calls;
            return false;
          }
        }
        else{
          //free_edge_counter += 1;
          //std::cout << "already valid" << std::endl;
        }
        base_->cumulative_collision_checks_ += hec->robot_calls_ - prev_calls;
        //std::cout << hec->calls_ << " " << hec->robot_calls_ << " " << hec->obs_calls_ << " " << hec->env_calls_ << std::endl;

        if (prev_calls == hec->robot_calls_ && e->annotatedValidAt != parametrization_){
          free_edge_counter += 1;
        }
      }

      return true;
    }

    std::vector<std::shared_ptr<Vertex>> addSamples(const uint n){
      std::chrono::steady_clock::time_point begin_sampling = std::chrono::steady_clock::now();

      std::vector<std::shared_ptr<Vertex>> vertices;
      uint newSamples{0u};

      while (vertices.size() < n){
        //bool isnew = false;
        if (baseBufferIndex_ >= base_->vertices_.size()){
          //isnew = true;
          while(true){
            const arr q = hec->hcc->cp.sample();
            if (hec->hcc->isValidForRobotEnv(q)){
              //hec->hcc->cp.C.watch(true);
              auto v = std::make_shared<Vertex>(q);
              base_->vertices_.push_back(v);
              newSamples++;
              break;
            }
          }
        }
        else{
          //std::cout << "reusin sample" << std::endl;
        }

        const auto v = base_->vertices_[baseBufferIndex_];
        // we do not need to check the collision with the env here
        bool can_improve_cost = true;
        if (start_ && goal_) {
          if (v->dist_to_goal < 0){
            v->dist_to_goal = euclideanDistance(goal_->q_, v->q_);
          }
          const double dist_to_goal = v->dist_to_goal;
          can_improve_cost = (euclideanDistance(start_->q_, v->q_) + dist_to_goal) < best_cost;
        }
        if (can_improve_cost && hec->hcc->isValid(v->q_, false, true, true)){
        //if (hcc->isValidForRobotObs(v->q_) && hcc->isValidForObsEnv(v->q_)){
          vertices.push_back(v);
          //if (isnew) std::cout << "A" << std::endl;
          //else{std::cout << "B" << std::endl;}
        }

        baseBufferIndex_++;
      }

      // std::cout << "\t\t\t" << base_->vertices_.size() << " " << newSamples << std::endl;

      std::chrono::steady_clock::time_point end_sampling = std::chrono::steady_clock::now();
      duration_sampling += std::chrono::duration_cast<std::chrono::microseconds>(end_sampling - begin_sampling).count() / (1.*1e6);
      return vertices;
    };

    std::shared_ptr<Vertex> start_;
    std::shared_ptr<Vertex> goal_;

    const uint parametrization_;

    const uint batchSize_{100u};

    uint baseBufferIndex_{0u};
    std::shared_ptr<SharedGraphInformation> base_;

    HierarchicalGraph G_;
    //std::shared_ptr<HierarchicalCollisionCheckerV4> hcc;
    std::shared_ptr<HierarchicalEdgeCheckerV2> hec;
};

enum class OMPLPlannerType{
  RRTConnect=0,
  EITStar,
};

class OMPLPlanner: public ContinuingPlanner{
  ompl::base::SpaceInformationPtr si;
  std::shared_ptr<ompl::geometric::SimpleSetup> ss;
  const uint dim;

  const OMPLPlannerType planner_type;

  public:
    OMPLPlanner(const ConfigurationProblem& cp, std::shared_ptr<Problem> p, const OMPLPlannerType type)
      :ContinuingPlanner(cp), dim(cp.C.getJointState().N), planner_type(type){
      auto state_space = std::make_shared<ompl::base::RealVectorStateSpace>(dim);

      ompl::base::RealVectorBounds bounds(dim);

      //std::cout << cp.limits << std::endl;

      for(uint i=0; i < cp.limits.d0; ++i){
        if(cp.limits(i, 0) == cp.limits(i, 1)){
          bounds.low[i] = -10.;
          bounds.high[i] = 10.;
        }
        else{
          bounds.low[i] = cp.limits(i, 0) * 1.5 - 0.2;
          bounds.high[i] = cp.limits(i, 1) * 1.5 + 0.2;
        }
      }
      state_space->setBounds(bounds);

      si = std::make_shared<ompl::base::SpaceInformation>(state_space);

      vc = std::make_shared<OMPLValidityChecker>(p, cp.C, si);
      si->setStateValidityChecker(vc);
      const auto extent = si->getMaximumExtent();
      si->setStateValidityCheckingResolution(RESOLUTION/extent);
    };

    virtual void set_resolution(const double resolution){
      const auto extent = si->getMaximumExtent();
      si->setStateValidityCheckingResolution(resolution/extent);
    }

    virtual void setup(const arr &q0, const arr &q1){
      ompl::base::ScopedState<> start(si);
      for (int i = 0; i < q0.d0; ++i) {
        start[i] = q0(i);
      }

      ompl::base::ScopedState<> goal(si);
      for (int i = 0; i < q1.d0; ++i) {
        goal[i] = q1(i);
      }

      ss = std::make_shared<ompl::geometric::SimpleSetup>(si);
      ss->setStartState(start);
      ss->setGoalState(goal);
    };

    virtual arr plan(const arr &q0, const arr &q1, double time_in_s=-1){
      setup(q0, q1);

      if (planner_type == OMPLPlannerType::EITStar){
        auto *eit = new ompl::geometric::EITstar(si);
        eit->setBatchSize(500);
        ss->setPlanner(ompl::base::PlannerPtr(eit));
      }
      else if (planner_type == OMPLPlannerType::RRTConnect){
        auto *rrtConnect = new ompl::geometric::RRTConnect(si);
        ss->setPlanner(ompl::base::PlannerPtr(rrtConnect));
      }
      else{
        throw "no available planner specified";
      }

      if (stop_at_first_solution){
        ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
        obj->setCostThreshold(ompl::base::Cost(1e6));
        ss->setOptimizationObjective(obj);
      }
      
      return planMore(time_in_s);
    };

    virtual arr planMore(double time_in_s = -1){
      double clipped_time = time_in_s;
      if (time_in_s < 0){
        clipped_time= 100;
      }

      // solve this in a separate thread
      std::future<void> future = std::async(std::launch::async, [&]() {
        //planner->solve(maxSolveDuration);
        ss->solve(clipped_time);
      });

      // query the cost in another thread
      //double resolution = 1e-3;
      auto idle = std::chrono::milliseconds(1);
      
      auto start = std::chrono::system_clock::now();
      auto now = std::chrono::system_clock::now();
      do{
        double cost = std::numeric_limits<double>::max();
        if (planner_type == OMPLPlannerType::EITStar){
          cost = ss->getPlanner()->as<ompl::geometric::EITstar>()->bestCost().value();
        }
        else if (planner_type == OMPLPlannerType::RRTConnect){
        }
        //double cost = ss->getPlanner()->getBestCost();
        now = std::chrono::system_clock::now();
        auto current_time = std::chrono::duration_cast<std::chrono::microseconds>(now-start).count() / (1.*1e6);
        if (cost > 0){
          if (costs.size() == 0 || cost < costs.back().second){
            costs.push_back(std::make_pair(current_time, cost));
          }
        }
        //std::cout << current_time << " " << cost << std::endl;
      } while (future.wait_until(now + idle) != std::future_status::ready);

      future.get();

      auto sol = ss->getSolutionPath();

      now = std::chrono::system_clock::now();
      auto current_time = std::chrono::duration_cast<std::chrono::microseconds>(now-start).count() / (1.*1e6);
      costs.push_back(std::make_pair(current_time, sol.length()));

      // parse path
      arr path;

      uint cnt = 0;
      for (auto s: sol.getStates()){
        const double* _state =
          s->as<ompl::base::RealVectorStateSpace::StateType>()->values;

        arr q(_state, dim, true);
        path.append(q);
        cnt++;
      }

      path.resize(cnt, dim);
      //std::cout << vc->numQueries << std::endl;

      return path;
    };

    virtual double getCollCalls(){return 0.;};
    virtual void resetCollCalls(){};

    double duration_sampling{0.};
    double duration_search{0.};
    double duration_nn{0.};
    double duration_verifying{0.};

    std::shared_ptr<OMPLValidityChecker> vc;
  private:

};

uint EIRM_FWD_EFFORT{0u};

class HierarchicalGraphV2{
  public:
    typedef std::shared_ptr<Vertex> vPtr;
    typedef std::shared_ptr<HierarchicalEdge> ePtr;

    HierarchicalGraphV2(std::shared_ptr<SharedGraphInformation> base, const bool effort_ordered=false)
      :base_(base), effort_ordered_(effort_ordered)
#ifdef USE_HEAP
      ,open(effort_ordered ? getEffortComparisonOperator(): getCostComparisonOperator())
#endif
  {
#ifndef USE_ANN
        gnat.setDistanceFunction([](const vPtr l, const vPtr r){return euclideanDistance(l->q_, r->q_);});
#endif
      };
    bool effort_ordered_ = false;

    void set_queue_order(const bool effort_ordered){
      effort_ordered_ = effort_ordered;

      if (search_running){
        throw "cant change cost during running search";
      }

      open.clear();
      open.getComparisonOperator() = effort_ordered ? getEffortComparisonOperator(): getCostComparisonOperator();
    }

    void addVertices(const std::vector<vPtr> &v){
      for (const auto &a: v){
        //localVertices_.push_back(a);
        localVertices_.push_back(std::make_shared<Vertex>(a->q_, a->id_));
#ifdef USE_ANN
        ann.append(a->q_);
#else
        gnat.add(localVertices_.back());
#endif
      }

    };

    static std::pair<uint, uint> key(const vPtr &l, const vPtr &r){
      if (l->id_ < r->id_){
        return std::make_pair(l->id_, r->id_);
      }
      return std::make_pair(r->id_, l->id_);
    }

    bool couldEdgeBeValid(const vPtr &l, const vPtr &r, const uint param){
      const auto k = key(l, r);
      if (blacklist_.count(k) > 0){
        return false;
      }/*

      if (whitelist_.count(k) > 0){
        return true;
      }*/

      const auto e = getEdgeIfExists(l, r);
      //if (base_->edges_.count(k) > 0){
        //const auto e = getEdge(l, r);
        return couldEdgeBeValid(e, param);
      //}

      //return true;
    }

    bool couldEdgeBeValid(const std::shared_ptr<HierarchicalEdge> &e, const uint param){
      if (e && e->isInvalid(param)){
        blacklist_.insert(key(e->n1_, e->n2_));
        return false;
      }
      return true;
    }

    bool isEdgeKnownValid(const std::shared_ptr<HierarchicalEdge> &e, const uint param){
      if (!e){
        return false;
      }

      if (e->isValid(param)){
        //whitelist_.insert(k);
        return true;
      }

      return false;
    }

    bool isEdgeKnownValid(const vPtr &l, const vPtr &r, const uint param){
      const auto k = key(l, r);
      if (blacklist_.count(k) > 0){
        return false;
      }/*

      if (whitelist_.count(k) > 0){
        return true;
      }*/

      const auto e = getEdgeIfExists(l, r);
      return isEdgeKnownValid(e, param);
    }

    std::vector<vPtr> getNeighbors(const vPtr &v, const uint param) {
      std::vector<vPtr> unfiltered;
      std::chrono::steady_clock::time_point begin_nn = std::chrono::steady_clock::now();

      if (v->nnTag_ != localVertices_.size()){
        // knn
        if (true){ // using nn-structure
          const uint k =
            std::ceil(1.001 * (2.718 + (2.718 / v->q_.d0)) * std::log(1.*localVertices_.size()));

          intA idx;
          if (localVertices_.size() > k){
#ifdef USE_ANN
            ann.getkNN(idx, v->q_, k);
#else
            gnat.nearestK(v, k, unfiltered);
#endif
          }
          else{
#ifdef USE_ANN
            ann.getkNN(idx, v->q_, localVertices_.size());
#else
            gnat.nearestK(v, localVertices_.size(), unfiltered);
#endif
          }

#ifdef USE_ANN
          unfiltered.reserve(idx.d0);
          for (uint i=0; i<idx.d0; ++i){
            const auto n = localVertices_[idx(i)];

            if (n == v) {continue;}

            unfiltered.push_back(n);
          }
#endif
        }
        else{ // radius based
          for (auto n: localVertices_){
            if (n == v) {continue;}

            //std::cout << length(n->q_ - v->q_) << std::endl;
            if (euclideanDistance(n->q_, v->q_) > 2){
              continue;
            }

            unfiltered.push_back(n);
          }
        }

        /*for (const auto n: unfiltered){
          const auto e = getEdgeIfExists(n, v);
          if (e && !couldEdgeBeValid(e, param)) {continue;}
          //if (couldEdgeBeValid(v, n, param)){
            neighbors.push_back(n);
          //}
        }*/
        /*std::vector<vPtr> filtered;
        for (const auto n: unfiltered){
          const auto e = getEdgeIfExists(n, v);
          if (e && !couldEdgeBeValid(e, param)) {continue;}
          //if (couldEdgeBeValid(v, n, param)){
            filtered.push_back(n);
          //}
        }

        unfiltered = filtered;*/
        v->neighbors_ = unfiltered;
        v->nnTag_ = localVertices_.size();
        //std::cout << "not reusing" << std::endl;
      }
      else{
        //std::cout << "reusing" << std::endl;
        unfiltered = v->neighbors_;
      }

      const bool keepWhitelistedNeighbors{false};
      if (keepWhitelistedNeighbors){
        unfiltered.insert(unfiltered.end(), v->whitelisted_.begin(), v->whitelisted_.end());
        // remove the elements that are duplicates
        std::set<vPtr> s( unfiltered.begin(), unfiltered.end() );
        unfiltered.assign( s.begin(), s.end() );
      }

      //std::vector<vPtr> neighbors = unfiltered;
      /*for (const auto n: unfiltered){
        const auto e = getEdgeIfExists(n, v);
        if (e && !couldEdgeBeValid(e, param)) {continue;}
        //if (couldEdgeBeValid(v, n, param)){
          neighbors.push_back(n);
        //}
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

      return v->neighbors_;
    };

    virtual double d_dist(const std::shared_ptr<HierarchicalEdge> &edge, const uint param, const uint numCollChecks) const {
      return d_dist(edge->n1_, edge->n2_);
    }
    virtual double d_dist(const vPtr &l, const vPtr &r) const {
      return euclideanDistance(l->q_, r->q_);
    }
    virtual double h_dist(const vPtr &r, const std::vector<vPtr> &goals) const{
      double dist = 1e6;
      for (auto goal: goals){
        dist = std::min({d_dist(r, goal), dist});
      }
      return dist;
    }

    virtual double d_effort(const std::shared_ptr<HierarchicalEdge> &edge, const uint param, const uint totalCollChecks) const {
      //if (edge){
      if (edge && edge->collCheckLevelRobotEnv_ > 2){
        const uint requiredCollChecksRobotEnv = totalCollChecks - edge->collCheckLevelRobotEnv_;
        const uint requiredCollChecksRobotObs = totalCollChecks - edge->collCheckLevelObsEnv_[param];
        const uint requiredCollChecksObsEnv = totalCollChecks - edge->collCheckLevelRobotObs_[param];

        //std::cout << edge->collCheckLevelRobotEnv_ << " " << edge->collCheckLevelObsEnv_[param] << " " << edge->collCheckLevelRobotObs_[param] << std::endl;
        //std::cout << "A" << edge->collCheckLevelRobotEnv_ <<  std::endl;
        //std::cout << totalCollChecks << " " << requiredCollChecksRobotEnv << " " << requiredCollChecksRobotObs << " " << requiredCollChecksObsEnv << std::endl;

        //constexpr double weightRobotEnv{12./20.};
        //constexpr double weightRobotObs{3/20.};
        //constexpr double weightObsEnv{3./20};

        constexpr double weightRobotEnv{1./3.};
        constexpr double weightRobotObs{1/3.};
        constexpr double weightObsEnv{1./3.};

        //std::cout << "B" << weightRobotEnv * requiredCollChecksRobotEnv << std::endl;

        return weightRobotEnv * requiredCollChecksRobotEnv + 
          weightRobotObs * requiredCollChecksRobotObs + 
          weightObsEnv * requiredCollChecksObsEnv;
      }

      return totalCollChecks;
    }
    virtual double h_effort(const vPtr &r, const vPtr &goal) const{
      /*if (totalCollChecks < hec->robot_calls_){
        return 0u;
      }
      else{
      //const uint totalCollChecks - hec->robot_calls_ + claimedCollChecks;
      }*/
      return 0;
    }

    struct costs{
      double dist;
      double effort;
    };

    virtual bool cmp(const costs& l, const costs& r) const {
      if (effort_ordered_){
        if (l.effort != r.effort){
          return l.effort < r.effort;
        }
        return l.dist < r.dist;
      }
      return l.dist < r.dist;
    }

    //std::unordered_map<uint, costs> g;
    //std::map<vPtr, vPtr> parents;
    std::unordered_map<uint, costs> g;
    std::unordered_map<vPtr, vPtr> parents;

    struct Element{
      vPtr source;
      vPtr target;
      costs f;
      costs edge_cost;
      //costs g;
    };

    std::function<bool(const Element &, const Element &)> getEffortComparisonOperator() const{
      return [](const Element &lhs, const Element &rhs){
        if (lhs.f.effort != rhs.f.effort){
          return lhs.f.effort < rhs.f.effort;
        }
        return lhs.f.dist < rhs.f.dist;
      };
    }

    std::function<bool(const Element &, const Element &)> getCostComparisonOperator() const{
      return [](const Element &lhs, const Element &rhs){
        return lhs.f.dist < rhs.f.dist;
      };
    }


    const std::vector<std::pair<vPtr, vPtr>> expand(const vPtr& v){
      const auto& neighbors = getNeighbors(v, 0);
      std::vector<std::pair<vPtr, vPtr>> expand_buffer;
      expand_buffer.reserve(neighbors.size());
      for (const auto &n: neighbors){
        expand_buffer.emplace_back(v, n);
      }

      return expand_buffer;
    }

    void add_to_queue(const std::vector<std::pair<vPtr, vPtr>>& edges, const uint& param, const std::vector<vPtr>& goals, std::shared_ptr<HierarchicalEdgeCheckerV2> hec=nullptr, const double max_cost = std::numeric_limits<double>::max()){
      for (const auto& edge: edges){
        const auto tmp = getEdgeIfExists(edge.first, edge.second);

        const uint necessaryCollisionChecks = hec->getNumberOfCollisionChecks(edge.first->q_, edge.second->q_);
        if (tmp){
          claimed_effort += (necessaryCollisionChecks - tmp->collCheckLevelRobotEnv_);
        }

        if (tmp && !couldEdgeBeValid(tmp, param)){
          continue;
        }

        if (g.count(edge.second->id_) > 0){
          continue;
        }

        const costs& cost_to_get_to_source = g[edge.first->id_];

        const costs edge_cost_{
          .dist = d_dist(edge.first, edge.second),
          .effort = d_effort(tmp, param, necessaryCollisionChecks),
        };

        //std::cout << base_->cumulative_collision_checks_ << " " << claimed_effort << std::endl;

        uint effort_to_goal = 1e6;
        for (auto goal: goals){
          const uint checks_to_goal = hec->getNumberOfCollisionChecks(edge.second->q_, goal->q_);

          uint local_effort_to_goal;
          const auto goal_edge = getEdgeIfExists(edge.second, goal);
          if (goal_edge){
            local_effort_to_goal = d_effort(goal_edge, param, checks_to_goal);
          }
          else if (checks_to_goal > base_->cumulative_collision_checks_ - claimed_effort - EIRM_FWD_EFFORT){
            if (claimed_effort + EIRM_FWD_EFFORT > base_->cumulative_collision_checks_){
              std::cout << "AAAAAAAAAAAAAAAAAAA" << std::endl;
            }
            local_effort_to_goal = checks_to_goal - (base_->cumulative_collision_checks_ - claimed_effort - EIRM_FWD_EFFORT);
          }
          else{
            local_effort_to_goal = 0;
          }

          if (local_effort_to_goal < effort_to_goal){
            effort_to_goal = local_effort_to_goal;
          }
        }

        const costs target_to_goal = costs{
          .dist = h_dist(edge.second, goals),
          .effort = effort_to_goal,
        };

        const Element elem{
          .source = edge.first,
          .target = edge.second,
          .f = costs{
            .dist = cost_to_get_to_source.dist + edge_cost_.dist + target_to_goal.dist,
            .effort = cost_to_get_to_source.effort + edge_cost_.effort + target_to_goal.effort,
          },
          .edge_cost = edge_cost_,
        };

        if (elem.f.dist > max_cost){
          continue;
        }
#ifdef USE_HEAP
        open.insert(elem);
        //std::cout << open.size() << std::endl;
#else
        open.insert(std::upper_bound(open.begin(), open.end(), elem, [this](const Element& l, const Element& r){
                return cmp(r.f, l.f);
              }), elem);
#endif
      }
    }

#ifdef USE_HEAP
    using Heap =
        ompl::BinaryHeap<Element, std::function<bool(const Element &, const Element &)>>;
    Heap open;
#else
    std::vector<Element> open;
    //std::priority_queue<> open
    //std::set<uint> closed;
#endif

    costs goal_costs = costs{.dist=std::numeric_limits<double>::max(), .effort=std::numeric_limits<double>::max()};

    bool search_running{false};
    uint calls{0u};

    uint claimed_effort = 0;

    std::vector<vPtr> search(const vPtr &start, const std::vector<vPtr> &goals, const uint param, std::shared_ptr<HierarchicalEdgeCheckerV2> hec=nullptr, const double max_cost = std::numeric_limits<double>::max()){
      ++calls;
      if(!search_running){
        parents.clear();
        open.clear();
        g.clear();
        //closed.clear();

        //std::cout << start << " " << goal << std::endl;
        //open.push_back(start);
#ifdef USE_HEAP
        //open.insert(Element{.v = start});
#else
        //open.push_back(Element{.v = start});
#endif
        add_to_queue(expand(start), param, goals, hec, max_cost);

        g[start->id_] = costs{.dist=0, .effort=0};

        goal_costs = costs{.dist=std::numeric_limits<double>::max(), .effort=std::numeric_limits<double>::max()};

        search_running = true;
        claimed_effort = 0;
      }

      // pop top from open
      /*for (const auto elem: open){
        std::cout << elem.f.dist << " " << elem.f.effort << std::endl;
      }
      std::cout << std::endl;
      */
      //std::cout << open.back().f.dist << " " << open.back().f.effort << std::endl;
      if (open.size() == 0){
        search_running = false;
        return {};
      }
#ifdef USE_HEAP
      //vPtr v = open.top();
      //open.pop();
      
      const Element element = open.top()->data;
      const vPtr source = element.source;
      const vPtr target = element.target;

      //std::cout << element.f.dist << " " << element.f.effort << std::endl;

      open.pop();
#else
      //vPtr v = open.back().v;
      //open.pop_back();

      Element element = open.back();
      vPtr source = open.back().source;
      vPtr target = open.back().target;

      open.pop_back();
#endif

      if (element.f.dist > max_cost){
        search_running = false;
        return {};
      }

      if (g.count(target->id_) > 0){
        return {};
      }

      const auto edge = getEdge(source, target);
      if (hec && edge->collCheckLevelRobotEnv_ == 0){
        sparselyValidatedEdge_ += 1;
        const bool res = hec->couldBeValid(*edge, param);

        // this effort can not be claimed by something else
        if (res){
          base_->cumulative_collision_checks_ += 1;
          claimed_effort += 1;
        }
      }
      if (!couldEdgeBeValid(edge, param)) {
        return{};
      }

      parents[target] = source;
      const costs g_source = g[source->id_]; 
      g[target->id_] = costs{
        .dist = g_source.dist + element.edge_cost.dist,
        .effort = g_source.effort + element.edge_cost.effort,
      };

      //closed.insert(v->id_);

      //std::cout << v->id_ << " " << goal->id_ << std::endl;
      //std::cout << "A" << std::endl;

      //if (v == goal){
      for (auto& goal: goals){
        if (target->id_ == goal->id_){
        //if (v->id_ ==  goal->id_){
        //if (length(v->q_ - goal->q_) < 0.01){
          // extract path, return   
          std::vector<vPtr> path;
          auto p = target;
          //auto p = v;
          while(true){
            //std::cout << p->q_ << std::endl;
            path.push_back(p);
            if (p == start){
              // reverse the path, since we started from the goal
              std::reverse(path.begin(), path.end());

              //std::cout << "path" << std::endl;
              search_running = false;
              return path;
            }
            p = parents[p];
          }
        }
      }

      add_to_queue(expand(target), param, goals, hec, max_cost);

      if (open.size() == 0){
        search_running = false;
      }
      //std::cout << "no path found in graph" << std::endl;
      return {};
    };

  //private:
    std::shared_ptr<HierarchicalEdge> getEdgeIfExists(const vPtr &l, const vPtr &r){
      //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      const auto k = key(l, r);
      //std::shared_ptr<HierarchicalEdge> edge;

      const auto it = base_->edges_.find(k);
      if (it != base_->edges_.end()){
        return it->second;
      }
      return nullptr;
      /*if (base_->edges_.count(k) > 0){
        edge = base_->edges_[k];
      }*/
      /*const auto diff = std::chrono::duration_cast<
        std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begin).count();
      std::cout << diff << std::endl;*/
      //return edge;
    }
    std::shared_ptr<HierarchicalEdge> getEdge(const vPtr &l, const vPtr &r){
      //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      const auto k = key(l, r);
      //std::shared_ptr<HierarchicalEdge> edge;

      const auto it = base_->edges_.find(k);
      if (it != base_->edges_.end()){
        return it->second;
      }
      else{
        auto edge = std::make_shared<HierarchicalEdge>(l, r);
        base_->edges_[k] = edge;
        return edge;
      }

      /*if (base_->edges_.count(k) == 0){
        edge = std::make_shared<HierarchicalEdge>(l, r);
        base_->edges_[k] = edge;
      }
      else{
        edge = base_->edges_[k];
      }*/
      /*const auto diff = std::chrono::duration_cast<
        std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begin).count();
      std::cout << diff << std::endl;*/
      //return edge;
    }
    
    double duration_nn{0.};

    ompl::NearestNeighborsGNAT<vPtr> gnat;
    //ompl::NearestNeighborsLinear<vPtr> gnat;
    //ompl::NearestNeighborsSqrtApprox<vPtr> gnat;

    ANN ann;
    std::vector<vPtr> localVertices_;

    struct pair_hash {
      template <class T1, class T2>
      std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        return h1 + 0x9e3779b9 + (h2<<6) + (h2>>2);
      }
    };

    std::unordered_set<std::pair<uint, uint>, pair_hash> whitelist_;
    std::unordered_set<std::pair<uint, uint>, pair_hash> blacklist_;

    //std::set<std::pair<uint, uint>> whitelist_;
    //std::set<std::pair<uint, uint>> blacklist_;

    std::shared_ptr<SharedGraphInformation> base_;

    uint sparselyValidatedEdge_ = 0;
};

class HierarchicalLPRMV2: public ContinuingPlanner{
  public:
    const bool manageGraphSize_;

    double best_cost{std::numeric_limits<double>::max()};
    arr best_path;

    HierarchicalLPRMV2(ConfigurationProblem cp, 
        std::shared_ptr<Problem> p, 
        const uint param, const bool effortOrdered, const bool manageGraphSize,
        std::shared_ptr<SharedGraphInformation> base)
      :ContinuingPlanner(cp),
      parametrization_(param), 
      base_(base), 
      G_(base_, effortOrdered), 
      batchSize_(500), 
      manageGraphSize_(manageGraphSize){

      //hcc = std::make_shared<HierarchicalCollisionCheckerV4>(p, cp.C);
      hec = std::make_shared<HierarchicalEdgeCheckerV2>(p, cp.C);
    };

    virtual void set_resolution(const double resolution){
      hec->resolution_ = resolution;
    }

    /*virtual double getCollCalls(){return hec->hcc.calls_;};
    virtual void resetCollCalls(){
      hec->hcc.calls_ = 0;
    };*/

    typedef std::shared_ptr<Vertex> vPtr;
    std::vector<vPtr> start_tree;
    std::unordered_map<vPtr, vPtr> parents;

    virtual void setup(const arr &q0, const arr &q1){
      //std::cout << q0 << std::endl;
      //std::cout << q1 << std::endl;
      G_.addVertices(base_->startGoalVertices_);
      if (manageGraphSize_){
        G_.addVertices(addSamples(batchSize_));
      }
      else{
        const uint current_num_vertices = base_->vertices_.size();
        G_.addVertices(addSamples(std::max({batchSize_, current_num_vertices})));
      }

      //std::cout << q0 << " " << q1 << std::endl;

      start_ = nullptr;
      goal_ = nullptr;
      for (const auto &v: G_.localVertices_){
        if (!start_ && euclideanDistance(v->q_, q0) < 1e-4){
          start_ = v;
          //std::cout << "using start vertex" << std::endl;
        }
        if (!goal_ && euclideanDistance(v->q_, q1) < 1e-4){
          //std::cout << "using goal vertex" << std::endl;
          goal_ = v;
        }

        if (start_ && goal_){
          break;
        }
      }

      // start and goal vertices are not added to the shared information
      if (start_ == nullptr){
        auto tmp = std::make_shared<Vertex>(q0);
        G_.addVertices({tmp});
        start_ = G_.localVertices_.back();

        base_->startGoalVertices_.push_back(tmp);
      }
      if (goal_ == nullptr){
        auto tmp = std::make_shared<Vertex>(q1);
        G_.addVertices({tmp});
        goal_ = G_.localVertices_.back();

        base_->startGoalVertices_.push_back(tmp);
      }

      start_tree.push_back(start_);
      parents[start_] = nullptr;
      EIRM_FWD_EFFORT = 0u;
    }

    virtual arr plan(const arr &q0, const arr &q1, double time_in_s=-1){
      setup(q0, q1);
      return planMore(time_in_s);
    };
    virtual arr planMore(double time_in_s=-1){
      //std::cout << "LPRM" << std::endl;
      if (duration_nn == 0){
        G_.duration_nn = 0.;
      }

      uint cnt = 0;

      if (false){
        rai::ConfigurationViewer Vf;
        Vf.setConfiguration(hec->hcc->cp.C, "\"Real World\"");

        Points localPoints;

        for (auto &v: G_.localVertices_){
          arr p{v->q_(0), v->q_(1), 1.};
          localPoints.points.push_back(p);
        }

        Vf.add(localPoints);

        Points allPoints;
        allPoints.color = {1., 0., 0.};

        for (auto &v: base_->vertices_){
          arr p{v->q_(0), v->q_(1), 1.};
          allPoints.points.push_back(p);
        }

        Vf.add(allPoints);

        Vf.watch();
      }

      if (false){
        std::cout << G_.getNeighbors(start_, parametrization_).size() << std::endl;
        std::cout << G_.getNeighbors(goal_, parametrization_).size() << std::endl;

        rai::ConfigurationViewer Vf;
        Vf.setConfiguration(cc_->cp.C, "\"Real World\"");

        Lines line;

        for (auto &v: G_.localVertices_){
          auto nei = G_.getNeighbors(v, parametrization_);
          for (auto n: nei){
            //if (!G_.isEdgeKnownValid(v, n, parametrization_)){
            //if (G_.couldEdgeBeValid(v, n, parametrization_)){
            if(G_.getEdge(v, n)->validForRobotEnv_ == EdgeStatus::valid){
            //{
              arr p1{n->q_(0), n->q_(1), 1.};
              arr p2{v->q_(0), v->q_(1), 1.};
              line.pairs.push_back(std::make_pair(p1, p2));
            }
          }
        }

        Vf.add(line);
        Vf.watch();
      }

      std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      while (true){
        //std::cout << G_.localVertices_.size() << std::endl;
        //std::cout << base_->vertices_.size() << std::endl;
        const auto diff = std::chrono::duration_cast<
          std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();

        if (time_in_s > 0 && diff > time_in_s * 1000){
          return best_path;
        }

        if (false){
          std::cout << G_.getNeighbors(start_, parametrization_).size() << std::endl;
          std::cout << G_.getNeighbors(goal_, parametrization_).size() << std::endl;

          rai::ConfigurationViewer Vf;
          Vf.setConfiguration(hec->hcc->cp.C, "\"Real World\"");

          Lines line;

          for (auto &v: G_.localVertices_){
            auto nei = G_.getNeighbors(v, parametrization_);
            for (auto n: nei){
              //if (!G_.isEdgeKnownValid(v, n)){
              //if (G_.couldEdgeBeValid(v, n)){
              //if(G_.getEdge(v, n)->validForRobotEnv_ == EdgeStatus::valid){
              if (G_.getEdge(v, n)->collCheckLevelRobotEnv_ >= 1){
              //{
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
            // std::cout << "exiting before search (" << diff_inner_loop << ")" << std::endl;
            return best_path;
          }

          std::chrono::steady_clock::time_point begin_search = std::chrono::steady_clock::now();

          std::vector<std::shared_ptr<Vertex>> vertexPath;

          vertexPath = G_.search(goal_, start_tree, parametrization_, hec, best_cost);
          if (vertexPath.size() != 0){
            std::reverse(vertexPath.begin(), vertexPath.end());
          }

          while(G_.search_running){
            const auto diff_search_loop = std::chrono::duration_cast<
              std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();

            if (time_in_s > 0 && diff_search_loop > time_in_s * 1000){
              // std::cout << "exiting at search (" << diff_search_loop << ")" << std::endl;
              std::chrono::steady_clock::time_point end_search = std::chrono::steady_clock::now();
              duration_search += std::chrono::duration_cast<std::chrono::microseconds>(end_search - begin_search).count() / (1.*1e6);
              return best_path;
            }
            vertexPath = G_.search(goal_, start_tree, parametrization_, hec, best_cost);
            if (vertexPath.size() != 0){
              std::reverse(vertexPath.begin(), vertexPath.end());
            }
          }

          std::chrono::steady_clock::time_point end_search = std::chrono::steady_clock::now();
          duration_search += std::chrono::duration_cast<std::chrono::microseconds>(end_search - begin_search).count() / (1.*1e6);
          //auto vertexPath = G_.search(start_, goal_);
          //std::cout << "done search" << std::endl;
          if (vertexPath.size() != 0){
            // add the start of the path to it
            {
              auto p = vertexPath[0];
              //std::cout << start_->q_ << std::endl;
              //std::cout << p->q_ << std::endl;
              std::vector<vPtr> path;
              while(parents[p]){
                p = parents[p];
                path.push_back(p);
              }
              std::reverse(path.begin(), path.end());
              path.insert(path.end(), vertexPath.begin(), vertexPath.end());
              vertexPath = path;
            }

            // verify path
            //std::cout << "start feasibility" << std::endl;
            //std::cout << "\t verifying" << std::endl;
            std::chrono::steady_clock::time_point begin_feas = std::chrono::steady_clock::now();

            const bool feasible = verifyPath(vertexPath);

            std::chrono::steady_clock::time_point end_feas = std::chrono::steady_clock::now();
            duration_verifying += std::chrono::duration_cast<std::chrono::microseconds>(end_feas - begin_feas).count() / (1.*1e6);

            //const bool feasible = verifyPathIncrementally(vertexPath);
            //std::cout << "feasibility:" << feasible << std::endl;

            if (feasible){
              arr path(vertexPath.size(), vertexPath[0]->q_.d0);
              for (uint i=0; i<vertexPath.size(); ++i){
                path[i] = vertexPath[i]->q_;
                //std::cout << "P" << vertexPath[i]->id_ << std::endl;
              }
              //std::cout << path.d0 << " " << length(path[0] - path[1]) << " " << length(path[-1] - path[-2]) << std::endl;
              if (verbose_ > 1) std::cout << hec->calls_ << " " << hec->robot_calls_ << " " << hec->obs_calls_ << " " << hec->env_calls_ << std::endl;
              //if (verbose_ > 1) std::cout << path.d0 << " " << free_edge_counter << std::endl;
              std::cout << path.d0 << " " << free_edge_counter << std::endl;
              if (verbose_ > 1) std::cout << G_.sparselyValidatedEdge_ << std::endl;
              //std::cout << hec->calls_ << std::endl;

              const auto diff = std::chrono::duration_cast<
                std::chrono::microseconds>(std::chrono::steady_clock::now() - begin).count() / (1.*1e6);
              
              const double pathlength = get_path_length(path);
              if (pathlength < best_cost){
                best_path = path;
                best_cost = pathlength;

                costs.push_back(std::make_pair(diff, pathlength));
              }

              G_.set_queue_order(false);
              start_tree.clear();
              start_tree.push_back(start_);

              if (stop_at_first_solution){
                return path;
              }

              break;
            }
          }
          else{break;}
        }

        duration_nn = G_.duration_nn;

        // add batch of samples
        //std::cout << "\t\t\tadding samples in param " << parametrization_ << std::endl;
        G_.addVertices(addSamples(batchSize_));

        // std::cout << G_.localVertices_.size() << std::endl;
        // std::cout << G_.base_->vertices_.size() << " " << G_.base_->edges_.size() << std::endl;

        ++cnt;

        //if (cnt == 1){return{};}
      }
    };

  //private:
    uint free_edge_counter{0};

    bool verifyPath(const std::vector<std::shared_ptr<Vertex>> &vertexPath) {
      free_edge_counter = 0u;

      std::vector<uint> order(vertexPath.size()-1);
      std::iota(std::begin(order), std::end(order), 1);

      for (const auto i: order){
        const auto l = vertexPath[i-1];
        const auto r = vertexPath[i];
        //std::cout << "A " << l->id_ << " " << r->id_ << std::endl;
        if (false){
          //std::cout << G_.getNeighbors(start_, parametrization_).size() << std::endl;
          //std::cout << G_.getNeighbors(goal_, parametrization_).size() << std::endl;

          rai::ConfigurationViewer Vf;
          Vf.setConfiguration(hec->hcc->cp.C, "\"Real World\"");

          Lines line2;
          line2.color(0) = 1;

          for (auto &v: G_.localVertices_){
            auto nei = G_.getNeighbors(v, parametrization_);
            for (auto n: nei){
              //if (!G_.isEdgeKnownValid(v, n, parametrization_)){
              //if (G_.couldEdgeBeValid(v, n, parametrization_)){
              //if(G_.getEdge(v, n)->validForRobotEnv_ == EdgeStatus::valid){
              {
                arr p1{n->q_(0), n->q_(1), 0.9};
                arr p2{v->q_(0), v->q_(1), 0.9};
                line2.pairs.push_back(std::make_pair(p1, p2));
                //std::cout << n->id_ << " " << v->id_ <<std::endl;
              }
            }
          }

          Vf.add(line2);

          Lines line3;

          for (auto &v: G_.localVertices_){
            auto nei = G_.getNeighbors(v, parametrization_);
            for (auto n: nei){
              //if (!G_.isEdgeKnownValid(v, n, parametrization_)){
              //if (G_.couldEdgeBeValid(v, n, parametrization_)){
              if(G_.getEdge(v, n)->validForRobotEnv_ == EdgeStatus::valid){
              //{
                arr p1{n->q_(0), n->q_(1), 1.1};
                arr p2{v->q_(0), v->q_(1), 1.1};
                line3.pairs.push_back(std::make_pair(p1, p2));
                //std::cout << n->id_ << " " << v->id_ <<std::endl;
              }
            }
          }

          Vf.add(line3);
          Lines line;
          line.color(0) = 0.5;
          line.color(1) = 0.5;
          line.color(2) = 0;

          for (auto &v: G_.localVertices_){
            auto nei = G_.getNeighbors(v, parametrization_);
            for (auto n: nei){
              //if (!G_.isEdgeKnownValid(v, n, parametrization_)){
              if (G_.getEdge(v, n)->collCheckLevelRobotEnv_ >= 1){
              //if(G_.getEdge(v, n)->validForRobotEnv_ == EdgeStatus::valid){
              //{
                arr p1{n->q_(0), n->q_(1), 1.};
                arr p2{v->q_(0), v->q_(1), 1.};
                line.pairs.push_back(std::make_pair(p1, p2));
                //std::cout << n->id_ << " " << v->id_ <<std::endl;
              }
            }
          }

          Vf.add(line);

          Vf.watch();
        }

        //hec->hcc->cp.C.setJointState(l->q_);
        //hec->hcc->cp.C.watch(true);

        const uint prev_calls = hec->robot_calls_;
        const auto e = G_.getEdge(l, r);
        //std::cout << hec->calls_ << " " << hec->robot_calls_ << " " << hec->obs_calls_ << " " << hec->env_calls_ << std::endl;
        if(!G_.isEdgeKnownValid(e, parametrization_)){ // only check if we didnt mark yet
          //std::cout << e->collCheckLevelRobotEnv_ << std::endl;
          //std::cout << "QQQQQQQQQQQQQQQQQQQQQ" << std::endl;
          if (!hec->isValid(*e, parametrization_)){
            //base_->cumulative_collision_checks_ += hec->robot_calls_ - prev_calls;
            //std::cout << "i: " << i << std::endl;
            //std::cout << l->q_ << std::endl << r->q_ << std::endl;
            return false;
          }
          start_tree.push_back(r);
          parents[r] = l;
          base_->cumulative_collision_checks_ += hec->robot_calls_ - prev_calls;
          EIRM_FWD_EFFORT += hec->robot_calls_ - prev_calls;
        }
        else{
          //free_edge_counter += 1;
          //std::cout << "already valid" << std::endl;
        }
        //std::cout << hec->calls_ << " " << hec->robot_calls_ << " " << hec->obs_calls_ << " " << hec->env_calls_ << std::endl;

        if (prev_calls == hec->robot_calls_ && e->annotatedValidAt != parametrization_){
          free_edge_counter += 1;
        }
      }

      return true;
    }

    std::vector<std::shared_ptr<Vertex>> addSamples(const uint n){
      std::chrono::steady_clock::time_point begin_sampling = std::chrono::steady_clock::now();

      std::vector<std::shared_ptr<Vertex>> vertices;
      uint newSamples{0u};
      while (vertices.size() < n){
        //bool isnew = false;
        if (baseBufferIndex_ >= base_->vertices_.size()){
          //isnew = true;
          while(true){
            const arr q = hec->hcc->cp.sample();
            if (hec->hcc->isValidForRobotEnv(q)){
              //hec->hcc->cp.C.watch(true);
              auto v = std::make_shared<Vertex>(q);
              base_->vertices_.push_back(v);
              newSamples++;
              break;
            }
          }
        }
        else{
          //std::cout << "reusin sample" << std::endl;
        }

        const auto v = base_->vertices_[baseBufferIndex_];
        bool can_improve_cost = true;
        if (start_ && goal_) {
          if (v->dist_to_goal < 0){
            v->dist_to_goal = euclideanDistance(goal_->q_, v->q_);
          }
          const double dist_to_goal = v->dist_to_goal;
          can_improve_cost = (euclideanDistance(start_->q_, v->q_) + dist_to_goal) < best_cost;
        }
        if (can_improve_cost && hec->hcc->isValid(v->q_, false, true, true)){
        //if (hcc->isValidForRobotObs(v->q_) && hcc->isValidForObsEnv(v->q_)){
          vertices.push_back(v);
          //if (isnew) std::cout << "A" << std::endl;
          //else{std::cout << "B" << std::endl;}
        }

        baseBufferIndex_++;
      }

      // std::cout << "\t\t\t" << base_->vertices_.size() << " " << newSamples << std::endl;

      std::chrono::steady_clock::time_point end_sampling = std::chrono::steady_clock::now();
      duration_sampling += std::chrono::duration_cast<std::chrono::microseconds>(end_sampling - begin_sampling).count() / (1.*1e6);
      return vertices;
    };

    std::shared_ptr<Vertex> start_;
    std::shared_ptr<Vertex> goal_;

    const uint parametrization_;

    const uint batchSize_{100u};

    uint baseBufferIndex_{0u};
    std::shared_ptr<SharedGraphInformation> base_;

    HierarchicalGraphV2 G_;
    //std::shared_ptr<HierarchicalCollisionCheckerV4> hcc;
    std::shared_ptr<HierarchicalEdgeCheckerV2> hec;
};
#endif
