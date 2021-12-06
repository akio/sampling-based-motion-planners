#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nearest_neighbor.h"
#include "space.h"

namespace rrt {

class Rrt {
 public:
  enum class Status {
    kTrapped,
    kCollided,
    kReached,
    kAdvanced,
  };

  Rrt(SpacePtr space, NearestNeighborPtr nn);

  Rrt(const Rrt&) = delete;
  Rrt& operator=(const Rrt&) = delete;

  void initialize(const NodePtr& x);

  Status extend(const NodePtr& x_sample, NodePtr& x_new_out);

  Status connect(const NodePtr& x_sample, NodePtr& x_new_out);

  std::vector<NodePtr> nodes() const { return nodes_; }

  double gamma() const { return gamma_; }
  void set_gamma(double value) { gamma_ = value; }

  void reset();

 private:
  double card();

  std::vector<NodePtr> nodes_;
  NearestNeighborPtr nn_;
  SpacePtr space_;
  double gamma_ = 1.0;
};

}  // namespace rrt
