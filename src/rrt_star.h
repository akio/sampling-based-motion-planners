#pragma once

#include <cmath>
#include <vector>
#include <memory>
#include <string>

#include "space.h"
#include "nearest_neighbor.h"

namespace rrt {

class RrtStar {
 public:
  enum class Status {
    kTrapped,
    kCollided,
    kReached,
    kAdvanced,
  };

  RrtStar(SpacePtr space, NearestNeighborPtr nn);

  RrtStar(const RrtStar&) = delete;
  RrtStar& operator=(const RrtStar&) = delete;

  void initialize(const NodePtr& x);

  Status extend(const NodePtr& x_sample, NodePtr& x_new_out);

  Status connect(const NodePtr& x_sample, NodePtr& x_new_out);

  std::vector<NodePtr> nodes() const { return nodes_; }

  double gamma() const { return gamma_; }
  void set_gamma(double value) { gamma_ = value; }

  void reset();
 private:

  std::vector<NodePtr> nodes_;
	NearestNeighborPtr nn_;
	SpacePtr space_;
  double gamma_ = 1.0;
};

}  // namespace rrt

