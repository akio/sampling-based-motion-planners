#pragma once

#include <cmath>
#include <vector>
#include <memory>
#include <string>

#include "space.h"
#include "nearest_neighbor.h"

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

  Status extend(const NodePtr& x_sample, NodePtr& out_x_new);

  Status connect(const NodePtr& x_sample, NodePtr& out_x_new);

  std::vector<NodePtr> nodes() const { return nodes_; }

  void reset();
 private:
  std::vector<NodePtr> nodes_;
	NearestNeighborPtr nn_;
	SpacePtr space_;
};

}  // namespace rrt

