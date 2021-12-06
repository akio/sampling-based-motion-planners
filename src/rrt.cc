#include "rrt.h"

#include <cassert>
#include <iostream>
#include <random>

namespace rrt {

Rrt::Rrt(SpacePtr space, NearestNeighborPtr nn) : nn_(nn), space_(space) {}

void Rrt::initialize(const NodePtr& x) {
  nodes_.push_back(x);
  nn_->add(x);
}

Rrt::Status Rrt::extend(const NodePtr& x_sample, NodePtr& x_new_out) {
  assert(!nodes_.empty());
  auto x_near = nn_->find_nearest(x_sample);
  auto x_new = space_->steer(x_near, x_sample);
  if (x_new) {
    if (!space_->collision_free(x_new)) {
      return Status::kCollided;
    }
    nn_->add(x_new);
    nodes_.push_back(x_new);
    x_new_out = x_new;
    if (x_sample->distance(*x_new) <= std::numeric_limits<double>::epsilon()) {
      return Status::kReached;
    } else {
      return Status::kAdvanced;
    }
  }
  return Status::kTrapped;
}

Rrt::Status Rrt::connect(const NodePtr& x_sample, NodePtr& x_new_out) {
  while (true) {
    Status status = extend(x_sample, x_new_out);
    if (status != Status::kAdvanced) {
      return status;
    }
  }
}

void Rrt::reset() {
  nodes_.clear();
  nn_->reset();
}

}  // namespace rrt
