#include "rrt_planner.h"

#include <cassert>
#include <limits>

namespace rrt {

RrtPlanner::RrtPlanner(SpacePtr space)
    : space_(space),
      tree_(new Rrt(space, std::make_shared<LinearSearchNN>())),
      max_samples_(-1) {}

bool RrtPlanner::solve(const NodePtr& init, const NodePtr& goal) {
  tree_->initialize(init);

  int i = 0;
  while (true) {
    if (max_samples_ >= 0 && i >= max_samples_) {
      break;
    }
    ++i;
    NodePtr x_random = space_->sample();
    NodePtr x_new;
    Rrt::Status status;
    if (use_connect_) {
      status = tree_->connect(x_random, x_new);
    } else {
      status = tree_->extend(x_random, x_new);
    }

    if (x_new) {
      auto distance = x_new->distance(*goal);
      if (distance <= goal_tolerance_) {
        nodes_.clear();
        nodes_ = tree_->nodes();

        solution_.clear();
        NodePtr node = x_new;
        while (node) {
          solution_.push_back(node);
          node = node->parent();
        }
        return true;
      }
    }
  }
  return false;
}

void RrtPlanner::reset() {
  tree_->reset();
  nodes_.clear();
  solution_.clear();
}

}  // namespace rrt
