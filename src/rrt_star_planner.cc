#include "rrt_star_planner.h"

#include <cassert>
#include <limits>

namespace rrt {

RrtStarPlanner::RrtStarPlanner(SpacePtr space)
    : space_(space),
      tree_(new RrtStar(space, std::make_shared<LinearSearchNN>())) {}

bool RrtStarPlanner::solve(const NodePtr& init, const NodePtr& goal) {
  tree_->initialize(init);

  int i = 0;
  while (true) {
    if (max_samples_ >= 0 && i >= max_samples_) {
      break;
    }
    ++i;
    NodePtr x_random = space_->sample();
    NodePtr x_new;
    (void)tree_->extend(x_random, x_new);

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
  nodes_.clear();
  nodes_ = tree_->nodes();
  return false;
}

void RrtStarPlanner::reset() {
  tree_->reset();
  nodes_.clear();
  solution_.clear();
}

}  // namespace rrt
