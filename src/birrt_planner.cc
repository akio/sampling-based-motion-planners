#include "birrt_planner.h"

#include <cassert>
#include <limits>

namespace rrt {

BidirectionalRrtPlanner::BidirectionalRrtPlanner(SpacePtr space)
    : space_(space),
      init_tree_(new Rrt(space, std::make_shared<LinearSearchNN>())),
      goal_tree_(new Rrt(space, std::make_shared<LinearSearchNN>())),
      max_samples_(-1) {
}

bool BidirectionalRrtPlanner::solve(const NodePtr& init, const NodePtr& goal) {
  init_tree_->initialize(init);
  goal_tree_->initialize(goal);

  int i = 0;
  while (true) {
    if (max_samples_ >= 0 && i >= max_samples_) {
      break;
    }
    ++i;
		NodePtr x_rand = space_->sample();
    NodePtr x_new;
    auto status = init_tree_->extend(x_rand, x_new);
    if (status != Rrt::Status::kTrapped && status != Rrt::Status::kCollided) {
      NodePtr x_new2;
      if (goal_tree_->connect(x_new, x_new2) == Rrt::Status::kReached) {
        auto init_nodes = init_tree_->nodes();
        auto goal_nodes = goal_tree_->nodes();
        nodes_.clear();
        nodes_.reserve(init_nodes.size() + goal_nodes.size());
        nodes_.insert(end(nodes_), begin(init_nodes), end(init_nodes));
        nodes_.insert(end(nodes_), begin(goal_nodes), end(goal_nodes));

        solution_.clear();
        NodePtr node = x_new;
        while (node) {
          solution_.push_back(node);
          node = node->parent();
        }
        node = x_new2;
        while (node) {
          solution_.push_back(node);
          node = node->parent();
        }
        return true;
      }
    }
    std::swap(init_tree_, goal_tree_);
  }
  return false;
}

void BidirectionalRrtPlanner::reset() {
  init_tree_->reset();
  goal_tree_->reset();
  nodes_.clear();
}

}  // namespace rrt
