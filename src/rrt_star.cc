#include "rrt_star.h"

#include <cassert>
#include <iostream>
#include <random>

#include "space_2d.h"

namespace rrt {

RrtStar::RrtStar(SpacePtr space, NearestNeighborPtr nn)
    : nn_(nn), space_(space) {}

void RrtStar::initialize(const NodePtr& x) {
  nodes_.push_back(x);
  nn_->add(x);
}

/// [1] S. Karaman and E. Frazzoli, “Sampling-based Algorithms for Optimal
/// Motion Planning.”
RrtStar::Status RrtStar::extend(const NodePtr& x_sample, NodePtr& x_new_out) {
  assert(!nodes_.empty());
  auto x_nearest = nn_->find_nearest(x_sample);
  auto x_new = space_->steer(x_nearest, x_sample);
  if (x_new) {
    // if (!space_->collision_free(x_new)) {
    //  return Status::kCollided;
    //}
    if (!space_->collision_free_path(x_nearest, x_new)) {
      return Status::kCollided;
    }

    double n = nodes_.size();
    double d = space_->num_dimensions();
    double radius = gamma_ * std::pow(std::log(n) / n, 1.0 / d);
    // auto x_near_vec = nn_->radius_search(x_new, radius);
    auto x_near_vec = nn_->nearest_neighbors(x_new, 20);

    std::cout << "radius = " << radius << ", near nodes = " << x_near_vec.size()
              << std::endl;

    // Select parent
    double min_cost = x_new->cost();
    int i = 0;
    int min_index = -1;
    for (auto& x_near : x_near_vec) {
      double cost = x_new->distance(*x_near.first) + x_near.first->cost();
      std::cout << "  cost = " << cost << " min_cost = " << min_cost
                << "  x_new = (" << std::static_pointer_cast<Motion2D>(x_new)->x
                << ", " << std::static_pointer_cast<Motion2D>(x_new)->y
                << ")  x_near = ("
                << std::static_pointer_cast<Motion2D>(x_near.first)->x << ", "
                << std::static_pointer_cast<Motion2D>(x_near.first)->y << ")"
                << std::endl;
      if (cost < min_cost && space_->collision_free_path(x_new, x_near.first)) {
        std::cout << "  NEW PARENT FOUND" << std::endl;
        x_new->set_parent(x_near.first);
        min_cost = cost;
        min_index = i;
      }
      ++i;
    }
    if (min_index >= 0) {
      x_near_vec.erase(begin(x_near_vec) + i);
    }

    // Reconnect neighbors
    for (auto& x_near : x_near_vec) {
      double current_cost = x_near.first->cost();
      double cost = x_near.first->distance(*x_new) + x_new->cost();
      std::cout << "  cost = " << cost << " current_cost = " << current_cost
                << std::endl;
      if (cost < current_cost &&
          space_->collision_free_path(x_near.first, x_new)) {
        std::cout << "  RECONNECT" << std::endl;
        x_near.first->set_parent(x_new);
      }
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

/// [1] B. Akgun and M. Stilman, “Sampling Heuristics for Optimal Motion
/// Planning in High Dimensions,”
///     in IEEE/RSJ International Conference on Intelligent Robots and Systems,
///     2011.
RrtStar::Status RrtStar::connect(const NodePtr& x_sample, NodePtr& x_new_out) {
  while (true) {
    Status status = extend(x_sample, x_new_out);
    if (status != Status::kAdvanced) {
      return status;
    }
  }
}

void RrtStar::reset() {
  nodes_.clear();
  nn_->reset();
}

}  // namespace rrt
