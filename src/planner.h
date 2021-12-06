#pragma once

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "space.h"

namespace rrt {

class PlannerInterface {
 public:
  PlannerInterface() = default;

  virtual ~PlannerInterface() = default;

  PlannerInterface(const PlannerInterface&) = delete;

  PlannerInterface& operator=(const PlannerInterface&) = delete;

  virtual bool solve(const NodePtr& init, const NodePtr& goal) = 0;

  virtual void reset() = 0;

  virtual const std::vector<NodePtr>& nodes() const = 0;

  virtual const std::vector<NodePtr>& solution() const = 0;

  virtual SpacePtr space() const = 0;

  virtual double goal_tolerance() const = 0;

  virtual void set_goal_tolerance(double value) = 0;
};

inline std::vector<NodePtr> path_pruning(SpacePtr space,
                                         std::vector<NodePtr> path) {
  std::vector<NodePtr> new_path;
  for (int i = 0; i < path.size(); ++i) {
    auto new_node = path[i]->clone();
    if (i == 0) {
      new_node->set_parent({});
    } else {
      new_node->set_parent(new_path.back());
    }
    new_path.push_back(new_node);
  }
  if (new_path.size() >= 3) {
    int i = 0;
    while (i + 2 < new_path.size()) {
      if (space->collision_free_path(new_path[i], new_path[i + 2])) {
        new_path[i + 2]->set_parent(new_path[i]);
        new_path.erase(begin(new_path) + i + 1);
      } else {
        ++i;
      }
    }
  }
  return new_path;
}

}  // namespace rrt
