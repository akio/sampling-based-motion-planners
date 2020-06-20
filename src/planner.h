#pragma once

#include <cmath>
#include <vector>
#include <memory>
#include <string>

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

}  // namespace rrt


