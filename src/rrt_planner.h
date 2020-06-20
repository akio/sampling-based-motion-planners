#pragma once

#include "planner.h"
#include "rrt.h"
#include "nearest_neighbor.h"

namespace rrt {

class RrtPlanner : public PlannerInterface {
 public:
  RrtPlanner(SpacePtr space);

  virtual ~RrtPlanner() {}

  bool solve(const NodePtr& init, const NodePtr& goal) override;

  void reset() override;

	const std::vector<NodePtr>& nodes() const override {
		return nodes_;
	}

	const std::vector<NodePtr>& solution() const override {
		return solution_;
	}

  double goal_tolerance() const override {
    return goal_tolerance_;
  }

  void set_goal_tolerance(double value) override {
    goal_tolerance_ = value;
  }

  bool use_connect() const { return use_connect_; }

  void set_use_connect(bool value) { use_connect_ = value; }

	SpacePtr space() const override { return space_; }
 private:
	SpacePtr space_;
  std::unique_ptr<Rrt> tree_;
  std::vector<NodePtr> solution_;
  std::vector<NodePtr> nodes_;
  int max_samples_;
  double goal_tolerance_{0.0};
  bool use_connect_{false};
};

}  // namespace rrt
