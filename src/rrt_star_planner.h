#pragma once

#include "planner.h"
#include "rrt_star.h"
#include "nearest_neighbor.h"

namespace rrt {

class RrtStarPlanner : public PlannerInterface {
 public:
  RrtStarPlanner(SpacePtr space);

  virtual ~RrtStarPlanner() {}

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

	SpacePtr space() const override { return space_; }

  void set_gamma(double value) { tree_->set_gamma(value); }

  void set_max_samples(int value) { max_samples_ = value; }
 private:
	SpacePtr space_;
  std::unique_ptr<RrtStar> tree_;
  std::vector<NodePtr> solution_;
  std::vector<NodePtr> nodes_;
  int max_samples_{-1};
  double goal_tolerance_{0.0};
  bool use_connect_{false};
};

}  // namespace rrt
