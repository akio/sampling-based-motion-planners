#include "planner.h"

#include <cassert>
#include <vector>

#include "rrt.h"
#include "planner.h"
#include "nearest_neighbor.h"

namespace rrt {

class BidirectionalRrtPlanner : public PlannerInterface {
 public:
  BidirectionalRrtPlanner(SpacePtr space);

  virtual ~BidirectionalRrtPlanner() {}

  bool solve(const NodePtr& init, const NodePtr& goal);

  void reset() override;

	const std::vector<NodePtr>& nodes() const {
		return nodes_;
	}

	const std::vector<NodePtr>& solution() const override {
		return solution_;
	}

  double goal_tolerance() const {
    return goal_tolerance_;
  }

  void set_goal_tolerance(double value) {
    goal_tolerance_ = value;
  }

  bool use_connect1() const { return use_connect1_; }
  void set_use_connect1(bool value) { use_connect1_ = value; }

  bool use_connect2() const { return use_connect2_; }
  void set_use_connect2(bool value) { use_connect2_ = value; }

	SpacePtr space() const override { return space_; }
 private:
	SpacePtr space_;
  std::unique_ptr<Rrt> init_tree_;
  std::unique_ptr<Rrt> goal_tree_;
  std::vector<NodePtr> nodes_;
  std::vector<NodePtr> solution_;
  int max_samples_;
  double goal_tolerance_{0.0};
  bool use_connect1_{false};
  bool use_connect2_{false};
};

}  // namespace rrt
