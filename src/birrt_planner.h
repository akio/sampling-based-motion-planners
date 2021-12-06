#include <cassert>
#include <vector>

#include "nearest_neighbor.h"
#include "planner.h"
#include "rrt.h"

namespace rrt {

class BidirectionalRrtPlanner : public PlannerInterface {
 public:
  BidirectionalRrtPlanner(SpacePtr space);

  virtual ~BidirectionalRrtPlanner() {}

  bool solve(const NodePtr& init, const NodePtr& goal);

  void reset() override;

  const std::vector<NodePtr>& nodes() const { return nodes_; }

  const std::vector<NodePtr>& solution() const override { return solution_; }

  double goal_tolerance() const { return goal_tolerance_; }

  void set_goal_tolerance(double value) { goal_tolerance_ = value; }

  SpacePtr space() const override { return space_; }

 private:
  SpacePtr space_;
  std::unique_ptr<Rrt> init_tree_;
  std::unique_ptr<Rrt> goal_tree_;
  std::vector<NodePtr> nodes_;
  std::vector<NodePtr> solution_;
  int max_samples_;
  double goal_tolerance_{0.0};
};

}  // namespace rrt
