#include <cassert>
#include <limits>
#include <vector>

#include "nearest_neighbor.h"
#include "planner.h"
#include "rrt_star.h"

namespace rrt {

class BiRrtStarPlanner : public PlannerInterface {
 public:
  BiRrtStarPlanner(SpacePtr space);

  virtual ~BiRrtStarPlanner() {}

  bool solve(const NodePtr& init, const NodePtr& goal);

  void reset() override;

  const std::vector<NodePtr>& nodes() const { return nodes_; }

  const std::vector<NodePtr>& solution() const override { return solution_; }

  double goal_tolerance() const { return goal_tolerance_; }

  void set_goal_tolerance(double value) { goal_tolerance_ = value; }

  SpacePtr space() const override { return space_; }

  void set_gamma(double value) {
    init_tree_->set_gamma(value);
    goal_tree_->set_gamma(value);
  }

 private:
  SpacePtr space_;
  std::unique_ptr<RrtStar> init_tree_;
  std::unique_ptr<RrtStar> goal_tree_;
  std::vector<NodePtr> nodes_;
  std::vector<NodePtr> solution_;
  int max_samples_;
  double goal_tolerance_{std::numeric_limits<double>::epsilon()};
};

}  // namespace rrt
