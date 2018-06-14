#ifndef RRT_H_
#define RRT_H_

#include <vector>
#include <memory>
#include <string>

namespace rrt {

class NodeInterface {
 public:
  virtual double Distance(const NodeInterface& node) const = 0;
	virtual std::shared_ptr<NodeInterface> parent() const = 0;
	virtual void set_parent(std::shared_ptr<NodeInterface> parent) = 0;
};

typedef std::shared_ptr<NodeInterface> NodePtr;

class Motion2D : public NodeInterface {
 public:
	double x;
	double y;
	double u;

  double Distance(const NodeInterface& node) const override {
		auto m = dynamic_cast<const Motion2D&>(node);
		double dx = m.x - x;
		double dy = m.y - y;
		return std::sqrt(dx * dx + dy * dy);
	}

	NodePtr parent() const override {
		return parent_;
	}

	void set_parent(NodePtr parent) override {
		parent_ = parent;
	}

 private:
	NodePtr parent_;
};

class CollisionInterface {
 public:
  virtual bool Collide(const NodePtr& node) const = 0;
};

typedef std::shared_ptr<CollisionInterface> CollisionPtr;

class Box2D : public CollisionInterface {
 public:
  Box2D(double x0, double y0, double width, double height);

  bool Collide(const NodePtr& node) const override;

  double x0;
  double y0;
  double width;
  double height;
};

class SpaceInterface {
 public:
	virtual NodePtr Sample() const = 0;
	virtual NodePtr NewNode(NodePtr origin, NodePtr target) const = 0;
  virtual std::vector<CollisionPtr> collisions() const = 0;
};

typedef std::shared_ptr<SpaceInterface> SpacePtr;

class Space2D : public SpaceInterface {
 public:
	Space2D(double xmin, double xmax, double ymin, double ymax, double control);
	NodePtr Sample() const override;
	NodePtr NewNode(NodePtr x0, NodePtr x1) const override;
  std::vector<CollisionPtr> collisions() const override {
    return collisions_;
  }

  void AddCollisionBox(double x0, double y0, double width, double height);

 private:
  double xmin_{0.0};
  double xmax_{0.0};
  double ymin_{0.0};
  double ymax_{0.0};
	double control_{0.0};
  std::vector<CollisionPtr> collisions_;
};

class NearestNeighborInterface {
 public:
  virtual void Add(NodePtr node) = 0;
	virtual NodePtr Query(NodePtr motion) const = 0;
};

typedef std::shared_ptr<NearestNeighborInterface> NearestNeighborPtr;

class LinearSearchNN : public NearestNeighborInterface {
 public:
	LinearSearchNN();
  void Add(NodePtr node) override;
	NodePtr Query(NodePtr node) const override;
 private:
	std::vector<NodePtr> nodes_;
};


class PlannerInterface {
 public:
};

enum class RrtStatus {
  kTrapped,
  kReached,
  kAdvanced,
};

class Rrt {
 public:

  Rrt(SpacePtr space);

  bool Solve(const NodePtr& init, const NodePtr& goal);

  RrtStatus Extend(const NodePtr& x_sample, NodePtr& out_x_new);

  RrtStatus Connect(const NodePtr& x_sample, NodePtr& out_x_new);

  void Clear();

	const std::vector<NodePtr>& nodes() const {
		return nodes_;
	}

  double goal_tolerance() const {
    return goal_tolerance_;
  }

  void set_goal_tolerance(double value) {
    goal_tolerance_ = value;
  }

  bool use_connect() const { return use_connect_; }

  void set_use_connect(bool value) { use_connect_ = value; }

	SpacePtr space() const { return space_; }
 private:
  std::vector<NodePtr> nodes_;
	NearestNeighborPtr nn_;
	SpacePtr space_;
  int max_samples_;
  double goal_tolerance_{0.0};
  bool use_connect_{false};
};

class BidirectionalRrt {
 public:
  BidirectionalRrt(SpacePtr space);

  bool Solve(const NodePtr& init, const NodePtr& goal);

  RrtStatus Extend(const NodePtr& x_sample, NodePtr& out_x_new);

  RrtStatus Connect(const NodePtr& x_sample, NodePtr& out_x_new);

  void Clear();

	const std::vector<NodePtr>& nodes() const {
    std::vector<NodePtr> nodes;
    nodes.reserve(init_nodes_.size() + goal_nodes_.size());
    nodes.insert(end(nodes), begin(init_nodes_), end(init_nodes_));
    nodes.insert(end(nodes), begin(goal_nodes_), end(goal_nodes_));
		return nodes;
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

 private:
  std::vector<NodePtr> init_nodes_;
  std::vector<NodePtr> goal_nodes_;
	NearestNeighborPtr init_nn_;
	NearestNeighborPtr goal_nn_;
	SpacePtr space_;
  int max_samples_;
  double goal_tolerance_{0.0};
  bool use_connect1_{false};
  bool use_connect2_{false};
};

}  // namespace rrt

#endif/*RRT_H_*/
