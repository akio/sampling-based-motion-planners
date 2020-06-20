#pragma once

#include <cassert>
#include <cmath>
#include <vector>
#include <memory> #include <string>

#include "space.h"

namespace rrt {

class Motion2D : public NodeInterface {
 public:
	double x;
	double y;
	double u;

  double distance(const NodeInterface& node) const override {
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

class Box2D : public CollisionInterface {
 public:
  Box2D(double x0, double y0, double width, double height);

  bool collide(const NodePtr& node) const override;

  double x0;
  double y0;
  double width;
  double height;
};


class Space2D : public SpaceInterface {
 public:
	Space2D(double xmin, double xmax, double ymin, double ymax, double control);

	NodePtr sample() const override;

	NodePtr steer(NodePtr x0, NodePtr x1) const override;

  bool has_collision(NodePtr node) const override;

  std::vector<CollisionPtr> collisions() const override {
    return collisions_;
  }

  void add_collision_box(double x0, double y0, double width, double height);

 private:
  double xmin_{0.0};
  double xmax_{0.0};
  double ymin_{0.0};
  double ymax_{0.0};
	double control_{0.0};
  std::vector<CollisionPtr> collisions_;
};

}  // namespace rrt
