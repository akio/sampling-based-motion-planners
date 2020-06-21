#pragma once

#include <cassert>
#include <cmath>
#include <vector>
#include <memory>
#include <string>
#include <random>

#include "space.h"

namespace rrt {

class Motion2D : public NodeInterface {
 public:
	double x;
	double y;
	double u;
  Motion2D() : x(0), y(0), u(0) {}

  Motion2D(const Motion2D&) = delete;
  Motion2D& operator=(const Motion2D&) = delete;


  double distance(const NodeInterface& node) const override {
		auto& other = static_cast<const Motion2D&>(node);
		double dx = other.x - x;
		double dy = other.y - y;
		return std::hypot(dx, dy);
	}

  NodePtr midpoint(const NodeInterface& node) const override {
    auto& other = static_cast<const Motion2D&>(node);
    auto mid = std::make_shared<Motion2D>();
    mid->x = 0.5 * (x + other.x);
    mid->y = 0.5 * (y + other.y);
    mid->u = 0;
    return std::static_pointer_cast<NodeInterface>(mid);
  }

	NodePtr parent() const override {
		return parent_;
	}

	void set_parent(NodePtr parent) override {
		parent_ = parent;
    if (parent_) {
      cost_ = distance(*parent_) + parent_->cost();
    } else {
      cost_ = 0;
    }
	}

  double cost() const override {
    return cost_;
  }

  std::shared_ptr<NodeInterface> clone() const override {
    auto new_node = std::make_shared<Motion2D>();
    new_node->x = x;
    new_node->y = y;
    new_node->u = u;
    new_node->parent_ = parent_;
    new_node->cost_ = cost_;
    return std::static_pointer_cast<NodeInterface>(new_node);
  };
 private:
	NodePtr parent_;
  double cost_ = 0;
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

  bool collision_free(NodePtr node) const override;

  bool collision_free_path(NodePtr node_a, NodePtr node_b) const override;

  std::vector<CollisionPtr> collisions() const override {
    return collisions_;
  }

  void add_collision_box(double x0, double y0, double width, double height);

  size_t num_dimensions() const override { return 2; }

  double control() const override { return control_; }

 private:
	mutable std::random_device seed_gen_;
	mutable std::default_random_engine random_engine_{seed_gen_()};
  double xmin_{0.0};
  double xmax_{0.0};
  double ymin_{0.0};
  double ymax_{0.0};
	double control_{0.0};
  std::vector<CollisionPtr> collisions_;
};

}  // namespace rrt
