#include <iostream>
#include <random>
#include <cassert>

#include "rrt.h"

namespace rrt {

Box2D::Box2D(double x0, double y0, double width, double height)
    : x0(x0), y0(y0), width(width), height(height) {
}

bool Box2D::Collide(const NodePtr& node) const {
  auto m = std::dynamic_pointer_cast<Motion2D>(node);

  if (x0 <= m->x && m->x <= x0 + width &&
      y0 <= m->y && m->y <= y0 + height) {
    return true;
  } else {
    return false;
  }
}


Space2D::Space2D(double xmin, double xmax, double ymin, double ymax, double control)
		: xmin_(xmin),
			xmax_(xmax),
			ymin_(ymin),
			ymax_(ymax),
			control_(control) {
	assert(xmin_ < xmax_);
	assert(ymin_ < ymax_);
}

NodePtr Space2D::Sample() const {
	std::random_device seed_gen;

	std::default_random_engine engine(seed_gen());
	std::uniform_real_distribution<> x_dist(xmin_, xmax_);
	std::uniform_real_distribution<> y_dist(ymin_, ymax_);
	auto n = std::make_shared<Motion2D>();
	n->x = x_dist(engine);
	n->y = y_dist(engine);
	n->u = 0;
	n->set_parent(nullptr);
	return std::static_pointer_cast<NodeInterface>(n);
}

NodePtr Space2D::NewNode(NodePtr origin, NodePtr target) const {
	auto n0 = std::dynamic_pointer_cast<Motion2D>(origin);
	auto n1 = std::dynamic_pointer_cast<Motion2D>(target);

	double u = n1->x - n0->x;
	double v = n1->y - n0->y;
  double norm = std::sqrt(u * u + v * v);

	double coeff = control_ / norm;
	if (coeff > 1.0) {
		coeff = 1.0;
	}

	double new_x = n0->x + u * coeff;
	double new_y = n0->y + v * coeff;
	auto new_node = std::make_shared<Motion2D>();
	new_node->x = new_x;
	new_node->y = new_y;
	new_node->u = coeff * norm;
	new_node->set_parent(origin);

  for (auto c : collisions_) {
    if (c->Collide(new_node)) {
      return NodePtr{};
    }
  }

	return new_node;
}


void Space2D::AddCollisionBox(double x0, double y0, double width, double height) {
  auto box = std::make_shared<Box2D>(x0, y0, width, height);
  collisions_.push_back(box);
}

LinearSearchNN::LinearSearchNN() {}

void LinearSearchNN::Add(NodePtr motion) {
	nodes_.push_back(motion);
}

NodePtr LinearSearchNN::Query(NodePtr node) const {
	double min_distance = std::numeric_limits<double>::infinity();
  NodePtr result;
	for (const auto& n : nodes_) {
		double d = node->Distance(*n);
		if (d < min_distance) {
			result = n;
			min_distance = d;
		}
	}
	return result;
}


Rrt::Rrt(SpacePtr space)
    : nn_(new LinearSearchNN()),
      space_(space),
      max_samples_(-1) {

}

bool Rrt::Solve(const NodePtr& init, const NodePtr& goal) {
	nn_->Add(init);
  nodes_.push_back(init);

  int i = 0;
  while (true) {
    if (max_samples_ >= 0 && i >= max_samples_) {
      break;
    }
    ++i;
		NodePtr x_random = space_->Sample();
    NodePtr x_new;
    if (use_connect_) {
      (void)Connect(x_random, x_new);
    } else {
      (void)Extend(x_random, x_new);
    }

    if (x_new) {
      auto distance = x_new->Distance(*goal);
      if (distance <= goal_tolerance_) {
        return true;
      }
    }
	}
  return false;
}

RrtStatus Rrt::Extend(const NodePtr& x_sample, NodePtr& out_x_new) {
  auto x_near = nn_->Query(x_sample);
  assert(x_near);
  auto x_new = space_->NewNode(x_near, x_sample);
  if (x_new) {
    out_x_new = x_new;
    nn_->Add(x_new);
    nodes_.push_back(x_new);
    if (x_sample->Distance(*x_new) <= std::numeric_limits<double>::epsilon()) {
      return RrtStatus::kReached;
    } else {
      return RrtStatus::kAdvanced;
    }
  }
  return RrtStatus::kTrapped;
}

RrtStatus Rrt::Connect(const NodePtr& x_sample, NodePtr& out_x_new) {
  RrtStatus status;
  while (true) {
    status = Extend(x_sample, out_x_new);
    if (status != RrtStatus::kAdvanced) {
      break;
    }
  }
  return status;
}

void Rrt::Clear() {
  nn_.reset(new LinearSearchNN());
  nodes_.clear();
}

BidirectionalRrt::BidirectionalRrt(SpacePtr space)
    : init_nn_(new LinearSearchNN()),
      goal_nn_(new LinearSearchNN()),
      space_(space),
      max_samples_(-1) {
}

bool BidirectionalRrt::Solve(const NodePtr& init, const NodePtr& goal) {
  init_nn_->Add(init);
  init_nodes_.push_back(init);
  goal_nn_->Add(goal);
  goal_nodes_.push_back(goal);

  int i = 0;
  while (true) {
    if (max_samples_ >= 0 && i >= max_samples_) {
      break;
    }
    ++i;
		NodePtr x_random = space_->Sample();
    NodePtr x_new;

  }
  return false;
}

RrtStatus BidirectionalRrt::Extend(const NodePtr& x_sample, NodePtr& out_x_new) {
  auto x_near = init_nn_->Query(x_sample);
  assert(x_near);
  auto x_new = space_->NewNode(x_near, x_sample);
  if (x_new) {
    out_x_new = x_new;
    init_nn_->Add(x_new);
    init_nodes_.push_back(x_new);
    if (x_sample->Distance(*x_new) <= std::numeric_limits<double>::epsilon()) {
      return RrtStatus::kReached;
    } else {
      return RrtStatus::kAdvanced;
    }
  }
  return RrtStatus::kTrapped;
}

void BidirectionalRrt::Clear() {
  init_nn_.reset(new LinearSearchNN());
  goal_nn_.reset(new LinearSearchNN());
  init_nodes_.clear();
  goal_nodes_.clear();
}


}  // namespace rrt
