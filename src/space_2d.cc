#include "space_2d.h"

#include <random>

namespace rrt {

Box2D::Box2D(double x0, double y0, double width, double height)
    : x0(x0), y0(y0), width(width), height(height) {
}

bool Box2D::collide(const NodePtr& node) const {
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

NodePtr Space2D::sample() const {
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

NodePtr Space2D::steer(NodePtr origin, NodePtr target) const {
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

	return new_node;
}


bool Space2D::has_collision(NodePtr node) const {
  for (auto c : collisions_) {
    if (c->collide(node)) {
      return true;
    }
  }
  return false;
}

void Space2D::add_collision_box(double x0, double y0, double width, double height) {
  auto box = std::make_shared<Box2D>(x0, y0, width, height);
  collisions_.push_back(box);
}

}  // namespace rrt
