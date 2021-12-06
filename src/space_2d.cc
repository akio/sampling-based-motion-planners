#include "space_2d.h"

namespace rrt {

Box2D::Box2D(double x0, double y0, double width, double height)
    : x0(x0), y0(y0), width(width), height(height) {}

bool Box2D::collide(const NodePtr& node) const {
  auto m = std::dynamic_pointer_cast<Motion2D>(node);

  if (x0 <= m->x && m->x <= x0 + width && y0 <= m->y && m->y <= y0 + height) {
    return true;
  } else {
    return false;
  }
}

Space2D::Space2D(double xmin, double xmax, double ymin, double ymax,
                 double control)
    : xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax), control_(control) {
  assert(xmin_ < xmax_);
  assert(ymin_ < ymax_);
}

NodePtr Space2D::sample() const {
  std::uniform_real_distribution<> x_dist(xmin_, xmax_);
  std::uniform_real_distribution<> y_dist(ymin_, ymax_);
  auto n = std::make_shared<Motion2D>();
  n->x = x_dist(random_engine_);
  n->y = y_dist(random_engine_);
  n->u = 0;
  n->set_parent(nullptr);
  return std::static_pointer_cast<NodeInterface>(n);
}

NodePtr Space2D::steer(NodePtr origin, NodePtr target) const {
  auto n0 = std::static_pointer_cast<Motion2D>(origin);
  auto n1 = std::static_pointer_cast<Motion2D>(target);

  double u = n1->x - n0->x;
  double v = n1->y - n0->y;
  double norm = std::hypot(u, v);

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

bool Space2D::collision_free(NodePtr node) const {
  for (auto c : collisions_) {
    if (c->collide(node)) {
      return false;
    }
  }
  return true;
}

// Line search
bool Space2D::collision_free_path(NodePtr node_a, NodePtr node_b) const {
  auto n0 = std::static_pointer_cast<Motion2D>(node_a);
  auto n1 = std::static_pointer_cast<Motion2D>(node_b);
  auto work = std::make_shared<Motion2D>();

  double u = n1->x - n0->x;
  double v = n1->y - n0->y;
  double norm = std::hypot(u, v);

  double resolution = 0.5;
  double control_step = resolution * control_;

  work->x = n0->x;
  work->y = n0->y;
  int i = 0;
  while (true) {
    double t = control_step * i / norm;
    if (t > 1.0) {
      return collision_free(node_b);
    }
    work->x = n0->x + u * t;
    work->y = n0->y + v * t;
    if (!collision_free(work)) {
      return false;
    }
    ++i;
  }
}

void Space2D::add_collision_box(double x0, double y0, double width,
                                double height) {
  auto box = std::make_shared<Box2D>(x0, y0, width, height);
  collisions_.push_back(box);
}

}  // namespace rrt
