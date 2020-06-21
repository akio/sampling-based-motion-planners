#pragma once

#include <cmath>
#include <vector>
#include <memory>
#include <string>

namespace rrt {

class NodeInterface {
 public:
  virtual double distance(const NodeInterface& node) const = 0;

  virtual std::shared_ptr<NodeInterface> midpoint(const NodeInterface& node) const = 0;

	virtual std::shared_ptr<NodeInterface> parent() const = 0;

	virtual void set_parent(std::shared_ptr<NodeInterface> parent) = 0;

  virtual double cost() const = 0;

  virtual std::shared_ptr<NodeInterface> clone() const = 0;
};

typedef std::shared_ptr<NodeInterface> NodePtr;


class CollisionInterface {
 public:
  virtual bool collide(const NodePtr& node) const = 0;
};

typedef std::shared_ptr<CollisionInterface> CollisionPtr;

class SpaceInterface {
 public:
	virtual NodePtr sample() const = 0;

	virtual NodePtr steer(NodePtr origin, NodePtr target) const = 0;

  virtual bool collision_free(NodePtr node) const = 0;

  virtual bool collision_free_path(NodePtr node_a, NodePtr node_b) const = 0;

  virtual std::vector<CollisionPtr> collisions() const = 0;

  virtual size_t num_dimensions() const = 0;

  virtual double control() const = 0;
};

typedef std::shared_ptr<SpaceInterface> SpacePtr;

}  // namespace rrt


