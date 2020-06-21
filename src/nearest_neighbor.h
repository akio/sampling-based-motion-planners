#pragma once

#include <vector>

#include "space.h"

namespace rrt {

class NearestNeighborInterface {
 public:
  typedef std::pair<NodePtr, double> NodeWithCost;

  virtual void add(NodePtr node) = 0;

	virtual NodePtr find_nearest(NodePtr node) const = 0;

	virtual std::vector<NodeWithCost> nearest_neighbors(NodePtr node, size_t k) const = 0;

	virtual std::vector<NodeWithCost> radius_search(NodePtr node, double radius) const = 0;


  virtual void reset() = 0;
};

typedef std::shared_ptr<NearestNeighborInterface> NearestNeighborPtr;

class LinearSearchNN : public NearestNeighborInterface {
 public:
	LinearSearchNN();

  void add(NodePtr node) override;

	NodePtr find_nearest(NodePtr node) const override;

	std::vector<NodeWithCost> nearest_neighbors(NodePtr node, size_t k) const override;

	std::vector<NodeWithCost> radius_search(NodePtr node, double radius) const override;

  void reset() override;
 private:
	std::vector<NodePtr> nodes_;
};

}  // namespace rrt
