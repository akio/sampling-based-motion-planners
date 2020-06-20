#pragma once

#include <vector>

#include "space.h"

namespace rrt {

class NearestNeighborInterface {
 public:
  virtual void add(NodePtr node) = 0;
	virtual NodePtr query(NodePtr motion) const = 0;
  virtual void reset() = 0;
};

typedef std::shared_ptr<NearestNeighborInterface> NearestNeighborPtr;

class LinearSearchNN : public NearestNeighborInterface {
 public:
	LinearSearchNN();

  void add(NodePtr node) override;

	NodePtr query(NodePtr node) const override;

  void reset() override;
 private:
	std::vector<NodePtr> nodes_;
};

}  // namespace rrt
