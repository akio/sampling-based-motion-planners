#include "nearest_neighbor.h"

#include <limits>

namespace rrt {

LinearSearchNN::LinearSearchNN() {}

void LinearSearchNN::add(NodePtr motion) {
	nodes_.push_back(motion);
}

NodePtr LinearSearchNN::query(NodePtr node) const {
	double min_distance = std::numeric_limits<double>::infinity();
  NodePtr result;
	for (const auto& n : nodes_) {
		double d = node->distance(*n);
		if (d < min_distance) {
			result = n;
			min_distance = d;
		}
	}
	return result;
}

void LinearSearchNN::reset() {
  nodes_.clear();
}


} // namespace rrt
