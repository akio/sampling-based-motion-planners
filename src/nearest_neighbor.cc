#include "nearest_neighbor.h"

#include <algorithm>
#include <cassert>
#include <limits>

namespace rrt {

LinearSearchNN::LinearSearchNN() {}

void LinearSearchNN::add(NodePtr motion) { nodes_.push_back(motion); }

NodePtr LinearSearchNN::find_nearest(NodePtr node) const {
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

std::vector<NearestNeighborInterface::NodeWithCost>
LinearSearchNN::nearest_neighbors(NodePtr node, size_t k) const {
  size_t num_results = std::min(k, nodes_.size());
  std::vector<NodeWithCost> work;
  work.reserve(nodes_.size());
  for (const auto& n : nodes_) {
    double d = node->distance(*n);
    work.push_back({n, d});
  }
  std::sort(begin(work), end(work),
            [](const NodeWithCost& a, const NodeWithCost& b) {
              return a.second < b.second;
            });
  std::vector<NodeWithCost> result(num_results);
  std::copy(begin(work), begin(work) + num_results, begin(result));
  return result;
}

std::vector<NearestNeighborInterface::NodeWithCost>
LinearSearchNN::radius_search(NodePtr node, double radius) const {
  assert(radius >= 0);
  if (radius == 0) {
    return {};
  }
  std::vector<NodeWithCost> result;
  for (const auto& n : nodes_) {
    double d = node->distance(*n);
    if (d <= radius) {
      result.push_back({n, d});
    }
  }
  return result;
}

void LinearSearchNN::reset() { nodes_.clear(); }

}  // namespace rrt
