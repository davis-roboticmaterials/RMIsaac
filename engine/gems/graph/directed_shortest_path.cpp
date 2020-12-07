/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "directed_shortest_path.hpp"

#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"

namespace isaac {
namespace graph {

void DirectedShortestPath::addEdge(int from, int to, double dist) {
  const int num_nodes = getNumNodes();
  ASSERT(0 <= from && from < num_nodes, "Invalid source index: 0 <= %d < %d", from, num_nodes);
  ASSERT(0 <= to && to < num_nodes, "Invalid destimation index: 0 <= %d < %d", to, num_nodes);
  ISAAC_ASSERT_LT(0.0, dist);
  nodes_[to].in.push_back({from, dist});
  nodes_[from].out.push_back({to, dist});
}

void DirectedShortestPath::resetNumNodes(int num_nodes) {
  ISAAC_ASSERT_LE(0, num_nodes);
  nodes_.resize(num_nodes);
  dijkstra_visited_.resize(num_nodes, 0);
  dijkstra_processed_.resize(num_nodes, 0);
  dijkstra_from_.resize(num_nodes);
  dijkstra_distance_.resize(num_nodes);
  current_target_ = -1;
}

void DirectedShortestPath::resetTarget(int end_id) {
  const int num_nodes = getNumNodes();
  ASSERT(0 <= end_id && end_id < num_nodes,
         "Invalid source index: 0 <= %d < %d", end_id, num_nodes);
  // Make sure the memory has been allocated properly
  if (virtual_init_ == 0) {
    // Reset which node have been visited for the first time, or in the very unlikely case we do
    // 4 Billions planning.
    std::fill(dijkstra_visited_.begin(), dijkstra_visited_.end(), 0);
    std::fill(dijkstra_processed_.begin(), dijkstra_processed_.end(), 0);
  }
  virtual_init_++;
  while (!queue_.empty()) queue_.pop();
  // Initialize dijkstra by inserting the last node in the queue.
  queue_.push({0.0, end_id});
  dijkstra_distance_[end_id] = 0.0;
  dijkstra_from_[end_id] = -1;
  dijkstra_visited_[end_id] = virtual_init_;
  current_target_ = end_id;
}

std::vector<int> DirectedShortestPath::findShortestPath(int start_id) {
  ISAAC_ASSERT_LE(0, current_target_);
  // While we have not reached the start position we keep going
  while (!queue_.empty() && dijkstra_processed_[start_id] != virtual_init_) {
    // Get the node with the minimum distance not processed yet.
    const QueueNode current = queue_.top();
    queue_.pop();
    // If we have already processed this node, we can move to the next one
    if (dijkstra_processed_[current.node_id] == virtual_init_) continue;
    // Mark the node as processed, at this point there will be no shorter path from that node.
    dijkstra_processed_[current.node_id] = virtual_init_;
    // For each of the node that lead to the current node, we try to see if we are getting a shorter
    // path.
    for (const auto& node : nodes_[current.node_id].in) {
      const double weight = current.weight + node.weight;
      const int node_idx = node.node_id;
      if (dijkstra_visited_[node_idx] != virtual_init_ ||  // Never visited yet
          dijkstra_distance_[node_idx] > weight) {         // Or we find a better path
        dijkstra_visited_[node_idx] = virtual_init_;
        dijkstra_distance_[node_idx] = weight;
        dijkstra_from_[node_idx] = current.node_id;
        queue_.push({weight, node_idx});
      }
    }
  }
  std::vector<int> path;
  if (dijkstra_processed_[start_id] == virtual_init_) {
    // We have reached the start, let's extract the path.
    while (start_id >= 0) {
      path.push_back(start_id);
      start_id = dijkstra_from_[start_id];
    }
  }
  return path;
}

}  // namespace graph
}  // namespace isaac
