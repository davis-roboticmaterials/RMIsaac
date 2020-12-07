/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/graph/directed_shortest_path.hpp"

#include <vector>

#include "gtest/gtest.h"

namespace isaac {
namespace graph {

TEST(DirectedShortestPath, small) {
  DirectedShortestPath graph(5);
  graph.addEdge(0, 2, 3.0);
  graph.addEdge(0, 1, 1.0);
  graph.addEdge(1, 2, 1.0);
  graph.addEdge(2, 3, 0.5);
  graph.addEdge(3, 0, 0.5);
  graph.addEdge(4, 0, 1.0);
  graph.resetTarget(2);
  { // Check path 0 -> 2 (should go though 1)
    const auto& path = graph.findShortestPath(0);
    ASSERT_EQ(path.size(), 3);
    EXPECT_EQ(path[0], 0);
    EXPECT_EQ(path[1], 1);
    EXPECT_EQ(path[2], 2);
  }
  { // Check path 3 -> 2 (should go though 0 & 1)
    const auto& path = graph.findShortestPath(3);
    ASSERT_EQ(path.size(), 4);
    EXPECT_EQ(path[0], 3);
    EXPECT_EQ(path[1], 0);
    EXPECT_EQ(path[2], 1);
    EXPECT_EQ(path[3], 2);
  }
  { // Check path 4 -> 2 (should go though 0 & 1)
    const auto& path = graph.findShortestPath(4);
    ASSERT_EQ(path.size(), 4);
    EXPECT_EQ(path[0], 4);
    EXPECT_EQ(path[1], 0);
    EXPECT_EQ(path[2], 1);
    EXPECT_EQ(path[3], 2);
  }
  graph.resetTarget(4);
  for (int i = 0; i < 4; i++) {
    EXPECT_TRUE(graph.findShortestPath(i).empty());
  }
}

} // namespace graph
} // namespace isaac