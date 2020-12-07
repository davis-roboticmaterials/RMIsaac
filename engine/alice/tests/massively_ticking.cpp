/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace alice {

class Ticker : public Codelet {
 public:
  void start() override {
    tickPeriodically();
  }
};

TEST(Alice, SimpleCodelet) {
  std::vector<Ticker*> tickers;
  Application app;
  for (int i = 0; i < 50; i++) {
    Node* node = app.createNode("test_" + std::to_string(i));
    auto* ticker = node->addComponent<Ticker>();
    ticker->async_set_tick_period("10ms");
    tickers.push_back(ticker);
  }
  app.startWaitStop(1.0);
  for (Ticker* ticker : tickers) {
    EXPECT_NEAR(ticker->getTickCount(), 100, 10);
  }
}

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::Ticker);
