// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors
#pragma once
#include <vector>

namespace utl {
class Logger;
}  // namespace utl
namespace odb {
class dbDatabase;
}
namespace dpl {
class Node;
class Pin;
class Network;
class DetailedHPWL;
class Journal;
class PinSwapper
{
 public:
  using Swap = std::pair<Pin*, Pin*>;
  using SwapGroup = std::vector<Swap>;

  PinSwapper(utl::Logger* logger, Network* network, odb::dbDatabase* db);
  void start();

 private:
  void swapPins(Node* node);
  SwapGroup swapPins(const std::vector<Pin*>& pins1,
                     const std::vector<Pin*>& pins2);
  Swap swapPins(Pin* pin1, Pin* pin2);
  // Generate a neighboring placement by perturbing the current solution
  void generate_neighbor();
  bool accept(double delta_cost);

  utl::Logger* logger_{nullptr};
  Network* network_{nullptr};
  odb::dbDatabase* db_{nullptr};
  DetailedHPWL* hpwl_evaluator_{nullptr};
  std::vector<Node*> nodes_;
  Journal* journal_;
  int last_id_{0};
  // Tracking costs (no placement state saved)
  double current_cost_{0.0};
  double best_cost_{0.0};
  int count_{0};
};
}  // namespace dpl