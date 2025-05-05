// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors
#pragma once
#include <random>
#include <string>
#include <vector>

#include "util/journal.h"
namespace utl {
class Logger;
}  // namespace utl
namespace odb {
class dbDatabase;
}
namespace dpl {
class Opendp;
class Node;
class Master;
class Network;
class Optdp;
class DetailedHPWL;
class Annealer
{
 public:
  Annealer(utl::Logger* logger,
           Opendp* opendp,
           Network* network,
           odb::dbDatabase* db);
  // Set simulated annealing parameters
  void set_sa_parameters(double initial_temp,
                         float alpha,
                         int max_iterations,
                         int seed);
  void start();

 private:
  bool swapNodes(std::vector<Node*> small_nodes, Master* target_master);
  void dismantleNode(Node* node);
  void assignToNodeGroup(Node* node);
  // Generate a neighboring placement by perturbing the current solution
  bool generate_neighbor(int& group_idx, std::vector<int>& sub_group);
  // Acceptance criterion (Metropolis rule)
  bool accept(double delta_cost);

  utl::Logger* logger_{nullptr};
  Opendp* opendp_{nullptr};
  Network* network_{nullptr};
  odb::dbDatabase* db_{nullptr};
  DetailedHPWL* hpwl_evaluator_{nullptr};
  std::vector<std::vector<Node*>> node_groups_;
  Journal* journal_;
  int last_id_{0};
  // SA
  std::mt19937 generator_;
  std::uniform_real_distribution<float> distribution_;
  double temperature_{1e3};
  float alpha_{0.95};
  int max_iterations_{1000};
  // Tracking costs (no placement state saved)
  double current_cost_{0.0};
  double best_cost_{0.0};

  // testing
  int last_node_idx_{0};
};
}  // namespace dpl