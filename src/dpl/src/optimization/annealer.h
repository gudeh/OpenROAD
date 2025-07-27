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
  /**
   * @brief Sets the parameters for simulated annealing optimization.
   *
   * This function configures the key parameters that control the simulated
   * annealing process including temperature schedule, cooling rate, iteration
   * count, and random seed for reproducible results.
   * @param initial_temp the starting temperature for annealing
   * @param alpha the cooling factor (0 < alpha < 1) applied each iteration
   * @param max_iterations the maximum number of annealing iterations to perform
   * @param seed the random seed for reproducible optimization runs
   */
  void set_sa_parameters(double initial_temp,
                         float alpha,
                         int max_iterations,
                         int seed);
  /**
   * @brief Starts the simulated annealing optimization process.
   *
   * This function initiates the main annealing loop that iteratively generates
   * neighboring solutions by merging cells to create multibit instances,
   * evaluates cost changes, and accepts/rejects moves. The process continues
   * until max_iterations is reached or no more node groups are available for
   * optimization.
   */
  void start();

 private:
  /**
   * @brief Swaps multiple smaller nodes to create a single larger multibit
   * cell.
   *
   * This function combines smaller single-bit or few-bit cells into a larger
   * multibit cell of the target master type. It handles the complex process
   * of creating a new database instance, reconnecting nets, and updating the
   * placement grid.
   * @param small_nodes the vector of smaller nodes to be combined
   * @param target_master the master cell type for the resulting multibit cell
   * @return true if the swap operation was successful, false otherwise
   */
  bool swapNodes(std::vector<Node*> small_nodes, Master* target_master);
  /**
   * @brief Dismantles a node by removing it from placement and disconnecting
   * its pins.
   *
   * This function removes a node from the placement grid, disconnects all its
   * pins from their respective nets, and marks the node to be deleted by the
   * end of annealing. This is typically called as part of the cell swapping
   * process when smaller nodes are being replaced by larger multibit cells.
   * @param node the node to be dismantled and removed
   */
  void dismantleNode(Node* node);
  /**
   * @brief Assigns a node to an appropriate node group based on function
   * compatibility.
   *
   * This function groups nodes that have compatible functions and can
   * potentially be merged into larger multibit cells. Nodes are grouped
   * based on their master function, region, and net connectivity compatibility.
   * @param node the node to be assigned to a compatible group
   */
  void assignToNodeGroup(Node* node);
  /**
   * @brief Generates a neighboring placement solution by perturbing the
   * current configuration.
   *
   * This function implements the neighbor generation strategy for simulated
   * annealing by selecting a random group of compatible nodes and attempting
   * to swap them for a larger multibit cell. It uses a search algorithm to
   * find the best combination of nodes that match the target master's
   * requirements.
   * @param group_idx reference to store the index of the selected node group
   * @param sub_group reference to store the indices of nodes selected for
   * swapping within the group
   * @return true if a valid neighbor was generated and swap was successful,
   * false otherwise
   */
  bool generate_neighbor(int& group_idx, std::vector<int>& sub_group);
  /**
   * @brief Implements the Metropolis acceptance criterion for simulated
   * annealing.
   *
   * This function determines whether to accept a proposed move based on the
   * cost difference and current temperature. It always accepts improving
   * moves (negative delta_cost) and probabilistically accepts worsening
   * moves based on the probability distribution.
   * @param delta_cost the change in cost (new_cost - current_cost) for the
   * proposed move
   * @return true if the move should be accepted, false if it should be rejected
   */
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