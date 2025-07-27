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
  /**
   * @brief Starts the pin swapping optimization process.
   *
   * This function iterates through all standard cells in the design and
   * applies various pin optimization strategies including bit placement
   * optimization for multibit cells, pin swaps based on master-defined
   * swap groups, and pin permutations. The process continues until no
   * further improvements can be found for each node.
   */
  void start();

 private:
  /**
   * @brief Optimizes bit placement for multibit cells by swapping equivalent
   * bit groups.
   *
   * This function analyzes multibit cells and attempts to swap equivalent
   * bit groups (pins with the same function but different bit indices) to
   * minimize wire length. It groups pins by their bit index and tries all
   * possible swaps between groups that have complete sets of function pins.
   * @param node the multibit node whose bit placement needs optimization
   * @return true if any beneficial swaps were made, false otherwise
   */
  bool optimizeBitPlacement(Node* node);
  /**
   * @brief Optimizes pin connectivity using master-defined pin swap groups.
   *
   * This function performs pin swaps based on predefined swap groups in the
   * cell master definition. For each bit position in multibit cells, it
   * attempts to swap pins according to the master's pin swap specifications
   * to achieve better wire length.
   * @param node the node whose pins need swap optimization
   * @return true if any beneficial swaps were made, false otherwise
   */
  bool optimizePinSwaps(Node* node);
  /**
   * @brief Optimizes pin connectivity using master-defined pin permutation
   * groups.
   *
   * This function performs pin permutations based on predefined permutation
   * groups in the cell master definition. It tries different permutations
   * of equivalent pins for each bit position to find the configuration that
   * minimizes total wire length.
   * @param node the node whose pins need permutation optimization
   * @return true if any beneficial permutations were made, false otherwise
   */
  bool optimizePinPermutes(Node* node);
  /**
   * @brief Swaps multiple groups of pins if the swap improves wire length.
   *
   * This function attempts to swap corresponding pins between two groups,
   * evaluates the wire length change, and commits the swap only if it
   * results in an improvement. It also updates the database connectivity
   * to reflect the changes.
   * @param pins1 the first group of pins to be swapped
   * @param pins2 the second group of pins to be swapped
   * @return true if any beneficial swaps were made, false otherwise
   */
  bool swapPins(const std::vector<Pin*>& pins1, const std::vector<Pin*>& pins2);
  /**
   * @brief Swaps two individual pins and records the action in the journal.
   *
   * This function performs the low-level pin swap operation by disconnecting
   * pins from their current nets and reconnecting them to each other's nets.
   * The swap is recorded in the journal for potential rollback and wirelength
   * evaluation.
   * @param pin1 the first pin to be swapped
   * @param pin2 the second pin to be swapped
   * @return a Swap pair representing the completed swap operation
   */
  Swap swapPins(Pin* pin1, Pin* pin2);
  /**
   * @brief Implements a greedy acceptance criterion for pin swaps.
   *
   * This function determines whether to accept a proposed pin swap based
   * on the cost difference. This uses a greedy approach that only accepts
   * moves that improve the wire length.
   * @param delta_cost the change in cost (new_cost - current_cost) for the
   * proposed swap
   * @return true if delta_cost is negative, false otherwise
   */
  bool accept(double delta_cost);

  utl::Logger* logger_{nullptr};
  Network* network_{nullptr};
  odb::dbDatabase* db_{nullptr};
  DetailedHPWL* hpwl_evaluator_{nullptr};
  Journal* journal_;
  int last_id_{0};
  // Tracking costs (no placement state saved)
  double current_cost_{0.0};
  double best_cost_{0.0};
  int count_{0};
};
}  // namespace dpl