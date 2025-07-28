// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors
#include "annealer.h"

#include <boost/random/uniform_int_distribution.hpp>
#include <cmath>

#include "dpl/Opendp.h"
#include "infrastructure/Grid.h"
#include "infrastructure/network.h"
#include "objective/detailed_hpwl.h"
#include "odb/db.h"
#include "util/journal.h"
#include "util/utility.h"
#include "utl/Logger.h"
namespace dpl {
namespace {

/**
 * @brief Recursive helper function for finding the optimal combination of nodes
 * to merge with a seed node.
 *
 * This function performs a depth-first search with pruning to find the best
 * subset of nodes from a group that, when combined with a seed node, will
 * best match the target dimensions. It uses a dimension metric (|ΔW| + |ΔH|)
 * to score combinations and prunes branches that cannot improve the best score.
 *
 * @param seed the seed node that anchors the combination
 * @param tgt_w target width for the combined bounding box
 * @param tgt_h target height for the combined bounding box
 * @param group the vector of candidate nodes to choose from
 * @param start the starting index in the group for this branch
 * @param remain the remaining function bits needed to complete the combination
 * @param ux_min current minimum x-coordinate of the union bounding box
 * @param uy_min current minimum y-coordinate of the union bounding box
 * @param ux_max current maximum x-coordinate of the union bounding box
 * @param uy_max current maximum y-coordinate of the union bounding box
 * @param current the current combination being built (node indices)
 * @param best reference to store the best combination found so far
 * @param best_score reference to track the best score achieved
 */
void dfs_helper(const Node* seed,
                int tgt_w,
                int tgt_h,
                const std::vector<Node*>& group,
                int start,
                int remain,
                int ux_min,
                int uy_min,
                int ux_max,
                int uy_max,
                std::vector<int>& current,
                std::vector<int>& best,
                int& best_score)
{
  if (remain == 0) {
    int w = ux_max - ux_min;
    int h = uy_max - uy_min;
    // dimension metric (|ΔW| + |ΔH|)
    int score = std::abs(w - tgt_w) + std::abs(h - tgt_h);
    if (score < best_score) {
      best_score = score;
      best = current;
    }
    return;
  }

  int n = group.size();
  for (int i = start; i <= n - remain; ++i) {
    const Node* node = group[i];
    if (node == seed) {
      continue;
    }
    if (node->isToBeRemoved()) {
      continue;
    }
    if (node->getMaster()->getFunctionBits() > remain) {
      continue;
    }

    const auto& r = node->getBBox();
    int x0 = std::min(ux_min, r.xMin());
    int y0 = std::min(uy_min, r.yMin());
    int x1 = std::max(ux_max, r.xMax());
    int y1 = std::max(uy_max, r.yMax());

    // partial score pruning using dimension metric
    int w = x1 - x0;
    int h = y1 - y0;
    int partial = std::abs(w - tgt_w) + std::abs(h - tgt_h);
    if (partial >= best_score) {
      continue;
    }

    current.push_back(i);
    dfs_helper(seed,
               tgt_w,
               tgt_h,
               group,
               i + 1,
               remain - node->getMaster()->getFunctionBits(),
               x0,
               y0,
               x1,
               y1,
               current,
               best,
               best_score);
    current.pop_back();
  }
}

/**
 * @brief Finds the optimal set of nodes to merge with a seed node to match
 * target dimensions.
 *
 * This function uses a depth-first search algorithm to find the best
 * combination of (k_total - 1) nodes from the group that, when combined
 * with the seed node, will create a bounding box closest to the target
 * rectangle dimensions. The optimization criterion is minimizing the
 * Manhattan distance between actual and target dimensions.
 *
 * @param seed the seed node that anchors the combination
 * @param target_rect the target rectangle dimensions to match
 * @param group the vector of candidate nodes to choose from
 * @param k_total the total number of function bits required in the final
 * combination
 * @return vector of indices within 'group' representing the chosen nodes
 */
std::vector<int> findBestGroup(const Node* seed,
                               const odb::Rect& target_rect,
                               const std::vector<Node*>& group,
                               int k_total)
{
  const auto& s = seed->getBBox();
  int tgt_w = target_rect.dx();
  int tgt_h = target_rect.dy();
  int need = k_total - seed->getMaster()->getFunctionBits();

  std::vector<int> best;
  std::vector<int> current;
  int best_score = std::numeric_limits<int>::max();

  dfs_helper(seed,
             tgt_w,
             tgt_h,
             group,
             0,
             need,
             s.xMin(),
             s.yMin(),
             s.xMax(),
             s.yMax(),
             current,
             best,
             best_score);

  return best;
}
}  // namespace

Annealer::Annealer(utl::Logger* logger,
                   dpl::Opendp* opendp,
                   Network* network,
                   odb::dbDatabase* db)
    : logger_(logger), opendp_(opendp), network_(network), db_(db)
{
  generator_ = std::mt19937(19);
  std::uniform_real_distribution<float> distribution(0.0, 1.0);
  distribution_ = distribution;
  hpwl_evaluator_ = new DetailedHPWL(network);
  hpwl_evaluator_->init();
  current_cost_ = hpwl_evaluator_->curr();
  best_cost_ = current_cost_;
}
void Annealer::set_sa_parameters(double initial_temp,
                                 float alpha,
                                 int max_iterations,
                                 int seed)
{
  temperature_ = initial_temp;
  alpha_ = alpha;
  max_iterations_ = max_iterations;
  generator_ = std::mt19937(seed);
}

void Annealer::dismantleNode(Node* node)
{
  node->setPlaced(true);
  node->setFixed(false);
  opendp_->getGrid()->erasePixel(node);
  for (auto pin : node->getPins()) {
    auto edge = pin->getEdge();
    edge->removePin(pin);
  }
  node->setToBeRemoved(true);
}

bool Annealer::swapNodes(std::vector<Node*> small_nodes, Master* target_master)
{
  std::shuffle(small_nodes.begin(), small_nodes.end(), generator_);
  auto block = db_->getChip()->getBlock();
  std::string new_name = fmt::format("MULTIBIT_{}", last_id_++);
  while (block->findInst(new_name.c_str()) != nullptr) {
    new_name = fmt::format("MULTIBIT_{}", last_id_++);
  }
  auto function = target_master->getFunction();
  auto big_inst = odb::dbInst::create(block,
                                      target_master->getDbMaster(),
                                      new_name.c_str(),
                                      small_nodes[0]->getDbInst()->getRegion());
  DbuX left = std::numeric_limits<DbuX>::max();
  DbuY bottom = std::numeric_limits<DbuY>::max();
  int current_bit = target_master->getLsb();

  // Step 1: Reconnect multibit pins by mapping each small node's bits to the
  // big instance
  for (auto node : small_nodes) {
    left = std::min(left, node->getLeft());
    bottom = std::min(bottom, node->getBottom());
    auto inst = node->getDbInst();
    // Map each bit position from the small node to the corresponding bit in the
    // big instance
    for (int small_bit = node->getMaster()->getLsb();
         small_bit <= node->getMaster()->getMsb();
         small_bit++) {
      for (const auto& [single_bit, pattern] : function->getMultibitPinMap()) {
        std::string small_iterm_name
            = small_bit == -1 ? single_bit
                              : Utility::getPinName(pattern, small_bit);
        auto iterm = inst->findITerm(small_iterm_name.c_str());
        auto big_iterm = big_inst->findITerm(
            Utility::getPinName(pattern, current_bit).c_str());
        if (iterm == nullptr) {
          logger_->error(utl::DPL, 503, "ITerm {} not found for {}", small_iterm_name, inst->getMaster()->getConstName());
          continue;
        }
        if (big_iterm == nullptr) {
          logger_->error(utl::DPL,
                         504,
                         "ITerm {} not found for {}",
                         Utility::getPinName(pattern, current_bit),
                         big_inst->getMaster()->getConstName());
          continue;
        }
        if (iterm->getNet() == nullptr) {
          logger_->error(
              utl::DPL, 505, "ITerm {} not connected", small_iterm_name);
          continue;
        }
        big_iterm->connect(iterm->getNet());
      }
      current_bit++;
    }
  }

  // Step 2: Connect any remaining unconnected pins from the first small node
  // (e.g., power, ground, or other non-multibit pins)
  for (auto big_iterm : big_inst->getITerms()) {
    if (big_iterm->isConnected()) {
      continue;
    }
    auto small_iterm = small_nodes[0]->getDbInst()->findITerm(
        big_iterm->getMTerm()->getName().c_str());
    if (small_iterm == nullptr) {
      continue;
    }
    if (small_iterm->getNet() == nullptr) {
      continue;
    }
    big_iterm->connect(small_iterm->getNet());
  }
  big_inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
  network_->addNode(big_inst);
  auto big_node = network_->getNode(big_inst);
  {
    SwapCellsAction action;
    action.setRemovedCells(small_nodes);
    action.setAddedCell(big_node);
    journal_->addAction(action);
  }
  big_node->setLeft(left);
  big_node->setBottom(bottom);
  big_inst->setLocation(
      big_node->getLeft().v + opendp_->getGrid()->getCore().xMin(),
      big_node->getBottom().v + opendp_->getGrid()->getCore().yMin());
  big_node->setGroupId(small_nodes[0]->getGroupId());
  big_node->setGroup(small_nodes[0]->getGroup());
  big_node->setRegion(small_nodes[0]->getRegion());  // removed
  big_node->setPlaced(false);
  // because of checkRegionOverLap
  for (auto iterm : big_inst->getITerms()) {
    if (iterm->getNet() == nullptr) {
      continue;
    }
    auto edge = network_->getEdge(iterm->getNet());
    if (edge == nullptr) {
      continue;
    }
    auto pin = network_->addPin(iterm);
    network_->connect(pin, big_node);
    network_->connect(pin, edge);
  }
  for (auto node : small_nodes) {
    dismantleNode(node);
  }
  bool success = opendp_->mapMove(big_node);
  if (!success) {
    success = opendp_->shiftMove(big_node);
  }
  return success;
}

void Annealer::assignToNodeGroup(Node* node)
{
  if (node->isFixed() || !node->isPlaced() || node->isToBeRemoved()) {
    return;
  }
  if (node->getMaster()->getFunction() == nullptr) {
    return;
  }
  auto node_master = node->getMaster();
  auto function = node_master->getFunction();
  if (node_master->getFunctionBits() == function->getMaxBits()) {
    return;
  }
  auto db_inst = node->getDbInst();

  // Try to find an existing compatible group for this node
  for (auto& node_group : node_groups_) {
    auto group_head = node_group[0];
    auto group_inst = group_head->getDbInst();
    // Check basic compatibility: same group, region, and function
    if (group_inst->getGroup() != db_inst->getGroup()) {
      continue;
    }
    if (group_head->getRegion() != node->getRegion()) {
      continue;
    }
    if (group_head->getMaster()->getFunction() != function) {
      continue;
    }

    // Check net connectivity equivalence for non-multibit pins
    bool equivalent = true;
    for (auto iterm : db_inst->getITerms()) {
      bool skip_iterm = false;
      const std::string term_name = iterm->getMTerm()->getName();
      // Skip multibit pins (they will be handled differently during swapping)
      for (const auto& [single_bit, pattern] : function->getMultibitPinMap()) {
        if (term_name == single_bit || Utility::match(term_name, pattern)) {
          skip_iterm = true;
          break;
        }
      }
      if (skip_iterm) {
        continue;
      }
      // For non-multibit pins, ensure identical net connectivity
      auto head_iterm
          = group_inst->findITerm(iterm->getMTerm()->getName().c_str());
      if (head_iterm == nullptr) {
        equivalent = false;
        break;
      }
      if (iterm->getNet() != head_iterm->getNet()) {
        equivalent = false;
        break;
      }
    }
    if (equivalent) {
      node_group.push_back({node});
      return;
    }
  }
  // No compatible group found, create a new group
  node_groups_.push_back({node});
}

void Annealer::start()
{
  journal_ = new Journal(opendp_->getGrid(), nullptr);
  opendp_->setJournal(journal_);
  node_groups_.clear();
  for (size_t i = 0; i < network_->getNumNodes(); i++) {
    auto node = network_->getNode(i);
    assignToNodeGroup(node);
  }
  for (int iter = 0; iter < max_iterations_; ++iter) {
    if (iter % 100 == 0) {
      logger_->info(utl::DPL, 501, "Annealing iteration: {}", iter);
    }
    if (node_groups_.empty()) {
      logger_->info(
          utl::DPL, 502, "Annealing completed: no more groups to process.");
      break;
    }
    journal_->clearJournal();
    int group_idx;
    std::vector<int> sub_group;
    if (generate_neighbor(group_idx, sub_group)) {
      auto delta_cost = hpwl_evaluator_->delta(*journal_);
      if (accept(-delta_cost)) {
        hpwl_evaluator_->accept();
        // accept the new configuration
        current_cost_ -= delta_cost;
        if (current_cost_ < best_cost_) {
          best_cost_ = current_cost_;
        }
        auto node
            = network_->getNode(network_->getNumNodes()
                                - 1);  // the last added node is the new node

        assignToNodeGroup(node);
        // update the node groups
        auto& group = node_groups_[group_idx];
        // find if group is full of removed nodes
        bool all_removed = true;
        for (const auto& node : group) {
          if (node->isToBeRemoved()) {
            continue;
          }
          all_removed = false;
          break;
        }

        if (all_removed) {
          node_groups_.erase(node_groups_.begin() + group_idx);
        }
      } else {
        // reject the new configuration
        journal_->undoAll();
      }
    } else {
      journal_->undoAll();
    }
    temperature_ *= alpha_;
  }
}

bool Annealer::generate_neighbor(int& group_idx, std::vector<int>& sub_group)
{
  using idx_range = boost::random::uniform_int_distribution<int>;
  group_idx = idx_range(0, node_groups_.size() - 1)(generator_);
  // TODO: implement neighbor generation (e.g., swap cells or move cell)
  auto group = node_groups_[group_idx];
  int node_idx;
  Node* node;
  // Find a valid seed node that hasn't been marked for removal
  do {
    node_idx = idx_range(0, group.size() - 1)(generator_);
    node = group[node_idx];
  } while (node->isToBeRemoved());

  auto node_master = node->getMaster();
  auto function = node_master->getFunction();
  auto node_bits = node_master->getFunctionBits();
  auto max_bits = function->getMaxBits();
  // Calculate how many size levels we can go up (node_bits and max_bits are
  // powers of 2)
  int dist = std::log2(max_bits) - std::log2(node_bits);
  auto bits_to_master = function->getMasters();
  // Randomly select a target size that's larger than current (1 to dist levels
  // up)
  auto upper_size_idx = idx_range(1, dist)(generator_);
  auto masters = function->getMasters();
  auto it = bits_to_master.find(node_bits);
  std::advance(it, upper_size_idx);
  auto target_master = it->second;

  // Find the best combination of nodes to merge with the seed node
  sub_group = findBestGroup(group[node_idx],
                            target_master->getBBox(),
                            group,
                            target_master->getFunctionBits());
  sub_group.push_back(node_idx);
  std::stable_sort(sub_group.begin(), sub_group.end());

  // Verify that the selected nodes have exactly the required number of bits
  int total_bits = 0;
  for (auto idx : sub_group) {
    total_bits += group[idx]->getMaster()->getFunctionBits();
  }
  if (total_bits != target_master->getFunctionBits()) {
    // not enough cells to swap
    return false;
  }
  std::vector<Node*> nodes_to_swap;
  nodes_to_swap.reserve(sub_group.size());
  for (int idx : sub_group) {
    nodes_to_swap.emplace_back(group[idx]);
  }
  return swapNodes(nodes_to_swap, target_master);
}

bool Annealer::accept(double delta_cost)
{
  // delta_cost = new_wl - old_wl
  const float num = distribution_(generator_);
  // Metropolis criterion: always accept improvements (delta_cost <= 0),
  // probabilistically accept worse solutions based on exp(-ΔE/T)
  const float prob
      = (delta_cost > 0.0) ? std::exp((-1) * delta_cost / temperature_) : 1;
  return prob > num;
}

}  // namespace dpl