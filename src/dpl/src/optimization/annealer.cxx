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
#include "utl/Logger.h"
namespace dpl {
namespace {
bool startsWith(const std::string& str, const std::string& prefix)
{
  return str.find(prefix, 0) == 0;
}
int extractPinIdx(const std::string& str, const std::string& prefix)
{
  auto idx = str.substr(prefix.size());
  if (idx.empty()) {
    return 1;
  }
  return std::stoi(idx);
}

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

// finds the best set of (k_total - 1) other cells to merge with 'seed'
// returns the indices within 'group' of the chosen cells
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
  std::string new_name = fmt::format("osama_{}", last_id_++);
  while (block->findInst(new_name.c_str()) != nullptr) {
    new_name = fmt::format("osama_{}", last_id_++);
  }
  auto function = target_master->getFunction();
  auto big_inst = odb::dbInst::create(block,
                                      target_master->getDbMaster(),
                                      new_name.c_str(),
                                      small_nodes[0]->getDbInst()->getRegion());
  int added_bits = 0;
  DbuX left = std::numeric_limits<DbuX>::max();
  DbuY bottom = std::numeric_limits<DbuY>::max();
  for (auto node : small_nodes) {
    left = std::min(left, node->getLeft());
    bottom = std::min(bottom, node->getBottom());
    auto inst = node->getDbInst();
    for (auto iterm : inst->getITerms()) {
      auto name = iterm->getMTerm()->getName();
      bool swappable = false;
      for (const auto& pin_name : function->getFunctionPins()) {
        if (!startsWith(name, pin_name)) {
          continue;
        }
        auto idx = extractPinIdx(name, pin_name);
        std::string big_iterm_name
            = pin_name + std::to_string(idx + added_bits);
        auto big_iterm = big_inst->findITerm(big_iterm_name.c_str());
        big_iterm->connect(iterm->getNet());
        swappable = true;
        break;
      }
      if (swappable) {
        continue;
      }
      auto big_iterm = big_inst->findITerm(name.c_str());
      if (big_iterm->isConnected()) {
        continue;
      }
      big_iterm->connect(iterm->getNet());
    }
    added_bits += node->getMaster()->getFunctionBits();
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
  for (auto& node_group : node_groups_) {
    auto group_head = node_group[0];
    auto group_inst = group_head->getDbInst();
    if (group_inst->getGroup() != db_inst->getGroup()) {
      continue;
    }
    if (group_head->getRegion() != node->getRegion()) {
      continue;
    }
    if (group_head->getMaster()->getFunction() != function) {
      continue;
    }
    bool equivalent = true;

    for (auto iterm : db_inst->getITerms()) {
      bool skip_iterm = false;
      for (const auto& pin_name : function->getFunctionPins()) {
        if (startsWith(iterm->getMTerm()->getName(), pin_name)) {
          skip_iterm = true;
          break;
        }
      }
      if (skip_iterm) {
        continue;
      }
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
  do {
    node_idx = idx_range(0, group.size() - 1)(generator_);
    node = group[node_idx];
  } while (node->isToBeRemoved());
  auto node_master = node->getMaster();
  auto function = node_master->getFunction();
  auto node_bits = node_master->getFunctionBits();
  auto max_bits = function->getMaxBits();
  // node_bits and max_bits are in base 2, get their log2 distance
  int dist = std::log2(max_bits) - std::log2(node_bits);
  auto bits_to_master = function->getMasters();
  auto upper_size_idx = idx_range(1, dist)(generator_);
  auto masters = function->getMasters();
  auto it = bits_to_master.find(node_bits);
  std::advance(it, upper_size_idx);
  auto target_master = it->second;
  sub_group = findBestGroup(group[node_idx],
                            target_master->getBBox(),
                            group,
                            target_master->getFunctionBits());
  sub_group.push_back(node_idx);
  std::stable_sort(sub_group.begin(), sub_group.end());
  // count total number of bits
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
  const float prob
      = (delta_cost > 0.0) ? std::exp((-1) * delta_cost / temperature_) : 1;
  return prob > num;
}

}  // namespace dpl