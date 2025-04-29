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
Equivalence::Equivalence(Master* small_master, Master* big_master, int ratio)
    : small_master_(small_master), big_master_(big_master), ratio_(ratio)
{
}

Master* Equivalence::getBigMaster() const
{
  return big_master_;
}

Master* Equivalence::getSmallMaster() const
{
  return small_master_;
}

int Equivalence::getRatio() const
{
  return ratio_;
}

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
void Annealer::addEquivalentCells(Equivalence entry)
{
  const size_t idx = equivalence_list_.size();
  equivalence_list_.emplace_back(entry);
  master_to_equivalence_[entry.getSmallMaster()].emplace_back(idx);
  // figure if there is other equivalence that can be calculated from the given
  // one and the list of already added equivalences
  const size_t size = equivalence_list_.size();
  for (size_t i = 0; i < size - 1; ++i) {
    auto& eq = equivalence_list_[i];
    if (eq.getBigMaster() == entry.getSmallMaster()) {
      Equivalence new_entry(eq.getSmallMaster(),
                            entry.getBigMaster(),
                            eq.getRatio() * entry.getRatio());
      new_entry.setSwappablePins(entry.getSwappablePins());
      master_to_equivalence_[eq.getSmallMaster()].emplace_back(
          equivalence_list_.size());
      equivalence_list_.emplace_back(new_entry);
    } else if (eq.getSmallMaster() == entry.getSmallMaster()) {
      if (eq.getRatio() > entry.getRatio()) {
        Equivalence new_entry(entry.getBigMaster(),
                              eq.getBigMaster(),
                              eq.getRatio() / entry.getRatio());
        new_entry.setSwappablePins(entry.getSwappablePins());
        master_to_equivalence_[new_entry.getSmallMaster()].emplace_back(
            equivalence_list_.size());
        equivalence_list_.emplace_back(new_entry);
      } else if (eq.getRatio() < entry.getRatio()) {
        Equivalence new_entry(eq.getBigMaster(),
                              entry.getBigMaster(),
                              entry.getRatio() / eq.getRatio());
        new_entry.setSwappablePins(entry.getSwappablePins());
        master_to_equivalence_[new_entry.getSmallMaster()].emplace_back(
            equivalence_list_.size());
        equivalence_list_.emplace_back(new_entry);
      }
    }
  }
}
namespace {
bool startsWith(const std::string& str, const std::string& prefix)
{
  if (prefix.size() > str.size())
    return false;
  return str.substr(0, prefix.size()) == prefix;
}
int extractPinIdx(const std::string& str, const std::string& prefix)
{
  auto idx = str.substr(prefix.size());
  if (idx.size() == 0) {
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
    if (node == seed)
      continue;

    const auto& r = node->getBBox();
    int x0 = std::min(ux_min, r.xMin());
    int y0 = std::min(uy_min, r.yMin());
    int x1 = std::max(ux_max, r.xMax());
    int y1 = std::max(uy_max, r.yMax());

    // partial score pruning using dimension metric
    int w = x1 - x0;
    int h = y1 - y0;
    int partial = std::abs(w - tgt_w) + std::abs(h - tgt_h);
    if (partial >= best_score)
      continue;

    current.push_back(i);
    dfs_helper(seed,
               tgt_w,
               tgt_h,
               group,
               i + 1,
               remain - 1,
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
  int need = k_total - 1;

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

bool Annealer::swapNodes(std::vector<Node*> small_nodes, Equivalence entry)
{
  std::shuffle(small_nodes.begin(), small_nodes.end(), generator_);
  auto block = db_->getChip()->getBlock();
  std::string new_name = fmt::format("osama_{}", last_id_++);
  while (block->findInst(new_name.c_str()) != nullptr) {
    new_name = fmt::format("osama_{}", last_id_++);
  }
  auto big_inst = odb::dbInst::create(block,
                                      entry.getBigMaster()->getDbMaster(),
                                      new_name.c_str(),
                                      small_nodes[0]->getDbInst()->getRegion());
  int n = 1;
  DbuX left = std::numeric_limits<DbuX>::max();
  DbuY bottom = std::numeric_limits<DbuY>::max();
  for (auto node : small_nodes) {
    left = std::min(left, node->getLeft());
    bottom = std::min(bottom, node->getBottom());
    auto inst = node->getDbInst();
    for (auto iterm : inst->getITerms()) {
      auto name = iterm->getMTerm()->getName();
      bool swappable = false;
      for (auto pin_name : entry.getSwappablePins()) {
        if (!startsWith(name, pin_name)) {
          continue;
        }
        auto idx = extractPinIdx(name, pin_name);
        std::string big_iterm_name = pin_name + std::to_string(idx * n);
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
    ++n;
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

void Annealer::assignToNodeGroup(Node* node, Equivalence entry)
{
  auto db_inst = node->getDbInst();
  int group_idx = -1;
  for (auto& node_group : node_groups_) {
    ++group_idx;
    auto group_head = node_group[0];
    auto group_inst = group_head->getDbInst();
    if (group_inst->getGroup() != db_inst->getGroup()) {
      continue;
    }
    if (group_head->getRegion() != node->getRegion()) {
      continue;
    }
    bool equivalent = true;
    for (int iterm_idx = 0; iterm_idx < db_inst->getITerms().size();
         iterm_idx++) {
      auto iterm = db_inst->getITerm(iterm_idx);
      auto head_iterm = group_inst->getITerm(iterm_idx);
      bool skip_iterm = false;
      for (auto pin_name : entry.getSwappablePins()) {
        if (startsWith(iterm->getMTerm()->getName(), pin_name)) {
          skip_iterm = true;
          break;
        }
      }
      if (skip_iterm) {
        continue;
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
    if (node->isFixed() || !node->isPlaced() || node->isToBeRemoved()) {
      continue;
    }
    auto master = node->getMaster();
    if (master_to_equivalence_.find(master) == master_to_equivalence_.end()) {
      continue;
    }
    assignToNodeGroup(node,
                      equivalence_list_[master_to_equivalence_[master][0]]);
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
      if (accept(-delta_cost)) {  // accept the new configuration
        hpwl_evaluator_->accept();
        // accept the new configuration
        current_cost_ -= delta_cost;
        if (current_cost_ < best_cost_) {
          best_cost_ = current_cost_;
        }
        // update the node groups
        auto& group = node_groups_[group_idx];
        for (int j = sub_group.size() - 1; j >= 0; --j) {
          group.erase(group.begin() + sub_group[j]);
        }
        auto node
            = network_->getNode(network_->getNumNodes()
                                - 1);  // the last added node is the new node
        if (master_to_equivalence_.find(node->getMaster())
            != master_to_equivalence_.end()) {
          assignToNodeGroup(
              node,
              equivalence_list_[master_to_equivalence_[node->getMaster()][0]]);
        }
        if (group.size() == 0) {
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
  auto node_idx = idx_range(0, group.size() - 1)(generator_);
  auto node = group[node_idx];
  auto eq_list = master_to_equivalence_[node->getMaster()];
  auto entry_idx = idx_range(0, eq_list.size() - 1)(generator_);
  auto entry = equivalence_list_[eq_list[entry_idx]];
  sub_group = findBestGroup(group[node_idx],
                            entry.getBigMaster()->getBBox(),
                            group,
                            entry.getRatio());
  sub_group.push_back(node_idx);
  std::stable_sort(sub_group.begin(), sub_group.end());
  if (sub_group.size() != entry.getRatio()) {
    // not enough cells to swap
    return false;
  }
  std::vector<Node*> nodes_to_swap;
  for (int idx : sub_group) {
    nodes_to_swap.push_back(group[idx]);
  }
  return swapNodes(nodes_to_swap, entry);
}

bool Annealer::accept(double delta_cost)
{
  const float num = distribution_(generator_);
  const float prob
      = (delta_cost > 0.0) ? std::exp((-1) * delta_cost / temperature_) : 1;
  return prob > num;
}

}  // namespace dpl