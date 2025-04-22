// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors
#include "annealer.h"

#include "dpl/Opendp.h"
#include "infrastructure/Grid.h"
#include "infrastructure/network.h"
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
}
void Annealer::addEquivalentCells(Equivalence entry)
{
  equivalence_list_.emplace_back(entry);
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
int findBestPair(const Node* small_node,
                 const odb::Rect& big_rect,
                 const std::vector<Node*>& group)
{
  const auto small_rect = small_node->getBBox();
  const auto target_dx = big_rect.dx();
  const auto target_dy = big_rect.dy();

  int best_dxdy = std::numeric_limits<int>::max();
  int best_idx = -1;
  int i = 0;
  for (const auto& node : group) {
    if (node == small_node) {
      ++i;
      continue;
    }
    auto rect = node->getBBox();
    rect.merge(small_rect, rect);
    const auto dx_diff = std::abs(rect.dx() - target_dx);
    const auto dy_diff = std::abs(rect.dy() - target_dy);
    const auto dxdy = dx_diff + dy_diff;
    if (dxdy < best_dxdy) {
      best_dxdy = dxdy;
      best_idx = i;
    }
    ++i;
  }
  return best_idx;
}
}  // namespace

void Annealer::dismantleNode(Node* node)
{
  opendp_->getGrid()->erasePixel(node);
  for (auto pin : node->getPins()) {
    auto edge = pin->getEdge();
    edge->removePin(pin);
  }
  node->setFixed(true);
  auto db_inst = node->getDbInst();
  db_inst->setPlacementStatus(odb::dbPlacementStatus::UNPLACED);
  for (auto iterm : db_inst->getITerms()) {
    iterm->disconnect();
  }
  db_inst->setUserFlag1();
}

bool Annealer::swapNodes(std::vector<Node*> small_nodes, Equivalence entry)
{
  auto block = db_->getChip()->getBlock();
  auto core = block->getCoreArea();
  auto big_inst
      = odb::dbInst::create(block,
                            entry.getBigMaster()->getDbMaster(),
                            fmt::format("osama_{}", last_id_++).c_str(),
                            small_nodes[0]->getDbInst()->getRegion());
  int n = 1;
  DbuX left{0};
  DbuY bottom{0};
  for (auto node : small_nodes) {
    left += node->getLeft();
    bottom += node->getBottom();
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
        logger_->report("Connecting {} to net {} instead of {}",
                        big_iterm->getName(),
                        big_iterm->getNet()->getName(),
                        iterm->getName());
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
  big_node->setLeft(left / DbuX{small_nodes.size()});
  big_node->setBottom(bottom / DbuY{small_nodes.size()});
  big_inst->setLocation(
      big_node->getLeft().v + opendp_->getGrid()->getCore().xMin(),
      big_node->getBottom().v + opendp_->getGrid()->getCore().yMin());
  big_node->setGroupId(small_nodes[0]->getGroupId());
  big_node->setGroup(small_nodes[0]->getGroup());
  // big_node->setRegion(&(small_nodes[0]->getGroup()->getBBox())); // removed
  // because of checkRegionOverLap
  for (auto iterm : big_inst->getITerms()) {
    logger_->report("Handling pin {} with net {}",
                    iterm->getName(),
                    iterm->getNet()->getName());
    auto pin = network_->addPin(iterm);
    network_->connect(pin, big_node);
  }
  for (auto node : small_nodes) {
    dismantleNode(node);
  }
  bool map_move = opendp_->mapMove(big_node);
  return map_move;
}

void Annealer::assignToNodeGroup(Node* node, Equivalence entry)
{
  auto db_inst = node->getDbInst();
  for (auto& node_group : node_groups_) {
    auto group_head = node_group[0];
    auto group_inst = group_head->getDbInst();
    if (group_inst->getGroup() != db_inst->getGroup()) {
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
      node_group.emplace_back(node);
      return;
    }
  }
  node_groups_.emplace_back(node);
}

void Annealer::start()
{
  for (auto entry : equivalence_list_) {
    node_groups_.clear();
    for (size_t i = 0; i < network_->getNumNodes(); i++) {
      auto node = network_->getNode(i);
      if (node->getMaster() == entry.getSmallMaster()) {
        assignToNodeGroup(node, entry);
      }
    }
    for (auto& group : node_groups_) {
      for (size_t i = 0; i < group.size();) {
        const auto best_pair_idx
            = findBestPair(group[i], entry.getBigMaster()->getBBox(), group);
        if (best_pair_idx >= 0) {
          swapNodes({group[i], group[best_pair_idx]}, entry);
          group.erase(group.begin() + best_pair_idx);
          group.erase(group.begin() + i);
        } else {
          ++i;
        }
      }
    }
    break;
  }
}

}  // namespace dpl