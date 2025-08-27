// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors

#include "pin_swapper.h"

#include <map>
#include <string>
#include <vector>

#include "infrastructure/network.h"
#include "objective/detailed_hpwl.h"
#include "odb/db.h"
#include "util/journal.h"
#include "utl/Logger.h"

namespace dpl {
PinSwapper::PinSwapper(utl::Logger* logger,
                       Network* network,
                       odb::dbDatabase* db)
    : logger_(logger), network_(network), db_(db)
{
  hpwl_evaluator_ = new DetailedHPWL(network);
  hpwl_evaluator_->init();
  current_cost_ = hpwl_evaluator_->curr();
  best_cost_ = current_cost_;
}

PinSwapper::Swap PinSwapper::swapPins(Pin* pin1, Pin* pin2)
{
  SwapPinsAction action(pin1, pin2);
  journal_->addAction(action);
  auto edge1 = pin1->getEdge();
  auto edge2 = pin2->getEdge();
  edge1->removePin(pin1);
  edge2->removePin(pin2);
  pin1->setEdge(edge2);
  pin2->setEdge(edge1);
  edge1->addPin(pin2);
  edge2->addPin(pin1);

  return std::make_pair(pin1, pin2);
}

bool PinSwapper::swapPins(const std::vector<Pin*>& pins1,
                          const std::vector<Pin*>& pins2)
{
  journal_->clear();
  SwapGroup swaps;
  // swap pins
  for (size_t i = 0; i < pins1.size(); ++i) {
    swaps.push_back(swapPins(pins1[i], pins2[i]));
  }
  auto delta_cost = hpwl_evaluator_->delta(*journal_);
  if (accept(-delta_cost)) {
    count_++;
    hpwl_evaluator_->accept();
    current_cost_ -= delta_cost;
    if (current_cost_ < best_cost_) {
      best_cost_ = current_cost_;
    }
    for (const auto& swap : swaps) {
      auto iterm1 = swap.first->getDbITerm();
      auto iterm2 = swap.second->getDbITerm();
      auto net1 = iterm1->getNet();
      auto net2 = iterm2->getNet();
      iterm1->disconnect();
      iterm2->disconnect();
      iterm1->connect(net2);
      iterm2->connect(net1);
    }
    return true;
  }
  journal_->undo();
  journal_->clear();
  return false;
}

bool PinSwapper::optimizeBitPlacement(Node* node)
{
  bool change = false;
  auto function = node->getMaster()->getFunction();
  auto swappable_pins = function->getMultibitPinMap();

  // Map to temporarily store pins by their index
  std::map<int, std::vector<Pin*>> index_to_pins;

  // Go through all pins of the node
  for (Pin* pin : node->getPins()) {
    if (auto iterm = pin->getDbITerm()) {
      std::string pin_name = iterm->getMTerm()->getName();

      // Check if this pin matches any of our patterns
      for (const auto& [single_bit, pattern] : swappable_pins) {
        if (Utility::match(pin_name, pattern)) {
          // Extract the index number from the pin name
          int idx = Utility::extractPinIdx(pin_name, pattern);
          // Store pin in the corresponding index group
          index_to_pins[idx].push_back(pin);
          break;  // Found the matching pattern, no need to check others
        }
      }
    }
  }

  // Convert map to vector of vectors, ensuring each group has all function pins
  std::vector<std::vector<Pin*>> pin_groups;
  for (auto& [idx, pins] : index_to_pins) {
    // Only add groups that have all the function pins
    if (pins.size() == swappable_pins.size()) {
      // sort pins by name
      std::sort(pins.begin(), pins.end(), [](Pin* a, Pin* b) {
        return a->getDbITerm()->getMTerm()->getName()
               < b->getDbITerm()->getMTerm()->getName();
      });
      pin_groups.push_back(pins);
    }
  }
  // try all possible swaps
  if (pin_groups.size() >= 2) {
    for (size_t i = 0; i < pin_groups.size(); ++i) {
      for (size_t j = 0; j < pin_groups.size(); ++j) {
        if (i == j) {
          continue;
        }
        change |= swapPins(pin_groups[i], pin_groups[j]);
      }
    }
  }
  return change;
}

bool PinSwapper::optimizePinSwaps(Node* node)
{
  auto master = node->getMaster();
  auto pin_swaps = master->getPinSwaps();
  bool change = false;
  for (int current_bit = master->getLsb(); current_bit <= master->getMsb();
       current_bit++) {
    for (const auto& swap_group : pin_swaps) {
      std::vector<std::vector<Pin*>> pin_groups(2);
      for (const auto& [pin1_name, pin2_name] : swap_group) {
        auto pin1 = node->getPin(Utility::getPinName(pin1_name, current_bit));
        auto pin2 = node->getPin(Utility::getPinName(pin2_name, current_bit));
        pin_groups[0].push_back(pin1);
        pin_groups[1].push_back(pin2);
      }
      change |= swapPins(pin_groups[0], pin_groups[1]);
    }
  }
  return change;
}

bool PinSwapper::optimizePinPermutes(Node* node)
{
  bool change = false;
  auto master = node->getMaster();
  auto pin_permutes = master->getPinPermutes();
  for (int current_bit = master->getLsb(); current_bit <= master->getMsb();
       current_bit++) {
    std::vector<Pin*> base_pins;
    for (const auto& pin_name : pin_permutes[0]) {
      base_pins.push_back(
          node->getPin(Utility::getPinName(pin_name, current_bit)));
    }
    for (int permute = 1; permute < pin_permutes.size(); permute++) {
      std::vector<Pin*> swap_pins;
      for (const auto& pin_name : pin_permutes[permute]) {
        swap_pins.push_back(
            node->getPin(Utility::getPinName(pin_name, current_bit)));
      }
      std::vector<std::vector<Pin*>> pin_groups(2);
      std::set<std::pair<Pin*, Pin*>> pin_set;
      for (int i = 0; i < base_pins.size(); i++) {
        auto pin1 = base_pins[i];
        auto pin2 = swap_pins[i];
        if (pin1 == pin2) {
          continue;
        }
        if (pin1->getDbITerm()->getMTerm()->getName()
            > pin2->getDbITerm()->getMTerm()->getName()) {
          std::swap(pin1, pin2);
        }
        pin_set.insert(std::make_pair(pin1, pin2));
      }
      for (const auto& [pin1, pin2] : pin_set) {
        pin_groups[0].push_back(pin1);
        pin_groups[1].push_back(pin2);
      }
      change |= swapPins(pin_groups[0], pin_groups[1]);
    }
  }
  return change;
}

bool PinSwapper::accept(double delta_cost)
{
  return delta_cost < 0;
}
void PinSwapper::start()
{
  journal_ = new Journal(nullptr, nullptr);
  for (auto& node : network_->getNodes()) {
    if (!node->isStdCell()) {
      continue;
    }
    // brute force optimization on each node
    bool change;
    do {
      change = false;
      if (node->getMaster()->isMultibit()) {
        change |= optimizeBitPlacement(node.get());
      }
      if (node->getMaster()->hasPinSwaps()) {
        change |= optimizePinSwaps(node.get());
      }
      if (node->getMaster()->hasPinPermutes()) {
        change |= optimizePinPermutes(node.get());
      }
    } while (change);
  }
  logger_->report("Pin swapper found {} improvements", count_);
  delete journal_;
}
}  // namespace dpl