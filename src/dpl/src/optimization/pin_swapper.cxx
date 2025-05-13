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
}  // namespace
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

PinSwapper::SwapGroup PinSwapper::swapPins(const std::vector<Pin*>& pins1,
                                           const std::vector<Pin*>& pins2)
{
  SwapGroup swaps;
  // swap pins
  for (size_t i = 0; i < pins1.size(); ++i) {
    swaps.push_back(swapPins(pins1[i], pins2[i]));
  }
  return swaps;
}

void PinSwapper::swapPins(Node* node)
{
  auto function = node->getMaster()->getFunction();
  auto swappable_pins = function->getFunctionPins();

  // Map to temporarily store pins by their index
  std::map<int, std::vector<Pin*>> index_to_pins;

  // Go through all pins of the node
  for (Pin* pin : node->getPins()) {
    if (auto iterm = pin->getDbITerm()) {
      std::string pin_name = iterm->getMTerm()->getName();

      // Check if this pin matches any of our patterns
      for (const auto& pattern : swappable_pins) {
        if (startsWith(pin_name, pattern)) {
          // Extract the index number from the pin name
          int idx = extractPinIdx(pin_name, pattern);
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
        journal_->clearJournal();
        auto swaps = swapPins(pin_groups[i], pin_groups[j]);
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
        } else {
          journal_->undoAll();
        }
      }
    }
  }
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
    if (node->getMaster()->getFunction() == nullptr) {
      continue;
    }
    if (node->getMaster()->getFunctionBits() <= 1) {
      continue;
    }
    swapPins(node.get());
    nodes_.push_back(node.get());
  }
  logger_->report("Pin swapper found {} improvements", count_);
  delete journal_;
}
}  // namespace dpl