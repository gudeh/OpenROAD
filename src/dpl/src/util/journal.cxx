// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025-2025, The OpenROAD Authors

#include "journal.h"

#include "infrastructure/Grid.h"
#include "optimization/detailed_manager.h"

namespace dpl {
void Journal::clearJournal()
{
  actions_.clear();
  affected_nodes_.clear();
  affected_edges_.clear();
}
namespace {
void paintInGrid(Grid* grid, Node* node)
{
  const auto grid_x = grid->gridX(DbuX(node->getLeft()));
  const auto grid_y = grid->gridRoundY(DbuY(node->getBottom()));
  auto pixel = grid->gridPixel(grid_x, grid_y);
  grid->paintPixel(node, grid_x, grid_y);
  node->adjustCurrOrient(
      pixel->sites.at(node->getDbInst()->getMaster()->getSite()));
}
void doSwapAction(const SwapPinsAction* swap_action)
{
  auto pin1 = swap_action->getPin1();
  auto pin2 = swap_action->getPin2();
  auto edge1 = pin1->getEdge();
  auto edge2 = pin2->getEdge();
  edge1->removePin(pin1);
  edge2->removePin(pin2);
  pin1->setEdge(edge2);
  pin2->setEdge(edge1);
  edge1->addPin(pin2);
  edge2->addPin(pin1);
}
};  // namespace
void Journal::undo(const JournalAction* action, const bool positions_only) const
{
  switch (action->typeId()) {
    case JournalActionTypeEnum::MOVE_CELL: {
      auto move_action = static_cast<const MoveCellAction*>(action);
      auto node = move_action->getNode();
      if (!positions_only) {
        grid_->erasePixel(node);
        for (auto seg : move_action->getNewSegs()) {
          if (seg < 0) {
            continue;
          }
          mgr_->removeCellFromSegment(node, seg);
        }
      }
      node->setLeft(move_action->getOrigLeft());
      node->setBottom(move_action->getOrigBottom());
      if (!move_action->wasPlaced()) {
        node->setPlaced(false);
        return;
      }
      if (!positions_only) {
        paintInGrid(grid_, node);
        for (auto seg : move_action->getOrigSegs()) {
          if (seg < 0) {
            continue;
          }
          mgr_->addCellToSegment(node, seg);
        }
      }
      break;
    }
    case JournalActionTypeEnum::UNPLACE_CELL: {
      auto unplace_action = static_cast<const UnplaceCellAction*>(action);
      auto node = unplace_action->getNode();
      grid_->paintPixel(node);
      node->setPlaced(true);
      node->setHold(unplace_action->wasHold());
      break;
    }
    case JournalActionTypeEnum::SWAP_CELLS: {
      auto swap_action = static_cast<const SwapCellsAction*>(action);

      const auto added_node = swap_action->getAddedCell();
      grid_->erasePixel(added_node);
      for (auto pin : added_node->getPins()) {
        if (pin->getEdge() == nullptr) {
          continue;
        }
        pin->getEdge()->removePin(pin);
      }
      added_node->setToBeRemoved(true);

      for (const auto& node : swap_action->getRemovedCells()) {
        for (auto pin : node->getPins()) {
          pin->getEdge()->addPin(pin);
        }
        grid_->paintPixel(node);
        node->setToBeRemoved(false);
      }

      break;
    }
    case JournalActionTypeEnum::SWAP_PINS: {
      doSwapAction(static_cast<const SwapPinsAction*>(action));
      break;
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void Journal::undoAll()
{
  for (auto it = actions_.rbegin(); it != actions_.rend(); ++it) {
    auto action = (*it).get();
    undo(action, false);
  }
  clearJournal();
}
////////////////////////////////////////////////////////////////////////////////
void Journal::redo(const JournalAction* action, const bool positions_only) const
{
  switch (action->typeId()) {
    case JournalActionTypeEnum::MOVE_CELL: {
      auto move_action = static_cast<const MoveCellAction*>(action);
      auto node = move_action->getNode();
      if (!positions_only) {
        grid_->erasePixel(node);
        for (auto seg : move_action->getOrigSegs()) {
          if (seg < 0) {
            continue;
          }
          mgr_->removeCellFromSegment(node, seg);
        }
      }
      node->setLeft(move_action->getNewLeft());
      node->setBottom(move_action->getNewBottom());
      if (!positions_only) {
        paintInGrid(grid_, node);
        for (auto seg : move_action->getNewSegs()) {
          if (seg < 0) {
            continue;
          }
          mgr_->addCellToSegment(node, seg);
        }
      }
      break;
    }
    case JournalActionTypeEnum::UNPLACE_CELL: {
      auto unplace_action = static_cast<const UnplaceCellAction*>(action);
      auto node = unplace_action->getNode();
      grid_->erasePixel(node);
      node->setPlaced(false);
      node->setHold(false);
      break;
    }
    case JournalActionTypeEnum::SWAP_PINS: {
      doSwapAction(static_cast<const SwapPinsAction*>(action));
      break;
    }
    default:
      break;
  }
}

}  // namespace dpl
