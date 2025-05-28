// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025-2025, The OpenROAD Authors

#pragma once

#include <cstddef>
#include <set>
#include <vector>

#include "infrastructure/network.h"
namespace dpl {
class Grid;

class DetailedMgr;
enum JournalActionTypeEnum
{
  MOVE_CELL,
  UNPLACE_CELL,
  SWAP_CELLS,
  SWAP_PINS
};
class JournalAction
{
 public:
  virtual JournalActionTypeEnum typeId() const = 0;

 protected:
};
class MoveCellAction : public JournalAction
{
 public:
  MoveCellAction() = default;
  void setOrigLocation(const DbuX x, const DbuY y)
  {
    orig_x_ = x;
    orig_y_ = y;
  }
  void setNewLocation(const DbuX x, const DbuY y)
  {
    new_x_ = x;
    new_y_ = y;
  }
  void setOrigSegs(const std::vector<int>& segs) { orig_segs_ = segs; }
  void setNewSegs(const std::vector<int>& segs) { new_segs_ = segs; }
  void setNode(Node* node) { node_ = node; }
  void setWasPlaced(bool was_placed) { was_placed_ = was_placed; }
  // getters
  Node* getNode() const { return node_; }
  DbuX getOrigLeft() const { return orig_x_; }
  DbuY getOrigBottom() const { return orig_y_; }
  DbuX getNewLeft() const { return new_x_; }
  DbuY getNewBottom() const { return new_y_; }
  const std::vector<int>& getOrigSegs() const { return orig_segs_; }
  const std::vector<int>& getNewSegs() const { return new_segs_; }
  bool wasPlaced() const { return was_placed_; }
  JournalActionTypeEnum typeId() const override
  {
    return JournalActionTypeEnum::MOVE_CELL;
  }

 private:
  Node* node_{nullptr};
  DbuX orig_x_{0};
  DbuY orig_y_{0};
  DbuX new_x_{0};
  DbuY new_y_{0};
  std::vector<int> orig_segs_;
  std::vector<int> new_segs_;
  bool was_placed_{true};
};
class UnplaceCellAction : public JournalAction
{
 public:
  UnplaceCellAction() = default;
  void setNode(Node* node) { node_ = node; }
  Node* getNode() const { return node_; }
  void setWasHold(bool was_hold) { was_hold_ = was_hold; }
  bool wasHold() const { return was_hold_; }
  JournalActionTypeEnum typeId() const override
  {
    return JournalActionTypeEnum::UNPLACE_CELL;
  }

 private:
  Node* node_{nullptr};
  bool was_hold_{false};
};
class SwapCellsAction : public JournalAction
{
 public:
  SwapCellsAction() = default;
  void setRemovedCells(const std::vector<Node*>& cells)
  {
    removed_cells_ = cells;
  }
  void setAddedCell(Node* cell) { added_cell_ = cell; }
  JournalActionTypeEnum typeId() const override
  {
    return JournalActionTypeEnum::SWAP_CELLS;
  }
  const std::vector<Node*>& getRemovedCells() const { return removed_cells_; }
  Node* getAddedCell() const { return added_cell_; }

 private:
  std::vector<Node*> removed_cells_;
  Node* added_cell_;
};
class SwapPinsAction : public JournalAction
{
 public:
  SwapPinsAction(Pin* pin1, Pin* pin2) : pin1_(pin1), pin2_(pin2) {}
  JournalActionTypeEnum typeId() const override
  {
    return JournalActionTypeEnum::SWAP_PINS;
  }
  Pin* getPin1() const { return pin1_; }
  Pin* getPin2() const { return pin2_; }

 private:
  Pin* pin1_{nullptr};
  Pin* pin2_{nullptr};
};

class Journal
{
 public:
  Journal(Grid* grid, DetailedMgr* mgr) : grid_(grid), mgr_(mgr) {}
  // setters
  void addAction(const MoveCellAction& action)
  {
    affected_nodes_.insert(action.getNode());
    for (auto pin : action.getNode()->getPins()) {
      affected_edges_.insert(pin->getEdge());
    }
    actions_.push_back(std::make_unique<MoveCellAction>(action));
  }
  void addAction(const UnplaceCellAction& action)
  {
    affected_nodes_.insert(action.getNode());
    for (auto pin : action.getNode()->getPins()) {
      affected_edges_.insert(pin->getEdge());
    }
    actions_.push_back(std::make_unique<UnplaceCellAction>(action));
  }
  void addAction(const SwapCellsAction& action)
  {
    for (auto pin : action.getAddedCell()->getPins()) {
      if (pin->getEdge() != nullptr) {
        affected_edges_.insert(pin->getEdge());
      }
    }
    actions_.push_back(std::make_unique<SwapCellsAction>(action));
  }
  void addAction(const SwapPinsAction& action)
  {
    actions_.push_back(std::make_unique<SwapPinsAction>(action));
    affected_nodes_.insert(action.getPin1()->getNode());
    affected_nodes_.insert(action.getPin2()->getNode());
    affected_edges_.insert(action.getPin1()->getEdge());
    affected_edges_.insert(action.getPin2()->getEdge());
  }
  // getters
  JournalAction* getLastAction() const { return actions_.back().get(); }
  bool isEmpty() const { return actions_.empty(); }
  size_t size() const { return actions_.size(); }
  const std::set<Node*>& getAffectedNodes() const { return affected_nodes_; }
  const std::set<Edge*>& getAffectedEdges() const { return affected_edges_; }
  const std::vector<std::unique_ptr<JournalAction>>& getActions() const
  {
    return actions_;
  }
  // other
  void removeLastAction() { actions_.pop_back(); }
  void clearJournal();
  void undo(const JournalAction* action, bool positions_only = false) const;
  void redo(const JournalAction* action, bool positions_only = false) const;
  void undoAll();

 private:
  Grid* grid_{nullptr};
  DetailedMgr* mgr_{nullptr};
  std::vector<std::unique_ptr<JournalAction>> actions_;
  std::set<Node*> affected_nodes_;
  std::set<Edge*> affected_edges_;
};

}  // namespace dpl
