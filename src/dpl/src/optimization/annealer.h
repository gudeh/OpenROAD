// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors
#pragma once
#include <string>
#include <vector>
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
class Equivalence
{
 public:
  Equivalence(Master* small_master, Master* big_master, int ratio);
  Master* getSmallMaster() const;
  Master* getBigMaster() const;
  int getRatio() const;
  void addSwappablePin(std::string pin_name)
  {
    swapable_pins_.emplace_back(pin_name);
  }
  std::vector<std::string> getSwappablePins() { return swapable_pins_; }

 private:
  Master* small_master_{nullptr};
  Master* big_master_{nullptr};
  std::vector<std::string> swapable_pins_;
  int ratio_{0};
};
class Annealer
{
 public:
  Annealer(utl::Logger* logger,
           Opendp* opendp,
           Network* network,
           odb::dbDatabase* db);
  void addEquivalentCells(Equivalence entry);
  void start();

 private:
  bool swapNodes(std::vector<Node*> small_nodes, Equivalence entry);
  void dismantleNode(Node* node);
  void assignToNodeGroup(Node* node, Equivalence entry);
  utl::Logger* logger_{nullptr};
  Opendp* opendp_{nullptr};
  Network* network_{nullptr};
  odb::dbDatabase* db_{nullptr};
  std::vector<Equivalence> equivalence_list_;
  std::vector<std::vector<Node*>> node_groups_;
  int last_id_{0};
};
}  // namespace dpl