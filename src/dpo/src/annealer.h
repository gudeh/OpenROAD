// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors
#pragma once
namespace utl {
class Logger;
}  // namespace utl

namespace dpl {
class Opendp;
class Node;
}  // namespace dpl
namespace dpo {
using dpl::Node;
class Annealer
{
 public:
  Annealer(utl::Logger* logger, dpl::Opendp* opendp);
  void move(Node* node);

 private:
  utl::Logger* logger_{nullptr};
  dpl::Opendp* opendp_{nullptr};
};
}  // namespace dpo