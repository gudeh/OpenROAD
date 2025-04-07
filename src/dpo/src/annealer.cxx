// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors
#include "annealer.h"

#include "dpl/Opendp.h"
#include "journal.h"
#include "utl/Logger.h"

namespace dpo {
Annealer::Annealer(utl::Logger* logger, dpl::Opendp* opendp)
    : logger_(logger), opendp_(opendp)
{
  // opendp_->importDb();
}
void Annealer::move(Node* node)
{
  logger_->report("MapMove {}",
                  opendp_->mapMove(node, {GridX{100}, GridY{100}}));
}

}  // namespace dpo