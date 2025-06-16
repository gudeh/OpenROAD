// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors

#include "dpl/Opendp.h"
#include "odb/util.h"
#include "utl/Logger.h"

// My stuff.
#include "legalize_shift.h"
#include "optimization/annealer.h"
#include "optimization/detailed.h"
#include "optimization/detailed_manager.h"
#include "optimization/pin_swapper.h"
#include "util/CellParser.h"

namespace dpl {

////////////////////////////////////////////////////////////////
void Opendp::parseCells(const std::string& cells_file)
{
  CellParser cell_parser;
  auto cells = cell_parser.parse(cells_file);
  // first create needed functions
  for (const auto& [name, config] : cells) {
    if (config.multibitMap.has_value()) {
      if (network_->getMasterFunction(config.function) != nullptr) {
        continue;
      }
      MasterFunction* function = network_->addMasterFunction(config.function);
      for (const auto& [single_bit, pattern] : (*config.multibitMap).pinMap) {
        function->addMultibitPinEntry(single_bit, pattern);
      }
    }
  }
  for (const auto& [name, config] : cells) {
    auto db_master = db_->findMaster(name.c_str());
    if (db_master == nullptr) {
      continue;
    }
    auto master
        = network_->addMaster(db_master, grid_.get(), drc_engine_.get());
    if (config.pinSwaps.has_value()) {
      master->setPinSwaps(*config.pinSwaps);
    }
    if (config.pinPermutes.has_value()) {
      master->setPinPermutes(*config.pinPermutes);
    }
    auto function = network_->getMasterFunction(config.function);
    if (function == nullptr) {
      continue;
    }
    if (config.multibitMap.has_value()) {
      master->setLsb((*config.multibitMap).LSb);
      master->setMsb((*config.multibitMap).MSb);
    }
    master->setFunction(function);
  }
}
////////////////////////////////////////////////////////////////
void Opendp::runSimulatedAnnealing(int max_iterations,
                                   double initial_temperature,
                                   float alpha,
                                   int seed,
                                   int max_displacement_x,
                                   int max_displacement_y)
{
  setMaxDisplacement(max_displacement_x, max_displacement_y);
  odb::WireLengthEvaluator eval(db_->getChip()->getBlock());
  const int64_t hpwlBefore = eval.hpwl();
  importDb();
  parseCells(cells_file_);
  adjustNodesOrient();
  initGrid();
  DetailedMgr mgr(arch_.get(), network_.get(), grid_.get(), drc_engine_.get());
  mgr.setLogger(logger_);
  mgr.setSeed(0);
  mgr.setMaxDisplacement(0, 0);
  mgr.setDisallowOneSiteGaps(false);
  ShiftLegalizer lg;
  lg.legalize(mgr);
  Annealer annealer(logger_, this, network_.get(), db_);
  annealer.set_sa_parameters(initial_temperature, alpha, max_iterations, seed);
  annealer.start();
  network_->removeMarkedNodes();

  updateDbInstLocations();
  const int64_t hpwlAfter = eval.hpwl();
  const double dbu_micron = db_->getTech()->getDbUnitsPerMicron();
  // Statistics.
  logger_->report("Detailed Improvement Results");
  logger_->report("------------------------------------------");
  logger_->report("Original HPWL         {:10.1f} u", hpwlBefore / dbu_micron);
  logger_->report("Final HPWL            {:10.1f} u", hpwlAfter / dbu_micron);
  const double hpwl_delta = (hpwlAfter - hpwlBefore) / (double) hpwlBefore;
  logger_->report("Delta HPWL            {:10.1f} %", hpwl_delta * 100);
  logger_->report("");
}
////////////////////////////////////////////////////////////////
void Opendp::optimizePinPlacement()
{
  odb::WireLengthEvaluator eval(db_->getChip()->getBlock());
  const int64_t hpwlBefore = eval.hpwl();
  importDb();
  parseCells(cells_file_);
  adjustNodesOrient();
  initGrid();
  DetailedMgr mgr(arch_.get(), network_.get(), grid_.get(), drc_engine_.get());
  mgr.setLogger(logger_);
  mgr.setSeed(0);
  mgr.setMaxDisplacement(0, 0);
  mgr.setDisallowOneSiteGaps(false);
  ShiftLegalizer lg;
  lg.legalize(mgr);
  PinSwapper pin_swapper(logger_, network_.get(), db_);
  pin_swapper.start();

  const int64_t hpwlAfter = eval.hpwl();
  const double dbu_micron = db_->getTech()->getDbUnitsPerMicron();
  // Statistics.
  logger_->report("Detailed Improvement Results");
  logger_->report("------------------------------------------");
  logger_->report("Original HPWL         {:10.1f} u", hpwlBefore / dbu_micron);
  logger_->report("Final HPWL            {:10.1f} u", hpwlAfter / dbu_micron);
  const double hpwl_delta = (hpwlAfter - hpwlBefore) / (double) hpwlBefore;
  logger_->report("Delta HPWL            {:10.1f} %", hpwl_delta * 100);
  logger_->report("");
}
////////////////////////////////////////////////////////////////
void Opendp::improvePlacement(const int seed,
                              const int max_displacement_x,
                              const int max_displacement_y)
{
  logger_->report("Detailed placement improvement.");

  odb::WireLengthEvaluator eval(db_->getChip()->getBlock());
  const int64_t hpwlBefore = eval.hpwl();

  if (hpwlBefore == 0) {
    logger_->report("Skipping detailed improvement since hpwl is zero.");
    return;
  }

  // Get needed information from DB.
  importDb();
  // TODO: adjustNodesOrient() but it's currently causing an unrelated CI
  // failure
  initGrid();

  const bool disallow_one_site_gaps = !odb::hasOneSiteMaster(db_);

  // A manager to track cells.
  DetailedMgr mgr(arch_.get(), network_.get(), grid_.get(), drc_engine_.get());
  mgr.setLogger(logger_);
  // Various settings.
  mgr.setSeed(seed);
  mgr.setMaxDisplacement(max_displacement_x, max_displacement_y);
  mgr.setDisallowOneSiteGaps(disallow_one_site_gaps);

  // Legalization.  Doesn't particularly do much.  It only
  // populates the data structures required for detailed
  // improvement.  If it errors or prints a warning when
  // given a legal placement, that likely means there is
  // a bug in my code somewhere.
  ShiftLegalizer lg;
  lg.legalize(mgr);

  // Detailed improvement.  Runs through a number of different
  // optimizations aimed at wirelength improvement.  The last
  // call to the random improver can be set to consider things
  // like density, displacement, etc. in addition to wirelength.
  // Everything done through a script string.

  DetailedParams dtParams;
  dtParams.script_ = "";
  // Maximum independent set matching.
  dtParams.script_ += "mis -p 10 -t 0.005;";
  // Global swaps.
  dtParams.script_ += "gs -p 10 -t 0.005;";
  // Vertical swaps.
  dtParams.script_ += "vs -p 10 -t 0.005;";
  // Small reordering.
  dtParams.script_ += "ro -p 10 -t 0.005;";
  // Random moves and swaps with hpwl as a cost function.  Use
  // random moves and hpwl objective right now.
  dtParams.script_ += "default -p 5 -f 20 -gen rng -obj hpwl -cost (hpwl);";

  if (disallow_one_site_gaps) {
    dtParams.script_ += "disallow_one_site_gaps;";
  }

  // Run the script.
  Detailed dt(dtParams);
  dt.improve(mgr);

  // Write solution back.
  updateDbInstLocations();

  // Get final hpwl.
  const int64_t hpwlAfter = eval.hpwl();

  const double dbu_micron = db_->getTech()->getDbUnitsPerMicron();

  // Statistics.
  logger_->report("Detailed Improvement Results");
  logger_->report("------------------------------------------");
  logger_->report("Original HPWL         {:10.1f} u", hpwlBefore / dbu_micron);
  logger_->report("Final HPWL            {:10.1f} u", hpwlAfter / dbu_micron);
  const double hpwl_delta = (hpwlAfter - hpwlBefore) / (double) hpwlBefore;
  logger_->report("Delta HPWL            {:10.1f} %", hpwl_delta * 100);
  logger_->report("");
}

////////////////////////////////////////////////////////////////
}  // namespace dpl
