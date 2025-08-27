// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

// clang-format off
%{

#include "ord/OpenRoad.hh"
#include "graphics/Graphics.h"
#include "graphics/DplObserver.h"
#include "dpl/Opendp.h"
#include "utl/Logger.h"

%}

%include "../../Exception.i"

%import <std_vector.i>
%import "dbtypes.i"

%inline %{

namespace dpl {

void
detailed_placement_cmd(int max_displacment_x,
                       int max_displacment_y,
                       const char* report_file_name){
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->detailedPlacement(max_displacment_x, max_displacment_y, std::string(report_file_name));
}

void
report_legalization_stats()
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->reportLegalizationStats();
}

void
check_placement_cmd(bool verbose, const char* report_file_name)
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->checkPlacement(verbose, std::string(report_file_name));
}


void
set_padding_global(int left,
                   int right)
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->setPaddingGlobal(left, right);
}

void
set_padding_master(odb::dbMaster *master,
                   int left,
                   int right)
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->setPadding(master, left, right);
}

void
set_padding_inst(odb::dbInst *inst,
                 int left,
                 int right)
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->setPadding(inst, left, right);
}

void
filler_placement_cmd(const std::vector<odb::dbMaster*>& filler_masters,
                     const char* prefix,
                     bool verbose)
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->fillerPlacement(filler_masters, prefix, verbose);
}

void
remove_fillers_cmd()
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->removeFillers();
}

void
optimize_mirroring_cmd()
{
  dpl::Opendp *opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->optimizeMirroring();
}

void
set_debug_cmd(float min_displacement,
              const odb::dbInst* debug_instance)
{
  dpl::Opendp* opendp = ord::OpenRoad::openRoad()->getOpendp();
  if (dpl::Graphics::guiActive()) {
      std::unique_ptr<DplObserver> graphics = std::make_unique<dpl::Graphics>(
          opendp, min_displacement, debug_instance);
      opendp->setDebug(graphics);
  }
}

void improve_placement_cmd(int seed,
  int max_displacement_x,
  int max_displacement_y)
{
  dpl::Opendp* opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->improvePlacement(seed, max_displacement_x, max_displacement_y);
}

void disallow_odd_sites_cmd()
{
  dpl::Opendp* opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->disallowOddSites();
}

void set_phi_cut_cell_cmd(odb::dbMaster* master)
{
  dpl::Opendp* opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->setPhiCutCell(master);
}

void place_phi_cut_cells_cmd()
{
  dpl::Opendp* opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->placePhiCutCells();
}

void set_tap_phi_cell_cmd(odb::dbMaster* master)
{
  dpl::Opendp* opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->setTapPhiCell(master);
}

void set_cells_file_cmd(const char* cells_file)
{
  dpl::Opendp* opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->setCellsFile(std::string(cells_file));
}

void anneal(int max_iterations,
            double initial_temperature,
            float alpha,
            int seed,
            int max_displacement_x,
            int max_displacement_y)
{
  dpl::Opendp* opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->runSimulatedAnnealing(max_iterations,
                                initial_temperature,
                                alpha,
                                seed,
                                max_displacement_x,
                                max_displacement_y);
}

void optimize_pin_placement_cmd()
{
  dpl::Opendp* opendp = ord::OpenRoad::openRoad()->getOpendp();
  opendp->optimizePinPlacement();
}

} // namespace

%} // inline
