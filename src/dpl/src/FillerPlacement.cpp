// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2020-2025, The OpenROAD Authors

#include <algorithm>
#include <string>
#include <utility>

#include "dpl/Opendp.h"
#include "infrastructure/Grid.h"
#include "infrastructure/Objects.h"
#include "infrastructure/network.h"
#include "odb/dbTypes.h"
#include "utl/Logger.h"

namespace dpl {

using std::to_string;

using utl::DPL;

using odb::dbMaster;
using odb::dbPlacementStatus;

using utl::format_as;

static dbTechLayer* getImplant(dbMaster* master)
{
  if (!master) {
    return nullptr;
  }

  for (auto obs : master->getObstructions()) {
    auto layer = obs->getTechLayer();
    if (layer->getType() == odb::dbTechLayerType::IMPLANT) {
      return layer;
    }
  }
  return nullptr;
}

Opendp::MasterByImplant Opendp::splitByImplant(
    const dbMasterSeq& filler_masters)
{
  MasterByImplant mapping;
  for (auto master : filler_masters) {
    mapping[getImplant(master)].emplace_back(master);
  }

  return mapping;
}

void Opendp::fillerPlacement(const dbMasterSeq& filler_masters,
                             const char* prefix,
                             bool verbose)
{
  if (network_->getNumCells() == 0) {
    importDb();
    adjustNodesOrient();
  }

  auto filler_masters_by_implant = splitByImplant(filler_masters);

  for (auto& [layer, masters] : filler_masters_by_implant) {
    std::sort(masters.begin(),
              masters.end(),
              [](dbMaster* master1, dbMaster* master2) {
                return master1->getWidth() > master2->getWidth();
              });
  }

  gap_fillers_.clear();
  filler_count_.clear();
  initGrid();
  setGridCells();

  for (GridY row{0}; row < grid_->getRowCount(); row++) {
    placeRowFillers(row, prefix, filler_masters_by_implant);
  }

  int filler_count = 0;
  int max_filler_master = 0;
  for (const auto& [master, count] : filler_count_) {
    filler_count += count;
    max_filler_master = std::max(max_filler_master, count);
  }
  logger_->info(DPL, 1, "Placed {} filler instances.", filler_count);

  if (verbose) {
    logger_->report("Filler usage:");
    int max_master_len = 0;
    for (const auto& [master, count] : filler_count_) {
      max_master_len = std::max(max_master_len,
                                static_cast<int>(master->getName().size()));
    }
    const int count_offset = fmt::format("{}", max_filler_master).size();
    for (const auto& [master, count] : filler_count_) {
      const int line_offset
          = count_offset + max_master_len - master->getName().size();
      logger_->report("  {}: {:>{}}", master->getName(), count, line_offset);
    }
  }
}

void Opendp::setGridCells()
{
  for (auto& cell : network_->getNodes()) {
    if (cell->getType() != Node::CELL) {
      continue;
    }
    grid_->visitCellPixels(*cell, false, [&](Pixel* pixel, bool padded) {
      setGridCell(*cell, pixel);
    });
  }
}

void Opendp::placeRowFillers(GridY row,
                             const std::string& prefix,
                             const MasterByImplant& filler_masters_by_implant)
{
  // DbuY row_height;
  GridX j{0};

  const DbuX site_width = grid_->getSiteWidth();
  GridX row_site_count = grid_->getRowSiteCount();
  while (j < row_site_count) {
    Pixel* pixel = grid_->gridPixel(j, row);
    if (pixel->cell || !pixel->is_valid) {
      ++j;
      continue;
    }
    // Select the site and orientation to fill this row with.  Use the shortest
    // site.
    auto [site, orient] = grid_->getShortestSite(j, row);
    GridX k = j;
    while (k < row_site_count && grid_->gridPixel(k, row)->cell == nullptr
           && grid_->gridPixel(k, row)->is_valid) {
      k++;
    }

    dbTechLayer* implant = nullptr;
    if (j > 0) {
      auto pixel = grid_->gridPixel(j - 1, row);
      if (pixel->cell && pixel->cell->getDbInst()) {
        implant = getImplant(pixel->cell->getDbInst()->getMaster());
      }
    } else if (k < row_site_count) {
      auto pixel = grid_->gridPixel(k, row);
      if (pixel->cell && pixel->cell->getDbInst()) {
        implant = getImplant(pixel->cell->getDbInst()->getMaster());
      }
    } else {  // totally empty row - use anything
      implant = filler_masters_by_implant.begin()->first;
    }

    GridX gap = k - j;
    dbMasterSeq& fillers = gapFillers(implant, gap, filler_masters_by_implant);
    if (fillers.empty()) {
      DbuX x{core_.xMin() + gridToDbu(j, site_width)};
      DbuY y{core_.yMin() + grid_->gridYToDbu(row)};
      logger_->error(
          DPL,
          2,
          "could not fill gap of size {} at {},{} dbu between {} and {}",
          gap,
          x,
          y,
          gridInstName(row, j - 1),
          gridInstName(row, k + 1));
    } else {
      k = j;
      debugPrint(
          logger_, DPL, "filler", 2, "fillers size is {}.", fillers.size());
      for (dbMaster* master : fillers) {
        std::string inst_name
            = prefix + to_string(row.v) + "_" + to_string(k.v);
        dbInst* inst = dbInst::create(block_,
                                      master,
                                      inst_name.c_str(),
                                      /* physical_only */ true);
        DbuX x{core_.xMin() + gridToDbu(k, site_width)};
        DbuY y{core_.yMin() + grid_->gridYToDbu(row)};
        inst->setOrient(orient);
        inst->setLocation(x.v, y.v);
        inst->setPlacementStatus(dbPlacementStatus::PLACED);
        inst->setSourceType(odb::dbSourceType::DIST);
        filler_count_[master]++;
        k += master->getWidth() / site_width.v;
      }
      j += gap;
    }
  }
}

const char* Opendp::gridInstName(GridY row, GridX col)
{
  if (col < 0) {
    return "core_left";
  }
  if (col > grid_->getRowSiteCount()) {
    return "core_right";
  }

  const auto cell = grid_->gridPixel(col, row)->cell;
  if (cell) {
    return cell->getDbInst()->getConstName();
  }
  return "?";
}

// Return list of masters to fill gap (in site width units).
dbMasterSeq& Opendp::gapFillers(
    dbTechLayer* implant,
    GridX gap,
    const MasterByImplant& filler_masters_by_implant)
{
  auto iter = filler_masters_by_implant.find(implant);
  if (iter == filler_masters_by_implant.end()) {
    logger_->error(DPL, 50, "No fillers found for {}.", implant->getName());
  }
  const dbMasterSeq& filler_masters = iter->second;

  GapFillers& gap_fillers = gap_fillers_[implant];
  if (gap_fillers.size() < gap + 1) {
    gap_fillers.resize(gap.v + 1);
  }
  dbMasterSeq& fillers = gap_fillers[gap.v];
  if (fillers.empty()) {
    int width = 0;
    dbMaster* smallest_filler = filler_masters[filler_masters.size() - 1];
    const DbuX site_width = grid_->getSiteWidth();
    bool have_filler1 = smallest_filler->getWidth() == site_width;
    for (dbMaster* filler_master : filler_masters) {
      int filler_width = filler_master->getWidth() / site_width.v;
      while ((width + filler_width) <= gap
             && (have_filler1 || (width + filler_width) != gap - 1)) {
        fillers.push_back(filler_master);
        width += filler_width;
        if (width == gap) {
          return fillers;
        }
      }
    }
    // Fail. Return empty fillers.
    fillers.clear();
  }
  return fillers;
}

void Opendp::removeFillers()
{
  block_ = db_->getChip()->getBlock();
  for (dbInst* db_inst : block_->getInsts()) {
    if (isFiller(db_inst)) {
      odb::dbInst::destroy(db_inst);
    }
  }
}

/* static */
bool Opendp::isFiller(dbInst* db_inst)
{
  dbMaster* db_master = db_inst->getMaster();
  return db_master->getType() == odb::dbMasterType::CORE_SPACER
         // Filter spacer cells used as tapcells.
         && db_inst->getPlacementStatus() != odb::dbPlacementStatus::LOCKED;
}

// Return true if cell is a single site Core Spacer.
bool Opendp::isOneSiteCell(odb::dbMaster* db_master) const
{
  return db_master->getType() == odb::dbMasterType::CORE_SPACER
         && db_master->getWidth() == grid_->getSiteWidth();
}
namespace {
// Helper function to check if a cell is connected to phi nets
bool isPhiCell(const Node* cell)
{
  if (!cell || cell->getType() != Node::CELL) {
    return false;
  }
  return !cell->getConnections().empty();
}

// Helper function to get the phi net ID for a cell
int getPhiNetId(const Node* cell)
{
  if (!cell || cell->getType() != Node::CELL) {
    return -1;
  }

  // Return the first net ID found
  const auto& connections = cell->getConnections();
  if (!connections.empty()) {
    return connections.begin()->second;  // Return the net ID
  }
  return -1;
}

int getCurrSitePhiNetId(Pixel* pixel1, Pixel* pixel2)
{
  if (pixel1 && pixel1->cell && isPhiCell(pixel1->cell)) {
    return getPhiNetId(pixel1->cell);
  }
  if (pixel2 && pixel2->cell && isPhiCell(pixel2->cell)) {
    return getPhiNetId(pixel2->cell);
  }
  return -1;
}

bool haveDifferentPhiNets(Pixel* pixel1, Pixel* pixel2)
{
  if (pixel1 == nullptr || pixel2 == nullptr) {
    return false;
  }
  if (pixel1->cell == nullptr || pixel2->cell == nullptr) {
    return false;
  }
  if (!isPhiCell(pixel1->cell) && !isPhiCell(pixel2->cell)) {
    return false;
  }
  return getPhiNetId(pixel1->cell) != getPhiNetId(pixel2->cell);
}
}  // namespace

// Helper function to find a valid position for phi cut cell
static std::optional<std::pair<GridX, GridY>> findPhiCutPosition(
    Grid* grid,
    GridX gap_start_x,
    GridX gap_end_x,
    GridY row,
    GridX phi_cut_width,
    GridY phi_cut_height)
{
  // Try each possible position in the gap, prioritizing lower y values
  for (GridX x = gap_start_x; x <= gap_end_x - phi_cut_width; x++) {
    // Check if this position is valid (no overlapping cells)
    bool position_valid = true;
    for (GridX check_x = x; check_x < x + phi_cut_width && position_valid;
         check_x++) {
      for (GridY check_y = row;
           check_y < row + phi_cut_height && position_valid;
           check_y++) {
        Pixel* pixel = grid->gridPixel(check_x, check_y);
        if (!pixel->is_valid || pixel->cell) {
          position_valid = false;
          break;
        }
      }
    }
    if (position_valid) {
      return std::make_pair(x, row);
    }
  }
  return std::nullopt;
}

// Helper function to safely swap a cell's master
static void swapCellMaster(Node* node, dbMaster* new_master, dbBlock* block)
{
  dbInst* old_inst = node->getDbInst();
  if (!old_inst || !new_master) {
    return;
  }

  // Save old instance properties
  std::string inst_name = old_inst->getName();
  dbOrientType orient = old_inst->getOrient();
  int x = old_inst->getLocation().x();
  int y = old_inst->getLocation().y();
  dbPlacementStatus placement_status = old_inst->getPlacementStatus();
  odb::dbSourceType source_type = old_inst->getSourceType();

  // Delete old instance
  dbInst::destroy(old_inst);

  // Create new instance with the desired master at the same position
  dbInst* new_inst = dbInst::create(block,
                                    new_master,
                                    inst_name.c_str(),
                                    /* physical_only */ true);

  // Restore properties
  new_inst->setOrient(orient);
  new_inst->setLocation(x, y);
  new_inst->setPlacementStatus(placement_status);
  new_inst->setSourceType(source_type);

  // Update node's instance pointer
  node->setDbInst(new_inst);
}

void Opendp::placeRowPhiCutCells(GridY row, int& phi_cut_count)
{
  if (!phi_cut_cell_) {
    return;
  }

  const DbuX site_width = grid_->getSiteWidth();
  GridX row_site_count = grid_->getRowSiteCount();
  GridX j{0};

  while (j < row_site_count) {
    Pixel* pixel1 = grid_->gridPixel(j, row);
    Pixel* pixel2 = grid_->gridPixel(j, row + 1);
    if (haveDifferentPhiNets(pixel1, pixel2)) {
      logger_->error(DPL,
                     58,
                     "Cells {} and {} are on different phi nets.",
                     pixel1->cell->name(),
                     pixel2->cell->name());
    }

    // Find next valid cell
    int curr_phi_net = getCurrSitePhiNetId(pixel1, pixel2);
    int next_phi_net = -1;

    if (curr_phi_net == -1) {
      ++j;
      continue;
    }

    GridX gap_start_x = j;
    GridX gap_end_x{-1};
    // Find next phi cell
    Node* welltap_cell = nullptr;
    bool found_cut_cell = false;
    while (++j < row_site_count) {
      Pixel* next_pixel1 = grid_->gridPixel(j, row);
      Pixel* next_pixel2 = grid_->gridPixel(j, row + 1);
      // if there is a cut cell in this row, skip
      if (next_pixel1->cell
          && next_pixel1->cell->getMaster()->getDbMaster() == phi_cut_cell_) {
        found_cut_cell = true;
        break;
      }
      if (next_pixel1->cell
          && next_pixel1->cell->getMaster()->getDbMaster()->getType()
                 == odb::dbMasterType::CORE_WELLTAP) {
        welltap_cell = next_pixel1->cell;
        continue;
      }
      if (haveDifferentPhiNets(next_pixel1, next_pixel2)) {
        logger_->error(DPL,
                       59,
                       "Cells {} and {} are on different phi nets.",
                       next_pixel1->cell->name(),
                       next_pixel2->cell->name());
      }
      int phi_net_id = getCurrSitePhiNetId(next_pixel1, next_pixel2);
      if (phi_net_id == curr_phi_net) {
        gap_start_x = j;
      } else if (phi_net_id != -1) {
        next_phi_net = phi_net_id;
        gap_end_x = j;
        break;
      }
    }
    if (found_cut_cell) {
      continue;
    }
    if (next_phi_net == -1) {
      break;  // No more phi cells in this row
    }

    // Check if cells are on different phi nets
    if (welltap_cell) {
      if (tap_phi_cell_) {
        swapCellMaster(welltap_cell, tap_phi_cell_, block_);
      }
      continue;
    }
    gap_start_x += 1;
    // Calculate gap between cells
    GridX gap_x = gap_end_x - gap_start_x;
    // Check if there's enough space for phi cut cell
    GridX phi_cut_width{(int) phi_cut_cell_->getWidth() / site_width.v};
    GridY phi_cut_height{grid_->gridHeight(phi_cut_cell_)};
    if (gap_x < phi_cut_width) {
      continue;
    }
    auto position = findPhiCutPosition(grid_.get(),
                                       gap_start_x,
                                       gap_end_x,
                                       row,
                                       phi_cut_width,
                                       phi_cut_height);

    if (position) {
      auto [best_x, best_y] = *position;
      // Get site and orientation
      auto [site, orient] = grid_->getShortestSite(best_x, best_y);

      // Create and place phi cut cell
      std::string inst_name = "PHI_CUT_" + std::to_string(phi_cut_count++);
      dbInst* inst = dbInst::create(block_,
                                    phi_cut_cell_,
                                    inst_name.c_str(),
                                    /* physical_only */ true);

      DbuX x{core_.xMin() + gridToDbu(best_x, site_width)};
      DbuY y{core_.yMin() + grid_->gridYToDbu(best_y)};
      inst->setOrient(orient);
      inst->setLocation(x.v, y.v);
      inst->setPlacementStatus(dbPlacementStatus::PLACED);
      inst->setSourceType(odb::dbSourceType::DIST);

      auto left_phi_net = odb::dbNet::getNet(block_, curr_phi_net);
      auto right_phi_net = odb::dbNet::getNet(block_, next_phi_net);
      if (left_phi_net && right_phi_net) {
        auto left_phi_iterm = inst->findITerm("phil");
        auto right_phi_iterm = inst->findITerm("phir");
        if (left_phi_iterm && right_phi_iterm) {
          left_phi_iterm->connect(left_phi_net);
          right_phi_iterm->connect(right_phi_net);
        } else {
          logger_->warn(DPL,
                        57,
                        "Phi cut cell {} has no phil or phir iterms.",
                        inst->getName());
        }
      }
      for (auto iterm : inst->getITerms()) {
        const std::string iterm_name = iterm->getMTerm()->getName();
        if (iterm_name.find("phi", 0) == 0) {
          // skip phi pins
          continue;
        }
        auto net = block_->findNet(iterm_name.c_str());
        if (net) {
          iterm->connect(net);
        }
      }

      // Update grid
      network_->addNode(inst);
      auto& node = network_->getNodes().back();
      grid_->paintPixel(node.get());
    } else {
      logger_->error(DPL,
                     53,
                     "No valid position found for phi cut cell in row {} "
                     "between {} and {}",
                     grid_->gridYToDbu(row),
                     gridToDbu(gap_start_x, site_width),
                     gridToDbu(gap_end_x, site_width));
    }
  }
}

void Opendp::placePhiCutCells()
{
  if (!phi_cut_cell_) {
    return;
  }

  if (network_->getNumCells() == 0) {
    importDb();
    adjustNodesOrient();
  }

  initGrid();
  setGridCells();

  network_->addMaster(phi_cut_cell_, grid_.get(), drc_engine_.get());
  int phi_cut_count = 0;
  for (GridY row{0}; row < grid_->getRowCount(); row += 2) {
    placeRowPhiCutCells(row, phi_cut_count);
  }

  if (phi_cut_count > 0) {
    logger_->info(DPL, 52, "Placed {} phi cut cells.", phi_cut_count);
  }
}
}  // namespace dpl
