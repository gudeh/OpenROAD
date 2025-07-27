#include "PlacementDRC.h"

#include <set>
#include <string>

#include "infrastructure/Grid.h"
#include "infrastructure/Objects.h"
#include "odb/db.h"
#include "odb/dbTransform.h"

namespace dpl {

namespace cell_edges {
Rect transformEdgeRect(const Rect& edge_rect,
                       const Node* cell,
                       const DbuX x,
                       const DbuY y,
                       const odb::dbOrientType& orient)
{
  Rect bbox;
  cell->getDbInst()->getMaster()->getPlacementBoundary(bbox);
  odb::dbTransform transform(orient);
  transform.apply(bbox);
  Point offset(x.v - bbox.xMin(), y.v - bbox.yMin());
  transform.setOffset(offset);
  Rect result(edge_rect);
  transform.apply(result);
  return result;
}
Rect getQueryRect(const Rect& edge_box, const int spc)
{
  Rect query_rect(edge_box);
  bool is_vertical_edge = edge_box.getDir() == 0;
  if (is_vertical_edge) {
    // vertical edge
    query_rect = query_rect.bloat(spc, odb::Orientation2D::Horizontal);
  } else {
    // horizontal edge
    query_rect = query_rect.bloat(spc, odb::Orientation2D::Vertical);
  }
  return query_rect;
}
uint64_t squaredDistanceBetweenRects(const Rect& rect1, const Rect& rect2)
{
  // Calculate the distance along the x-axis
  int x_dist = 0;
  if (rect1.xMax() < rect2.xMin()) {
    x_dist = rect2.xMin() - rect1.xMax();
  } else if (rect2.xMax() < rect1.xMin()) {
    x_dist = rect1.xMin() - rect2.xMax();
  } else {
    x_dist = 0;  // rectangles overlap or touch on x-axis
  }

  // Calculate the distance along the y-axis
  int y_dist = 0;
  if (rect1.yMax() < rect2.yMin()) {
    y_dist = rect2.yMin() - rect1.yMax();
  } else if (rect2.yMax() < rect1.yMin()) {
    y_dist = rect1.yMin() - rect2.yMax();
  } else {
    y_dist = 0;  // rectangles overlap or touch on y-axis
  }

  // Return the squared Euclidean distance
  return x_dist * (int64_t) x_dist + y_dist * (int64_t) y_dist;
}

};  // namespace cell_edges

// Constructor
PlacementDRC::PlacementDRC(Grid* grid, odb::dbTech* tech) : grid_(grid)
{
  makeCellEdgeSpacingTable(tech);
  makeEolSpacingRules(tech);
}

bool PlacementDRC::checkEdgeSpacing(const Node* cell) const
{
  const GridX x = grid_->gridX(cell);
  const GridY y = grid_->gridRoundY(cell);
  return checkEdgeSpacing(cell, x, y, cell->getOrient());
}

// Check edge spacing for a cell at a given location and orientation
bool PlacementDRC::checkEdgeSpacing(const Node* cell,
                                    const GridX x,
                                    const GridY y,
                                    const dbOrientType& orient) const
{
  if (!hasCellEdgeSpacingTable()) {
    return true;
  }
  auto master = cell->getMaster();
  if (master == nullptr) {
    // Filler Cell
    return true;
  }
  // Get the real grid coordinates from the grid indices.
  DbuX x_real = gridToDbu(x, grid_->getSiteWidth());
  DbuY y_real = grid_->gridYToDbu(y);
  for (const auto& edge1 : master->getEdges()) {
    int max_spc = getMaxSpacing(edge1.getEdgeType())
                  + 1;  // +1 to account for EXACT rules
    Rect edge1_box = cell_edges::transformEdgeRect(
        edge1.getBBox(), cell, x_real, y_real, orient);
    bool is_vertical_edge = edge1_box.getDir() == 0;
    Rect query_rect = cell_edges::getQueryRect(edge1_box, max_spc);
    GridX xMin = grid_->gridX(DbuX(query_rect.xMin()));
    GridX xMax = grid_->gridEndX(DbuX(query_rect.xMax()));
    GridY yMin = grid_->gridEndY(DbuY(query_rect.yMin())) - 1;
    GridY yMax = grid_->gridEndY(DbuY(query_rect.yMax()));
    std::set<Node*> checked_cells;
    // Loop over the area covered by queryRect to find neighboring edges and
    // check violations.
    for (GridY y1 = yMin; y1 <= yMax; y1++) {
      for (GridX x1 = xMin; x1 <= xMax; x1++) {
        const Pixel* pixel = grid_->gridPixel(x1, y1);
        if (pixel == nullptr || pixel->cell == nullptr || pixel->cell == cell) {
          // Skip if pixel is empty or occupied only by the current cell.
          continue;
        }
        auto cell2 = static_cast<Node*>(pixel->cell);
        if (checked_cells.find(cell2) != checked_cells.end()) {
          // Skip if cell was already checked
          continue;
        }
        checked_cells.insert(cell2);
        auto master2 = cell2->getMaster();
        if (master2 == nullptr) {
          continue;
        }
        for (const auto& edge2 : master2->getEdges()) {
          auto spc_entry
              = edge_spacing_table_[edge1.getEdgeType()][edge2.getEdgeType()];
          int spc = spc_entry.spc;
          Rect edge2_box = cell_edges::transformEdgeRect(edge2.getBBox(),
                                                         cell2,
                                                         cell2->getLeft(),
                                                         cell2->getBottom(),
                                                         cell2->getOrient());
          if (edge1_box.getDir() != edge2_box.getDir()) {
            // Skip if edges are not parallel.
            continue;
          }
          if (!query_rect.overlaps(edge2_box)) {
            // Skip if there is no PRL between the edges.
            continue;
          }
          Rect test_rect(edge1_box);
          // Generalized intersection between the two edges.
          test_rect.merge(edge2_box);
          int dist = is_vertical_edge ? test_rect.dx() : test_rect.dy();
          if (spc_entry.is_exact) {
            if (dist == spc) {
              // Violation only if the distance between the edges is exactly the
              // specified spacing.
              return false;
            }
          } else if (dist < spc) {
            return false;
          }
        }
      }
    }
  }
  return true;
}
bool PlacementDRC::checkAbuttedPins(const Node* cell) const
{
  const GridX x = grid_->gridX(cell);
  const GridY y = grid_->gridRoundY(cell);
  return checkAbuttedPins(cell, x, y, cell->getOrient());
}
GridRect PlacementDRC::getEolQueryRect(const Node* node,
                                       const DbuX left,
                                       const DbuY bottom,
                                       const odb::dbOrientType& orient) const
{
  Rect query_rect;
  query_rect.mergeInit();
  bool valid = false;
  const auto& master = node->getMaster();
  for (auto [pin1_idx, net1_idx] : node->getConnections()) {
    auto pin1 = master->getPins().at(pin1_idx);
    Rect pin1_rect = cell_edges::transformEdgeRect(
        pin1->getBox(), node, left, bottom, orient);
    if (eol_spacing_rules_.find(pin1->getTechLayer()->getNumber())
        == eol_spacing_rules_.end()) {
      continue;
    }
    for (auto rule : eol_spacing_rules_.at(pin1->getTechLayer()->getNumber())) {
      if (pin1_rect.minDXDY() > rule.eol_width) {
        continue;
      }
      Rect bloat_rect(pin1_rect);
      bloat_rect.bloat(rule.spc, bloat_rect);
      query_rect.merge(bloat_rect);
      valid = true;
    }
  }
  if (!valid) {
    return {GridX{0}, GridY{0}, GridX{-1}, GridY{-1}};
  }
  GridX begin_x = grid_->gridX(DbuX(query_rect.xMin()));
  GridX end_x = grid_->gridX(DbuX(query_rect.xMax()));
  GridY begin_y = grid_->gridSnapDownY(DbuY(query_rect.yMin()));
  GridY end_y = grid_->gridSnapDownY(DbuY(query_rect.yMax()));
  return {begin_x, begin_y, end_x, end_y};
}
bool PlacementDRC::checkAbuttedPins(const Node* cell,
                                    const GridX x,
                                    const GridY y,
                                    const odb::dbOrientType& orient) const
{
  if (cell->getConnections().empty()) {
    return true;
  }
  const auto& master = cell->getMaster();
  DbuX x_real = gridToDbu(x, grid_->getSiteWidth());
  DbuY y_real = grid_->gridYToDbu(y);
  GridRect query_rect = getEolQueryRect(cell, x_real, y_real, orient);
  std::set<Node*> checked_cells;
  for (GridX xi = query_rect.xlo; xi <= query_rect.xhi; xi++) {
    for (GridY yi = query_rect.ylo; yi <= query_rect.yhi; yi++) {
      const Pixel* pixel = grid_->gridPixel(xi, yi);
      if (pixel == nullptr || pixel->cell == nullptr || pixel->cell == cell) {
        // Skip if pixel is empty or occupied only by the current cell.
        continue;
      }
      auto cell2 = static_cast<Node*>(pixel->cell);
      if (checked_cells.find(cell2) != checked_cells.end()) {
        // Skip if cell was already checked
        continue;
      }
      checked_cells.insert(cell2);
      auto master2 = cell2->getMaster();
      for (auto [pin1_idx, net1_idx] : cell->getConnections()) {
        auto pin1 = master->getPins().at(pin1_idx);
        Rect pin1_rect = cell_edges::transformEdgeRect(
            pin1->getBox(), cell, x_real, y_real, orient);
        for (auto [pin2_idx, net2_idx] : cell2->getConnections()) {
          if (net1_idx == net2_idx) {
            continue;
          }
          auto pin2 = master2->getPins().at(pin2_idx);
          if (pin2->getTechLayer() != pin1->getTechLayer()) {
            continue;
          }
          Rect pin2_rect = cell_edges::transformEdgeRect(pin2->getBox(),
                                                         cell2,
                                                         cell2->getLeft(),
                                                         cell2->getBottom(),
                                                         cell2->getOrient());
          if (eol_spacing_rules_.find(pin1->getTechLayer()->getNumber())
              == eol_spacing_rules_.end()) {
            continue;
          }
          // check eol spacing
          const uint64_t dist
              = cell_edges::squaredDistanceBetweenRects(pin1_rect, pin2_rect);
          for (const auto& rule :
               eol_spacing_rules_.at(pin1->getTechLayer()->getNumber())) {
            if (pin1_rect.minDXDY() > rule.eol_width
                && pin2_rect.minDXDY() > rule.eol_width) {
              continue;
            }
            uint64_t req_spc = rule.spc * (uint64_t) rule.spc;
            if (dist < req_spc) {
              return false;
            }
          }
        }
      }
    }
  }
  return checkPhiSpacing(cell, x, y, orient);
}

bool PlacementDRC::checkColoring(const Node* cell) const
{
  return checkColoring(grid_->gridX(cell));
}

bool PlacementDRC::checkColoring(const GridX x) const
{
  // Do not allow placing the cell on an oldd site.
  if (disallow_odd_sites_) {
    return x.v % 2 == 0;
  }
  return true;
}

bool PlacementDRC::check(const Node* cell,
                         const GridX x,
                         const GridY y,
                         const odb::dbOrientType& orient) const
{
  return checkEdgeSpacing(cell, x, y, orient)
         && checkAbuttedPins(cell, x, y, orient) && checkColoring(x);
}

bool PlacementDRC::check(const Node* cell) const
{
  return checkEdgeSpacing(cell) && checkAbuttedPins(cell)
         && checkColoring(cell);
}

// Initialize the edge spacing table from the technology
void PlacementDRC::makeCellEdgeSpacingTable(odb::dbTech* tech)
{
  auto spacing_rules = tech->getCellEdgeSpacingTable();
  if (spacing_rules.empty()) {
    return;
  }
  for (auto rule : spacing_rules) {
    edge_types_indices_.try_emplace(rule->getFirstEdgeType(),
                                    edge_types_indices_.size());
    edge_types_indices_.try_emplace(rule->getSecondEdgeType(),
                                    edge_types_indices_.size());
  }
  // Resize
  const size_t size = edge_types_indices_.size();
  edge_spacing_table_.resize(size);
  for (size_t i = 0; i < size; i++) {
    edge_spacing_table_[i].resize(size, EdgeSpacingEntry(0, false, false));
  }
  // Fill Table
  for (auto rule : spacing_rules) {
    std::string first_edge = rule->getFirstEdgeType();
    std::string second_edge = rule->getSecondEdgeType();
    const int spc = rule->getSpacing();
    const bool exact = rule->isExact();
    const bool except_abutted = rule->isExceptAbutted();
    const EdgeSpacingEntry entry(spc, exact, except_abutted);
    const int idx1 = edge_types_indices_[first_edge];
    const int idx2 = edge_types_indices_[second_edge];
    edge_spacing_table_[idx1][idx2] = entry;
    edge_spacing_table_[idx2][idx1] = entry;
  }
}

void PlacementDRC::makeEolSpacingRules(odb::dbTech* tech)
{
  for (auto layer : tech->getLayers()) {
    if (layer->getType() != odb::dbTechLayerType::ROUTING) {
      continue;
    }
    if (layer->getTechLayerSpacingEolRules().empty()) {
      // setting default spacing rule to disallow shorts.
      eol_spacing_rules_[layer->getNumber()].emplace_back(
          1, std::numeric_limits<uint>::max());
      continue;
    }
    for (auto rule : layer->getTechLayerSpacingEolRules()) {
      eol_spacing_rules_[layer->getNumber()].emplace_back(rule->getEolSpace(),
                                                          rule->getEolWidth());
    }
  }
}
// Check if the edge spacing table is populated
bool PlacementDRC::hasCellEdgeSpacingTable() const
{
  return !edge_spacing_table_.empty();
}

// Get the maximum spacing for a given edge type index
int PlacementDRC::getMaxSpacing(const int edge_type_idx) const
{
  return std::max_element(edge_spacing_table_[edge_type_idx].begin(),
                          edge_spacing_table_[edge_type_idx].end())
      ->spc;
}

// Get the index of an edge type from its name
int PlacementDRC::getEdgeTypeIdx(const std::string& edge_type) const
{
  auto it = edge_types_indices_.find(edge_type);
  if (it != edge_types_indices_.end()) {
    return it->second;
  }
  return -1;  // Edge type not found
}

// Convert grid coordinates to DBU coordinates
DbuX PlacementDRC::gridToDbu(const GridX grid_x, const DbuX site_width) const
{
  return DbuX(grid_x.v * site_width.v);
}

bool PlacementDRC::checkPhiSpacing(const Node* cell) const
{
  const GridX x = grid_->gridX(cell);
  const GridY y = grid_->gridRoundY(cell);
  return checkPhiSpacing(cell, x, y, cell->getOrient());
}

bool PlacementDRC::checkPhiSpacing(const Node* cell,
                                   const GridX x,
                                   const GridY y,
                                   const odb::dbOrientType& orient) const
{
  if (cell->getConnections().empty() || phi_spacing_ == 0) {
    return true;
  }
  // Cells with PHI nets are either single or double height.

  const GridX phi_cut_width = grid_->gridX(phi_spacing_);
  const GridX begin_x = x - phi_cut_width;
  const GridX end_x = x + grid_->gridPaddedWidth(cell) + phi_cut_width;
  const GridY y_begin = y.v % 2 == 0 ? y : y - 1;
  const GridY end_y = y_begin + 2;
  std::set<Node*> checked_cells;
  for (GridY yi = y_begin; yi < end_y; yi++) {
    for (GridX xi = begin_x; xi < end_x; xi++) {
      const Pixel* pixel = grid_->gridPixel(xi, yi);
      if (pixel == nullptr || pixel->cell == nullptr || pixel->cell == cell) {
        // Skip if pixel is empty or occupied only by the current cell
        continue;
      }

      auto cell2 = static_cast<Node*>(pixel->cell);

      if (checked_cells.find(cell2) != checked_cells.end()) {
        // Skip if cell was already checked
        continue;
      }
      checked_cells.insert(cell2);

      for (auto [_, net1_idx] : cell->getConnections()) {
        for (auto [_, net2_idx] : cell2->getConnections()) {
          if (net1_idx != net2_idx) {
            return false;
          }
        }
      }
    }
  }
  return true;
}

}  // namespace dpl