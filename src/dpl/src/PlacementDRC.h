#pragma once
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "infrastructure/Coordinates.h"

namespace odb {
class dbTech;
class dbOrientType;
}  // namespace odb

namespace dpl {
class Grid;
class Node;

struct EdgeSpacingEntry
{
  EdgeSpacingEntry(const int spc_in,
                   const bool is_exact_in,
                   const bool except_abutted_in)
      : spc(spc_in), is_exact(is_exact_in), except_abutted(except_abutted_in)
  {
  }
  bool operator<(const EdgeSpacingEntry& rhs) const { return spc < rhs.spc; }
  int spc;
  bool is_exact;
  bool except_abutted;
};
struct EolSpacingEntry
{
  EolSpacingEntry(const uint spc_in, const uint eol_width_in)
      : spc(spc_in), eol_width(eol_width_in)
  {
  }
  uint spc;
  uint eol_width;
};
class PlacementDRC
{
 public:
  // Constructor
  PlacementDRC(Grid* grid, odb::dbTech* tech);
  bool checkEdgeSpacing(const Node* cell) const;
  // Check edge spacing for a cell at a given location and orientation
  bool checkEdgeSpacing(const Node* cell,
                        GridX x,
                        GridY y,
                        const odb::dbOrientType& orient) const;
  bool checkAbuttedPins(const Node* cell) const;
  bool checkAbuttedPins(const Node* cell,
                        const GridX x,
                        const GridY y,
                        const odb::dbOrientType& orient) const;
  bool checkPhiSpacing(const Node* cell) const;
  bool checkPhiSpacing(const Node* cell,
                       GridX x,
                       GridY y,
                       const odb::dbOrientType& orient) const;
  bool checkColoring(const Node* cell,
                     GridX x,
                     GridY y,
                     const odb::dbOrientType& orient) const;
  bool checkColoring(const Node* cell) const;
  bool check(const Node* cell,
             GridX x,
             GridY y,
             const odb::dbOrientType& orient) const;
  bool check(const Node* cell) const;
  int getEdgeTypeIdx(const std::string& edge_type) const;
  bool hasCellEdgeSpacingTable() const;
  int getMaxSpacing(int edge_type_idx) const;
  // TODO: remove this
  void disallowOddSites() { disallow_odd_sites_ = true; }
  void setPhiSpacing(DbuX phi_spacing) { phi_spacing_ = phi_spacing; }
  DbuX getPhiSpacing() const { return phi_spacing_; }

  // Set phi cut cell and update phi spacing
  void setPhiCutCell(const std::string& cell_name);

 private:
  // Member variables
  Grid* grid_{nullptr};  // Pointer to the grid for placement
  std::unordered_map<std::string, int> edge_types_indices_;
  std::vector<std::vector<EdgeSpacingEntry>>
      edge_spacing_table_;  // LEF58_CELLEDGESPACINGTABLE between edge type
                            // pairs [from_idx][to_idx]
  std::map<int, std::vector<EolSpacingEntry>> eol_spacing_rules_;
  // TODO: remove this
  bool disallow_odd_sites_ = false;
  DbuX phi_spacing_{0};

  // Helper functions
  DbuX gridToDbu(GridX grid_x, DbuX site_width) const;
  void makeCellEdgeSpacingTable(odb::dbTech* tech);
  void makeEolSpacingRules(odb::dbTech* tech);
  GridRect getEolQueryRect(const Node* node,
                           DbuX left,
                           DbuY bottom,
                           const odb::dbOrientType& orient) const;
};

}  // namespace dpl