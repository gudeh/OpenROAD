#pragma once
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "dpl/Opendp.h"
#include "infrastructure/Coordinates.h"

namespace odb {
class dbTech;
class dbOrientType;
}  // namespace odb

namespace dpl {
class Grid;
class Node;
class Padding;

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

class PlacementDRC
{
 public:
  // Constructor
  PlacementDRC(Grid* grid,
               odb::dbTech* tech,
               Padding* padding,
               bool disallow_one_site_gap,
               utl::Logger* logger);
  bool isEdgeSpacingClean(const Node* cell) const;
  // Check edge spacing for a cell at a given location and orientation
  bool isEdgeSpacingClean(const Node* cell,
                        GridX x,
                        GridY y,
                        const odb::dbOrientType& orient) const;
  bool isBlockedLayersClean(const Node* cell, DplObserver* debug_observer=nullptr) const;
  bool isBlockedLayersClean(const Node* cell, GridX x, GridY y, DplObserver* debug_observer=nullptr) const;
  // Check shared padding spacing conflicts
  bool isPaddingClean(const Node* cell) const;
  bool isPaddingClean(const Node* cell, GridX x, GridY y) const;

  // Check one site gap
  bool isOneSiteGapClean(const Node* cell) const;
  bool isOneSiteGapClean(const Node* cell, GridX x, GridY y) const;

  // aggregate function to check against all DRC types
  bool isDRCclean(const Node* cell, DplObserver* debug_observer_=nullptr) const;
  bool isDRCclean(const Node* cell,
                GridX x,
                GridY y,
                const odb::dbOrientType& orient,
                DplObserver* debug_observer_) const;

  int getEdgeTypeIdx(const std::string& edge_type) const;
  bool hasCellEdgeSpacingTable() const;
  int getMaxSpacing(int edge_type_idx) const;

 private:
  // Member variables
  utl::Logger* logger_{nullptr};
  Grid* grid_{nullptr};        // Pointer to the grid for placement
  Padding* padding_{nullptr};  // Pointer to the padding
  std::unordered_map<std::string, int> edge_types_indices_;
  std::vector<std::vector<EdgeSpacingEntry>>
      edge_spacing_table_;  // LEF58_CELLEDGESPACINGTABLE between edge type
                            // pairs [from_idx][to_idx]
  bool disallow_one_site_gap_{false};

  // Helper functions
  DbuX gridToDbu(GridX grid_x, DbuX site_width) const;
  void makeCellEdgeSpacingTable(odb::dbTech* tech);
  bool hasPaddingConflict(const Node* cell, const Node* padding_cell) const;
};

}  // namespace dpl