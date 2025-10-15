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
class dbMaster;
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
  PlacementDRC(Grid* grid,
               odb::dbTech* tech,
               Padding* padding,
               bool disallow_one_site_gap);
  bool checkEdgeSpacing(const Node* cell) const;
  // Check edge spacing for a cell at a given location and orientation
  bool checkEdgeSpacing(const Node* cell,
                        GridX x,
                        GridY y,
                        const odb::dbOrientType& orient) const;
  bool checkBlockedLayers(const Node* cell) const;
  bool checkBlockedLayers(const Node* cell, GridX x, GridY y) const;
  // Check shared padding spacing conflicts
  bool checkPadding(const Node* cell) const;
  bool checkPadding(const Node* cell, GridX x, GridY y) const;

  // Check one site gap
  bool checkOneSiteGap(const Node* cell) const;
  bool checkOneSiteGap(const Node* cell, GridX x, GridY y) const;

  // aggregate function to check against all DRC types
  bool checkDRC(const Node* cell) const;
  bool checkDRC(const Node* cell,
                GridX x,
                GridY y,
                const odb::dbOrientType& orient) const;
  /**
   * @brief Checks end-of-line (EOL) spacing violations for abutted pins at
   * the cell's current placement.
   *
   * @param cell the cell whose abutted pins need to be checked
   * @return true if no EOL spacing violations are found, false otherwise
   */
  bool checkAbuttedPins(const Node* cell) const;
  /**
   * @brief Checks end-of-line (EOL) spacing violations for abutted pins
   * connected to different nets at a specified location and orientation.
   *
   * This function validates that pins on the same layer but connected to
   * different nets maintain the required EOL spacing distance. It checks
   * all neighboring cells within the EOL spacing range and verifies that
   * pins on the same technology layer have adequate spacing when connected
   * to different nets.
   * @param cell the cell whose abutted pins need to be checked
   * @param x the grid x-coordinate for the placement location
   * @param y the grid y-coordinate for the placement location
   * @param orient the orientation of the cell at the specified location
   * @return true if no EOL spacing violations are found, false otherwise
   */
  bool checkAbuttedPins(const Node* cell,
                        const GridX x,
                        const GridY y,
                        const odb::dbOrientType& orient) const;
  /**
   * @brief Checks phi spacing requirements at the cell's current placement.
   *
   * @param cell the cell whose phi spacing needs to be checked
   * @return true if phi spacing requirements are met, false otherwise
   */
  bool checkPhiSpacing(const Node* cell) const;
  /**
   * @brief Checks phi spacing requirements for cells with phi pins at a
   * specified location and orientation.
   *
   * This function ensures that cells with phi pins connected to different
   * phi nets maintain the minimum required spacing distance. It searches
   * within a phi_spacing_ distance around the cell and verifies that all
   * neighboring cells with connections are connected to the same nets.
   * @param cell the cell whose phi spacing needs to be checked
   * @param x the grid x-coordinate for the placement location
   * @param y the grid y-coordinate for the placement location
   * @param orient the orientation of the cell at the specified location
   * @return true if phi spacing requirements are met, false otherwise
   */
  bool checkPhiSpacing(const Node* cell,
                       GridX x,
                       GridY y,
                       const odb::dbOrientType& orient) const;
  /**
   * @brief Checks coloring constraints for placement at a specified grid
   * x-coordinate.
   *
   * This function validates site coloring rules, particularly for designs
   * that require placement only on even-numbered sites when odd sites are
   * disallowed.
   * @param x the grid x-coordinate to check for coloring constraints
   * @return true if placement is allowed at the x-coordinate, false otherwise
   */
  bool checkColoring(GridX x) const;
  /**
   * @brief Checks coloring constraints at the cell's current placement.
   *
   * @param cell the cell whose current location needs to be checked
   * @return true if placement is allowed, false otherwise
   */
  bool checkColoring(const Node* cell) const;
  int getEdgeTypeIdx(const std::string& edge_type) const;
  bool hasCellEdgeSpacingTable() const;
  int getMaxSpacing(int edge_type_idx) const;
  // TODO: remove this
  void disallowOddSites() { disallow_odd_sites_ = true; }

  // Set the minimum spacing required between cells with phi pins connected to
  // different phi nets
  void setPhiCutCell(odb::dbMaster* master);

  // Get the current minimum spacing required between cells with phi pins
  // connected to different phi nets
  DbuX getPhiSpacing() const { return phi_spacing_; }

 private:
  // Member variables
  Grid* grid_{nullptr};        // Pointer to the grid for placement
  Padding* padding_{nullptr};  // Pointer to the padding
  std::unordered_map<std::string, int> edge_types_indices_;
  std::vector<std::vector<EdgeSpacingEntry>>
      edge_spacing_table_;  // LEF58_CELLEDGESPACINGTABLE between edge type
                            // pairs [from_idx][to_idx]
  bool disallow_one_site_gap_{false};
  std::map<int, std::vector<EolSpacingEntry>> eol_spacing_rules_;
  // TODO: remove this
  bool disallow_odd_sites_ = false;
  DbuX phi_spacing_{0};  // Required spacing between cells with phi pins
                         // connected to different phi  nets
  odb::dbMaster* phi_cut_cell_{nullptr};

  // Helper functions
  DbuX gridToDbu(GridX grid_x, DbuX site_width) const;
  void makeCellEdgeSpacingTable(odb::dbTech* tech);
  bool hasPaddingConflict(const Node* cell, const Node* padding_cell) const;
  void makeEolSpacingRules(odb::dbTech* tech);
  GridRect getEolQueryRect(const Node* node,
                           DbuX left,
                           DbuY bottom,
                           const odb::dbOrientType& orient) const;
};

}  // namespace dpl