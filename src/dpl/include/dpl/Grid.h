/////////////////////////////////////////////////////////////////////////////
// Original authors: SangGi Do(sanggido@unist.ac.kr), Mingyu
// Woo(mwoo@eng.ucsd.edu)
//          (respective Ph.D. advisors: Seokhyeong Kang, Andrew B. Kahng)
// Rewrite by James Cherry, Parallax Software, Inc.
//
// Copyright (c) 2019, The Regents of the University of California
// Copyright (c) 2018, SangGi Do and Mingyu Woo
// All rights reserved.
//
// BSD 3-Clause License
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <unordered_set>
#include <vector>

#include "Coordinates.h"
#include "Opendp.h"

namespace dpl {

using odb::dbOrientType;
using odb::dbSite;

class GridNode
{
 public:
  virtual ~GridNode() = default;
  virtual bool isFixed() const = 0;
  virtual bool isPlaced() const = 0;
  virtual bool isHybrid() const = 0;
  virtual DbuX xMin() const = 0;
  virtual DbuY yMin() const = 0;
  virtual DbuX dx() const = 0;
  virtual DbuY dy() const = 0;
  virtual dbInst* getDbInst() const = 0;
  virtual DbuX siteWidth() const = 0;
};

struct GridIntervalX
{
  GridX lo;
  GridX hi;
};

struct GridIntervalY
{
  GridY lo;
  GridY hi;
};

struct Pixel
{
  GridNode* cell = nullptr;
  Group* group = nullptr;
  double util = 0.0;
  bool is_valid = false;     // false for dummy cells
  bool is_hopeless = false;  // too far from sites for diamond search
  std::map<dbSite*, dbOrientType> sites;
};

// Return value for grid searches.
class PixelPt
{
 public:
  PixelPt() = default;
  PixelPt(Pixel* pixel, GridX grid_x, GridY grid_y);
  Pixel* pixel = nullptr;
  GridX x{0};
  GridY y{0};
};

// The "Grid" is a 2D array of pixels.  The pixels represent the
// single height sites onto which multi-height sites are overlaid.
// The sites are assumed to be of a single width but the rows are of
// variable height in order to support hybrid rows.
class Grid
{
 public:
  void init(Logger* logger) { logger_ = logger; }
  void initBlock(dbBlock* block) { core_ = block->getCoreArea(); }
  void initGrid(dbDatabase* db,
                dbBlock* block,
                std::shared_ptr<Padding> padding,
                int max_displacement_x,
                int max_displacement_y);
  void allocateGrid();
  void examineRows(dbBlock* block);
  std::unordered_set<int> getRowCoordinates() const;

  GridX gridX(DbuX x) const;
  GridX gridEndX(DbuX x) const;

  GridX gridX(const GridNode* cell) const;
  GridX gridEndX(const GridNode* cell) const;
  GridX gridPaddedX(const GridNode* cell) const;
  GridX gridPaddedEndX(const GridNode* cell) const;

  GridY gridSnapDownY(DbuY y) const;
  GridY gridRoundY(DbuY y) const;
  GridY gridEndY(DbuY y) const;

  // Snap outwards to fully contain
  GridRect gridCovering(const Rect& rect) const;
  GridRect gridCovering(const GridNode* cell) const;
  GridRect gridCoveringPadded(const GridNode* cell) const;

  // Snap inwards to be fully contained
  GridRect gridWithin(const DbuRect& rect) const;

  GridY gridSnapDownY(const GridNode* cell) const;
  GridY gridRoundY(const GridNode* cell) const;
  GridY gridEndY(const GridNode* cell) const;

  DbuY gridYToDbu(GridY y) const;

  GridX gridPaddedWidth(const GridNode* cell) const;
  GridY gridHeight(const GridNode* cell) const;
  GridY gridHeight(odb::dbMaster* master) const;
  DbuY rowHeight(GridY index);

  void paintPixel(GridNode* cell, GridX grid_x, GridY grid_y);
  void erasePixel(GridNode* cell);
  void visitCellPixels(GridNode& cell,
                       bool padded,
                       const std::function<void(Pixel* pixel)>& visitor) const;
  void visitCellBoundaryPixels(
      GridNode& cell,
      const std::function<
          void(Pixel* pixel, odb::Direction2D edge, GridX x, GridY y)>& visitor)
      const;

  GridY getRowCount() const { return row_count_; }
  GridX getRowSiteCount() const { return row_site_count_; }
  DbuX getSiteWidth() const { return site_width_; }

  Pixel* gridPixel(GridX x, GridY y) const;
  Pixel& pixel(GridY y, GridX x) { return pixels_[y.v][x.v]; }
  const Pixel& pixel(GridY y, GridX x) const { return pixels_[y.v][x.v]; }

  void resize(int size) { pixels_.resize(size); }
  void resize(GridY size) { pixels_.resize(size.v); }
  void resize(GridY y, GridX size) { pixels_[y.v].resize(size.v); }
  void clear();

  bool hasHybridRows() const { return has_hybrid_rows_; }

  GridY getRowCount(DbuY row_height) const;

  Rect getCore() const { return core_; }
  bool cellFitsInCore(GridNode* cell) const;

  bool isMultiHeight(dbMaster* master) const;

 private:
  void markHopeless(dbBlock* block,
                    int max_displacement_x,
                    int max_displacement_y);
  void markBlocked(dbBlock* block);
  void visitDbRows(dbBlock* block,
                   const std::function<void(odb::dbRow*)>& func) const;

  using Pixels = std::vector<std::vector<Pixel>>;
  Logger* logger_ = nullptr;
  dbBlock* block_ = nullptr;
  std::shared_ptr<Padding> padding_;
  Pixels pixels_;
  // Contains all the rows' yLo plus the yHi of the last row.  The extra
  // value is useful for operations like region snapping to rows
  std::map<DbuY, GridY> row_y_dbu_to_index_;
  std::vector<DbuY> row_index_to_y_dbu_;         // index is GridY
  std::vector<DbuY> row_index_to_pixel_height_;  // index is GridY

  bool has_hybrid_rows_ = false;
  Rect core_;

  std::optional<DbuY> uniform_row_height_;  // unset if hybrid
  DbuX site_width_{0};

  GridY row_count_{0};
  GridX row_site_count_{0};
};

}  // namespace dpl
