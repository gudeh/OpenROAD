/////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2024, Precision Innovations Inc.
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

#include "dpl/Grid.h"
#include "dpl/Opendp.h"

namespace dpl {

using odb::dbOrientType;
using odb::dbSite;

struct Edge
{
  Edge(unsigned int type, const odb::Rect& box)
      : edge_type_idx_(type), bbox_(box)
  {
  }
  unsigned int getEdgeType() const { return edge_type_idx_; }
  const odb::Rect& getBBox() const { return bbox_; }

 private:
  unsigned int edge_type_idx_;
  odb::Rect bbox_;
};

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

struct Master
{
  bool is_multi_row = false;
  std::vector<Edge> edges_;
};

struct Cell : public GridNode
{
 public:
  const char* name() const;
  bool isFixed() const override;
  bool isPlaced() const override { return is_placed_; }
  bool isHybrid() const override;
  DbuX xMax() const { return x_ + width_; }
  DbuX xMin() const override { return x_; }
  DbuY yMin() const override { return y_; }
  DbuX dx() const override { return width_; }
  DbuY dy() const override { return height_; }
  dbInst* getDbInst() const override { return db_inst_; }
  DbuX siteWidth() const override;
  void setPlaced(bool in) { is_placed_ = in; }
  void setHold(bool in) { hold_ = in; }
  void setOrient(const dbOrientType& in) { orient_ = in; }
  void setLeft(DbuX in) { x_ = in; }
  void setBottom(DbuY in) { y_ = in; }

  bool inGroup() const { return group_ != nullptr; }
  int64_t area() const;
  bool isStdCell() const;

  bool isHybridParent() const;
  dbSite* getSite() const;

  bool isBlock() const;

  dbInst* db_inst_ = nullptr;
  DbuX x_{0};  // lower left wrt core DBU
  DbuY y_{0};
  dbOrientType orient_;
  DbuX width_{0};
  DbuY height_{0};
  bool is_placed_ = false;
  bool hold_ = false;
  Group* group_ = nullptr;
  Rect* region_ = nullptr;  // group rect
};

struct Group
{
  string name;
  vector<Rect> region_boundaries;
  vector<Cell*> cells_;
  Rect boundary;
  double util = 0.0;
};

}  // namespace dpl
