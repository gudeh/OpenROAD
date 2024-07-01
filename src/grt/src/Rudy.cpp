///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2024, Precision Innovations Inc.
// All rights reserved.
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

#include "grt/Rudy.h"

#include "grt/GRoute.h"
#include "grt/GlobalRouter.h"
#include "odb/dbShape.h"
#include "utl/Logger.h"

#include <iostream>

namespace grt {

Rudy::Rudy(odb::dbBlock* block, grt::GlobalRouter* grouter)
    : block_(block), grouter_(grouter)
{
  grid_block_ = block_->getDieArea();
  if (grid_block_.area() == 0) {
    std::cout<<"\n\ngrid_block_.area() is 0\n\n"<<std::endl;
    return;
  }

  if (!grouter_->isInitialized()) {
    int min_layer, max_layer;
    grouter_->setDbBlock(block);
    grouter_->getMinMaxLayer(min_layer, max_layer);
    grouter_->initFastRoute(min_layer, max_layer);
  }

  // The wire width is the harmonic average pitch divided by the number of
  // routing layers.
  double pitch_terms = 0;
  const int min_routing_layer = grouter->getMinRoutingLayer();
  const int max_routing_layer = grouter->getMaxRoutingLayer();
  std::cout<<"min_routing_layer: "<<min_routing_layer<<std::endl;
  std::cout<<"max_routing_layer: "<<max_routing_layer<<std::endl;
  const auto tech = block_->getTech();
  for (int layer_idx = min_routing_layer; layer_idx <= max_routing_layer;
       ++layer_idx) {
    const auto layer = tech->findRoutingLayer(layer_idx);
    int pitch = layer->getPitch();
    if (pitch == 0) {
      pitch = layer->getWidth() + layer->getSpacing();
    }
    pitch_terms += 1.0 / pitch;
  }
  wire_width_ = 1 / pitch_terms;  // = harm. mean / num_routing_layers

  std::cout<<"wire_width_: "<<wire_width_<<std::endl;
  int x_grids, y_grids;
  grouter_->getGridSize(x_grids, y_grids);
  tile_size_ = grouter_->getGridTileSize();
  setGridConfig(grid_block_, x_grids, y_grids);
  makeGrid();
}

void Rudy::setGridConfig(odb::Rect block, int tile_cnt_x, int tile_cnt_y)
{
  grid_block_ = block;
  tile_cnt_x_ = tile_cnt_x;
  tile_cnt_y_ = tile_cnt_y;
}

void Rudy::makeGrid()
{
  const int grid_lx = grid_block_.xMin();
  const int grid_ly = grid_block_.yMin();

  odb::Point upper_die_bounds(grid_block_.xMax(), grid_block_.yMax());
  odb::Point upper_grid_bounds(tile_cnt_x_ * tile_size_,
                               tile_cnt_y_ * tile_size_);
  int x_extra = upper_die_bounds.x() - upper_grid_bounds.x();
  int y_extra = upper_die_bounds.y() - upper_grid_bounds.y();

  grid_.resize(tile_cnt_x_);
  int cur_x = grid_lx;
  for (int x = 0; x < grid_.size(); x++) {
    grid_[x].resize(tile_cnt_y_);
    int cur_y = grid_ly;
    for (int y = 0; y < grid_[x].size(); y++) {
      Tile& grid = grid_[x][y];
      int x_ext = x == grid_.size() - 1 ? x_extra : 0;
      int y_ext = y == grid_[x].size() - 1 ? y_extra : 0;
      grid.setRect(
          cur_x, cur_y, cur_x + tile_size_ + x_ext, cur_y + tile_size_ + y_ext);
      cur_y += tile_size_;
    }
    cur_x += tile_size_;
  }
}

void Rudy::getResourceReductions()
{
  CapacityReductionData cap_usage_data;
  grouter_->getCapacityReductionData(cap_usage_data);
  std::cout<<"grid_.size(): "<<grid_.size()<<std::endl;
  std::cout<<"grid_[0].size(): "<<grid_[0].size()<<std::endl;
  std::cout<<"grid area: "<<(grid_.size()*grid_[0].size())<<std::endl;
  for (int x = 0; x < grid_.size(); x++) {
    for (int y = 0; y < grid_[x].size(); y++) {
      Tile& tile = getEditableTile(x, y);
      uint8_t tile_cap = cap_usage_data[x][y].capacity;
      float tile_reduction = cap_usage_data[x][y].reduction;
      float cap_usage_data = tile_reduction / tile_cap;
      tile.addRudy(cap_usage_data * 100);
      total_cap_usage+=(cap_usage_data * 100);
      //std::cout<<"cap_usage_data: "<<(cap_usage_data*100)<<std::endl;
    }
  }
}

void Rudy::calculateRudy()
{
//  grouter_->globalRoute();
  std::cout<<"\n\ncalculate rudy!\n\n";
  // Clear previous computation
  for (auto& grid_column : grid_) {
    for (auto& tile : grid_column) {
      tile.clearRudy();
    }
  }

  getResourceReductions();

  std::cout<<"number of nets: "<<block_->getNets().size()<<std::endl;
  // refer: https://ieeexplore.ieee.org/document/4211973
  for (auto net : block_->getNets()) {
    if (!net->getSigType().isSupply()) {
      std::cout<<"net name: "<< net->getName() <<std::endl;
      const auto net_rect = net->getTermBBox();
      processIntersectionSignalNet(net_rect);
    }
  }
   
  std::cout<<"total cap usage: "<<total_cap_usage<<std::endl;
  std::cout<<"total rudy:      "<<total_rudy<<std::endl;
  std::cout<<"nets_0_area :    "<<nets_0_area<<std::endl;
}

void Rudy::processIntersectionSignalNet(const odb::Rect net_rect)
{
  const auto net_area = net_rect.area();
  if (net_area == 0) {
    std::cout<<"net 0 area!\n";
    // TODO: handle nets with 0 area from getTermBBox()
    nets_0_area++;
    return;
  }
  std::cout<<"net_rect.area(): "<<net_rect.area()<<", dx()"<<net_rect.dx()<<", dy()"<<net_rect.dy();
  std::cout<<"("<<net_rect.xMin()<<","<<net_rect.yMin()<<") ("<<net_rect.xMax()<<","<<net_rect.yMax()<<")"<<std::endl;
  const auto hpwl = static_cast<float>(net_rect.dx() + net_rect.dy());
  const auto wire_area = hpwl * wire_width_;
  const auto net_congestion = wire_area / net_area;
  

  // Calculate the intersection range
  const int min_x_index
      = std::max(0, (net_rect.xMin() - grid_block_.xMin()) / tile_size_);
  const int max_x_index = std::min(
      tile_cnt_x_ - 1, (net_rect.xMax() - grid_block_.xMin()) / tile_size_);
  const int min_y_index
      = std::max(0, (net_rect.yMin() - grid_block_.yMin()) / tile_size_);
  const int max_y_index = std::min(
      tile_cnt_y_ - 1, (net_rect.yMax() - grid_block_.yMin()) / tile_size_);

//  std::cout<<"min_x_index: "<<min_x_index<<std::endl;
//  std::cout<<"max_x_index: "<<max_x_index<<std::endl;
//  std::cout<<"min_y_index: "<<min_y_index<<std::endl;
//  std::cout<<"max_y_index: "<<max_y_index<<std::endl;
  std::cout<<"grid indxs: x0,x1;y0,y1: "<<min_x_index<<","<<max_x_index<<", "<<min_y_index<<","<<max_y_index<<std::endl;

  std::cout<<"*hpwl: "<<hpwl<<", =wire_area: "<<wire_area<<", /net_area: "<<net_area<<", =net_congestion: "<<net_congestion<<std::endl;
  // Iterate over the tiles in the calculated range
  for (int x = min_x_index; x <= max_x_index; ++x) {
    for (int y = min_y_index; y <= max_y_index; ++y) {
      Tile& tile = getEditableTile(x, y);
      const auto tile_box = tile.getRect();
      if (net_rect.overlaps(tile_box)) {
        const auto intersect_area = net_rect.intersect(tile_box).area();
        const auto tile_area = tile_box.area();
        const auto tile_net_box_ratio = static_cast<float>(intersect_area)
                                        / static_cast<float>(tile_area);
        const auto rudy = net_congestion * tile_net_box_ratio * 100;
        tile.addRudy(rudy);
        total_rudy+=rudy;
        //std::cout<<"rudy: "<<rudy<<std::endl;
      }
    }
  }
}

std::pair<int, int> Rudy::getGridSize() const
{
  if (grid_.empty()) {
    return {0, 0};
  }
  return {grid_.size(), grid_.at(0).size()};
}

void Rudy::Tile::setRect(int lx, int ly, int ux, int uy)
{
  rect_ = odb::Rect(lx, ly, ux, uy);
}

void Rudy::Tile::addRudy(float rudy)
{
  rudy_ += rudy;
}

}  // namespace grt
