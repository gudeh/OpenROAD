// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors

#include "Graphics.h"

#include "dpl/Opendp.h"
#include "gui/gui.h"
#include "infrastructure/Grid.h"
#include "infrastructure/Objects.h"
#include "infrastructure/network.h"
#include "odb/db.h"
#include "odb/geom.h"

namespace dpl {

using odb::dbBox;

Graphics::Graphics(Opendp* dp,
                   float min_displacement,
                   const odb::dbInst* debug_instance)
    : dp_(dp),
      debug_instance_(debug_instance),
      min_displacement_(min_displacement)
{
  gui::Gui::get()->registerRenderer(this);
}

void Graphics::startPlacement(odb::dbBlock* block)
{
  block_ = block;
}

void Graphics::placeInstance(odb::dbInst* instance)
{
  if (!instance || instance != debug_instance_) {
    return;
  }

  auto gui = gui::Gui::get();

  auto selected = gui->makeSelected(instance);
  gui->setSelected(selected);
  gui->redraw();
  gui->pause();
}

void Graphics::binSearch(const Node* cell,
                         GridX xl,
                         GridY yl,
                         GridX xh,
                         GridY yh)
{
  if (!debug_instance_ || cell->getDbInst() != debug_instance_) {
    return;
  }
  odb::Rect core = dp_->grid_->getCore();
  int xl_dbu = core.xMin() + gridToDbu(xl, dp_->grid_->getSiteWidth()).v;
  int yl_dbu = core.yMin() + dp_->grid_->gridYToDbu(yl).v;
  int xh_dbu = core.xMin() + gridToDbu(xh, dp_->grid_->getSiteWidth()).v;
  int yh_dbu = core.yMin() + dp_->grid_->gridYToDbu(yh).v;
  searched_.emplace_back(xl_dbu, yl_dbu, xh_dbu, yh_dbu);
}

void Graphics::endPlacement()
{
  auto gui = gui::Gui::get();
  gui->redraw();
  gui->pause();
}

void Graphics::drawObjects(gui::Painter& painter)
{
  if (!block_) {
    return;
  }

  odb::Rect core = block_->getCoreArea();

  for (const auto& cell : dp_->network_->getNodes()) {
    if (!cell->isPlaced()) {
      continue;
    }
    // Compare the squared distances to save calling sqrt
    float min_length = min_displacement_ * dp_->grid_->gridHeight(cell.get()).v;
    min_length *= min_length;
    DbuX lx{core.xMin() + cell->getLeft()};
    DbuY ly{core.yMin() + cell->getBottom()};

    auto color = cell->getDbInst() ? gui::Painter::kGray : gui::Painter::kRed;
    painter.setPen(color);
    painter.setBrush(gui::Painter::kTransparent);
    painter.drawRect(odb::Rect(
        lx.v, ly.v, lx.v + cell->getWidth().v, ly.v + cell->getHeight().v));

    if (!cell->getDbInst()) {
      continue;
    }

    dbBox* bbox = cell->getDbInst()->getBBox();
    odb::Point initial_location(bbox->xMin(), bbox->yMin());
    odb::Point final_location(lx.v, ly.v);
    float len = odb::Point::squaredDistance(initial_location, final_location);
    if (len < min_length) {
      continue;
    }

    painter.setPen(gui::Painter::kYellow, /* cosmetic */ true);
    painter.drawLine(initial_location.x(),
                     initial_location.y(),
                     final_location.x(),
                     final_location.y());
    painter.drawCircle(final_location.x(), final_location.y(), 100);
  }

  auto color = gui::Painter::kCyan;
  painter.setPen(color);
  painter.setBrush(color);
  for (auto& rect : searched_) {
    painter.drawRect(rect);
  }



  if (!dp_ || !dp_->grid_)
    return;

  // // Draw the x and y grid values as a string at the center of the pixel if enabled
  // for (GridY y{0}; y < dp_->grid_->getRowCount(); ++y) {
  //   for (GridX x{0}; x < dp_->grid_->getRowSiteCount(); ++x) {
  //     std::string label = std::to_string(x.v) + "," + std::to_string(y.v);
  //     painter.setPen(gui::Painter::kWhite);
  //     int dbu_x = dp_->grid_->getCore().xMin() + gridToDbu(x, dp_->grid_->getSiteWidth()).v + (dp_->grid_->getSiteWidth().v / 2);
  //     int dbu_y = dp_->grid_->getCore().yMin() + dp_->grid_->gridYToDbu(y).v + (dp_->grid_->rowHeight(y).v / 2);
  //     painter.drawString(dbu_x, dbu_y, gui::Painter::Anchor::kCenter, label, /*rotate_90=*/false);
  //   }
  // }

  // Then, draw the blocked pixels
  for (GridY y{0}; y < dp_->grid_->getRowCount(); ++y) {
    for (GridX x{0}; x < dp_->grid_->getRowSiteCount(); ++x) {
      const Pixel& pixel = dp_->grid_->pixel(y, x);
      int dbu_x = dp_->grid_->getCore().xMin() + gridToDbu(x, dp_->grid_->getSiteWidth()).v;
      int dbu_y = dp_->grid_->getCore().yMin() + dp_->grid_->gridYToDbu(y).v;
      if (pixel.blocked_layers != 0) {
        painter.setPen(gui::Painter::kRed);
        painter.setBrush(gui::Painter::kRed);
        odb::Rect rect(dbu_x, dbu_y, dbu_x + dp_->grid_->getSiteWidth().v, 
                      dbu_y + dp_->grid_->rowHeight(y).v);
        painter.drawRect(rect);
      }
    }
  }
}

/* static */
bool Graphics::guiActive()
{
  return gui::Gui::enabled();
}
}  // namespace dpl
