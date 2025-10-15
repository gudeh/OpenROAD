// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors

#include "utility.h"

#include <algorithm>
#include <cstdint>

#include "dpl/Opendp.h"
#include "infrastructure/Coordinates.h"
#include "infrastructure/network.h"
#include "odb/geom.h"

namespace dpl {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
double Utility::disp_l1(Network* nw, double& tot, double& max, double& avg)
{
  // Returns L1 displacement of the current placement from that stored.
  tot = 0.;
  max = 0.;
  avg = 0.;
  for (int i = 0; i < nw->getNumNodes(); i++) {
    const Node* ndi = nw->getNode(i);

    const DbuX dx = abs(ndi->getLeft() - ndi->getOrigLeft());
    const DbuY dy = abs(ndi->getBottom() - ndi->getOrigBottom());
    tot += dx.v + dy.v;
    max = std::max(max, (double) dx.v + dy.v);
  }
  avg = tot / (double) nw->getNumNodes();

  return tot;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
uint64_t Utility::hpwl(const Network* nw)
{
  // Compute the wire length for the given placement.
  uint64_t totWL = 0;
  for (unsigned e = 0; e < nw->getNumEdges(); e++) {
    const Edge* ed = nw->getEdge(e);
    totWL += hpwl(ed);
  }
  return totWL;
}

uint64_t Utility::hpwl(const Network* nw, uint64_t& hpwlx, uint64_t& hpwly)
{
  hpwlx = 0;
  hpwly = 0;
  // Compute the wire length for the given placement.
  unsigned numEdges = nw->getNumEdges();
  odb::Rect box;
  for (unsigned e = 0; e < numEdges; e++) {
    const Edge* ed = nw->getEdge(e);

    const int numPins = ed->getNumPins();
    if (numPins <= 1) {
      continue;
    }

    box.mergeInit();
    for (const Pin* pin : ed->getPins()) {
      const Node* ndi = pin->getNode();
      const DbuY py = ndi->getCenterY() + pin->getOffsetY();
      const DbuX px = ndi->getCenterX() + pin->getOffsetX();
      box.merge(odb::Point(px.v, py.v));
    }
    hpwlx += box.dx();
    hpwly += box.dy();
  }
  return hpwlx + hpwly;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
uint64_t Utility::hpwl(const Edge* ed)
{
  uint64_t hpwlx = 0;
  uint64_t hpwly = 0;
  return hpwl(ed, hpwlx, hpwly);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
uint64_t Utility::hpwl(const Edge* ed, uint64_t& hpwlx, uint64_t& hpwly)
{
  hpwlx = 0;
  hpwly = 0;

  const int numPins = ed->getNumPins();
  if (numPins <= 1) {
    return 0;
  }

  odb::Rect box;
  for (const Pin* pin : ed->getPins()) {
    const Node* ndi = pin->getNode();
    const DbuY py = ndi->getCenterY() + pin->getOffsetY();
    const DbuX px = ndi->getCenterX() + pin->getOffsetX();
    box.merge(odb::Point(px.v, py.v));
  }
  hpwlx = box.dx();
  hpwly = box.dy();
  return hpwlx + hpwly;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool Utility::match(const std::string& str, const std::string& pattern)
{
  // Convert the pattern to a regex pattern
  // Replace {#} with \d+ (one or more digits)
  std::string regex_pattern = pattern;
  size_t pos = regex_pattern.find("{#}");
  if (pos != std::string::npos) {
    regex_pattern.replace(pos, 3, "\\d+");
  }
  // Create regex object and match
  std::regex re(regex_pattern);
  return std::regex_match(str, re);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int Utility::extractPinIdx(const std::string& pin_name,
                           const std::string& pattern)
{
  // Convert pattern to regex pattern by replacing {#} with capture group for
  // digits
  std::string regex_pattern = pattern;
  size_t pos = regex_pattern.find("{#}");
  if (pos != std::string::npos) {
    regex_pattern.replace(pos, 3, "(\\d+)");  // Capture the digits
  }

  std::regex re(regex_pattern);
  std::smatch match;
  if (std::regex_match(pin_name, match, re) && match.size() > 1) {
    return std::stoi(match[1].str());  // Return the captured digits
  }
  return -1;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
std::string Utility::getPinName(const std::string& pattern, int idx)
{
  // Replace {#} with the actual index
  std::string result = pattern;
  size_t pos = result.find("{#}");
  if (pos != std::string::npos) {
    result.replace(pos, 3, std::to_string(idx));
  }
  return result;
}

}  // namespace dpl
