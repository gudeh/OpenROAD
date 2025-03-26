/////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2022, The Regents of the University of California
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
//
///////////////////////////////////////////////////////////////////////////////

#include "ord/Tech.h"

#include <tcl.h>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "ord/OpenRoad.hh"
#include "sta/Corner.hh"
#include "sta/Liberty.hh"
#include "sta/Units.hh"

namespace ord {

Tech::Tech(Tcl_Interp* interp,
           const char* log_filename,
           const char* metrics_filename)
    : app_(new OpenRoad())
{
  if (!interp) {
    interp = Tcl_CreateInterp();
    Tcl_Init(interp);
    app_->init(interp, log_filename, metrics_filename);
  }
}

Tech::~Tech()
{
  delete app_;
}

odb::dbDatabase* Tech::getDB()
{
  return app_->getDb();
}

odb::dbTech* Tech::getTech()
{
  return getDB()->getTech();
}

sta::dbSta* Tech::getSta()
{
  return app_->getSta();
}

void Tech::readLef(const std::string& file_name)
{
  const bool make_tech = getDB()->getTech() == nullptr;
  const bool make_library = true;
  std::string lib_name = file_name;

  // Hacky but easier than dealing with stdc++fs linking
  auto slash_pos = lib_name.find_last_of('/');
  if (slash_pos != std::string::npos) {
    lib_name.erase(0, slash_pos + 1);
  }
  auto dot_pos = lib_name.find_last_of('.');
  if (dot_pos != std::string::npos) {
    lib_name.erase(lib_name.begin() + dot_pos, lib_name.end());
  }

  app_->readLef(
      file_name.c_str(), lib_name.c_str(), "", make_tech, make_library);
}

void Tech::readLiberty(const std::string& file_name)
{
  // TODO: take corner & min/max args
  getSta()->readLiberty(file_name.c_str(),
                        getSta()->cmdCorner(),
                        sta::MinMaxAll::all(),
                        true /* infer_latches */);
}

float Tech::nominalProcess()
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::LibertyLibrary* lib = network->defaultLibertyLibrary();
  return lib->nominalProcess();
}

float Tech::nominalVoltage()
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::LibertyLibrary* lib = network->defaultLibertyLibrary();
  return lib->nominalVoltage();
}

float Tech::nominalTemperature()
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::LibertyLibrary* lib = network->defaultLibertyLibrary();
  return lib->nominalTemperature();
}

float Tech::timeScale()
{
  return getSta()->units()->timeUnit()->scale();
}

float Tech::resistanceScale()
{
  return getSta()->units()->resistanceUnit()->scale();
}

float Tech::capacitanceScale()
{
  return getSta()->units()->capacitanceUnit()->scale();
}

float Tech::voltageScale()
{
  return getSta()->units()->voltageUnit()->scale();
}

float Tech::currentScale()
{
  return getSta()->units()->currentUnit()->scale();
}

float Tech::powerScale()
{
  return getSta()->units()->powerUnit()->scale();
}

float Tech::distanceScale()
{
  return getSta()->units()->distanceUnit()->scale();
}

}  // namespace ord
