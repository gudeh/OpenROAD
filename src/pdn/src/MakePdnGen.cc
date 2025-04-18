// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2022-2025, The OpenROAD Authors

#include "pdn/MakePdnGen.hh"

#include <tcl.h>

#include "domain.h"
#include "grid.h"
#include "ord/OpenRoad.hh"
#include "pdn/PdnGen.hh"
#include "power_cells.h"
#include "renderer.h"

namespace sta {

extern const char* pdn_tcl_inits[];
extern void evalTclInit(Tcl_Interp*, const char*[]);

}  // namespace sta

namespace pdn {
extern "C" {
extern int Pdn_Init(Tcl_Interp* interp);
}
}  // namespace pdn

namespace ord {

void initPdnGen(OpenRoad* openroad)
{
  Tcl_Interp* interp = openroad->tclInterp();
  // Define swig TCL commands.
  pdn::Pdn_Init(interp);
  // Eval encoded sta TCL sources.
  sta::evalTclInit(interp, sta::pdn_tcl_inits);

  openroad->getPdnGen()->init(openroad->getDb(), openroad->getLogger());
}

pdn::PdnGen* makePdnGen()
{
  return new pdn::PdnGen();
}

void deletePdnGen(pdn::PdnGen* pdngen)
{
  delete pdngen;
}

}  // namespace ord
