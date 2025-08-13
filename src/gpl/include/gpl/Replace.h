// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2018-2025, The OpenROAD Authors

#pragma once

#include <memory>
#include <string>
#include <vector>
#include "odb/db.h"
#include "odb/geom.h"
#include "../src/placerBase.h"

namespace odb {
class dbDatabase;
class dbInst;
class dbBTerm;
class Rect;

}  // namespace odb
namespace sta {
class dbSta;
}

namespace grt {
class GlobalRouter;
}

namespace rsz {
class Resizer;
}

namespace utl {
class Logger;
}

namespace gpl {

class PlacerBaseCommon;
class PlacerBase;
class Pin;
class NesterovBaseCommon;
class NesterovBase;
class RouteBase;
class TimingBase;

class InitialPlace;
class NesterovPlace;

using Cluster = std::vector<odb::dbInst*>;
using Clusters = std::vector<Cluster>;

class Replace
{
 public:
  Replace();
  ~Replace();

  void init(odb::dbDatabase* odb,
            sta::dbSta* sta,
            rsz::Resizer* resizer,
            grt::GlobalRouter* router,
            utl::Logger* logger);
  void reset();

  void doIncrementalPlace(int threads);
  void doInitialPlace(int threads);
  void runMBFF(int max_sz, float alpha, float beta, int threads, int num_paths);

  void addPlacementCluster(const Cluster& cluster);
  int doNesterovPlace(int threads, int start_iter = 0);

  // Initial Place param settings
  void setInitialPlaceMaxIter(int iter);
  void setInitialPlaceMinDiffLength(int length);
  void setInitialPlaceMaxSolverIter(int iter);
  void setInitialPlaceMaxFanout(int fanout);
  void setInitialPlaceNetWeightScale(float scale);

  void setNesterovPlaceMaxIter(int iter);

  void setBinGridCnt(int binGridCntX, int binGridCntY);

  void setTargetDensity(float density);
  // Execute gpl with uniform density as target density
  void setUniformTargetDensityMode(bool mode);
  void setTargetOverflow(float overflow);
  void setInitDensityPenalityFactor(float penaltyFactor);
  void setInitWireLengthCoef(float coef);
  void setMinPhiCoef(float minPhiCoef);
  void setMaxPhiCoef(float maxPhiCoef);

  // Query for uniform density value
  float getUniformTargetDensity(int threads);

  // HPWL: half-parameter wire length.
  void setReferenceHpwl(float refHpwl);

  // temp funcs; OpenDB should have these values.
  void setPadLeft(int padding);
  void setPadRight(int padding);

  void setTimingDrivenMode(bool mode);

  // void setSkipIoMode(bool mode);
  void setDisableRevertIfDiverge(bool mode);

  void setRoutabilityDrivenMode(bool mode);
  void setRoutabilityUseGrt(bool mode);
  void setRoutabilityCheckOverflow(float overflow);
  void setRoutabilityMaxDensity(float density);
  void setRoutabilityMaxInflationIter(int iter);
  void setRoutabilityTargetRcMetric(float rc);
  void setRoutabilityInflationRatioCoef(float coef);
  void setRoutabilityMaxInflationRatio(float ratio);
  void setRoutabilityRcCoefficients(float k1, float k2, float k3, float k4);
  void setEnableRoutingCongestion(bool mode);

  void addTimingNetWeightOverflow(int overflow);
  void setTimingNetWeightMax(float max);
  void setKeepResizeBelowOverflow(float overflow);

  void setDebug(int pause_iterations,
                int update_iterations,
                bool draw_bins,
                bool initial,
                odb::dbInst* inst,
                int start_iter,
                bool generate_images,
                std::string images_path);

 private:
  bool initNesterovPlace(int threads);

  odb::dbDatabase* db_ = nullptr;
  sta::dbSta* sta_ = nullptr;
  rsz::Resizer* rs_ = nullptr;
  grt::GlobalRouter* fr_ = nullptr;
  utl::Logger* log_ = nullptr;

  std::shared_ptr<PlacerBaseCommon> pbc_;
  std::shared_ptr<NesterovBaseCommon> nbc_;
  std::vector<std::shared_ptr<PlacerBase>> pbVec_;
  std::vector<std::shared_ptr<NesterovBase>> nbVec_;
  std::shared_ptr<RouteBase> rb_;
  std::shared_ptr<TimingBase> tb_;

  std::unique_ptr<InitialPlace> ip_;
  std::unique_ptr<NesterovPlace> np_;

  int initialPlaceMaxIter_ = 20;
  int initialPlaceMinDiffLength_ = 1500;
  int initialPlaceMaxSolverIter_ = 100;
  int initialPlaceMaxFanout_ = 200;
  float initialPlaceNetWeightScale_ = 800;

  int total_placeable_insts_ = 0;

  int nesterovPlaceMaxIter_ = 5000;
  int binGridCntX_ = 0;
  int binGridCntY_ = 0;
  float overflow_ = 0.1;
  float density_ = 1.0;
  float initDensityPenalityFactor_ = 0.00008;
  float initWireLengthCoef_ = 0.25;
  float minPhiCoef_ = 0.95;
  float maxPhiCoef_ = 1.05;
  float referenceHpwl_ = 446000000;

  float routabilityCheckOverflow_ = 0.3;
  float routabilityMaxDensity_ = 0.99;
  float routabilityTargetRcMetric_ = 1.01;
  float routabilityInflationRatioCoef_ = 3;
  float routabilityMaxInflationRatio_ = 6;
  int routabilityMaxInflationIter_ = 4;

  // routability RC metric coefficients
  float routabilityRcK1_ = 1.0;
  float routabilityRcK2_ = 1.0;
  float routabilityRcK3_ = 0.0;
  float routabilityRcK4_ = 0.0;

  float timingNetWeightMax_ = 5;
  float keepResizeBelowOverflow_ = 1.0;

  bool timingDrivenMode_ = true;
  bool routabilityDrivenMode_ = true;
  bool routabilityUseRudy_ = true;
  bool uniformTargetDensityMode_ = false;
  // bool skipIoMode_ = false;
  bool disableRevertIfDiverge_ = false;
  bool enable_routing_congestion_ = false;

  std::vector<int> timingNetWeightOverflows_;
  Clusters clusters_;

  // temp variable; OpenDB should have these values.
  int padLeft_ = 0;
  int padRight_ = 0;
  bool gui_debug_ = false;
  int gui_debug_pause_iterations_ = 10;
  int gui_debug_update_iterations_ = 10;
  int gui_debug_draw_bins_ = false;
  int gui_debug_initial_ = false;
  odb::dbInst* gui_debug_inst_ = nullptr;
  int gui_debug_start_iter_ = 0;
  bool gui_debug_generate_images_ = false;
  std::string gui_debug_images_path_ = "REPORTS_DIR";
};

inline constexpr const char* format_label_int = "{:27} {:10}";
inline constexpr const char* format_label_float = "{:27} {:10.4f}";
inline constexpr const char* format_label_um2 = "{:27} {:10.3f} um^2";
inline constexpr const char* format_label_percent = "{:27} {:10.2f} %";
inline constexpr const char* format_label_um2_with_delta
    = "{:27} {:10.3f} um^2 ({:+.2f}%)";



// Determines the appropriate coordinates for placing an IO pin based on its predefined placement,
//  constraint region, or proximity to the net bounding box and die edges, while avoiding excluded regions.
inline std::pair<int, int> computeIOCoordi(
      odb::dbBTerm* bTerm,
      const odb::Rect& net_bbox,
      gpl::Pin* pb_pin,
      utl::Logger* logger)
  {
    bool is_predefined_placed = pb_pin->isBTermPredefinedPlacement();
   
    if (is_predefined_placed) {
      int cx = pb_pin->cx();
      int cy = pb_pin->cy();
      debugPrint(logger,
                 utl::GPL,
                 "io_constraint",
                 2,
                 "A: {} already placed in odb: ({}, {})",
                 bTerm->getConstName(),
                 cx,
                 cy);
      return {cx, cy};
    }

    int net_cx = (net_bbox.xMin() + net_bbox.xMax()) / 2;
    int net_cy = (net_bbox.yMin() + net_bbox.yMax()) / 2;
    debugPrint(logger, utl::GPL, "io_constraint", 2,
           "Net bbox center for {} is ({}, {})",
           bTerm->getConstName(), net_cx, net_cy);


    const auto& constraint_region = bTerm->getConstraintRegion();
    if (!is_predefined_placed && constraint_region) {
      if(pb_pin->getBTermStatus() != Pin::BTermPlacementStatus::CONSTRAINT_REGION) {
        logger->warn(utl::GPL, 9998, 
                     "BTerm {} is not marked as CONSTRAINT_REGION, but has a constraint region defined. ",
                     bTerm->getConstName());
      }
      int cx = 0;
      int cy = 0;
      int target_x = 0;
      int target_y = 0;
      target_x = (net_bbox.xMin() + net_bbox.xMax()) / 2;
      target_y = (net_bbox.yMin() + net_bbox.yMax()) / 2;

      int xMin = constraint_region->xMin();
      int xMax = constraint_region->xMax();
      int yMin = constraint_region->yMin();
      int yMax = constraint_region->yMax();

      debugPrint(logger,
                 utl::GPL,
                 "io_constraint",
                 2,
                 "target for {}: ({}, {})",
                 bTerm->getConstName(),
                 target_x,
                 target_y);

      debugPrint(logger,
                 utl::GPL,
                 "io_constraint",
                 2,
                 "Constraint region for {}: [({}, {}) --> ({}, {})]",
                 bTerm->getConstName(),
                 xMin,
                 yMin,
                 xMax,
                 yMax);

      cx = (xMin == xMax) ? xMin : std::clamp(target_x, xMin, xMax);
      cy = (yMin == yMax) ? yMin : std::clamp(target_y, yMin, yMax);

      debugPrint(logger,
                 utl::GPL,
                 "io_constraint",
                 2,
                 "Final clamped coord for {}: ({}, {})",
                 bTerm->getConstName(),
                 cx,
                 cy);

      return {cx, cy};
    }

    if(!is_predefined_placed && !constraint_region) {
      if(pb_pin->getBTermStatus() != Pin::BTermPlacementStatus::PROJECTED) {
        logger->warn(utl::GPL, 9997, 
                     "BTerm {} is not marked as PROJECTED, but has no constraint region defined. ",
                     bTerm->getConstName());
      }
      odb::Rect die = bTerm->getBlock()->getDieArea();
      const auto& excluded_regions = bTerm->getBlock()->getBlockedRegionsForPins();

      std::vector<std::pair<int, int>> candidates = {
          {die.xMin(), net_cy},
          {die.xMax(), net_cy},
          {net_cx, die.yMin()},
          {net_cx, die.yMax()}
      };

      int64_t best_dist = std::numeric_limits<int64_t>::max();
      int best_x = std::numeric_limits<int>::min();
      int best_y = std::numeric_limits<int>::min();

      for (const auto& [x, y] : candidates) {
        bool excluded_point = false;
        for (const odb::Rect& r : excluded_regions) {
          if (r.xMin() <= x && x <= r.xMax() && r.yMin() <= y && y <= r.yMax()) {
            excluded_point = true;
            break;
          }
        }
        if (excluded_point) { 
          continue;
  }

        // Manhattan distance
        int64_t dx = static_cast<int64_t>(x) - static_cast<int64_t>(net_cx);
        int64_t dy = static_cast<int64_t>(y) - static_cast<int64_t>(net_cy);
        int64_t dist = std::llabs(dx) + std::llabs(dy);

        if (dist < best_dist) {
          best_dist = dist;
          best_x = x;
          best_y = y;
        }
      }


      if (best_x >= 0) {
        debugPrint(logger,
                  utl::GPL,
                  "io_constraint",
                  2,
                  "D: {} projected to die edge avoiding excluded: ({}, {})",
                  bTerm->getConstName(),
                  best_x,
                  best_y);
        return {best_x, best_y};
      } else {
        debugPrint(logger,
                  utl::GPL,
                  "io_constraint",
                  2,
                  "E: {} unconstrained but no valid edge available → Ignored.",
                  bTerm->getConstName());
        return {0, 0};
      }
    }
    
    debugPrint(logger,
            utl::GPL,
            "io_constraint",
            2,
            "F: {} This should never happen.",
            bTerm->getConstName());
    return {-2000, -2000};
  }
}  // namespace gpl
