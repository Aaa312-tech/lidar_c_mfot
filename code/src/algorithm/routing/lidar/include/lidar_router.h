#pragma once

#include <iosfwd>
#include <string>
#include <vector>

#include "picdb_lidar_view.h"

namespace picpr::lidar
{

struct LidarRouteConfig
{
  std::string netOrder = "topo";
  bool        group    = true;
  int         maxIteration = 10;
  bool        enable45Neighbor = true;
  double      gridResolution = 2.0;
  double      bendRadius = 5.0;
  double      netBoundScaleFactor = 1.5;
  int         netDefaultBound = 100;
  double      lossPropagation = 1.5;
  double      lossBending = 50.0;
  double      lossCrossing = 0.0;
  double      lossCongestion = 500.0;
  double      ilCross = 0.5;
  double      bendPointsDistance = 0.02;
  int         historyCost = 1000;
  bool        enableMfOtRouting = true;
  int         mfotIterations = 3;
  int         mfotGridStride = 8;
  int         mfotMaxSamplesPerNet = 1024;
  int         mfotCorridorRadius = 5;
  double      mfotEpsilon = 90.0;
  double      mfotLambdaCongestion = 160.0;
  double      mfotDetourScale = 1.0;
  double      mfotObstaclePenalty = 600.0;
  double      mfotCorridorEnergySlack = 110.0;
  double      mfotOutsidePenalty = 0.0;
  double      mfotPotentialScale = 0.0;
  double      mfotHistoryScale = 0.0;
  double      mfotPriorityScale = 0.0;
  double      mfotSearchWeight = 1.23;
  bool        mfotHardCorridor = false;
};

class LidarGridRouter
{
 public:
  LidarGridRouter(LidarRuntimeView& db, LidarRouteConfig config = LidarRouteConfig());

  int processNetOrder();
  std::vector<std::string> globalPriorityOrder() const;

 private:
  bool segmentsIntersect(const LidarPoint& a,
                         const LidarPoint& b,
                         const LidarPoint& c,
                         const LidarPoint& d) const;

  LidarRuntimeView&  _db;
  LidarRouteConfig _config;
};

void writeRouteInitSummary(LidarRuntimeView& db,
                           const LidarRouteConfig& config,
                           std::ostream& os);

}  // namespace picpr::lidar
