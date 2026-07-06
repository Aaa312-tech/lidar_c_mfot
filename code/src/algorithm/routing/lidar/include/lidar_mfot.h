#pragma once

#include <cstddef>
#include <iosfwd>
#include <string>
#include <unordered_map>
#include <vector>

#include "lidar_drc.h"
#include "lidar_router.h"

namespace picpr::lidar
{

struct LidarMfOtNetPlan
{
  std::string netName;
  std::unordered_map<int, double> corridorCost;
  double priority = 0.0;
  double freeEnergy = 0.0;
  double corridorShrinkRatio = 1.0;
  int bboxCells = 0;
  int sampledCells = 0;
};

struct LidarMfOtPlan
{
  bool enabled = false;
  int width = 0;
  int height = 0;
  int stride = 1;
  int iterations = 0;
  std::unordered_map<std::string, LidarMfOtNetPlan> nets;
  std::vector<double> density;
  double totalFreeEnergy = 0.0;
  double maxPriority = 0.0;
  std::size_t totalCorridorCells = 0;
};

LidarMfOtPlan buildMfOtPlan(const LidarRuntimeView& db,
                            const LidarDrcManager& drc,
                            const LidarRouteConfig& config);

double mfotCellPenalty(const LidarMfOtPlan& plan,
                       const LidarRouteConfig& config,
                       const std::string& netName,
                       int x,
                       int y,
                       bool* inCorridor = nullptr);

double mfotNetPriority(const LidarMfOtPlan& plan, const std::string& netName);

double mfotSearchWeightForNet(const LidarMfOtPlan& plan,
                              const LidarRouteConfig& config,
                              const std::string& netName);

void applyMfOtGlobalPriorities(LidarRuntimeView& db,
                               const LidarMfOtPlan& plan,
                               const LidarRouteConfig& config);

void seedMfOtHistoryMap(const LidarMfOtPlan& plan,
                        const LidarRouteConfig& config,
                        std::vector<int>& historyMap,
                        int historyWidth,
                        int historyHeight);

void writeMfOtSummary(const LidarMfOtPlan& plan, std::ostream& os);

}  // namespace picpr::lidar
