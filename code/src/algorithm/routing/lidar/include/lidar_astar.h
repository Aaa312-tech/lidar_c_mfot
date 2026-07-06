#pragma once

#include <array>
#include <cstddef>
#include <iosfwd>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "lidar_drc.h"
#include "lidar_mfot.h"
#include "lidar_router.h"

namespace picpr::lidar
{

struct LidarAstarStep
{
  int         dx = 0;
  int         dy = 0;
  int         orientation = 0;
  std::string type;
};

struct LidarAstarInitState
{
  std::string netName;
  int         connectThreshold = 0;
  bool        repel = false;
  int         unrouted = 0;
  double      checkRegion = 0.0;
  std::array<std::array<int, 2>, 2> routingBound = {{{0, 0}, {0, 0}}};
  int         crossingBudget = 0;
  double      radius = 0.0;
  int         gridRadius = 0;
  int         bend45Part1 = 0;
  int         bend45Part2 = 0;
  int         predictLength = 0;
  std::map<std::string, double> stepG;
  std::map<int, std::vector<LidarAstarStep>> nextSteps;
  std::string startPortName;
  std::string endPortName;
  std::array<int, 3> startNode = {0, 0, 0};
  std::array<int, 3> endNode = {0, 0, 0};
  double      startCostF = 0.0;
  std::size_t accessPoints = 0;
};

struct LidarAstarRouteResult
{
  std::string                     netName;
  bool                            success = false;
  bool                            strictDrc = true;
  bool                            shortSbend = false;
  double                          shortSbendLength = 0.0;
  std::vector<std::array<int, 3>> originPath;
  std::vector<std::array<int, 3>> processedPath;
  LidarPythonStringSet            crossingNets;
  std::set<std::string>           violatedNets;
  std::size_t                     visitedNodes = 0;
  std::size_t                     expandedNodes = 0;
  double                          finalCostG = 0.0;
  double                          finalCostF = 0.0;
  std::vector<std::array<int, 3>> popTrace;
  std::vector<std::array<double, 2>> popTraceCost;
  std::vector<std::array<int, 3>> popTraceParent;
};

struct LidarGridRouteFlowEntry
{
  int                    iteration = 0;
  std::string            groupName;
  LidarAstarRouteResult  route;
};

struct LidarGridRouteFlowResult
{
  bool                            success = false;
  std::vector<LidarGridRouteFlowEntry> entries;
  std::set<std::string>           abnormalNets;
  std::size_t                     historyNonzero = 0;
  long long                       historySum = 0;
  bool                            mfotEnabled = false;
  std::size_t                     mfotNets = 0;
  std::size_t                     mfotCorridorCells = 0;
  double                          mfotFreeEnergy = 0.0;
};

struct LidarRouteWritebackResult
{
  bool        success = false;
  std::size_t routedNets = 0;
  std::size_t waveguideSegments = 0;
  std::size_t skippedNets = 0;
  std::size_t postprocessInvalidAccesses = 0;
  std::size_t postprocessRejectedNets = 0;
};

LidarAstarInitState buildAstarInitState(const LidarRuntimeView& db,
                                        const LidarDrcManager& drc,
                                        const LidarNet& net,
                                        const LidarRouteConfig& config,
                                        int drUnrouted);

LidarAstarRouteResult routeSingleNetGrid(LidarRuntimeView& db,
                                         LidarDrcManager& drc,
                                         LidarNet& net,
                                         const LidarRouteConfig& config,
                                         bool strictDrc = true,
                                         int drUnrouted = 2,
                                         const std::set<std::string>& groups = {},
                                         const std::vector<int>* historyMap = nullptr,
                                         int historyWidth = 0,
                                         int historyHeight = 0,
                                         const LidarMfOtPlan* mfotPlan = nullptr);

LidarGridRouteFlowResult routeAllNetsGrid(LidarRuntimeView& db,
                                          LidarDrcManager& drc,
                                          const LidarRouteConfig& config);

void writeAstarInitSummary(const LidarRuntimeView& db,
                           const LidarDrcManager& drc,
                           const LidarRouteConfig& config,
                           std::ostream& os);

void writeAstarRouteSummary(LidarRuntimeView& db,
                            LidarDrcManager& drc,
                            const LidarRouteConfig& config,
                            std::ostream& os,
                            bool updateBitmap = true);

void writeGridRouteFlowSummary(LidarRuntimeView& db,
                               LidarDrcManager& drc,
                               const LidarRouteConfig& config,
                               std::ostream& os);

void writeGridRouteFlowSummary(const LidarGridRouteFlowResult& flow,
                               std::ostream& os);

void writeGridRouteResultYml(LidarRuntimeView& db,
                             LidarDrcManager& drc,
                             const LidarRouteConfig& config,
                             std::ostream& os);

void writeGridRouteResultYml(const LidarRuntimeView& db,
                             const LidarRouteConfig& config,
                             const LidarGridRouteFlowResult& flow,
                             std::ostream& os);

LidarRouteWritebackResult routeAllNetsGridToDesign(
    Design& design,
    LidarRuntimeView& db,
    LidarDrcManager& drc,
    const LidarRouteConfig& config);

LidarRouteWritebackResult writeRoutedGridToDesign(
    Design& design,
    const LidarRuntimeView& db,
    const LidarRouteConfig& config,
    bool routeSuccess);

}  // namespace picpr::lidar
