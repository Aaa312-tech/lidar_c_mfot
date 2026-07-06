#include "lidar_mfot.h"

#include "lidar_astar.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <ostream>
#include <unordered_map>
#include <unordered_set>

namespace picpr::lidar
{

namespace
{

struct MfOtSample
{
  int x = 0;
  int y = 0;
  int key = 0;
  double baseEnergy = 0.0;
  std::vector<int> supportKeys;
};

struct MfOtCandidateNet
{
  std::string netName;
  std::vector<MfOtSample> samples;
  int bboxCells = 0;
  double topologyPressure = 0.0;
};

int cellKey(int x, int y, int height)
{
  return x * height + y;
}

double gridDistance(int x0, int y0, int x1, int y1)
{
  const double dx = static_cast<double>(x0 - x1);
  const double dy = static_cast<double>(y0 - y1);
  return std::hypot(dx, dy);
}

void appendLineSupport(std::vector<int>& keys,
                       int width,
                       int height,
                       int x0,
                       int y0,
                       int x1,
                       int y1,
                       int step)
{
  int x = x0;
  int y = y0;
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  int tick = 0;
  const int sampleStep = std::max(1, step);

  while (true) {
    if (x >= 0 && x < width && y >= 0 && y < height
        && (tick % sampleStep == 0 || (x == x1 && y == y1))) {
      keys.push_back(cellKey(x, y, height));
    }
    if (x == x1 && y == y1) {
      break;
    }
    const int twiceError = 2 * error;
    if (twiceError >= dy) {
      error += dy;
      x += sx;
    }
    if (twiceError <= dx) {
      error += dx;
      y += sy;
    }
    ++tick;
  }
}

std::vector<int> buildPathSupport(int width,
                                  int height,
                                  int sx,
                                  int sy,
                                  int wx,
                                  int wy,
                                  int tx,
                                  int ty,
                                  int step)
{
  std::vector<int> keys;
  const double estimatedLength = gridDistance(sx, sy, wx, wy)
                                 + gridDistance(wx, wy, tx, ty);
  keys.reserve(static_cast<std::size_t>(estimatedLength / std::max(1, step)) + 4);
  appendLineSupport(keys, width, height, sx, sy, wx, wy, step);
  appendLineSupport(keys, width, height, wx, wy, tx, ty, step);
  std::sort(keys.begin(), keys.end());
  keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
  return keys;
}

int clampInt(int value, int lo, int hi)
{
  return std::max(lo, std::min(value, hi));
}

double obstacleEnergy(const LidarDrcManager& drc,
                      const LidarRuntimeView& db,
                      const LidarNet& net,
                      int x,
                      int y,
                      const LidarRouteConfig& config)
{
  const auto check = drc.checkSingleNode(x, y, net.netName);
  if (!check.violated) {
    return 0.0;
  }
  if (check.netName.has_value() && check.netName.value() == net.netName) {
    return 0.0;
  }

  const auto& sourcePort = db.ports[net.sourcePortIndex];
  const auto& targetPort = db.ports[net.targetPortIndex];
  if (check.netName.has_value()
      && ((sourcePort.netName.has_value()
           && check.netName.value() == sourcePort.netName.value())
          || (targetPort.netName.has_value()
              && check.netName.value() == targetPort.netName.value()))) {
    return 0.0;
  }

  if (check.type == "blk") {
    return config.mfotObstaclePenalty;
  }
  if (check.type == "port") {
    return 0.45 * config.mfotObstaclePenalty;
  }
  if (check.type == "compound") {
    return 0.30 * config.mfotObstaclePenalty;
  }
  return 0.20 * config.mfotObstaclePenalty;
}

void addCorridorDisk(LidarMfOtNetPlan& netPlan,
                     int width,
                     int height,
                     int cx,
                     int cy,
                     int radius,
                     double cost)
{
  const int r = std::max(0, radius);
  for (int x = std::max(0, cx - r); x <= std::min(width - 1, cx + r); ++x) {
    for (int y = std::max(0, cy - r); y <= std::min(height - 1, cy + r); ++y) {
      if (gridDistance(x, y, cx, cy) > static_cast<double>(r) + 0.25) {
        continue;
      }
      const int key = cellKey(x, y, height);
      auto it = netPlan.corridorCost.find(key);
      if (it == netPlan.corridorCost.end() || cost < it->second) {
        netPlan.corridorCost[key] = cost;
      }
    }
  }
}

template <typename DensityAt>
double pathDensityEnergy(const MfOtSample& sample,
                         const DensityAt& densityAt,
                         double lambda)
{
  if (sample.supportKeys.empty()) {
    return lambda * densityAt(sample.key);
  }
  double densitySum = 0.0;
  for (const auto key : sample.supportKeys) {
    densitySum += densityAt(key);
  }
  return lambda * densitySum
         / std::max(1.0, static_cast<double>(sample.supportKeys.size()));
}

template <typename AddDensity>
void addPathDensity(const MfOtSample& sample,
                    double probability,
                    const AddDensity& addDensity)
{
  if (sample.supportKeys.empty()) {
    addDensity(sample.key, probability);
    return;
  }
  for (const auto key : sample.supportKeys) {
    addDensity(key, probability);
  }
}

}  // namespace

LidarMfOtPlan buildMfOtPlan(const LidarRuntimeView& db,
                            const LidarDrcManager& drc,
                            const LidarRouteConfig& config)
{
  LidarMfOtPlan plan;
  plan.enabled = config.enableMfOtRouting;
  plan.width = drc.bitmapWidth();
  plan.height = drc.bitmapHeight();
  plan.stride = std::max(1, config.mfotGridStride);
  plan.iterations = std::max(0, config.mfotIterations);
  if (db.nets.size() >= 256) {
    plan.stride = std::max(plan.stride, 16);
  }
  if (!plan.enabled || plan.width <= 0 || plan.height <= 0 || db.nets.empty()) {
    plan.enabled = false;
    return plan;
  }

  std::vector<MfOtCandidateNet> candidates;
  candidates.reserve(db.nets.size());

  for (const auto& net : db.nets) {
    if (net.sourcePortIndex >= db.ports.size()
        || net.targetPortIndex >= db.ports.size()) {
      continue;
    }
    const auto state = buildAstarInitState(db, drc, net, config, 2);
    const int minX = clampInt(state.routingBound[0][0], 0, plan.width - 1);
    const int minY = clampInt(state.routingBound[0][1], 0, plan.height - 1);
    const int maxX = clampInt(state.routingBound[1][0], 0, plan.width - 1);
    const int maxY = clampInt(state.routingBound[1][1], 0, plan.height - 1);
    if (minX >= maxX || minY >= maxY) {
      continue;
    }

    MfOtCandidateNet candidate;
    candidate.netName = net.netName;
    candidate.bboxCells = (maxX - minX + 1) * (maxY - minY + 1);
    candidate.topologyPressure =
        static_cast<double>(std::max(0, net.topologyCrossing))
        + 0.002 * std::max(0.0, net.eulerDistance);

    const int sx = state.startNode[0];
    const int sy = state.startNode[1];
    const int tx = state.endNode[0];
    const int ty = state.endNode[1];
    const double direct = std::max(1.0, gridDistance(sx, sy, tx, ty));
    int localStride = plan.stride;
    if (config.mfotMaxSamplesPerNet > 0) {
      const int spanX = std::max(1, maxX - minX + 1);
      const int spanY = std::max(1, maxY - minY + 1);
      const int strideLimit =
          std::max(localStride, std::max(spanX, std::max(spanY, 1)));
      auto estimateSamples = [&]() -> long long {
        const long long nx =
            std::max<long long>(1, (spanX + localStride - 1) / localStride);
        const long long ny =
            std::max<long long>(1, (spanY + localStride - 1) / localStride);
        return nx * ny;
      };
      while (estimateSamples()
                 > static_cast<long long>(config.mfotMaxSamplesPerNet)
             && localStride < strideLimit) {
        if (localStride > strideLimit / 2) {
          localStride = strideLimit;
          break;
        }
        localStride *= 2;
      }
    }
    const int stride = localStride;
    const int pathSupportStep = std::max(1, stride);
    const int alignedMinX = minX + ((stride - (minX % stride)) % stride);
    const int alignedMinY = minY + ((stride - (minY % stride)) % stride);

    for (int x = alignedMinX; x <= maxX; x += stride) {
      for (int y = alignedMinY; y <= maxY; y += stride) {
        const double viaDistance = gridDistance(sx, sy, x, y)
                                   + gridDistance(x, y, tx, ty);
        const double detour =
            std::max(0.0, viaDistance - direct) * config.gridResolution;
        std::vector<int> supportKeys = buildPathSupport(plan.width,
                                                        plan.height,
                                                        sx,
                                                        sy,
                                                        x,
                                                        y,
                                                        tx,
                                                        ty,
                                                        pathSupportStep);
        double pathObstacleEnergy = 0.0;
        for (const auto key : supportKeys) {
          pathObstacleEnergy += obstacleEnergy(
            drc, db, net, key / plan.height, key % plan.height, config);
        }
        const double meanPathObstacle =
          supportKeys.empty()
            ? 0.0
            : pathObstacleEnergy
                / std::max(1.0, static_cast<double>(supportKeys.size()));
        const double baseEnergy =
            config.mfotDetourScale * detour
          + std::max(obstacleEnergy(drc, db, net, x, y, config),
                 0.35 * meanPathObstacle);
        candidate.samples.push_back({x,
                       y,
                       cellKey(x, y, plan.height),
                       baseEnergy,
                       std::move(supportKeys)});
      }
    }

    if (!candidate.samples.empty()) {
      candidates.push_back(std::move(candidate));
    }
  }

  if (candidates.empty()) {
    plan.enabled = false;
    return plan;
  }
  const bool buildCorridor =
      config.mfotHardCorridor || config.mfotOutsidePenalty > 0.0
      || config.mfotPotentialScale > 0.0 || config.mfotHistoryScale > 0.0;

  const std::size_t gridSize =
      static_cast<std::size_t>(plan.width) * static_cast<std::size_t>(plan.height);
  constexpr std::size_t maxDenseDensityCells = 20000000;
  const bool useDenseDensity = gridSize <= maxDenseDensityCells;
  if (useDenseDensity) {
    plan.density.assign(gridSize, 0.0);
  }
  std::vector<double> nextDensity(useDenseDensity ? gridSize : 0, 0.0);
  std::vector<double> pointDensity(useDenseDensity ? gridSize : 0, 0.0);
  std::vector<double> nextPointDensity(useDenseDensity ? gridSize : 0, 0.0);
  std::unordered_map<int, double> sparseDensity;
  std::unordered_map<int, double> nextSparseDensity;
  std::unordered_map<int, double> sparsePointDensity;
  std::unordered_map<int, double> nextSparsePointDensity;
  auto densityAt = [&](int key) {
    if (useDenseDensity) {
      return plan.density[static_cast<std::size_t>(key)];
    }
    auto it = sparseDensity.find(key);
    return it == sparseDensity.end() ? 0.0 : it->second;
  };
  auto pointDensityAt = [&](int key) {
    if (useDenseDensity) {
      return pointDensity[static_cast<std::size_t>(key)];
    }
    auto it = sparsePointDensity.find(key);
    return it == sparsePointDensity.end() ? 0.0 : it->second;
  };
  auto addNextDensity = [&](int key, double value) {
    if (useDenseDensity) {
      nextDensity[static_cast<std::size_t>(key)] += value;
    } else {
      nextSparseDensity[key] += value;
    }
  };
  auto addNextPointDensity = [&](int key, double value) {
    if (useDenseDensity) {
      nextPointDensity[static_cast<std::size_t>(key)] += value;
    } else {
      nextSparsePointDensity[key] += value;
    }
  };
  const double epsilon = std::max(1e-3, config.mfotEpsilon);

  for (int iter = 0; iter < plan.iterations; ++iter) {
    if (useDenseDensity) {
      std::fill(nextDensity.begin(), nextDensity.end(), 0.0);
      std::fill(nextPointDensity.begin(), nextPointDensity.end(), 0.0);
    } else {
      nextSparseDensity.clear();
      nextSparsePointDensity.clear();
    }
    double iterFreeEnergy = 0.0;
    for (const auto& candidate : candidates) {
      double minEnergy = std::numeric_limits<double>::infinity();
      for (const auto& sample : candidate.samples) {
        const double energy =
            sample.baseEnergy
            + pathDensityEnergy(sample, densityAt, config.mfotLambdaCongestion);
        minEnergy = std::min(minEnergy, energy);
      }
      if (!std::isfinite(minEnergy)) {
        continue;
      }

      double weightSum = 0.0;
      for (const auto& sample : candidate.samples) {
        const double energy =
            sample.baseEnergy
            + pathDensityEnergy(sample, densityAt, config.mfotLambdaCongestion);
        const double exponent = -(energy - minEnergy) / epsilon;
        if (exponent < -36.0) {
          continue;
        }
        weightSum += std::exp(exponent);
      }
      if (weightSum <= 0.0) {
        continue;
      }

      const double invWeightSum = 1.0 / weightSum;
      for (const auto& sample : candidate.samples) {
        const double energy =
            sample.baseEnergy
            + pathDensityEnergy(sample, densityAt, config.mfotLambdaCongestion);
        const double exponent = -(energy - minEnergy) / epsilon;
        if (exponent < -36.0) {
          continue;
        }
        const double probability = std::exp(exponent) * invWeightSum;
        addPathDensity(sample, probability, addNextDensity);
      }

      double pointMinEnergy = std::numeric_limits<double>::infinity();
      for (const auto& sample : candidate.samples) {
        const double energy =
            sample.baseEnergy
            + config.mfotLambdaCongestion * pointDensityAt(sample.key);
        pointMinEnergy = std::min(pointMinEnergy, energy);
      }
      double pointWeightSum = 0.0;
      for (const auto& sample : candidate.samples) {
        const double energy =
            sample.baseEnergy
            + config.mfotLambdaCongestion * pointDensityAt(sample.key);
        const double exponent = -(energy - pointMinEnergy) / epsilon;
        if (exponent >= -36.0) {
          pointWeightSum += std::exp(exponent);
        }
      }
      if (pointWeightSum > 0.0) {
        const double invPointWeightSum = 1.0 / pointWeightSum;
        for (const auto& sample : candidate.samples) {
          const double energy =
              sample.baseEnergy
              + config.mfotLambdaCongestion * pointDensityAt(sample.key);
          const double exponent = -(energy - pointMinEnergy) / epsilon;
          if (exponent >= -36.0) {
            addNextPointDensity(sample.key, std::exp(exponent) * invPointWeightSum);
          }
        }
      }
      iterFreeEnergy += minEnergy - epsilon * std::log(weightSum);
    }

    constexpr double damping = 0.55;
    if (useDenseDensity) {
      for (std::size_t i = 0; i < plan.density.size(); ++i) {
        plan.density[i] =
            damping * nextDensity[i] + (1.0 - damping) * plan.density[i];
        pointDensity[i] =
            damping * nextPointDensity[i] + (1.0 - damping) * pointDensity[i];
      }
    } else {
      std::unordered_map<int, double> blendedDensity;
      blendedDensity.reserve(sparseDensity.size() + nextSparseDensity.size());
      for (const auto& [key, value] : sparseDensity) {
        blendedDensity[key] += (1.0 - damping) * value;
      }
      for (const auto& [key, value] : nextSparseDensity) {
        blendedDensity[key] += damping * value;
      }
      sparseDensity = std::move(blendedDensity);

      std::unordered_map<int, double> blendedPointDensity;
      blendedPointDensity.reserve(sparsePointDensity.size()
                                  + nextSparsePointDensity.size());
      for (const auto& [key, value] : sparsePointDensity) {
        blendedPointDensity[key] += (1.0 - damping) * value;
      }
      for (const auto& [key, value] : nextSparsePointDensity) {
        blendedPointDensity[key] += damping * value;
      }
      sparsePointDensity = std::move(blendedPointDensity);
    }
    plan.totalFreeEnergy = iterFreeEnergy;
  }

  for (const auto& candidate : candidates) {
    LidarMfOtNetPlan netPlan;
    netPlan.netName = candidate.netName;
    netPlan.bboxCells = candidate.bboxCells;
    netPlan.sampledCells = static_cast<int>(candidate.samples.size());

    double minEnergy = std::numeric_limits<double>::infinity();
    double waypointOverlapEnergy = 0.0;
    double pathOverlapEnergy = 0.0;
    for (const auto& sample : candidate.samples) {
      const double energy =
          sample.baseEnergy
          + pathDensityEnergy(sample, densityAt, config.mfotLambdaCongestion);
      minEnergy = std::min(minEnergy, energy);
        waypointOverlapEnergy += pointDensityAt(sample.key);
      if (sample.supportKeys.empty()) {
        pathOverlapEnergy += densityAt(sample.key);
      } else {
        for (const auto key : sample.supportKeys) {
          pathOverlapEnergy += densityAt(key)
                               / std::max(1.0,
                                          static_cast<double>(sample.supportKeys.size()));
        }
      }
    }
    if (!std::isfinite(minEnergy)) {
      continue;
    }

    double weightSum = 0.0;
    for (const auto& sample : candidate.samples) {
      const double energy =
          sample.baseEnergy
          + pathDensityEnergy(sample, densityAt, config.mfotLambdaCongestion);
      const double relative = energy - minEnergy;
      if (buildCorridor && relative <= config.mfotCorridorEnergySlack) {
        const double cost = std::max(0.0, relative / epsilon);
        if (sample.supportKeys.empty()) {
          addCorridorDisk(netPlan,
                          plan.width,
                          plan.height,
                          sample.x,
                          sample.y,
                          config.mfotCorridorRadius,
                          cost);
        } else {
          for (const auto key : sample.supportKeys) {
            addCorridorDisk(netPlan,
                            plan.width,
                            plan.height,
                            key / plan.height,
                            key % plan.height,
                            config.mfotCorridorRadius,
                            cost);
          }
        }
      }
      const double exponent = -relative / epsilon;
      if (exponent >= -36.0) {
        weightSum += std::exp(exponent);
      }
    }

    auto netIt = db.netIndex.find(candidate.netName);
    if (buildCorridor && netIt != db.netIndex.end()) {
      const auto& net = db.nets[netIt->second];
      const auto state = buildAstarInitState(db, drc, net, config, 2);
      addCorridorDisk(netPlan,
                      plan.width,
                      plan.height,
                      state.startNode[0],
                      state.startNode[1],
                      std::max(config.mfotCorridorRadius, state.gridRadius + 2),
                      0.0);
      addCorridorDisk(netPlan,
                      plan.width,
                      plan.height,
                      state.endNode[0],
                      state.endNode[1],
                      std::max(config.mfotCorridorRadius, state.gridRadius + 2),
                      0.0);
    }

    netPlan.freeEnergy =
        minEnergy - epsilon * std::log(std::max(1e-12, weightSum));
    const double meanOverlap =
        (waypointOverlapEnergy + 0.05 * pathOverlapEnergy)
        / std::max(1.0, static_cast<double>(candidate.samples.size()));
    netPlan.priority =
        candidate.topologyPressure
        + meanOverlap * 100.0
        + static_cast<double>(candidate.bboxCells)
              / std::max(1.0, static_cast<double>(candidate.samples.size()))
              * 0.05;
    netPlan.corridorShrinkRatio =
        candidate.bboxCells <= 0
            ? 1.0
            : static_cast<double>(netPlan.corridorCost.size())
                  / static_cast<double>(candidate.bboxCells);

    plan.totalCorridorCells += netPlan.corridorCost.size();
    plan.maxPriority = std::max(plan.maxPriority, netPlan.priority);
    plan.nets.emplace(candidate.netName, std::move(netPlan));
  }

  plan.enabled = !plan.nets.empty();
  return plan;
}

double mfotCellPenalty(const LidarMfOtPlan& plan,
                       const LidarRouteConfig& config,
                       const std::string& netName,
                       int x,
                       int y,
                       bool* inCorridor)
{
  if (inCorridor != nullptr) {
    *inCorridor = true;
  }
  if (!config.mfotHardCorridor && config.mfotOutsidePenalty <= 0.0
      && config.mfotPotentialScale <= 0.0) {
    return 0.0;
  }
  if (!plan.enabled || x < 0 || y < 0 || x >= plan.width || y >= plan.height) {
    return 0.0;
  }
  auto netIt = plan.nets.find(netName);
  if (netIt == plan.nets.end()) {
    return 0.0;
  }
  const int key = cellKey(x, y, plan.height);
  auto costIt = netIt->second.corridorCost.find(key);
  if (costIt == netIt->second.corridorCost.end()) {
    if (inCorridor != nullptr) {
      *inCorridor = false;
    }
    return config.mfotOutsidePenalty;
  }
  return config.mfotPotentialScale * costIt->second;
}

double mfotNetPriority(const LidarMfOtPlan& plan, const std::string& netName)
{
  auto it = plan.nets.find(netName);
  return it == plan.nets.end() ? 0.0 : it->second.priority;
}

double mfotSearchWeightForNet(const LidarMfOtPlan& plan,
                              const LidarRouteConfig& config,
                              const std::string& netName)
{
  if (!plan.enabled) {
    return 1.0;
  }
  const double targetWeight = std::max(1.0, config.mfotSearchWeight);
  double boundedTargetWeight = targetWeight;
  if (plan.nets.size() < 384) {
    boundedTargetWeight = std::min(boundedTargetWeight, 1.06);
  }
  if (boundedTargetWeight <= 1.0) {
    return 1.0;
  }
  const double normalized =
      plan.maxPriority <= 0.0
          ? 0.0
          : std::clamp(mfotNetPriority(plan, netName) / plan.maxPriority,
                       0.0,
                       1.0);
  return 1.0 + (boundedTargetWeight - 1.0) * (0.45 + 0.55 * normalized);
}

void applyMfOtGlobalPriorities(LidarRuntimeView& db,
                               const LidarMfOtPlan& plan,
                               const LidarRouteConfig& config)
{
  if (!plan.enabled || config.mfotPriorityScale <= 0.0) {
    return;
  }
  double maxPriority = 0.0;
  for (const auto& [_, netPlan] : plan.nets) {
    maxPriority = std::max(maxPriority, netPlan.priority);
  }
  if (maxPriority <= 0.0) {
    return;
  }
  for (auto& net : db.nets) {
    const double normalized =
        mfotNetPriority(plan, net.netName) / std::max(1e-9, maxPriority);
    net.compDist -= config.mfotPriorityScale * normalized;
  }
}

void seedMfOtHistoryMap(const LidarMfOtPlan& plan,
                        const LidarRouteConfig& config,
                        std::vector<int>& historyMap,
                        int historyWidth,
                        int historyHeight)
{
  if (!plan.enabled || config.mfotHistoryScale <= 0.0
      || historyWidth != plan.width || historyHeight != plan.height) {
    return;
  }
  const auto maxIt = std::max_element(plan.density.begin(), plan.density.end());
  if (maxIt == plan.density.end() || *maxIt <= 0.0) {
    return;
  }
  const double invMax = 1.0 / *maxIt;
  for (int x = 0; x < historyWidth; ++x) {
    for (int y = 0; y < historyHeight; ++y) {
      const int key = cellKey(x, y, historyHeight);
      const int seed = static_cast<int>(
          std::round(config.historyCost * config.mfotHistoryScale
                     * plan.density[static_cast<std::size_t>(key)] * invMax));
      if (seed <= 0) {
        continue;
      }
      for (int ori = 0; ori < 8; ++ori) {
        historyMap[(static_cast<std::size_t>(x)
                    * static_cast<std::size_t>(historyHeight)
                    + static_cast<std::size_t>(y))
                       * 8
                   + static_cast<std::size_t>(ori)] += seed;
      }
    }
  }
}

void writeMfOtSummary(const LidarMfOtPlan& plan, std::ostream& os)
{
  os << std::fixed << std::setprecision(6);
  os << "MFOT_SUMMARY"
     << "\tenabled=" << (plan.enabled ? 1 : 0)
     << "\twidth=" << plan.width
     << "\theight=" << plan.height
     << "\tstride=" << plan.stride
     << "\titerations=" << plan.iterations
     << "\tnets=" << plan.nets.size()
     << "\ttotal_corridor_cells=" << plan.totalCorridorCells
     << "\ttotal_free_energy=" << plan.totalFreeEnergy
     << "\tmax_priority=" << plan.maxPriority
     << "\n";
  for (const auto& [_, netPlan] : plan.nets) {
    os << "MFOT_NET"
       << "\tnet=" << netPlan.netName
       << "\tpriority=" << netPlan.priority
       << "\tfree_energy=" << netPlan.freeEnergy
       << "\tbbox_cells=" << netPlan.bboxCells
       << "\tsampled_cells=" << netPlan.sampledCells
       << "\tcorridor_cells=" << netPlan.corridorCost.size()
       << "\tcorridor_shrink=" << netPlan.corridorShrinkRatio
       << "\n";
  }
}

}  // namespace picpr::lidar
