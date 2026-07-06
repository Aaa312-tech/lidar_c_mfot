#include "lidar_astar.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <sstream>
#include <unordered_map>
#include <utility>

#include "cell.h"
#include "net.h"
#include "pin.h"
#include "segment.h"
#include "shape.h"
#include "xsection.h"

namespace picpr::lidar
{

namespace
{

int pythonRoundPositive(double value)
{
  const double floorValue = std::floor(value);
  const double fraction = value - floorValue;
  constexpr double eps = 1e-12;
  if (fraction < 0.5 - eps) {
    return static_cast<int>(floorValue);
  }
  if (fraction > 0.5 + eps) {
    return static_cast<int>(floorValue + 1.0);
  }
  const auto floorInt = static_cast<int>(floorValue);
  return (floorInt % 2 == 0) ? floorInt : floorInt + 1;
}

double customH(const std::array<int, 2>& pos1,
               const std::array<int, 2>& pos2,
               double bendingLoss,
               double resolution)
{
  const double dx = std::abs(pos1[0] - pos2[0]) * resolution;
  const double dy = std::abs(pos1[1] - pos2[1]) * resolution;
  if (dx != 0.0 && dy != 0.0) {
    if (dx > dy) {
      return dx - dy + dy * std::sqrt(2.0) + bendingLoss / 2.0;
    }
    return dy - dx + dx * std::sqrt(2.0) + bendingLoss / 2.0;
  }
  return std::max(dx, dy);
}

double arcPolylineLength(double radius,
                         double angleDegrees,
                         double bendPointsDistance)
{
  const double absAngle = std::abs(angleDegrees);
  int npoints = std::abs(static_cast<int>(angleDegrees / 360.0 * radius / bendPointsDistance / 2.0));
  npoints = std::max(npoints, static_cast<int>(360.0 / absAngle) + 1);
  if (npoints <= 1) {
    return 0.0;
  }
  const double angleRadians = absAngle * std::acos(-1.0) / 180.0;
  const double segmentAngle = angleRadians / static_cast<double>(npoints - 1);
  return static_cast<double>(npoints - 1) * 2.0 * radius * std::sin(segmentAngle / 2.0);
}

double roundedCornerLength(double x1,
                           double y1,
                           double x2,
                           double y2,
                           double x3,
                           double y3,
                           double radius,
                           double bendPointsDistance)
{
  const double v1x = x2 - x1;
  const double v1y = y2 - y1;
  const double v2x = x3 - x2;
  const double v2y = y3 - y2;
  const double ds1 = std::hypot(v1x, v1y);
  const double ds2 = std::hypot(v2x, v2y);
  const double dot = (v1x * v2x + v1y * v2y) / (ds1 * ds2);
  const double clampedDot = std::max(-1.0, std::min(1.0, dot));
  const double angleRadians = std::acos(clampedDot);
  const double angleDegrees = angleRadians * 180.0 / std::acos(-1.0);
  const double trim = radius / std::tan((std::acos(-1.0) - angleRadians) / 2.0);
  const double length = ds1 + ds2 - 2.0 * trim
                        + arcPolylineLength(radius, angleDegrees, bendPointsDistance);
  return std::round(length * 1000.0) / 1000.0;
}

std::array<int, 3> portGridNode(const LidarPort& port,
                                const std::array<int, 2>& grid)
{
  return {grid[0], grid[1], static_cast<int>(std::round(port.orientation))};
}

int manhattanDH(const std::array<int, 3>& lhs, const std::array<int, 3>& rhs)
{
  return std::abs(lhs[0] - rhs[0]) + std::abs(lhs[1] - rhs[1]);
}

double eulerDist(const std::array<int, 3>& lhs, const std::array<int, 3>& rhs)
{
  const double dx = static_cast<double>(lhs[0] - rhs[0]);
  const double dy = static_cast<double>(lhs[1] - rhs[1]);
  return std::hypot(dx, dy);
}

double normalizeDegrees(double value)
{
  double normalized = std::fmod(value, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  if (std::abs(normalized - 360.0) < 1e-9) {
    return 0.0;
  }
  return normalized;
}

bool oppositeOrientations(double lhs, double rhs)
{
  double diff = std::abs(normalizeDegrees(lhs) - normalizeDegrees(rhs));
  if (diff > 180.0) {
    diff = 360.0 - diff;
  }
  return std::abs(diff - 180.0) < 1e-9;
}

std::array<double, 2> bezierPoint(const std::array<double, 2>& p0,
                                  const std::array<double, 2>& p1,
                                  const std::array<double, 2>& p2,
                                  const std::array<double, 2>& p3,
                                  double t)
{
  const double u = 1.0 - t;
  const double b0 = u * u * u;
  const double b1 = 3.0 * u * u * t;
  const double b2 = 3.0 * u * t * t;
  const double b3 = t * t * t;
  return {b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
          b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1]};
}

std::vector<std::array<double, 2>> shortSbendBezierPoints(
    const LidarPort& startPort,
    const LidarPort& endPort,
    int npoints = 99)
{
  const std::array<double, 2> p0 = {startPort.center.x,
                                    startPort.center.y};
  const std::array<double, 2> p3 = {endPort.center.x,
                                    endPort.center.y};
  const double radians = normalizeDegrees(startPort.orientation)
                         * std::acos(-1.0) / 180.0;
  const std::array<double, 2> direction = {std::cos(radians),
                                           std::sin(radians)};
  const std::array<double, 2> delta = {p3[0] - p0[0], p3[1] - p0[1]};
  const double projected =
      std::abs(delta[0] * direction[0] + delta[1] * direction[1]);
  const double handle = projected / 2.0;
  const std::array<double, 2> p1 = {p0[0] + direction[0] * handle,
                                    p0[1] + direction[1] * handle};
  const std::array<double, 2> p2 = {p3[0] - direction[0] * handle,
                                    p3[1] - direction[1] * handle};

  std::vector<std::array<double, 2>> points;
  points.reserve(static_cast<std::size_t>(std::max(2, npoints)));
  const int samples = std::max(2, npoints);
  for (int i = 0; i < samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples - 1);
    points.push_back(bezierPoint(p0, p1, p2, p3, t));
  }
  if (points.size() >= 3) {
    const double startStep = std::hypot(points[1][0] - p0[0],
                                        points[1][1] - p0[1]);
    const auto& beforeEnd = points[points.size() - 2];
    const double endStep = std::hypot(p3[0] - beforeEnd[0],
                                      p3[1] - beforeEnd[1]);
    points[1] = {p0[0] + direction[0] * startStep,
                 p0[1] + direction[1] * startStep};
    points[points.size() - 2] = {p3[0] - direction[0] * endStep,
                                 p3[1] - direction[1] * endStep};
  }
  return points;
}

double shortSbendLength(const LidarPort& startPort, const LidarPort& endPort)
{
  const auto points = shortSbendBezierPoints(startPort, endPort);
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    length += std::hypot(points[i][0] - points[i - 1][0],
                         points[i][1] - points[i - 1][1]);
  }
  return std::round(length * 1000.0) / 1000.0;
}

std::string posKey(const std::array<int, 3>& pos)
{
  return std::to_string(pos[0]) + "," + std::to_string(pos[1]) + ","
         + std::to_string(pos[2]);
}

std::uint64_t rotl64(std::uint64_t value, int shift)
{
  return (value << shift) | (value >> (64 - shift));
}

void sipRound13(std::uint64_t& v0,
                std::uint64_t& v1,
                std::uint64_t& v2,
                std::uint64_t& v3)
{
  v0 += v1;
  v1 = rotl64(v1, 13);
  v1 ^= v0;
  v0 = rotl64(v0, 32);
  v2 += v3;
  v3 = rotl64(v3, 16);
  v3 ^= v2;
  v0 += v3;
  v3 = rotl64(v3, 21);
  v3 ^= v0;
  v2 += v1;
  v1 = rotl64(v1, 17);
  v1 ^= v2;
  v2 = rotl64(v2, 32);
}

std::uint64_t readLittleEndian64(const char* data)
{
  std::uint64_t value = 0;
  for (int i = 0; i < 8; ++i) {
    value |= static_cast<std::uint64_t>(
                 static_cast<unsigned char>(data[i]))
             << (8 * i);
  }
  return value;
}

std::uint64_t pythonHashSeed0Bits(const std::string& value)
{
  if (value.empty()) {
    return 0;
  }
  std::uint64_t v0 = 0x736f6d6570736575ULL;
  std::uint64_t v1 = 0x646f72616e646f6dULL;
  std::uint64_t v2 = 0x6c7967656e657261ULL;
  std::uint64_t v3 = 0x7465646279746573ULL;

  const char* data = value.data();
  std::size_t offset = 0;
  while (offset + 8 <= value.size()) {
    const std::uint64_t mi = readLittleEndian64(data + offset);
    v3 ^= mi;
    sipRound13(v0, v1, v2, v3);
    v0 ^= mi;
    offset += 8;
  }

  std::uint64_t tail = static_cast<std::uint64_t>(value.size()) << 56;
  for (std::size_t i = 0; offset + i < value.size(); ++i) {
    tail |= static_cast<std::uint64_t>(
                static_cast<unsigned char>(data[offset + i]))
            << (8 * i);
  }
  v3 ^= tail;
  sipRound13(v0, v1, v2, v3);
  v0 ^= tail;
  v2 ^= 0xff;
  for (int i = 0; i < 3; ++i) {
    sipRound13(v0, v1, v2, v3);
  }
  std::uint64_t hash = v0 ^ v1 ^ v2 ^ v3;
  if (hash == std::numeric_limits<std::uint64_t>::max()) {
    hash = static_cast<std::uint64_t>(-2LL);
  }
  return hash;
}

std::size_t pythonSetSlotForHash(std::uint64_t hash,
                                 std::vector<std::optional<std::string>>& table)
{
  constexpr int perturbShift = 5;
  const std::size_t mask = table.size() - 1;
  std::size_t index = static_cast<std::size_t>(hash) & mask;
  std::uint64_t perturb = hash;
  while (table[index].has_value()) {
    perturb >>= perturbShift;
    index = (index * 5 + 1 + static_cast<std::size_t>(perturb)) & mask;
  }
  return index;
}

void pythonSetInsert(std::vector<std::optional<std::string>>& table,
                     const std::string& value)
{
  const auto slot = pythonSetSlotForHash(pythonHashSeed0Bits(value), table);
  table[slot] = value;
}

void pythonSetResize(std::vector<std::optional<std::string>>& table,
                     std::size_t used)
{
  std::size_t newSize = 8;
  const std::size_t minUsed = used > 50000 ? used * 2 : used * 4;
  while (newSize <= minUsed) {
    newSize <<= 1;
  }
  std::vector<std::string> oldValues;
  oldValues.reserve(used);
  for (const auto& item : table) {
    if (item.has_value()) {
      oldValues.push_back(*item);
    }
  }
  table.assign(newSize, std::nullopt);
  for (const auto& item : oldValues) {
    pythonSetInsert(table, item);
  }
}

std::vector<std::string> pythonSetIterationOrder(
    const std::set<std::string>& values)
{
  std::vector<std::optional<std::string>> table(8);
  std::size_t used = 0;
  std::size_t fill = 0;
  for (const auto& value : values) {
    pythonSetInsert(table, value);
    ++used;
    ++fill;
    const std::size_t mask = table.size() - 1;
    if (fill * 5 >= mask * 3) {
      pythonSetResize(table, used);
      fill = used;
    }
  }

  std::vector<std::string> ordered;
  ordered.reserve(values.size());
  for (const auto& item : table) {
    if (item.has_value()) {
      ordered.push_back(*item);
    }
  }
  return ordered;
}

std::set<std::array<int, 3>> parseTracePopPositions(const char* value)
{
  std::set<std::array<int, 3>> positions;
  if (value == nullptr) {
    return positions;
  }
  std::stringstream ss(value);
  std::string token;
  while (std::getline(ss, token, ';')) {
    if (token.empty()) {
      continue;
    }
    std::stringstream ts(token);
    char comma1 = 0;
    char comma2 = 0;
    std::array<int, 3> pos = {0, 0, 0};
    if (ts >> pos[0] >> comma1 >> pos[1] >> comma2 >> pos[2]
        && comma1 == ',' && comma2 == ',') {
      positions.insert(pos);
    }
  }
  return positions;
}

std::string pathToString(const std::vector<std::array<int, 3>>& path)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (i != 0) {
      oss << "|";
    }
    oss << path[i][0] << "," << path[i][1] << "," << path[i][2];
  }
  return oss.str();
}

int orientationHistoryIndex(int orientation)
{
  switch ((orientation % 360 + 360) % 360) {
    case 0:
      return 0;
    case 45:
      return 1;
    case 90:
      return 2;
    case 135:
      return 3;
    case 180:
      return 4;
    case 225:
      return 5;
    case 270:
      return 6;
    case 315:
      return 7;
    default:
      return 0;
  }
}

struct GridSearchNode
{
  std::array<int, 3>    pos = {0, 0, 0};
  double                costG = std::numeric_limits<double>::infinity();
  double                costF = std::numeric_limits<double>::infinity();
  bool                  visited = false;
  int                   parent = -1;
  int                   crossingBudget = 0;
  std::string           crossingNet;
  std::set<std::string> violatedNets;
  bool                  violated = false;
  int                   straightCount = 0;
  bool                  neighborsComputed = false;
  std::vector<std::pair<std::size_t, std::string>> neighbors;
};

struct PathPortDump
{
  std::string name;
  std::string instanceName;
  std::string pinName;
  std::array<double, 2> center = {0.0, 0.0};
  double orientation = 0.0;
  double width = 0.5;
};

struct PostRoutePath
{
  std::vector<std::array<double, 2>> points;
  PathPortDump startPort;
  PathPortDump endPort;
};

struct PostProcessAccessViolation
{
  std::string netName;
  std::size_t pathIndex = 0;
  std::string endpoint;
  std::string objectName;
  std::array<double, 2> center = {0.0, 0.0};
  double tangentOrientation = 0.0;
  double portOrientation = 0.0;
  double delta = 0.0;
};

struct SplitLineResult
{
  std::array<std::vector<std::array<double, 2>>, 2> parts;
  std::array<std::string, 2> ports;
  double orientation = 0.0;
};

struct SplitPathResult
{
  std::size_t path1Index = 0;
  std::size_t path2Index = 0;
  SplitLineResult split1;
  SplitLineResult split2;
  std::array<double, 2> crossingPoint = {0.0, 0.0};
  double crossingOrientation = 0.0;
};

class HeapDict
{
 public:
  struct Entry
  {
    std::size_t index = 0;
    double priority = 0.0;
    std::size_t pos = 0;
  };

  explicit HeapDict(std::vector<GridSearchNode>& nodes) : _nodes(nodes) {}

  bool empty() const { return _heap.empty(); }

  const std::vector<Entry*>& rawHeap() const { return _heap; }

  bool contains(std::size_t index) const
  {
    return _entries.find(index) != _entries.end();
  }

  void set(std::size_t index)
  {
    if (_entries.find(index) != _entries.end()) {
      erase(index);
    }
    auto [it, inserted] =
        _entries.emplace(index, Entry{index, _nodes[index].costF, _heap.size()});
    (void) inserted;
    _heap.push_back(&it->second);
    decreaseKey(_heap.size() - 1);
  }

  std::size_t popitem()
  {
    Entry* wrapper = _heap.front();
    const std::size_t returnIndex = wrapper->index;
    if (_heap.size() == 1) {
      _heap.pop_back();
      _entries.erase(returnIndex);
      return returnIndex;
    }
    _heap.front() = _heap.back();
    _heap.pop_back();
    _heap.front()->pos = 0;
    minHeapify(0);
    _entries.erase(returnIndex);
    return returnIndex;
  }

 private:
  bool less(std::size_t lhs, std::size_t rhs) const
  {
    return _heap[lhs]->priority < _heap[rhs]->priority;
  }

  void swapItems(std::size_t i, std::size_t j)
  {
    std::swap(_heap[i], _heap[j]);
    _heap[i]->pos = i;
    _heap[j]->pos = j;
  }

  void decreaseKey(std::size_t i)
  {
    while (i != 0) {
      const std::size_t parent = (i - 1) >> 1;
      if (less(parent, i)) {
        break;
      }
      swapItems(i, parent);
      i = parent;
    }
  }

  void minHeapify(std::size_t i)
  {
    while (true) {
      const std::size_t left = (i << 1) + 1;
      std::size_t low = (left < _heap.size() && less(left, i)) ? left : i;
      const std::size_t right = (i + 1) << 1;
      if (right < _heap.size() && less(right, low)) {
        low = right;
      }
      if (low == i) {
        break;
      }
      swapItems(i, low);
      i = low;
    }
  }

  void erase(std::size_t index)
  {
    auto it = _entries.find(index);
    if (it == _entries.end()) {
      return;
    }
    Entry* wrapper = &it->second;
    while (wrapper->pos != 0) {
      const std::size_t parent = (wrapper->pos - 1) >> 1;
      swapItems(wrapper->pos, parent);
    }
    popitem();
  }

  std::vector<GridSearchNode>& _nodes;
  std::vector<Entry*> _heap;
  std::map<std::size_t, Entry> _entries;
};

int parseEnvInt(const char* value, int fallback)
{
  if (value == nullptr) {
    return fallback;
  }
  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || parsed < 0) {
    return fallback;
  }
  return static_cast<int>(parsed);
}

void writeHeapSnapshot(std::ostream& os,
                       const char* label,
                       const HeapDict& heap,
                       const std::vector<GridSearchNode>& nodes,
                       std::size_t popIndex,
                       int limit)
{
  const auto oldFlags = os.flags();
  const auto oldPrecision = os.precision();
  os << std::defaultfloat << std::setprecision(17);
  const auto& rawHeap = heap.rawHeap();
  const int effectiveLimit = limit <= 0
                                 ? static_cast<int>(rawHeap.size())
                                 : std::min<int>(limit, static_cast<int>(rawHeap.size()));
  os << "TRACE_HEAP"
     << "\tlabel=" << label
     << "\tbefore_pop=" << popIndex
     << "\tsize=" << rawHeap.size()
     << "\tlimit=" << effectiveLimit
     << "\n";
  for (int i = 0; i < effectiveLimit; ++i) {
    const auto* entry = rawHeap[static_cast<std::size_t>(i)];
    const auto nodeIndex = entry->index;
    const auto& node = nodes[nodeIndex];
    os << "TRACE_HEAP_ITEM"
       << "\tlabel=" << label
       << "\tbefore_pop=" << popIndex
       << "\theap_index=" << i
       << "\tpos=" << node.pos[0] << "," << node.pos[1] << "," << node.pos[2]
       << "\tcost_g=" << node.costG
       << "\tcost_f=" << entry->priority;
    if (entry->priority != node.costF) {
      os << "\tnode_cost_f=" << node.costF;
    }
    if (node.parent >= 0) {
      const auto& parent = nodes[static_cast<std::size_t>(node.parent)];
      os << "\tparent=" << parent.pos[0] << "," << parent.pos[1] << ","
         << parent.pos[2];
    }
    os << "\n";
  }
  os.flags(oldFlags);
  os.precision(oldPrecision);
}

void writeTracePop(std::ostream& os,
                   const std::string& netName,
                   std::size_t popIndex,
                   const GridSearchNode& node,
                   const std::vector<GridSearchNode>& nodes)
{
  const auto oldFlags = os.flags();
  const auto oldPrecision = os.precision();
  os << std::defaultfloat << std::setprecision(17)
     << "TRACE_POP"
     << "\tnet=" << netName
     << "\tpop=" << popIndex
     << "\tpos=" << node.pos[0] << "," << node.pos[1] << "," << node.pos[2]
     << "\tcost_g=" << node.costG
     << "\tcost_f=" << node.costF;
  if (node.parent >= 0) {
    const auto& parent = nodes[static_cast<std::size_t>(node.parent)];
    os << "\tparent=" << parent.pos[0] << "," << parent.pos[1] << ","
       << parent.pos[2]
       << "\tparent_cost_g=" << parent.costG
       << "\tparent_cost_f=" << parent.costF;
  }
  os << "\n";
  os.flags(oldFlags);
  os.precision(oldPrecision);
}

}  // namespace

LidarAstarInitState buildAstarInitState(const LidarRuntimeView& db,
                                        const LidarDrcManager& drc,
                                        const LidarNet& net,
                                        const LidarRouteConfig& config,
                                        int drUnrouted)
{
  LidarAstarInitState state;
  state.netName = net.netName;
  state.connectThreshold = static_cast<int>(std::ceil(config.bendRadius * 2.0 / config.gridResolution));
  state.repel = config.gridResolution < 20.0;
  state.unrouted = std::max(2, std::min(10, drUnrouted));
  state.checkRegion = state.unrouted * config.gridResolution;
  state.crossingBudget = net.crossingBudget;
  state.radius = config.bendRadius;
  state.gridRadius = static_cast<int>(std::ceil(config.bendRadius / config.gridResolution));
  state.bend45Part1 = static_cast<int>(std::ceil(config.bendRadius * 0.415 / config.gridResolution));
  state.bend45Part2 = static_cast<int>(std::ceil(config.bendRadius * 0.3 / config.gridResolution));
  state.predictLength = pythonRoundPositive(net.halfSize / config.gridResolution);

  const auto& port1 = db.ports[net.sourcePortIndex];
  const auto& port2 = db.ports[net.targetPortIndex];
  const double scaleFactor = config.netBoundScaleFactor + (net.failedCount / 2) * 0.5;
  const double xMean = (port1.center.x + port2.center.x) / 2.0;
  const double yMean = (port1.center.y + port2.center.y) / 2.0;
  const double boundX = std::max(std::abs(port1.center.x - port2.center.x) * scaleFactor,
                                 config.gridResolution * config.netDefaultBound)
                        / 2.0;
  const double boundY = std::max(std::abs(port1.center.y - port2.center.y) * scaleFactor,
                                 config.gridResolution * config.netDefaultBound)
                        / 2.0;
  state.routingBound = {{{std::max(1, static_cast<int>(std::ceil((xMean - boundX) / config.gridResolution))),
                          std::max(1, static_cast<int>(std::ceil((yMean - boundY) / config.gridResolution)))},
                         {std::min(drc.bitmapWidth() - 1,
                                   static_cast<int>((xMean + boundX) / config.gridResolution)),
                          std::min(drc.bitmapHeight() - 1,
                                   static_cast<int>((yMean + boundY) / config.gridResolution))}}};

  state.stepG["straight_0"] = config.gridResolution * config.lossPropagation;
  state.stepG["straight_45"] = std::sqrt(2.0) * config.gridResolution * config.lossPropagation;
  const double smoothRadius = config.bendRadius - 1e-3;
  const double bend45Length = roundedCornerLength(
      0.0,
      0.0,
      state.bend45Part1 * config.gridResolution,
      0.0,
      (state.bend45Part1 + state.bend45Part2) * config.gridResolution,
      state.bend45Part2 * config.gridResolution,
      smoothRadius,
      config.bendPointsDistance);
  state.stepG["bend_45_1"] = bend45Length * config.lossPropagation + config.lossBending / 2.0;
  state.stepG["bend_45_2"] = state.stepG["bend_45_1"];
  const double bend90Length = roundedCornerLength(
      0.0,
      0.0,
      state.gridRadius * config.gridResolution,
      0.0,
      state.gridRadius * config.gridResolution,
      state.gridRadius * config.gridResolution,
      smoothRadius,
      config.bendPointsDistance);
  state.stepG["bend_90"] = bend90Length * config.lossPropagation + config.lossBending;

  const int gr = state.gridRadius;
  const int b1 = state.bend45Part1;
  const int b2 = state.bend45Part2;
  state.nextSteps = {
      {0, {{gr, -gr, 270, "bend_90"}, {1, 0, 0, "straight_0"}, {gr, gr, 90, "bend_90"}}},
      {90, {{gr, gr, 0, "bend_90"}, {0, 1, 90, "straight_0"}, {-gr, gr, 180, "bend_90"}}},
      {180, {{-gr, gr, 90, "bend_90"}, {-1, 0, 180, "straight_0"}, {-gr, -gr, 270, "bend_90"}}},
      {270, {{-gr, -gr, 180, "bend_90"}, {0, -1, 270, "straight_0"}, {gr, -gr, 0, "bend_90"}}},
      {45, {{b1 + b2, b2, 0, "bend_45_2"}, {1, 1, 45, "straight_45"}, {b2, b1 + b2, 90, "bend_45_2"}}},
      {135, {{-b2, b2 + b1, 90, "bend_45_2"}, {-1, 1, 135, "straight_45"}, {-b2 - b1, b2, 180, "bend_45_2"}}},
      {225, {{-b2 - b1, -b2, 180, "bend_45_2"}, {-1, -1, 225, "straight_45"}, {-b2, -b2 - b1, 270, "bend_45_2"}}},
      {315, {{b2, -b2 - b1, 270, "bend_45_2"}, {1, -1, 315, "straight_45"}, {b2 + b1, -b2, 0, "bend_45_2"}}},
  };

  if (net.enable45 && config.enable45Neighbor) {
    state.nextSteps[0].push_back({b2 + b1, -b2, 315, "bend_45_1"});
    state.nextSteps[0].push_back({b2 + b1, b2, 45, "bend_45_1"});
    state.nextSteps[90].push_back({-b2, b2 + b1, 135, "bend_45_1"});
    state.nextSteps[90].push_back({b2, b2 + b1, 45, "bend_45_1"});
    state.nextSteps[180].push_back({-b2 - b1, b2, 135, "bend_45_1"});
    state.nextSteps[180].push_back({-b2 - b1, -b2, 225, "bend_45_1"});
    state.nextSteps[270].push_back({-b2, -b2 - b1, 225, "bend_45_1"});
    state.nextSteps[270].push_back({b2, -b2 - b1, 315, "bend_45_1"});
  }

  const LidarPort* startPort = nullptr;
  const LidarPort* endPort = nullptr;
  if (config.group) {
    if (net.reverse) {
      startPort = &port2;
      endPort = &port1;
    } else {
      startPort = &port1;
      endPort = &port2;
    }
  } else {
    startPort = &port1;
    endPort = &port2;
  }

  state.startPortName = startPort->portName;
  state.endPortName = endPort->portName;
  state.accessPoints = endPort->portGrids.size();
  std::array<int, 2> endPos = endPort->portGrids.front();
  if (config.group && net.earlyAccess) {
    const int accessGrid = static_cast<int>(state.radius / config.gridResolution);
    endPos = *(endPort->portGrids.end() - accessGrid);
  }
  const auto startPos = startPort->portGrids.front();
  state.endNode = portGridNode(*endPort, endPos);
  state.startNode = portGridNode(*startPort, startPos);
  state.startCostF = customH(startPos, endPos, config.lossBending, config.gridResolution);
  return state;
}

namespace
{

std::vector<std::array<int, 3>> processBendPath(
    const std::vector<std::array<int, 3>>& path,
    const LidarAstarInitState& state)
{
  std::vector<std::array<int, 3>> vec;
  if (path.size() <= 2) {
    return path;
  }

  for (std::size_t i = 0; i < path.size(); ++i) {
    vec.push_back(path[i]);
    if (i == path.size() - 1) {
      break;
    }
    if (path[i][2] == path[i + 1][2]) {
      continue;
    }

    const int ori = path[i][2];
    int angle = std::abs(path[i + 1][2] - ori);
    if (angle > 180) {
      angle = 360 - angle;
    }
    if (angle == 45) {
      switch (ori) {
        case 0:
          vec.push_back({path[i][0] + state.bend45Part1, path[i][1], ori});
          break;
        case 90:
          vec.push_back({path[i][0], path[i][1] + state.bend45Part1, ori});
          break;
        case 180:
          vec.push_back({path[i][0] - state.bend45Part1, path[i][1], ori});
          break;
        case 270:
          vec.push_back({path[i][0], path[i][1] - state.bend45Part1, ori});
          break;
        case 45:
          vec.push_back({path[i][0] + state.bend45Part2,
                         path[i][1] + state.bend45Part2,
                         ori});
          break;
        case 135:
          vec.push_back({path[i][0] - state.bend45Part2,
                         path[i][1] + state.bend45Part2,
                         ori});
          break;
        case 225:
          vec.push_back({path[i][0] - state.bend45Part2,
                         path[i][1] - state.bend45Part2,
                         ori});
          break;
        case 315:
          vec.push_back({path[i][0] + state.bend45Part2,
                         path[i][1] - state.bend45Part2,
                         ori});
          break;
        default:
          break;
      }
    } else {
      switch (ori) {
        case 0:
          vec.push_back({path[i][0] + state.gridRadius, path[i][1], ori});
          break;
        case 90:
          vec.push_back({path[i][0], path[i][1] + state.gridRadius, ori});
          break;
        case 180:
          vec.push_back({path[i][0] - state.gridRadius, path[i][1], ori});
          break;
        case 270:
          vec.push_back({path[i][0], path[i][1] - state.gridRadius, ori});
          break;
        default:
          break;
      }
    }
  }
  return vec;
}

std::vector<std::array<int, 3>> simplifyCollinearPath(
    const std::vector<std::array<int, 3>>& path)
{
  if (path.size() <= 2) {
    return path;
  }

  std::vector<std::array<int, 3>> simplified;
  simplified.reserve(path.size());
  simplified.push_back(path.front());
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const auto& cur = path[i];
    const auto& next = path[i + 1];
    const auto& prev = simplified.back();

    const int v1x = cur[0] - prev[0];
    const int v1y = cur[1] - prev[1];
    const int v2x = next[0] - cur[0];
    const int v2y = next[1] - cur[1];
    const bool hasZeroSegment =
        (v1x == 0 && v1y == 0) || (v2x == 0 && v2y == 0);
    const long long cross =
        static_cast<long long>(v1x) * static_cast<long long>(v2y)
        - static_cast<long long>(v1y) * static_cast<long long>(v2x);
    const long long dot =
        static_cast<long long>(v1x) * static_cast<long long>(v2x)
        + static_cast<long long>(v1y) * static_cast<long long>(v2y);

    if (!hasZeroSegment && cross == 0 && dot >= 0) {
      continue;
    }
    simplified.push_back(cur);
  }
  simplified.push_back(path.back());
  return simplified;
}

bool checkConnection(const std::array<int, 3>& pos,
                     const std::array<int, 3>& endPos,
                     const std::set<std::array<int, 2>>& accessPoints)
{
  return accessPoints.find({pos[0], pos[1]}) != accessPoints.end()
         && std::abs(pos[2] - endPos[2]) == 180;
}

std::vector<std::array<int, 3>> backTrackOriginPath(
    const std::vector<GridSearchNode>& nodes,
    std::size_t nodeIndex)
{
  std::vector<std::array<int, 3>> path;
  while (true) {
    const auto& node = nodes[nodeIndex];
    path.push_back(node.pos);
    if (node.parent < 0) {
      break;
    }
    nodeIndex = static_cast<std::size_t>(node.parent);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

LidarPythonStringSet collectCrossingNets(
    const std::vector<GridSearchNode>& nodes,
    std::size_t nodeIndex,
    bool& duplicated)
{
  LidarPythonStringSet crossingNets;
  duplicated = false;
  while (true) {
    const auto& node = nodes[nodeIndex];
    if (!node.crossingNet.empty()) {
      if (crossingNets.count(node.crossingNet) != 0) {
        duplicated = true;
      } else {
        crossingNets.insert(node.crossingNet);
      }
    }
    if (node.parent < 0) {
      break;
    }
    nodeIndex = static_cast<std::size_t>(node.parent);
  }
  return crossingNets;
}

std::set<std::string> collectViolatedNets(const std::vector<GridSearchNode>& nodes,
                                          std::size_t nodeIndex)
{
  std::set<std::string> violatedNets;
  while (true) {
    const auto& node = nodes[nodeIndex];
    violatedNets.insert(node.violatedNets.begin(), node.violatedNets.end());
    if (node.parent < 0) {
      break;
    }
    nodeIndex = static_cast<std::size_t>(node.parent);
  }
  return violatedNets;
}

bool netLessForHeap(const LidarRuntimeView& db, std::size_t lhs, std::size_t rhs)
{
  const auto& left = db.nets[lhs];
  const auto& right = db.nets[rhs];
  return (left.compDist < right.compDist)
         || (left.compDist == right.compDist && left.routingOrder < right.routingOrder);
}

std::vector<std::size_t> pythonHeapOrder(const LidarRuntimeView& db,
                                         const std::vector<std::size_t>& indices)
{
  std::vector<std::size_t> heap;
  heap.reserve(indices.size());

  auto lessIndex = [&](std::size_t lhs, std::size_t rhs) {
    return netLessForHeap(db, lhs, rhs);
  };
  auto siftdown = [&](std::vector<std::size_t>& h, std::size_t startPos, std::size_t pos) {
    const auto newItem = h[pos];
    while (pos > startPos) {
      const std::size_t parentPos = (pos - 1) >> 1;
      const auto parent = h[parentPos];
      if (lessIndex(newItem, parent)) {
        h[pos] = parent;
        pos = parentPos;
        continue;
      }
      break;
    }
    h[pos] = newItem;
  };
  auto siftup = [&](std::vector<std::size_t>& h, std::size_t pos) {
    const std::size_t endPos = h.size();
    const std::size_t startPos = pos;
    const auto newItem = h[pos];
    std::size_t childPos = 2 * pos + 1;
    while (childPos < endPos) {
      const std::size_t rightPos = childPos + 1;
      if (rightPos < endPos && !lessIndex(h[childPos], h[rightPos])) {
        childPos = rightPos;
      }
      h[pos] = h[childPos];
      pos = childPos;
      childPos = 2 * pos + 1;
    }
    h[pos] = newItem;
    siftdown(h, startPos, pos);
  };
  auto heappush = [&](std::vector<std::size_t>& h, std::size_t index) {
    h.push_back(index);
    siftdown(h, 0, h.size() - 1);
  };
  auto heappop = [&](std::vector<std::size_t>& h) {
    const auto lastItem = h.back();
    h.pop_back();
    if (h.empty()) {
      return lastItem;
    }
    const auto returnItem = h.front();
    h.front() = lastItem;
    siftup(h, 0);
    return returnItem;
  };

  for (const auto index : indices) {
    heappush(heap, index);
  }

  std::vector<std::size_t> order;
  order.reserve(heap.size());
  while (!heap.empty()) {
    order.push_back(heappop(heap));
  }
  return order;
}

class PythonNetHeap
{
 public:
  explicit PythonNetHeap(const LidarRuntimeView& db) : _db(db) {}

  bool empty() const { return _heap.empty(); }

  void push(std::size_t index)
  {
    _heap.push_back(index);
    siftdown(0, _heap.size() - 1);
  }

  std::size_t pop()
  {
    const auto lastItem = _heap.back();
    _heap.pop_back();
    if (_heap.empty()) {
      return lastItem;
    }
    const auto returnItem = _heap.front();
    _heap.front() = lastItem;
    siftup(0);
    return returnItem;
  }

 private:
  bool lessIndex(std::size_t lhs, std::size_t rhs) const
  {
    return netLessForHeap(_db, lhs, rhs);
  }

  void siftdown(std::size_t startPos, std::size_t pos)
  {
    const auto newItem = _heap[pos];
    while (pos > startPos) {
      const std::size_t parentPos = (pos - 1) >> 1;
      const auto parent = _heap[parentPos];
      if (lessIndex(newItem, parent)) {
        _heap[pos] = parent;
        pos = parentPos;
        continue;
      }
      break;
    }
    _heap[pos] = newItem;
  }

  void siftup(std::size_t pos)
  {
    const std::size_t endPos = _heap.size();
    const std::size_t startPos = pos;
    const auto newItem = _heap[pos];
    std::size_t childPos = 2 * pos + 1;
    while (childPos < endPos) {
      const std::size_t rightPos = childPos + 1;
      if (rightPos < endPos && !lessIndex(_heap[childPos], _heap[rightPos])) {
        childPos = rightPos;
      }
      _heap[pos] = _heap[childPos];
      pos = childPos;
      childPos = 2 * pos + 1;
    }
    _heap[pos] = newItem;
    siftdown(startPos, pos);
  }

  const LidarRuntimeView& _db;
  std::vector<std::size_t> _heap;
};

std::string joinSet(const std::set<std::string>& values)
{
  std::ostringstream oss;
  bool first = true;
  for (const auto& value : values) {
    if (!first) {
      oss << ",";
    }
    first = false;
    oss << value;
  }
  return oss.str();
}

std::string joinSet(const LidarPythonStringSet& values)
{
  return joinSet(values.sortedValues());
}

void writeYamlStringSeq(std::ostream& os,
                        const std::set<std::string>& values,
                        int indent)
{
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  if (values.empty()) {
    os << "[]\n";
    return;
  }
  os << "\n";
  for (const auto& value : values) {
    os << pad << "- " << value << "\n";
  }
}

void writeYamlStringSeq(std::ostream& os,
                        const LidarPythonStringSet& values,
                        int indent)
{
  writeYamlStringSeq(os, values.sortedValues(), indent);
}

void writeYamlStringSeq(std::ostream& os,
                        const std::vector<std::string>& values,
                        int indent)
{
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  if (values.empty()) {
    os << "[]\n";
    return;
  }
  os << "\n";
  for (const auto& value : values) {
    os << pad << "- " << value << "\n";
  }
}

void writeYamlGridPath(std::ostream& os,
                       const std::vector<std::array<int, 3>>& path,
                       int indent)
{
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  if (path.empty()) {
    os << "[]\n";
    return;
  }
  os << "\n";
  for (const auto& point : path) {
    os << pad << "- [" << point[0] << ", " << point[1] << ", " << point[2]
       << "]\n";
  }
}

void writeYamlGridPointSet(std::ostream& os,
                           const std::set<std::array<int, 2>>& points,
                           int indent)
{
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  if (points.empty()) {
    os << "[]\n";
    return;
  }
  os << "\n";
  for (const auto& point : points) {
    os << pad << "- [" << point[0] << ", " << point[1] << "]\n";
  }
}

void writeYamlCostTrace(std::ostream& os,
                        const std::vector<std::array<double, 2>>& costs,
                        int indent)
{
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  if (costs.empty()) {
    os << "[]\n";
    return;
  }
  const auto oldFlags = os.flags();
  const auto oldPrecision = os.precision();
  os << std::defaultfloat << std::setprecision(17);
  os << "\n";
  for (const auto& cost : costs) {
    os << pad << "- [" << cost[0] << ", " << cost[1] << "]\n";
  }
  os.flags(oldFlags);
  os.precision(oldPrecision);
}

void writeYamlMicronPath(std::ostream& os,
                         const std::vector<std::array<int, 3>>& path,
                         double resolution,
                         int indent)
{
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  if (path.empty()) {
    os << "[]\n";
    return;
  }
  os << "\n";
  for (const auto& point : path) {
    os << pad << "- [" << point[0] * resolution + resolution / 2.0 << ", "
       << point[1] * resolution + resolution / 2.0 << "]\n";
  }
}

std::vector<std::array<double, 2>> toMicronPath(
    const std::vector<std::array<int, 3>>& path,
    double resolution)
{
  std::vector<std::array<double, 2>> result;
  result.reserve(path.size());
  for (const auto& point : path) {
    result.push_back({point[0] * resolution + resolution / 2.0,
                      point[1] * resolution + resolution / 2.0});
  }
  return result;
}

void applyPythonPortAlignment(std::vector<std::array<double, 2>>& path,
                              const LidarPort& startPort,
                              const LidarPort& endPort)
{
  if (path.size() < 2) {
    return;
  }

  const auto originalPath = path;
  const int startOrient = static_cast<int>(std::round(startPort.orientation));
  const int endOrient = static_cast<int>(std::round(endPort.orientation));
  const auto normalizedOrientation = [](int orient) {
    return (orient % 360 + 360) % 360;
  };
  const auto isHorizontal = [](int orient) {
    const int normalized = (orient % 360 + 360) % 360;
    return normalized == 0 || normalized == 180;
  };
  const auto alignPointToPortAxis = [&](std::array<double, 2>& point,
                                        const LidarPort& port,
                                        int orient) {
    if (isHorizontal(orient)) {
      point[1] = port.center.y;
    } else {
      point[0] = port.center.x;
    }
  };
  const auto hasAdjacentDuplicate = [&]() {
    for (std::size_t i = 1; i < path.size(); ++i) {
      if (std::hypot(path[i - 1][0] - path[i][0],
                     path[i - 1][1] - path[i][1])
          < 1e-9) {
        return true;
      }
    }
    return false;
  };
  const auto repairEndpointTangents = [&]() {
    alignPointToPortAxis(path.front(), startPort, startOrient);
    alignPointToPortAxis(path.back(), endPort, endOrient);
    if (path.size() >= 3) {
      alignPointToPortAxis(path[1], startPort, startOrient);
      alignPointToPortAxis(path[path.size() - 2], endPort, endOrient);
    }
  };

  if (path.size() == 2 && isHorizontal(startOrient) && isHorizontal(endOrient)) {
    path[0][1] = startPort.center.y;
    path[1][1] = endPort.center.y;
  } else if (path.size() == 2 && !isHorizontal(startOrient)
             && !isHorizontal(endOrient)) {
    path[0][0] = startPort.center.x;
    path[1][0] = endPort.center.x;
  } else {
    if (normalizedOrientation(startOrient) == 0
        || normalizedOrientation(startOrient) == 180) {
      path[0][1] = startPort.center.y;
      path[1][1] = startPort.center.y;
    } else {
      path[0][0] = startPort.center.x;
      path[1][0] = startPort.center.x;
    }

    // Python LiDAR's alignment() currently snaps the tail Y coordinate for all
    // end-port orientations. Keep this quirk first for route compatibility,
    // then repair the final physical endpoint tangent below if this alignment
    // collapses and has to fall back.
    path[path.size() - 1][1] = endPort.center.y;
    path[path.size() - 2][1] = endPort.center.y;

    // Python only commits alignment when smooth() accepts the aligned points.
    // The most common rejection is a vertical tail collapsing into a duplicate
    // segment after the tail-Y snap; in that case LiDAR keeps the raw grid path
    // and connects the physical port with access waveguides.
    if (hasAdjacentDuplicate()) {
      path = originalPath;
    }
  }

  repairEndpointTangents();
}

double pointDistance(const std::array<double, 2>& lhs,
                     const std::array<double, 2>& rhs)
{
  return std::hypot(lhs[0] - rhs[0], lhs[1] - rhs[1]);
}

bool samePoint(const std::array<double, 2>& lhs,
               const std::array<double, 2>& rhs,
               double tolerance = 1e-9)
{
  return pointDistance(lhs, rhs) <= tolerance;
}

std::vector<std::array<double, 2>> postRoutePointsWithAccess(
    const PostRoutePath& postPath)
{
  std::vector<std::array<double, 2>> physicalPath;
  physicalPath.reserve(postPath.points.size() + 2);

  auto appendUnique = [&](const std::array<double, 2>& point) {
    if (physicalPath.empty()
        || !samePoint(physicalPath.back(), point)) {
      physicalPath.push_back(point);
    }
  };

  appendUnique(postPath.startPort.center);
  for (const auto& point : postPath.points) {
    appendUnique(point);
  }
  appendUnique(postPath.endPort.center);
  return physicalPath;
}

std::vector<std::array<double, 2>> trimPostRoutePointsForPorts(
    std::vector<std::array<double, 2>> points,
    const PathPortDump& startPort,
    const PathPortDump& endPort)
{
  while (!points.empty() && samePoint(points.front(), startPort.center)) {
    points.erase(points.begin());
  }
  while (!points.empty() && samePoint(points.back(), endPort.center)) {
    points.pop_back();
  }

  std::vector<std::array<double, 2>> trimmed;
  trimmed.reserve(points.size());
  for (const auto& point : points) {
    if (trimmed.empty() || !samePoint(trimmed.back(), point)) {
      trimmed.push_back(point);
    }
  }
  return trimmed;
}

std::vector<Point> dbWaveguidePointsWithAccess(const PostRoutePath& postPath)
{
  const auto physicalPath = postRoutePointsWithAccess(postPath);

  std::vector<Point> points;
  points.reserve(physicalPath.size());
  for (const auto& point : physicalPath) {
    points.emplace_back(point[0], point[1]);
  }
  return points;
}

double polylineLength(const std::vector<std::array<double, 2>>& points)
{
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    length += pointDistance(points[i - 1], points[i]);
  }
  return length;
}

double projectOnPolyline(const std::vector<std::array<double, 2>>& points,
                         const std::array<double, 2>& point)
{
  double bestDistance = std::numeric_limits<double>::infinity();
  double bestProject = 0.0;
  double prefix = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto& first = points[i - 1];
    const auto& second = points[i];
    const double vx = second[0] - first[0];
    const double vy = second[1] - first[1];
    const double segmentLength2 = vx * vx + vy * vy;
    if (segmentLength2 <= 1e-18) {
      continue;
    }
    const double wx = point[0] - first[0];
    const double wy = point[1] - first[1];
    double t = (wx * vx + wy * vy) / segmentLength2;
    t = std::max(0.0, std::min(1.0, t));
    const std::array<double, 2> projection = {first[0] + t * vx,
                                              first[1] + t * vy};
    const double distance = pointDistance(point, projection);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestProject = prefix + std::sqrt(segmentLength2) * t;
    }
    prefix += std::sqrt(segmentLength2);
  }
  return bestProject;
}

double cross2d(double ax, double ay, double bx, double by)
{
  return ax * by - ay * bx;
}

double snapGdsFactoryGrid(double value)
{
  return std::round(value * 1000.0) / 1000.0;
}

std::optional<std::array<double, 2>> segmentIntersection(
    const std::array<double, 2>& p,
    const std::array<double, 2>& p2,
    const std::array<double, 2>& q,
    const std::array<double, 2>& q2)
{
  constexpr double eps = 1e-9;
  const double rx = p2[0] - p[0];
  const double ry = p2[1] - p[1];
  const double sx = q2[0] - q[0];
  const double sy = q2[1] - q[1];
  const double denom = cross2d(rx, ry, sx, sy);
  if (std::abs(denom) < eps) {
    return std::nullopt;
  }
  const double qpx = q[0] - p[0];
  const double qpy = q[1] - p[1];
  const double t = cross2d(qpx, qpy, sx, sy) / denom;
  const double u = cross2d(qpx, qpy, rx, ry) / denom;
  if (t < -eps || t > 1.0 + eps || u < -eps || u > 1.0 + eps) {
    return std::nullopt;
  }
  return std::array<double, 2>{p[0] + t * rx, p[1] + t * ry};
}

std::optional<SplitLineResult> splitLineString(
    const std::vector<std::array<double, 2>>& coords,
    const std::array<double, 2>& crossingPoint)
{
  constexpr double eps = 1e-9;
  constexpr double crossingClearance = 4.5;
  if (coords.size() < 2) {
    return std::nullopt;
  }

  const double distance = projectOnPolyline(coords, crossingPoint);
  const double totalLength = polylineLength(coords);
  if (distance <= eps || distance >= totalLength - eps) {
    return std::nullopt;
  }

  double prefix = 0.0;
  std::size_t nextIndex = coords.size();
  for (std::size_t i = 1; i < coords.size(); ++i) {
    prefix += pointDistance(coords[i - 1], coords[i]);
    if (prefix > distance + eps) {
      nextIndex = i;
      break;
    }
  }
  if (nextIndex == coords.size()) {
    return std::nullopt;
  }

  const double x = coords[nextIndex][0];
  const double y = coords[nextIndex][1];
  const double dx = crossingPoint[0] - x;
  const double dy = crossingPoint[1] - y;
  if (std::abs(dx) < eps && std::abs(dy) < eps) {
    return std::nullopt;
  }
  const int slope = std::abs(dx) < eps
                        ? 90
                        : static_cast<int>(std::round(dy / dx));

  auto splitWithClearance = [&](std::array<double, 2> firstPoint,
                                std::array<double, 2> secondPoint,
                                std::array<std::string, 2> ports,
                                double orientation) {
    SplitLineResult result;
    result.ports = ports;
    result.orientation = orientation;
    result.parts[0].insert(result.parts[0].end(),
                           coords.begin(),
                           coords.begin()
                               + static_cast<std::ptrdiff_t>(nextIndex));
    result.parts[0].push_back(firstPoint);
    result.parts[1].push_back(secondPoint);
    result.parts[1].insert(result.parts[1].end(),
                           coords.begin()
                               + static_cast<std::ptrdiff_t>(nextIndex),
                           coords.end());
    return result;
  };

  if (slope == 0) {
    if (dx < 0.0) {
      return splitWithClearance(
          {crossingPoint[0] - crossingClearance, crossingPoint[1]},
          {crossingPoint[0] + crossingClearance, crossingPoint[1]},
          {"o1", "o3"},
          0.0);
    }
    return splitWithClearance(
        {crossingPoint[0] + crossingClearance, crossingPoint[1]},
        {crossingPoint[0] - crossingClearance, crossingPoint[1]},
        {"o3", "o1"},
        0.0);
  }
  if (slope == 90) {
    if (dy < 0.0) {
      return splitWithClearance(
          {crossingPoint[0], crossingPoint[1] - crossingClearance},
          {crossingPoint[0], crossingPoint[1] + crossingClearance},
          {"o4", "o2"},
          0.0);
    }
    return splitWithClearance(
        {crossingPoint[0], crossingPoint[1] + crossingClearance},
        {crossingPoint[0], crossingPoint[1] - crossingClearance},
        {"o2", "o4"},
        0.0);
  }
  if (slope == -1) {
    if (dy > 0.0) {
      return splitWithClearance(
          {crossingPoint[0] - crossingClearance,
           crossingPoint[1] + crossingClearance},
          {crossingPoint[0] + crossingClearance,
           crossingPoint[1] - crossingClearance},
          {"o1", "o3"},
          -45.0);
    }
    return splitWithClearance(
        {crossingPoint[0] + crossingClearance,
         crossingPoint[1] - crossingClearance},
        {crossingPoint[0] - crossingClearance,
         crossingPoint[1] + crossingClearance},
        {"o3", "o1"},
        -45.0);
  }
  if (slope == 1) {
    if (dy > 0.0) {
      return splitWithClearance(
          {crossingPoint[0] + crossingClearance,
           crossingPoint[1] + crossingClearance},
          {crossingPoint[0] - crossingClearance,
           crossingPoint[1] - crossingClearance},
          {"o2", "o4"},
          -45.0);
    }
    return splitWithClearance(
        {crossingPoint[0] - crossingClearance,
         crossingPoint[1] - crossingClearance},
        {crossingPoint[0] + crossingClearance,
         crossingPoint[1] + crossingClearance},
        {"o4", "o2"},
        -45.0);
  }
  return std::nullopt;
}

std::optional<SplitPathResult> splitPostRoutePaths(
    const std::vector<PostRoutePath>& net1Paths,
    const std::vector<PostRoutePath>& net2Paths)
{
  for (std::size_t index1 = 0; index1 < net1Paths.size(); ++index1) {
    const auto& path1 = net1Paths[index1].points;
    for (std::size_t index2 = 0; index2 < net2Paths.size(); ++index2) {
      const auto& path2 = net2Paths[index2].points;
      for (std::size_t i = 1; i < path1.size(); ++i) {
        for (std::size_t j = 1; j < path2.size(); ++j) {
          const auto intersection =
              segmentIntersection(path1[i - 1],
                                  path1[i],
                                  path2[j - 1],
                                  path2[j]);
          if (!intersection.has_value()) {
            continue;
          }
          const auto split1 = splitLineString(path1, intersection.value());
          const auto split2 = splitLineString(path2, intersection.value());
          if (!split1.has_value() || !split2.has_value()) {
            continue;
          }
          if (std::abs(split1->orientation - split2->orientation) > 1e-6) {
            continue;
          }
          SplitPathResult result;
          result.path1Index = index1;
          result.path2Index = index2;
          result.split1 = split1.value();
          result.split2 = split2.value();
          result.crossingPoint = intersection.value();
          result.crossingOrientation = result.split1.orientation;
          return result;
        }
      }
    }
  }
  return std::nullopt;
}

double normalizePortOrientation(double value)
{
  double normalized = std::fmod(value, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  if (std::abs(normalized - 360.0) < 1e-9) {
    return 0.0;
  }
  return normalized;
}

double angleDelta(double lhs, double rhs)
{
  const double normalizedLhs = normalizePortOrientation(lhs);
  const double normalizedRhs = normalizePortOrientation(rhs);
  const double diff = std::abs(normalizedLhs - normalizedRhs);
  return std::min(diff, 360.0 - diff);
}

std::string pathPortObjectName(const PathPortDump& port)
{
  if (port.instanceName.empty()) {
    return port.name;
  }
  if (port.pinName.empty()) {
    return port.instanceName + "," + port.name;
  }
  return port.instanceName + "," + port.pinName;
}

std::optional<double> tangentOrientation(const std::array<double, 2>& from,
                                         const std::array<double, 2>& to)
{
  constexpr double eps = 1e-12;
  const double dx = to[0] - from[0];
  const double dy = to[1] - from[1];
  if (std::hypot(dx, dy) <= eps) {
    return std::nullopt;
  }
  return normalizePortOrientation(std::atan2(dy, dx) * 180.0 / std::acos(-1.0));
}

std::vector<PostProcessAccessViolation> validatePostRoutePathAccess(
    const std::string& netName,
    std::size_t pathIndex,
    const PostRoutePath& postPath,
    double toleranceDegrees = 1.0)
{
  std::vector<PostProcessAccessViolation> violations;
  const auto points = dbWaveguidePointsWithAccess(postPath);
  if (points.size() < 2) {
    return violations;
  }

  auto checkEndpoint = [&](const PathPortDump& port,
                           const std::array<double, 2>& from,
                           const std::array<double, 2>& to,
                           const std::string& endpoint) {
    const auto tangent = tangentOrientation(from, to);
    if (!tangent.has_value()) {
      return;
    }
    const double delta = angleDelta(tangent.value(), port.orientation);
    if (delta <= toleranceDegrees) {
      return;
    }
    PostProcessAccessViolation violation;
    violation.netName = netName;
    violation.pathIndex = pathIndex;
    violation.endpoint = endpoint;
    violation.objectName = pathPortObjectName(port);
    violation.center = port.center;
    violation.tangentOrientation = tangent.value();
    violation.portOrientation = normalizePortOrientation(port.orientation);
    violation.delta = delta;
    violations.push_back(std::move(violation));
  };

  checkEndpoint(postPath.startPort,
                {points.front().x(), points.front().y()},
                {points[1].x(), points[1].y()},
                "start");
  checkEndpoint(postPath.endPort,
                {points.back().x(), points.back().y()},
                {points[points.size() - 2].x(), points[points.size() - 2].y()},
                "end");
  return violations;
}

std::vector<PostProcessAccessViolation> validatePostRouteAccesses(
    const std::string& netName,
    const std::vector<PostRoutePath>& paths)
{
  std::vector<PostProcessAccessViolation> violations;
  for (std::size_t index = 0; index < paths.size(); ++index) {
    auto pathViolations =
        validatePostRoutePathAccess(netName, index, paths[index]);
    violations.insert(violations.end(),
                      std::make_move_iterator(pathViolations.begin()),
                      std::make_move_iterator(pathViolations.end()));
  }
  return violations;
}

std::map<std::string, PathPortDump> crossingPorts(
    const std::string& crossingName,
    const std::array<double, 2>& crossingPoint,
    double orientation)
{
  constexpr double portOffset = 4.0;
  const double radians = orientation * std::acos(-1.0) / 180.0;
  const double cosAngle = std::cos(radians);
  const double sinAngle = std::sin(radians);

  struct BasePort
  {
    const char* name;
    std::array<double, 2> offset;
    double orientation;
  };
  const std::array<BasePort, 4> basePorts = {
      {{"o1", {-portOffset, 0.0}, 180.0},
       {"o2", {0.0, portOffset}, 90.0},
       {"o3", {portOffset, 0.0}, 0.0},
       {"o4", {0.0, -portOffset}, 270.0}}};

  std::map<std::string, PathPortDump> ports;
  for (const auto& basePort : basePorts) {
    PathPortDump port;
    port.name = basePort.name;
    port.instanceName = crossingName;
    port.pinName = basePort.name;
    port.center = {crossingPoint[0] + basePort.offset[0] * cosAngle
                       - basePort.offset[1] * sinAngle,
                    crossingPoint[1] + basePort.offset[0] * sinAngle
                        + basePort.offset[1] * cosAngle};
    port.orientation =
        normalizePortOrientation(basePort.orientation + orientation);
    port.width = 0.5;
    ports[port.name] = port;
  }
  return ports;
}

PathPortDump pathPortFromLidarPort(const LidarPort& port)
{
  PathPortDump dump;
  dump.name = port.pinName;
  dump.instanceName = port.instanceName;
  dump.pinName = port.pinName;
  dump.center = {port.center.x, port.center.y};
  dump.orientation = port.orientation;
  dump.width = port.width;
  return dump;
}

void writeYamlMicronPointPath(std::ostream& os,
                              const std::vector<std::array<double, 2>>& path,
                              int indent)
{
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  if (path.empty()) {
    os << "[]\n";
    return;
  }
  os << "\n";
  for (const auto& point : path) {
    os << pad << "- [" << point[0] << ", " << point[1] << "]\n";
  }
}

void writeYamlPort(std::ostream& os, const LidarPort& port, int indent)
{
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  os << "\n";
  os << pad << "name: " << port.portName << "\n";
  os << pad << "center: [" << port.center.x << ", " << port.center.y << "]\n";
  os << pad << "orientation: " << port.orientation << "\n";
  os << pad << "width: " << port.width << "\n";
}

void writeYamlPathPort(std::ostream& os, const PathPortDump& port, int indent)
{
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  os << "\n";
  os << pad << "name: " << port.name << "\n";
  os << pad << "center: [" << port.center[0] << ", " << port.center[1]
     << "]\n";
  os << pad << "orientation: " << port.orientation << "\n";
  os << pad << "width: " << port.width << "\n";
}

void writeYamlPostRoutePaths(std::ostream& os,
                             const std::vector<PostRoutePath>& paths,
                             int indent)
{
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  if (paths.empty()) {
    os << "[]\n";
    return;
  }
  os << "\n";
  for (const auto& path : paths) {
    os << pad << "- points: ";
    writeYamlMicronPointPath(os, path.points, indent + 4);
    os << pad << "  start_port: ";
    writeYamlPathPort(os, path.startPort, indent + 4);
    os << pad << "  end_port: ";
    writeYamlPathPort(os, path.endPort, indent + 4);
  }
}

void writeYamlPostProcessAccessViolations(
    std::ostream& os,
    const std::vector<PostProcessAccessViolation>& violations,
    int indent)
{
  const std::string pad(static_cast<std::size_t>(indent), ' ');
  if (violations.empty()) {
    os << "[]\n";
    return;
  }
  os << "\n";
  const auto oldFlags = os.flags();
  const auto oldPrecision = os.precision();
  os << std::defaultfloat << std::setprecision(17);
  for (const auto& violation : violations) {
    os << pad << "- endpoint: " << violation.endpoint << "\n";
    os << pad << "  path_index: " << violation.pathIndex << "\n";
    os << pad << "  object: " << violation.objectName << "\n";
    os << pad << "  center: [" << violation.center[0] << ", "
       << violation.center[1] << "]\n";
    os << pad << "  tangent_orientation: "
       << violation.tangentOrientation << "\n";
    os << pad << "  port_orientation: " << violation.portOrientation << "\n";
    os << pad << "  delta: " << violation.delta << "\n";
  }
  os.flags(oldFlags);
  os.precision(oldPrecision);
}

struct PythonPostProcessRoutes
{
  struct Crossing
  {
    std::string name;
    std::string firstNet;
    std::string secondNet;
    std::array<double, 2> center = {0.0, 0.0};
    double orientation = 0.0;
    std::array<std::string, 2> firstNetPorts;
    std::array<std::string, 2> secondNetPorts;
  };

  std::unordered_map<std::string, std::vector<PostRoutePath>> paths;
  std::unordered_map<std::string, LidarPythonStringSet> crossingSets;
  std::vector<Crossing> crossings;
};

double overlapLength(double a0, double a1, double b0, double b1)
{
  const double lower = std::max(std::min(a0, a1), std::min(b0, b1));
  const double upper = std::min(std::max(a0, a1), std::max(b0, b1));
  return std::max(0.0, upper - lower);
}

double pathMaxX(const std::vector<std::array<double, 2>>& points)
{
  double maxX = -std::numeric_limits<double>::infinity();
  for (const auto& point : points) {
    maxX = std::max(maxX, point[0]);
  }
  return maxX;
}

bool repairOneFanoutAccessOverlap(PostRoutePath& throughPath,
                                  const PostRoutePath& accessPath,
                                  const LidarRouteConfig& config)
{
  constexpr double eps = 1e-9;
  constexpr double crossingClearance = 4.5;
  const auto throughPoints = postRoutePointsWithAccess(throughPath);
  const auto accessPoints = postRoutePointsWithAccess(accessPath);
  if (throughPoints.size() < 5 || accessPoints.size() < 2) {
    return false;
  }
  if (accessPath.startPort.instanceName == throughPath.startPort.instanceName
      || accessPath.startPort.instanceName == throughPath.endPort.instanceName) {
    return false;
  }

  const auto& accessStart = accessPoints[0];
  const auto& accessEnd = accessPoints[1];
  if (std::abs(accessStart[1] - accessEnd[1]) > eps) {
    return false;
  }

  for (std::size_t segIndex = 1; segIndex + 1 < throughPoints.size(); ++segIndex) {
    const auto& segStart = throughPoints[segIndex];
    const auto& segEnd = throughPoints[segIndex + 1];
    if (std::abs(segStart[1] - segEnd[1]) > eps
        || std::abs(segStart[1] - accessStart[1]) > eps) {
      continue;
    }
    if (overlapLength(segStart[0],
                      segEnd[0],
                      accessStart[0],
                      accessEnd[0])
        <= config.gridResolution) {
      continue;
    }

    std::optional<std::size_t> turnIndex;
    for (std::size_t i = segIndex + 2; i + 1 < throughPoints.size(); ++i) {
      const auto& first = throughPoints[i];
      const auto& second = throughPoints[i + 1];
      if (std::abs(first[0] - second[0]) <= eps
          && std::abs(second[1] - segStart[1]) > 2.0 * crossingClearance) {
        turnIndex = i + 1;
        break;
      }
    }
    if (!turnIndex.has_value() || turnIndex.value() == 0
        || turnIndex.value() > throughPath.points.size()) {
      continue;
    }

    const auto& turnPoint = throughPoints[turnIndex.value()];
    const double detourY =
        segStart[1] + (turnPoint[1] > segStart[1] ? -crossingClearance
                                                   : crossingClearance);
    double detourX =
        std::ceil(pathMaxX(accessPoints) + throughPath.startPort.width);
    detourX = std::max(detourX,
                        std::max(segStart[0], segEnd[0]) + crossingClearance);
    detourX = snapGdsFactoryGrid(detourX);

    std::vector<std::array<double, 2>> repaired;
    auto append = [&](const std::array<double, 2>& point) {
      if (repaired.empty() || !samePoint(repaired.back(), point)) {
        repaired.push_back(point);
      }
    };

    repaired.reserve(throughPath.points.size() + 4);
    for (std::size_t i = 0; i < segIndex && i < throughPath.points.size(); ++i) {
      append(throughPath.points[i]);
    }
    append({segStart[0], detourY});
    append({detourX, detourY});
    append({detourX, turnPoint[1]});
    append(turnPoint);
    for (std::size_t i = turnIndex.value(); i < throughPath.points.size(); ++i) {
      append(throughPath.points[i]);
    }

    throughPath.points = std::move(repaired);
    return true;
  }

  return false;
}

void repairFanoutAccessOverlaps(PythonPostProcessRoutes& routes,
                                 const LidarRouteConfig& config)
{
  std::vector<std::string> netNames;
  netNames.reserve(routes.paths.size());
  for (const auto& entry : routes.paths) {
    netNames.push_back(entry.first);
  }
  std::sort(netNames.begin(), netNames.end());

  constexpr int maxRepairs = 16;
  int repairs = 0;
  bool changed = true;
  while (changed && repairs < maxRepairs) {
    changed = false;
    for (const auto& throughName : netNames) {
      auto throughIt = routes.paths.find(throughName);
      if (throughIt == routes.paths.end()) {
        continue;
      }
      for (auto& throughPath : throughIt->second) {
        for (const auto& accessName : netNames) {
          if (throughName == accessName) {
            continue;
          }
          auto accessIt = routes.paths.find(accessName);
          if (accessIt == routes.paths.end()) {
            continue;
          }
          for (const auto& accessPath : accessIt->second) {
            if (repairOneFanoutAccessOverlap(throughPath, accessPath, config)) {
              ++repairs;
              changed = true;
              break;
            }
          }
          if (changed || repairs >= maxRepairs) {
            break;
          }
        }
        if (changed || repairs >= maxRepairs) {
          break;
        }
      }
      if (changed || repairs >= maxRepairs) {
        break;
      }
    }
  }
}

bool hasEarlyUnsupportedDiagonal(const PostRoutePath& path)
{
  constexpr double eps = 1e-9;
  const std::size_t searchEnd = std::min<std::size_t>(path.points.size(), 4);
  for (std::size_t i = 1; i < searchEnd; ++i) {
    const double dx = path.points[i][0] - path.points[i - 1][0];
    const double dy = path.points[i][1] - path.points[i - 1][1];
    if (std::abs(dx) <= eps || std::abs(dy) <= eps) {
      continue;
    }
    if (std::abs(std::abs(dx) - std::abs(dy)) > 1e-6) {
      return true;
    }
  }
  return false;
}

bool pathHasPhysicalIntersection(const PostRoutePath& path,
                                 const PythonPostProcessRoutes& routes,
                                 const std::string& netName)
{
  constexpr double eps = 1e-6;
  const auto pathPoints = postRoutePointsWithAccess(path);
  if (pathPoints.size() < 2) {
    return false;
  }
  const double pathLength = polylineLength(pathPoints);

  for (const auto& entry : routes.paths) {
    if (entry.first == netName) {
      continue;
    }
    for (const auto& otherPath : entry.second) {
      const auto otherPoints = postRoutePointsWithAccess(otherPath);
      if (otherPoints.size() < 2) {
        continue;
      }
      const double otherLength = polylineLength(otherPoints);
      for (std::size_t i = 1; i < pathPoints.size(); ++i) {
        for (std::size_t j = 1; j < otherPoints.size(); ++j) {
          const auto intersection = segmentIntersection(pathPoints[i - 1],
                                                        pathPoints[i],
                                                        otherPoints[j - 1],
                                                        otherPoints[j]);
          if (!intersection.has_value()) {
            continue;
          }
          const double pathDistance =
              projectOnPolyline(pathPoints, intersection.value());
          const double otherDistance =
              projectOnPolyline(otherPoints, intersection.value());
          if (pathDistance <= eps || pathDistance >= pathLength - eps
              || otherDistance <= eps || otherDistance >= otherLength - eps) {
            continue;
          }
          return true;
        }
      }
    }
  }
  return false;
}

bool pathHasSelfIntersection(const PostRoutePath& path)
{
  const auto points = postRoutePointsWithAccess(path);
  if (points.size() < 4) {
    return false;
  }
  for (std::size_t i = 1; i < points.size(); ++i) {
    for (std::size_t j = i + 2; j < points.size(); ++j) {
      const auto intersection = segmentIntersection(points[i - 1],
                                                    points[i],
                                                    points[j - 1],
                                                    points[j]);
      if (intersection.has_value()) {
        return true;
      }
    }
  }
  return false;
}

bool repairOneLeftHorizontalConflictDetour(PostRoutePath& path,
                                           const PythonPostProcessRoutes& routes,
                                           const std::string& netName,
                                           const LidarRouteConfig& config)
{
  constexpr double eps = 1e-6;
  const double minHorizontalLength = 12.0 * config.gridResolution;
  const double sideClearance = std::max(4.0, 2.5 * config.gridResolution);
  const double detourStep = std::max(6.0, 3.5 * config.gridResolution);
  if (path.points.size() < 3) {
    return false;
  }

  for (std::size_t i = 0; i + 2 < path.points.size(); ++i) {
    const auto& segmentStart = path.points[i];
    const auto& segmentEnd = path.points[i + 1];
    const auto& nextPoint = path.points[i + 2];
    if (std::abs(segmentStart[1] - segmentEnd[1]) > eps
        || segmentEnd[0] >= segmentStart[0] - eps
        || segmentStart[0] - segmentEnd[0] < minHorizontalLength
        || std::abs(nextPoint[0] - segmentEnd[0]) > eps) {
      continue;
    }

    std::vector<double> conflictXs;
    for (const auto& entry : routes.paths) {
      if (entry.first == netName) {
        continue;
      }
      for (const auto& otherPath : entry.second) {
        const auto otherPoints = postRoutePointsWithAccess(otherPath);
        for (std::size_t j = 1; j < otherPoints.size(); ++j) {
          const auto intersection = segmentIntersection(segmentStart,
                                                        segmentEnd,
                                                        otherPoints[j - 1],
                                                        otherPoints[j]);
          if (!intersection.has_value()) {
            continue;
          }
          const auto point = intersection.value();
          if (std::abs(point[1] - segmentStart[1]) > eps
              || point[0] <= segmentEnd[0] + eps
              || point[0] >= segmentStart[0] - eps) {
            continue;
          }
          conflictXs.push_back(point[0]);
        }
      }
    }
    if (conflictXs.empty()) {
      continue;
    }

    const double rightmostConflict =
        *std::max_element(conflictXs.begin(), conflictXs.end());
    double detourX = snapGdsFactoryGrid(rightmostConflict + sideClearance);
    if (detourX >= segmentStart[0] - config.gridResolution) {
      detourX = snapGdsFactoryGrid(segmentStart[0] - config.gridResolution);
    }
    if (detourX <= segmentEnd[0] + config.gridResolution) {
      continue;
    }

    const double verticalDirection = nextPoint[1] - segmentEnd[1];
    const double detourSign = verticalDirection < 0.0 ? -1.0 : 1.0;
    for (int attempt = 1; attempt <= 4; ++attempt) {
      const double detourY = snapGdsFactoryGrid(
          segmentStart[1] + detourSign * detourStep * attempt);
      std::vector<std::array<double, 2>> repaired;
      repaired.reserve(path.points.size() + 2);
      auto append = [&](const std::array<double, 2>& point) {
        if (repaired.empty() || !samePoint(repaired.back(), point)) {
          repaired.push_back(point);
        }
      };

      for (std::size_t k = 0; k <= i; ++k) {
        append(path.points[k]);
      }
      append({detourX, segmentStart[1]});
      append({detourX, detourY});
      append({segmentEnd[0], detourY});
      for (std::size_t k = i + 2; k < path.points.size(); ++k) {
        append(path.points[k]);
      }

      PostRoutePath candidate = path;
      candidate.points = std::move(repaired);
      if (pathHasSelfIntersection(candidate)
          || pathHasPhysicalIntersection(candidate, routes, netName)) {
        continue;
      }
      path = std::move(candidate);
      return true;
    }
  }

  return false;
}

void repairPostSplitHorizontalConflictDetours(PythonPostProcessRoutes& routes,
                                              const LidarRouteConfig& config)
{
  constexpr int maxRepairs = 8;
  for (int repair = 0; repair < maxRepairs; ++repair) {
    bool changed = false;
    for (auto& entry : routes.paths) {
      for (auto& path : entry.second) {
        if (repairOneLeftHorizontalConflictDetour(
                path, routes, entry.first, config)) {
          changed = true;
          break;
        }
      }
      if (changed) {
        break;
      }
    }
    if (!changed) {
      break;
    }
  }
}

bool repairOneFanoutMrrOrderInversion(PostRoutePath& path,
                                      const LidarRouteConfig& config)
{
  constexpr double eps = 1e-9;
  constexpr double crossingClearance = 4.5;
  if (path.points.size() < 4) {
    return false;
  }
  if (path.startPort.instanceName.find("fanout") == std::string::npos
      || path.endPort.instanceName.find("mrr_array") == std::string::npos) {
    return false;
  }
  if (angleDelta(path.startPort.orientation, 0.0) > eps
      || angleDelta(path.endPort.orientation, 180.0) > eps) {
    return false;
  }
  if (std::abs(path.endPort.center[1] - path.startPort.center[1])
      <= 20.0 * config.gridResolution) {
    return false;
  }
  if (!hasEarlyUnsupportedDiagonal(path)) {
    return false;
  }

  const auto first = path.points.front();
  const auto last = path.points.back();
  if (last[0] >= path.endPort.center[0] - path.endPort.width) {
    return false;
  }

  double detourX = std::ceil(path.endPort.center[0] + config.gridResolution);
  detourX = std::max(detourX, first[0] + crossingClearance);
  detourX = snapGdsFactoryGrid(detourX);

  const double targetApproachY =
      last[1] + (last[1] < first[1] ? -crossingClearance : crossingClearance);

  std::vector<std::array<double, 2>> repaired;
  repaired.reserve(5);
  auto append = [&](const std::array<double, 2>& point) {
    if (repaired.empty() || !samePoint(repaired.back(), point)) {
      repaired.push_back(point);
    }
  };

  append(first);
  if (last[1] < first[1]) {
    const double sourceBypassY = first[1] + crossingClearance;
    append({first[0], sourceBypassY});
    append({detourX, sourceBypassY});
  } else {
    append({detourX, first[1]});
  }
  append({detourX, targetApproachY});
  append({last[0], targetApproachY});
  append(last);
  path.points = std::move(repaired);
  return true;
}

bool repairOneFanoutMrrSelfIntersection(PostRoutePath& path,
                                        const LidarRouteConfig& config)
{
  constexpr double eps = 1e-9;
  constexpr double crossingClearance = 4.5;
  if (path.points.size() < 4) {
    return false;
  }
  if (path.startPort.instanceName.find("fanout") == std::string::npos
      || path.endPort.instanceName.find("mrr_array") == std::string::npos) {
    return false;
  }
  if (angleDelta(path.startPort.orientation, 0.0) > eps
      || angleDelta(path.endPort.orientation, 180.0) > eps) {
    return false;
  }
  if (std::abs(path.endPort.center[1] - path.startPort.center[1])
      > 2.0 * config.gridResolution) {
    return false;
  }
  if (!pathHasSelfIntersection(path)) {
    return false;
  }

  const auto first = path.points.front();
  const auto last = path.points.back();
  if (last[0] <= first[0]) {
    return false;
  }

  const double bypassY =
      first[1] + (last[1] >= first[1] ? -crossingClearance
                                      : crossingClearance);

  std::vector<std::array<double, 2>> repaired;
  repaired.reserve(4);
  auto append = [&](const std::array<double, 2>& point) {
    if (repaired.empty() || !samePoint(repaired.back(), point)) {
      repaired.push_back(point);
    }
  };

  append(first);
  append({first[0], bypassY});
  append({last[0], bypassY});
  append(last);
  path.points = std::move(repaired);
  return true;
}

void repairFanoutMrrSelfIntersections(PythonPostProcessRoutes& routes,
                                      const LidarRouteConfig& config)
{
  for (auto& entry : routes.paths) {
    for (auto& path : entry.second) {
      repairOneFanoutMrrSelfIntersection(path, config);
    }
  }
}

bool isFanoutToMrrPath(const PostRoutePath& path)
{
  constexpr double eps = 1e-9;
  return path.startPort.instanceName.find("fanout") != std::string::npos
         && path.endPort.instanceName.find("mrr_array") != std::string::npos
         && angleDelta(path.startPort.orientation, 0.0) <= eps
         && angleDelta(path.endPort.orientation, 180.0) <= eps;
}

std::optional<std::size_t> firstLongVerticalSegmentIndex(
    const PostRoutePath& path,
    double minLength)
{
  constexpr double eps = 1e-9;
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const auto& prev = path.points[i - 1];
    const auto& point = path.points[i];
    if (std::abs(point[0] - prev[0]) > eps) {
      continue;
    }
    const double length = std::abs(point[1] - prev[1]);
    if (length >= minLength) {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<double> firstExistingLaneAfterSegment(const PostRoutePath& path,
                                                   std::size_t segmentIndex,
                                                   double trunkX,
                                                   double minSeparation)
{
  for (std::size_t i = segmentIndex + 1; i < path.points.size(); ++i) {
    const double x = path.points[i][0];
    if (x > trunkX + minSeparation) {
      return snapGdsFactoryGrid(x);
    }
  }
  return std::nullopt;
}

bool shiftFanoutMrrSharedTrunk(PostRoutePath& path,
                               std::size_t verticalSegmentIndex,
                               double laneX)
{
  constexpr double eps = 1e-9;
  if (verticalSegmentIndex == 0 || verticalSegmentIndex >= path.points.size()) {
    return false;
  }
  const auto trunkStart = path.points[verticalSegmentIndex - 1];
  const auto trunkEnd = path.points[verticalSegmentIndex];
  if (std::abs(laneX - trunkStart[0]) <= eps) {
    return false;
  }

  std::vector<std::array<double, 2>> repaired;
  repaired.reserve(path.points.size() + 1);
  auto append = [&](const std::array<double, 2>& point) {
    if (repaired.empty() || !samePoint(repaired.back(), point)) {
      repaired.push_back(point);
    }
  };

  for (std::size_t i = 0; i + 1 < verticalSegmentIndex; ++i) {
    append(path.points[i]);
  }
  append({laneX, trunkStart[1]});
  append({laneX, trunkEnd[1]});
  for (std::size_t i = verticalSegmentIndex + 1; i < path.points.size(); ++i) {
    append(path.points[i]);
  }
  if (repaired.size() < 2) {
    return false;
  }
  path.points = std::move(repaired);
  return true;
}

bool rewriteDownwardFanoutMrrBypass(PostRoutePath& path,
                                    const PostRoutePath& adjacentPath,
                                    double sharedTrunkX,
                                    const LidarRouteConfig& config)
{
  constexpr double eps = 1e-9;
  if (path.points.size() < 2) {
    return false;
  }
  const auto first = path.points.front();
  const auto last = path.points.back();
  if (last[1] >= first[1] - eps) {
    return false;
  }

  const double entryX =
      snapGdsFactoryGrid(sharedTrunkX + 2.0 * config.gridResolution);
  const double bypassX =
      snapGdsFactoryGrid(sharedTrunkX - 5.0 * config.gridResolution);
  const double bypassY = snapGdsFactoryGrid(
      adjacentPath.endPort.center[1] + 3.0 * config.gridResolution);
  const double diagonalCenterY = snapGdsFactoryGrid(
      0.5 * (adjacentPath.endPort.center[1] + path.endPort.center[1])
      + 0.875 * config.gridResolution);
  const double diagonalHalfSpan = 4.5;
  const std::array<double, 2> diagonalEntry = {
      snapGdsFactoryGrid(bypassX - diagonalHalfSpan),
      snapGdsFactoryGrid(diagonalCenterY + diagonalHalfSpan)};
  const std::array<double, 2> diagonalExit = {
      snapGdsFactoryGrid(bypassX + diagonalHalfSpan),
      snapGdsFactoryGrid(diagonalCenterY - diagonalHalfSpan)};
  if (bypassY >= first[1] - 2.0 * config.gridResolution
      || bypassY <= diagonalEntry[1] + 2.0 * config.gridResolution
      || diagonalExit[1] <= last[1] + 2.0 * config.gridResolution) {
    return false;
  }

  std::vector<std::array<double, 2>> repaired;
  repaired.reserve(9);
  auto append = [&](const std::array<double, 2>& point) {
    if (repaired.empty() || !samePoint(repaired.back(), point)) {
      repaired.push_back(point);
    }
  };
  append(first);
  append({entryX, first[1]});
  append({entryX, bypassY});
  append({bypassX, bypassY});
  append({bypassX, diagonalEntry[1]});
  append(diagonalEntry);
  append(diagonalExit);
  append({diagonalExit[0], last[1]});
  append(last);
  path.points = std::move(repaired);
  return true;
}

bool repairOneAdjacentFanoutMrrSharedTrunk(PostRoutePath& firstPath,
                                           PostRoutePath& secondPath,
                                           const LidarRouteConfig& config)
{
  constexpr double eps = 1e-6;
  const double minTrunkLength = 12.0 * config.gridResolution;
  const double minLaneSeparation = 3.0 * config.gridResolution;
  if (!isFanoutToMrrPath(firstPath) || !isFanoutToMrrPath(secondPath)) {
    return false;
  }
  if (firstPath.startPort.instanceName != secondPath.startPort.instanceName) {
    return false;
  }
  if (std::abs(firstPath.startPort.center[0] - secondPath.startPort.center[0])
          > eps
      || std::abs(firstPath.startPort.center[1] - secondPath.startPort.center[1])
             > config.gridResolution) {
    return false;
  }

  const double firstDy = firstPath.endPort.center[1] - firstPath.startPort.center[1];
  const double secondDy =
      secondPath.endPort.center[1] - secondPath.startPort.center[1];
  if (firstDy * secondDy <= 0.0) {
    return false;
  }

  const auto firstSegment =
      firstLongVerticalSegmentIndex(firstPath, minTrunkLength);
  const auto secondSegment =
      firstLongVerticalSegmentIndex(secondPath, minTrunkLength);
  if (!firstSegment.has_value() || !secondSegment.has_value()) {
    return false;
  }

  const double firstTrunkX = firstPath.points[*firstSegment][0];
  const double secondTrunkX = secondPath.points[*secondSegment][0];
  if (std::abs(firstTrunkX - secondTrunkX) > eps) {
    return false;
  }

  const auto firstA = firstPath.points[*firstSegment - 1];
  const auto firstB = firstPath.points[*firstSegment];
  const auto secondA = secondPath.points[*secondSegment - 1];
  const auto secondB = secondPath.points[*secondSegment];
  if (overlapLength(firstA[1], firstB[1], secondA[1], secondB[1])
      < minTrunkLength) {
    return false;
  }

  PostRoutePath* pathToShift = nullptr;
  const PostRoutePath* adjacentPath = nullptr;
  std::size_t segmentToShift = 0;
  if (firstDy < 0.0) {
    if (firstPath.endPort.center[1] <= secondPath.endPort.center[1]) {
      pathToShift = &firstPath;
      adjacentPath = &secondPath;
      segmentToShift = *firstSegment;
    } else {
      pathToShift = &secondPath;
      adjacentPath = &firstPath;
      segmentToShift = *secondSegment;
    }
  } else {
    if (firstPath.endPort.center[1] >= secondPath.endPort.center[1]) {
      pathToShift = &firstPath;
      adjacentPath = &secondPath;
      segmentToShift = *firstSegment;
    } else {
      pathToShift = &secondPath;
      adjacentPath = &firstPath;
      segmentToShift = *secondSegment;
    }
  }

  if (firstDy < 0.0 && adjacentPath != nullptr
      && rewriteDownwardFanoutMrrBypass(
          *pathToShift, *adjacentPath, firstTrunkX, config)) {
    return true;
  }

  auto laneX = firstExistingLaneAfterSegment(
      *pathToShift, segmentToShift, firstTrunkX, minLaneSeparation);
  if (!laneX.has_value()) {
    const double fallbackOffset =
        firstDy < 0.0 ? 6.0 * config.gridResolution
                      : 15.0 * config.gridResolution;
    laneX = snapGdsFactoryGrid(firstTrunkX + fallbackOffset);
  }
  return shiftFanoutMrrSharedTrunk(*pathToShift, segmentToShift, *laneX);
}

void repairAdjacentFanoutMrrSharedTrunks(PythonPostProcessRoutes& routes,
                                         const LidarRouteConfig& config)
{
  std::vector<std::string> netNames;
  netNames.reserve(routes.paths.size());
  for (const auto& entry : routes.paths) {
    netNames.push_back(entry.first);
  }
  std::sort(netNames.begin(), netNames.end());

  for (std::size_t i = 0; i < netNames.size(); ++i) {
    auto firstPathsIt = routes.paths.find(netNames[i]);
    if (firstPathsIt == routes.paths.end() || firstPathsIt->second.size() != 1) {
      continue;
    }
    const auto firstCrossingIt = routes.crossingSets.find(netNames[i]);
    if (firstCrossingIt != routes.crossingSets.end()
        && !firstCrossingIt->second.empty()) {
      continue;
    }
    for (std::size_t j = i + 1; j < netNames.size(); ++j) {
      auto secondPathsIt = routes.paths.find(netNames[j]);
      if (secondPathsIt == routes.paths.end()
          || secondPathsIt->second.size() != 1) {
        continue;
      }
      const auto secondCrossingIt = routes.crossingSets.find(netNames[j]);
      if (secondCrossingIt != routes.crossingSets.end()
          && !secondCrossingIt->second.empty()) {
        continue;
      }
      repairOneAdjacentFanoutMrrSharedTrunk(firstPathsIt->second.front(),
                                            secondPathsIt->second.front(),
                                            config);
    }
  }
}

void repairFanoutMrrOrderInversions(PythonPostProcessRoutes& routes,
                                    const LidarRouteConfig& config)
{
  std::vector<std::string> netNames;
  netNames.reserve(routes.paths.size());
  for (const auto& entry : routes.paths) {
    netNames.push_back(entry.first);
  }
  std::sort(netNames.begin(), netNames.end());

  for (const auto& netName : netNames) {
    const auto crossingIt = routes.crossingSets.find(netName);
    if (crossingIt != routes.crossingSets.end() && !crossingIt->second.empty()) {
      continue;
    }
    auto entry = routes.paths.find(netName);
    if (entry == routes.paths.end()) {
      continue;
    }
    for (auto& path : entry->second) {
      if (!pathHasPhysicalIntersection(path, routes, netName)) {
        continue;
      }
      repairOneFanoutMrrOrderInversion(path, config);
    }
  }
}

bool removeFirstPolylineLoop(PostRoutePath& path)
{
  if (path.points.size() < 4) {
    return false;
  }
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    for (std::size_t j = i + 2; j < path.points.size(); ++j) {
      const auto intersection = segmentIntersection(path.points[i - 1],
                                                    path.points[i],
                                                    path.points[j - 1],
                                                    path.points[j]);
      if (!intersection.has_value()) {
        continue;
      }

      std::vector<std::array<double, 2>> repaired;
      repaired.reserve(path.points.size() - (j - i) + 2);
      auto append = [&](const std::array<double, 2>& point) {
        if (repaired.empty() || !samePoint(repaired.back(), point)) {
          repaired.push_back(point);
        }
      };
      for (std::size_t k = 0; k < i; ++k) {
        append(path.points[k]);
      }
      append(intersection.value());
      for (std::size_t k = j; k < path.points.size(); ++k) {
        append(path.points[k]);
      }
      path.points = std::move(repaired);
      return true;
    }
  }
  return false;
}

bool removeFirstCollinearBacktrack(PostRoutePath& path)
{
  constexpr double eps = 1e-9;
  if (path.points.size() < 3) {
    return false;
  }
  for (std::size_t i = 1; i + 1 < path.points.size(); ++i) {
    const auto& prev = path.points[i - 1];
    const auto& point = path.points[i];
    const auto& next = path.points[i + 1];
    const double ax = point[0] - prev[0];
    const double ay = point[1] - prev[1];
    const double bx = next[0] - point[0];
    const double by = next[1] - point[1];
    if (std::hypot(ax, ay) <= eps || std::hypot(bx, by) <= eps) {
      path.points.erase(path.points.begin() + static_cast<std::ptrdiff_t>(i));
      return true;
    }
    if (std::abs(cross2d(ax, ay, bx, by)) > eps) {
      continue;
    }
    if (ax * bx + ay * by >= -eps) {
      continue;
    }
    path.points.erase(path.points.begin() + static_cast<std::ptrdiff_t>(i));
    return true;
  }
  return false;
}

bool removeAccessValidatedCollinearBacktrack(PostRoutePath& path)
{
  constexpr double eps = 1e-9;
  const auto physicalPoints = postRoutePointsWithAccess(path);
  if (physicalPoints.size() < 3 || path.points.empty()) {
    return false;
  }

  for (std::size_t i = 1; i + 1 < physicalPoints.size(); ++i) {
    if (i > path.points.size()) {
      continue;
    }
    const std::array<double, 2> prev = {physicalPoints[i - 1][0],
                                        physicalPoints[i - 1][1]};
    const std::array<double, 2> point = {physicalPoints[i][0],
                                         physicalPoints[i][1]};
    const std::array<double, 2> next = {physicalPoints[i + 1][0],
                                        physicalPoints[i + 1][1]};
    const double ax = point[0] - prev[0];
    const double ay = point[1] - prev[1];
    const double bx = next[0] - point[0];
    const double by = next[1] - point[1];
    if (std::hypot(ax, ay) <= eps || std::hypot(bx, by) <= eps
        || std::abs(cross2d(ax, ay, bx, by)) > eps || ax * bx + ay * by >= -eps) {
      continue;
    }

    PostRoutePath candidate = path;
    candidate.points.erase(candidate.points.begin()
                           + static_cast<std::ptrdiff_t>(i - 1));
    if (!validatePostRoutePathAccess(std::string{}, 0, candidate).empty()) {
      continue;
    }
    path = std::move(candidate);
    return true;
  }

  return false;
}

bool isCrossingPort(const PathPortDump& port)
{
  return port.instanceName.rfind("lidar_crossing_", 0) == 0;
}

bool removeTinyCrossingEndBacktrack(PostRoutePath& path)
{
  constexpr double eps = 1e-9;
  constexpr double maxEndpointDistance = 0.5;
  if (!isCrossingPort(path.endPort) || path.points.size() < 2) {
    return false;
  }

  const auto& prev = path.points[path.points.size() - 2];
  const auto& point = path.points.back();
  const auto& end = path.endPort.center;
  if (pointDistance(point, end) > maxEndpointDistance) {
    return false;
  }

  const double ax = point[0] - prev[0];
  const double ay = point[1] - prev[1];
  const double bx = end[0] - point[0];
  const double by = end[1] - point[1];
  if (std::hypot(ax, ay) <= eps || std::hypot(bx, by) <= eps) {
    return false;
  }
  if (std::abs(cross2d(ax, ay, bx, by)) > eps || ax * bx + ay * by >= -eps) {
    return false;
  }

  const auto repairedEndTangent = tangentOrientation(end, prev);
  if (!repairedEndTangent.has_value()
      || angleDelta(repairedEndTangent.value(), path.endPort.orientation) > 1.0) {
    return false;
  }

  path.points.pop_back();
  return true;
}

bool repairShortCrossingConnector(PostRoutePath& path)
{
  constexpr double eps = 1e-9;
  constexpr double maxDirectDistance = 4.5;
  if (!isCrossingPort(path.startPort) || !isCrossingPort(path.endPort)) {
    return false;
  }
  const double distance = pointDistance(path.startPort.center, path.endPort.center);
  if (distance <= eps || distance > maxDirectDistance) {
    return false;
  }
  const auto startTangent =
      tangentOrientation(path.startPort.center, path.endPort.center);
  const auto endTangent =
      tangentOrientation(path.endPort.center, path.startPort.center);
  if (!startTangent.has_value() || !endTangent.has_value()
      || angleDelta(startTangent.value(), path.startPort.orientation) > 1.0
      || angleDelta(endTangent.value(), path.endPort.orientation) > 1.0) {
    return false;
  }

  const double margin = std::min(1.0, distance / 3.0);
  const double startRadians =
      path.startPort.orientation * std::acos(-1.0) / 180.0;
  const double endRadians =
      path.endPort.orientation * std::acos(-1.0) / 180.0;
  path.points = {
      {path.startPort.center[0] + std::cos(startRadians) * margin,
       path.startPort.center[1] + std::sin(startRadians) * margin},
      {path.endPort.center[0] + std::cos(endRadians) * margin,
       path.endPort.center[1] + std::sin(endRadians) * margin},
  };
  return true;
}

void cleanupPostSplitPathGeometry(PythonPostProcessRoutes& routes)
{
  constexpr int maxLoopRepairs = 8;
  for (auto& entry : routes.paths) {
    for (auto& path : entry.second) {
      for (int i = 0; i < maxLoopRepairs; ++i) {
        if (!removeFirstCollinearBacktrack(path)
            && !removeAccessValidatedCollinearBacktrack(path)
            && !removeTinyCrossingEndBacktrack(path)
            && !removeFirstPolylineLoop(path)) {
          break;
        }
      }
      repairShortCrossingConnector(path);
    }
  }
}

PythonPostProcessRoutes buildPythonPostProcessRoutes(
    const LidarRuntimeView& db,
    const LidarRouteConfig& config)
{
  PythonPostProcessRoutes result;
  std::unordered_map<std::string, std::string> designNetByRuntimeName;
  for (const auto& net : db.nets) {
    designNetByRuntimeName[net.netName] =
        net.designNetName.empty() ? net.netName : net.designNetName;
  }
  for (const auto& net : db.nets) {
    if (!net.routed || net.routedPath.empty()) {
      continue;
    }
    const auto& sourcePort = db.ports[net.sourcePortIndex];
    const auto& targetPort = db.ports[net.targetPortIndex];
    const auto& routeStartPort =
        (config.group && net.reverse) ? targetPort : sourcePort;
    const auto& routeEndPort =
        (config.group && net.reverse) ? sourcePort : targetPort;
    const auto simplifiedPath = simplifyCollinearPath(net.routedPath);
    auto alignedPath = toMicronPath(simplifiedPath, config.gridResolution);
    applyPythonPortAlignment(alignedPath, routeStartPort, routeEndPort);

    PostRoutePath path;
    path.points = std::move(alignedPath);
    path.startPort = pathPortFromLidarPort(routeStartPort);
    path.endPort = pathPortFromLidarPort(routeEndPort);
    result.paths[net.netName].push_back(std::move(path));
    result.crossingSets[net.netName] = net.crossingNets;
  }

  repairFanoutAccessOverlaps(result, config);
  repairAdjacentFanoutMrrSharedTrunks(result, config);
  repairFanoutMrrOrderInversions(result, config);
  repairFanoutMrrSelfIntersections(result, config);

  auto applyPostRouteSplit =
      [&](const std::string& netName,
          const std::string& crossingNetName,
          const SplitPathResult& split,
          std::vector<PostRoutePath>& currentPaths,
          std::vector<PostRoutePath>& otherPaths) {
        PythonPostProcessRoutes::Crossing crossing;
        crossing.name = "lidar_crossing_" + std::to_string(result.crossings.size());
        const auto ports =
            crossingPorts(crossing.name, split.crossingPoint, split.crossingOrientation);
        crossing.firstNet = designNetByRuntimeName[netName];
        auto crossingDesignNetIt = designNetByRuntimeName.find(crossingNetName);
        crossing.secondNet =
            crossingDesignNetIt == designNetByRuntimeName.end()
                ? crossingNetName
                : crossingDesignNetIt->second;
        crossing.center = split.crossingPoint;
        crossing.orientation = split.crossingOrientation;
        crossing.firstNetPorts = split.split1.ports;
        crossing.secondNetPorts = split.split2.ports;
        result.crossings.push_back(std::move(crossing));

        auto& currentPath = currentPaths[split.path1Index];
        auto& otherPath = otherPaths[split.path2Index];
        const auto originalCurrentPath = currentPath;
        const auto originalOtherPath = otherPath;

        currentPath.endPort = ports.at(split.split1.ports[0]);
        currentPath.points = trimPostRoutePointsForPorts(split.split1.parts[0],
                                                         originalCurrentPath.startPort,
                                                         currentPath.endPort);
        PostRoutePath currentTail;
        currentTail.startPort = ports.at(split.split1.ports[1]);
        currentTail.endPort = originalCurrentPath.endPort;
        currentTail.points = trimPostRoutePointsForPorts(split.split1.parts[1],
                                                         currentTail.startPort,
                                                         currentTail.endPort);
        currentPaths.push_back(std::move(currentTail));

        otherPath.endPort = ports.at(split.split2.ports[0]);
        otherPath.points = trimPostRoutePointsForPorts(split.split2.parts[0],
                                                       originalOtherPath.startPort,
                                                       otherPath.endPort);
        PostRoutePath otherTail;
        otherTail.startPort = ports.at(split.split2.ports[1]);
        otherTail.endPort = originalOtherPath.endPort;
        otherTail.points = trimPostRoutePointsForPorts(split.split2.parts[1],
                                                       otherTail.startPort,
                                                       otherTail.endPort);
        otherPaths.push_back(std::move(otherTail));
      };

  for (const auto& net : db.nets) {
    auto currentPathsIt = result.paths.find(net.netName);
    if (currentPathsIt == result.paths.end()) {
      continue;
    }
    const char* tracePostNet = std::getenv("PICDB_LIDAR_TRACE_POST_NET");
    const bool traceThisPostNet =
        tracePostNet != nullptr && net.netName == tracePostNet;
    auto crossingIt = result.crossingSets.find(net.netName);
    if (crossingIt == result.crossingSets.end()) {
      continue;
    }
    const std::vector<std::string> crossingNames =
        crossingIt->second.pythonIterationOrder();
    if (traceThisPostNet) {
      std::cerr << "TRACE_POST_ORDER"
                << "\tnet=" << net.netName
                << "\torder=";
      for (std::size_t i = 0; i < crossingNames.size(); ++i) {
        if (i != 0) {
          std::cerr << ",";
        }
        std::cerr << crossingNames[i];
      }
      std::cerr << "\n";
    }
    for (const auto& crossingNetName : crossingNames) {
      auto otherPathsIt = result.paths.find(crossingNetName);
      if (otherPathsIt == result.paths.end()) {
        continue;
      }
      auto otherCrossingIt = result.crossingSets.find(crossingNetName);
      if (otherCrossingIt == result.crossingSets.end()
          || !otherCrossingIt->second.count(net.netName)) {
        continue;
      }

      otherCrossingIt->second.erase(net.netName);
      const auto split =
          splitPostRoutePaths(currentPathsIt->second, otherPathsIt->second);
      if (!split.has_value()) {
        continue;
      }
      if (traceThisPostNet) {
        const auto oldFlags = std::cerr.flags();
        const auto oldPrecision = std::cerr.precision();
        std::cerr << std::defaultfloat << std::setprecision(17)
                  << "TRACE_POST_SPLIT"
                  << "\tnet=" << net.netName
                  << "\tcrossing_net=" << crossingNetName
                  << "\tpath1_index=" << split->path1Index
                  << "\tpath2_index=" << split->path2Index
                  << "\tcrossing=" << split->crossingPoint[0] << ","
                  << split->crossingPoint[1]
                  << "\torientation=" << split->crossingOrientation
                  << "\tports1=" << split->split1.ports[0] << ","
                  << split->split1.ports[1]
                  << "\tports2=" << split->split2.ports[0] << ","
                  << split->split2.ports[1]
                  << "\n";
        std::cerr.flags(oldFlags);
        std::cerr.precision(oldPrecision);
      }
      applyPostRouteSplit(net.netName,
                          crossingNetName,
                          split.value(),
                          currentPathsIt->second,
                           otherPathsIt->second);
    }
  }

  auto crossingNearExisting = [&](const std::array<double, 2>& point) {
    constexpr double minCrossingSeparation = 4.0;
    for (const auto& crossing : result.crossings) {
      if (pointDistance(crossing.center, point) < minCrossingSeparation) {
        return true;
      }
    }
    return false;
  };

  auto insertSupplementalCrossings = [&](int maxSupplementalCrossings) {
    int supplementalCrossings = 0;
    for (std::size_t i = 0; i < db.nets.size()
                            && supplementalCrossings < maxSupplementalCrossings;
         ++i) {
      const auto& firstNet = db.nets[i];
      auto firstPathsIt = result.paths.find(firstNet.netName);
      if (firstPathsIt == result.paths.end()) {
        continue;
      }
      for (std::size_t j = i + 1; j < db.nets.size()
                              && supplementalCrossings < maxSupplementalCrossings;
           ++j) {
        const auto& secondNet = db.nets[j];
        auto secondPathsIt = result.paths.find(secondNet.netName);
        if (secondPathsIt == result.paths.end()) {
          continue;
        }
        const auto split =
            splitPostRoutePaths(firstPathsIt->second, secondPathsIt->second);
        if (!split.has_value() || crossingNearExisting(split->crossingPoint)) {
          continue;
        }
        applyPostRouteSplit(firstNet.netName,
                            secondNet.netName,
                            split.value(),
                            firstPathsIt->second,
                            secondPathsIt->second);
        result.crossingSets[firstNet.netName].insert(secondNet.netName);
        result.crossingSets[secondNet.netName].insert(firstNet.netName);
        ++supplementalCrossings;
      }
    }
    return supplementalCrossings;
  };

  insertSupplementalCrossings(32);
  cleanupPostSplitPathGeometry(result);
  insertSupplementalCrossings(32);
  cleanupPostSplitPathGeometry(result);
  repairPostSplitHorizontalConflictDetours(result, config);
  cleanupPostSplitPathGeometry(result);

  return result;
}

void clearRoutedNet(LidarRuntimeView& db, LidarNet& net, bool increase = true)
{
  net.routed = false;
  for (const auto& crossingNetName : net.crossingNets.pythonIterationOrder()) {
    auto crossingIt = db.netIndex.find(crossingNetName);
    if (crossingIt != db.netIndex.end()) {
      db.nets[crossingIt->second].crossingNets.erase(net.netName);
    }
  }
  net.rwguide.clear();
  net.routedPath.clear();
  net.originPath.clear();
  net.rectRoute.clear();
  net.shortSbend = false;
  net.shortSbendLength = 0.0;
  net.crossingNets.clear();
  net.insertionLoss = 0.0;
  net.wirelength = 0;
  net.bending = 0.0;
  net.crossingNum = 0;
  net.vionets = 0;
  net.vioNets.clear();
  if (increase) {
    ++net.failedCount;
  }
  if (net.failedCount > 4) {
    net.enable45 = false;
  }
}

std::optional<int> fallbackSegmentOrientation(int dx, int dy)
{
  if (dx == 0 && dy > 0) {
    return 90;
  }
  if (dx == 0 && dy < 0) {
    return 270;
  }
  if (dy == 0 && dx > 0) {
    return 0;
  }
  if (dy == 0 && dx < 0) {
    return 180;
  }
  if (std::abs(dx) == std::abs(dy)) {
    if (dx > 0 && dy > 0) {
      return 45;
    }
    if (dx < 0 && dy > 0) {
      return 135;
    }
    if (dx < 0 && dy < 0) {
      return 225;
    }
    if (dx > 0 && dy < 0) {
      return 315;
    }
  }
  return std::nullopt;
}

std::vector<std::array<int, 3>> expandFallbackWaypoints(
    const std::vector<std::array<int, 2>>& waypoints)
{
  std::vector<std::array<int, 3>> path;
  if (waypoints.size() < 2) {
    return path;
  }

  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    const auto& from = waypoints[i - 1];
    const auto& to = waypoints[i];
    const int dx = to[0] - from[0];
    const int dy = to[1] - from[1];
    const auto orientation = fallbackSegmentOrientation(dx, dy);
    if (!orientation.has_value()) {
      return {};
    }
    const int steps = std::max(std::abs(dx), std::abs(dy));
    if (steps <= 0) {
      continue;
    }
    const int stepX = dx == 0 ? 0 : dx / std::abs(dx);
    const int stepY = dy == 0 ? 0 : dy / std::abs(dy);
    if (path.empty()) {
      path.push_back({from[0], from[1], orientation.value()});
    }
    for (int step = 1; step <= steps; ++step) {
      path.push_back({from[0] + stepX * step,
                      from[1] + stepY * step,
                      orientation.value()});
    }
  }
  return path;
}

LidarPythonStringSet collectFallbackCrossingNets(
    const LidarRuntimeView& db,
    const LidarNet& net,
    const LidarRouteConfig& config,
    const LidarPort& routeStartPort,
    const LidarPort& routeEndPort,
    const std::vector<std::array<int, 3>>& path)
{
  LidarPythonStringSet crossingNets;
  auto pathPoints = toMicronPath(simplifyCollinearPath(path), config.gridResolution);
  applyPythonPortAlignment(pathPoints, routeStartPort, routeEndPort);
  const double pathLength = polylineLength(pathPoints);
  if (pathPoints.size() < 2 || pathLength <= 1e-9) {
    return crossingNets;
  }

  constexpr double eps = 1e-6;
  for (const auto& otherNet : db.nets) {
    if (otherNet.netName == net.netName || !otherNet.routed
        || otherNet.routedPath.empty()) {
      continue;
    }
    const auto& otherSourcePort = db.ports[otherNet.sourcePortIndex];
    const auto& otherTargetPort = db.ports[otherNet.targetPortIndex];
    const auto& otherRouteStartPort =
        (config.group && otherNet.reverse) ? otherTargetPort : otherSourcePort;
    const auto& otherRouteEndPort =
        (config.group && otherNet.reverse) ? otherSourcePort : otherTargetPort;
    auto otherPoints =
        toMicronPath(simplifyCollinearPath(otherNet.routedPath),
                     config.gridResolution);
    applyPythonPortAlignment(
        otherPoints, otherRouteStartPort, otherRouteEndPort);
    const double otherLength = polylineLength(otherPoints);
    if (otherPoints.size() < 2 || otherLength <= 1e-9) {
      continue;
    }

    bool intersects = false;
    for (std::size_t i = 1; i < pathPoints.size() && !intersects; ++i) {
      for (std::size_t j = 1; j < otherPoints.size(); ++j) {
        const auto intersection =
            segmentIntersection(pathPoints[i - 1],
                                pathPoints[i],
                                otherPoints[j - 1],
                                otherPoints[j]);
        if (!intersection.has_value()) {
          continue;
        }
        const double pathProjection =
            projectOnPolyline(pathPoints, intersection.value());
        const double otherProjection =
            projectOnPolyline(otherPoints, intersection.value());
        if (pathProjection <= eps || pathProjection >= pathLength - eps
            || otherProjection <= eps || otherProjection >= otherLength - eps) {
          continue;
        }
        crossingNets.insert(otherNet.netName);
        intersects = true;
        break;
      }
    }
  }
  return crossingNets;
}

std::optional<LidarAstarRouteResult> tryFanoutTreeFallbackRoute(
    LidarRuntimeView& db,
    LidarDrcManager& drc,
    LidarNet& net,
    const LidarRouteConfig& config)
{
  if (net.routed || std::abs(config.gridResolution - 2.0) > 1e-9) {
    return std::nullopt;
  }

  const auto& sourcePort = db.ports[net.sourcePortIndex];
  const auto& targetPort = db.ports[net.targetPortIndex];
  const auto& routeStartPort =
      (config.group && net.reverse) ? targetPort : sourcePort;
  const auto& routeEndPort =
      (config.group && net.reverse) ? sourcePort : targetPort;
  if (routeStartPort.instanceName.find("fanout") == std::string::npos
      || routeEndPort.instanceName.find("fanout") == std::string::npos) {
    return std::nullopt;
  }

  const auto state = buildAstarInitState(db, drc, net, config, 2);
  if (state.startNode[2] != 0 || state.endNode[2] != 180
      || !oppositeOrientations(routeStartPort.orientation,
                               routeEndPort.orientation)) {
    return std::nullopt;
  }

  const double dx = std::abs(routeEndPort.center.x - routeStartPort.center.x);
  const double dy = routeEndPort.center.y - routeStartPort.center.y;
  if (dx > 8.0 || std::abs(std::abs(dy) - 100.625) > 8.0) {
    return std::nullopt;
  }

  const int sx = state.startNode[0];
  const int sy = state.startNode[1];
  const int ex = state.endNode[0];
  const int ey = state.endNode[1];
  if (ex >= sx) {
    return std::nullopt;
  }

  constexpr int sideOffset = 3;
  constexpr int tailOffset = 5;
  const int startX = sx + sideOffset;
  const int endX = ex - sideOffset;
  const int diagonalSpan = std::abs(startX - endX);
  const int crossingX = (startX + endX) / 2;
  std::vector<std::array<int, 2>> waypoints;
  if (dy < 0.0) {
    waypoints = {{sx, sy},
                 {startX, sy},
                 {startX, ey + tailOffset + diagonalSpan},
                 {crossingX, ey + tailOffset + diagonalSpan},
                 {crossingX, ey + tailOffset},
                 {endX, ey + tailOffset},
                 {endX, ey},
                 {ex, ey}};
  } else {
    waypoints = {{sx, sy},
                 {startX, sy},
                 {startX, ey - tailOffset - diagonalSpan},
                 {crossingX, ey - tailOffset - diagonalSpan},
                 {crossingX, ey - tailOffset},
                 {endX, ey - tailOffset},
                 {endX, ey},
                 {ex, ey}};
  }

  auto path = expandFallbackWaypoints(waypoints);
  if (path.size() < 2) {
    return std::nullopt;
  }
  const auto simplifiedPath = simplifyCollinearPath(path);
  auto alignedPath = toMicronPath(simplifiedPath, config.gridResolution);
  applyPythonPortAlignment(alignedPath, routeStartPort, routeEndPort);
  const double length = polylineLength(alignedPath);
  const auto crossingNets =
      collectFallbackCrossingNets(
          db, net, config, routeStartPort, routeEndPort, path);

  LidarAstarRouteResult result;
  result.netName = net.netName;
  result.success = true;
  result.strictDrc = false;
  result.originPath = path;
  result.processedPath = path;
  result.crossingNets = crossingNets;
  result.finalCostG = length * config.lossPropagation;
  result.finalCostF = result.finalCostG;

  net.routed = true;
  net.shortSbend = false;
  net.shortSbendLength = 0.0;
  net.originPath = result.originPath;
  net.routedPath = result.processedPath;
  net.crossingNets = result.crossingNets;
  net.crossingNum = static_cast<int>(result.crossingNets.size());
  net.vioNets.clear();
  net.vionets = 0;
  net.wirelength = length;
  net.rwguide.clear();
  return result;
}

}  // namespace

LidarAstarRouteResult routeSingleNetGrid(LidarRuntimeView& db,
                                         LidarDrcManager& drc,
                                         LidarNet& net,
                                         const LidarRouteConfig& config,
                                         bool strictDrc,
                                         int drUnrouted,
                                         const std::set<std::string>& groups,
                                         const std::vector<int>* historyMap,
                                         int historyWidth,
                                         int historyHeight,
                                         const LidarMfOtPlan* mfotPlan)
{
  LidarAstarRouteResult result;
  result.netName = net.netName;
  result.strictDrc = strictDrc;
  net.currentBudget = net.crossingBudget;
  const char* traceNet = std::getenv("PICDB_LIDAR_TRACE_NET");
  static std::map<std::string, int> traceRouteAttemptCounts;
  int traceAttempt = 0;
  if (traceNet != nullptr && net.netName == traceNet) {
    traceAttempt = ++traceRouteAttemptCounts[net.netName];
    std::cerr << "TRACE_ROUTE_BEGIN"
              << "\tnet=" << net.netName
              << "\tattempt=" << traceAttempt
              << "\tstrict=" << (strictDrc ? 1 : 0)
              << "\tfailed_count=" << net.failedCount
              << "\n";
  }
  const int traceAttemptFilter =
      traceNet != nullptr ? parseEnvInt(std::getenv("PICDB_LIDAR_TRACE_ATTEMPT"), 0) : 0;
  const bool traceThisNet = traceNet != nullptr && net.netName == traceNet
                            && (traceAttemptFilter <= 0
                                || traceAttemptFilter == traceAttempt);
  const int traceHeapAt = traceThisNet
                              ? parseEnvInt(std::getenv("PICDB_LIDAR_TRACE_HEAP_AT"), -1)
                              : -1;
  const int traceHeapLimit = traceThisNet
                                 ? parseEnvInt(
                                       std::getenv("PICDB_LIDAR_TRACE_HEAP_LIMIT"), 0)
                                 : 0;
  const int traceSetStart = traceThisNet
                                ? parseEnvInt(std::getenv("PICDB_LIDAR_TRACE_SET_START"), 0)
                                : 0;
  const int traceSetEnd = traceThisNet
                              ? parseEnvInt(std::getenv("PICDB_LIDAR_TRACE_SET_END"), 0)
                              : 0;
  const int tracePopStart =
      traceThisNet ? parseEnvInt(std::getenv("PICDB_LIDAR_TRACE_POP_START"), 0) : 0;
  const int tracePopEnd =
      traceThisNet ? parseEnvInt(std::getenv("PICDB_LIDAR_TRACE_POP_END"), 0) : 0;
  const auto tracePositions =
      traceThisNet ? parseTracePopPositions(std::getenv("PICDB_LIDAR_TRACE_POS"))
                   : std::set<std::array<int, 3>>{};
  const auto tracePopPositions =
      traceThisNet ? parseTracePopPositions(std::getenv("PICDB_LIDAR_TRACE_POP_POS"))
                   : std::set<std::array<int, 3>>{};
  const char* tracePopFile =
      traceThisNet ? std::getenv("PICDB_LIDAR_TRACE_POP_FILE") : nullptr;
  auto writeTracePopFile = [&](bool success) {
    if (!traceThisNet || tracePopFile == nullptr || std::string(tracePopFile).empty()) {
      return;
    }
    std::ofstream out(tracePopFile, std::ios::app);
    if (!out) {
      return;
    }
    const auto oldFlags = out.flags();
    const auto oldPrecision = out.precision();
    out << std::defaultfloat << std::setprecision(17);
    out << "ROUTE"
        << "\tnet=" << net.netName
        << "\tattempt=" << traceAttempt
        << "\tstrict=" << (strictDrc ? 1 : 0)
        << "\tsuccess=" << (success ? 1 : 0)
        << "\tpops=" << result.popTrace.size()
        << "\n";
    for (std::size_t i = 0; i < result.popTrace.size(); ++i) {
      const auto& pos = result.popTrace[i];
      const auto& cost = result.popTraceCost[i];
      const auto& parent = result.popTraceParent[i];
      out << "POP"
          << "\tpop=" << (i + 1)
          << "\tpos=" << pos[0] << "," << pos[1] << "," << pos[2]
          << "\tcost_g=" << cost[0]
          << "\tcost_f=" << cost[1];
      if (parent[0] >= 0) {
        out << "\tparent=" << parent[0] << "," << parent[1] << "," << parent[2];
      }
      out << "\n";
    }
    out.flags(oldFlags);
    out.precision(oldPrecision);
  };

  const auto& routeStartPort =
      db.ports[(config.group && net.reverse) ? net.targetPortIndex
                                             : net.sourcePortIndex];
  const auto& routeEndPort =
      db.ports[(config.group && net.reverse) ? net.sourcePortIndex
                                              : net.targetPortIndex];
  if (routeStartPort.portGrids.empty() || routeEndPort.portGrids.empty()) {
    result.violatedNets.insert("__empty_port_grid__");
    if (routeStartPort.portGrids.empty()) {
      result.violatedNets.insert(routeStartPort.portName);
    }
    if (routeEndPort.portGrids.empty()) {
      result.violatedNets.insert(routeEndPort.portName);
    }
    if (traceNet != nullptr && net.netName == traceNet) {
      std::cerr << "TRACE_ROUTE_END"
                << "\tnet=" << net.netName
                << "\tattempt=" << traceAttempt
                << "\tstrict=" << (strictDrc ? 1 : 0)
                << "\tsuccess=0"
                << "\tempty_port_grid=1"
                << "\n";
    }
    writeTracePopFile(false);
    return result;
  }

  const auto state = buildAstarInitState(db, drc, net, config, drUnrouted);
  drc.setRoutingBendParameters(
      state.gridRadius, state.bend45Part1, state.bend45Part2, state.predictLength);
  const bool mfotCellCostsActive =
    mfotPlan != nullptr && mfotPlan->enabled
    && (config.mfotHardCorridor || config.mfotOutsidePenalty > 0.0
      || config.mfotPotentialScale > 0.0);
  if (net.eulerDistance < 10.0
      && oppositeOrientations(state.startNode[2], state.endNode[2])) {
    const double length = shortSbendLength(routeStartPort, routeEndPort);
    result.success = true;
    result.shortSbend = true;
    result.shortSbendLength = length;
    result.finalCostG = length * config.lossPropagation;
    result.finalCostF = result.finalCostG;

    net.routed = true;
    net.shortSbend = true;
    net.shortSbendLength = length;
    net.originPath.clear();
    net.routedPath.clear();
    net.crossingNets.clear();
    net.crossingNum = 0;
    net.vioNets.clear();
    net.vionets = 0;
    net.wirelength = length;

    if (traceNet != nullptr && net.netName == traceNet) {
      std::cerr << "TRACE_ROUTE_END"
                << "\tnet=" << net.netName
                << "\tattempt=" << traceAttempt
                << "\tstrict=" << (strictDrc ? 1 : 0)
                << "\tsuccess=1"
                << "\tshort_sbend=1"
                << "\tpops=0"
                << "\n";
    }
    writeTracePopFile(true);
    return result;
  }

  std::set<std::array<int, 2>> accessPoints;
  const auto& endPort = db.ports[net.reverse && config.group ? net.sourcePortIndex
                                                              : net.targetPortIndex];
  for (const auto& grid : endPort.portGrids) {
    accessPoints.insert(grid);
  }

  std::vector<GridSearchNode> nodes;
  std::unordered_map<std::string, std::size_t> nodeIndex;
  auto getOrCreate = [&](const std::array<int, 3>& pos,
                         int crossingBudget,
                         const std::string& crossingNet,
                         bool violated,
                         const std::set<std::string>& violatedNets) {
    const auto key = posKey(pos);
    auto it = nodeIndex.find(key);
    if (it != nodeIndex.end()) {
      return it->second;
    }
    GridSearchNode node;
    node.pos = pos;
    node.crossingBudget = crossingBudget;
    node.crossingNet = crossingNet;
    node.violated = violated;
    node.violatedNets = violatedNets;
    nodes.push_back(std::move(node));
    const auto index = nodes.size() - 1;
    nodeIndex.emplace(key, index);
    return index;
  };

  HeapDict nodepq(nodes);
  auto traceSetOperation = [&](std::size_t index, bool existed) {
    if (!traceThisNet || traceSetStart <= 0) {
      return;
    }
    const auto popIndex = static_cast<int>(result.popTrace.size());
    if (popIndex < traceSetStart || (traceSetEnd > 0 && popIndex > traceSetEnd)) {
      return;
    }
    const auto& node = nodes[index];
    const auto oldFlags = std::cerr.flags();
    const auto oldPrecision = std::cerr.precision();
    std::cerr << std::defaultfloat << std::setprecision(17)
              << "TRACE_SET"
              << "\tnet=" << net.netName
              << "\tpop_index=" << popIndex
              << "\texisted=" << (existed ? 1 : 0)
              << "\tpos=" << node.pos[0] << "," << node.pos[1] << ","
              << node.pos[2]
              << "\tvalue=" << node.costF
              << "\tcost_g=" << node.costG
              << "\tcost_f=" << node.costF;
    if (node.parent >= 0) {
      const auto& parent = nodes[static_cast<std::size_t>(node.parent)];
      std::cerr << "\tparent=" << parent.pos[0] << "," << parent.pos[1]
                << "," << parent.pos[2];
    }
    std::cerr << "\n";
    std::cerr.flags(oldFlags);
    std::cerr.precision(oldPrecision);
  };
  const auto startIndex = getOrCreate(state.startNode, net.crossingBudget, {}, false, {});
  nodes[startIndex].costG = 0.0;
  nodes[startIndex].costF = state.startCostF;
  nodes[startIndex].straightCount = 0;
  traceSetOperation(startIndex, nodepq.contains(startIndex));
  nodepq.set(startIndex);

  while (!nodepq.empty()) {
    if (traceThisNet && traceHeapAt >= 0
        && result.popTrace.size() == static_cast<std::size_t>(traceHeapAt)) {
      writeHeapSnapshot(
          std::cerr, net.netName.c_str(), nodepq, nodes, result.popTrace.size(), traceHeapLimit);
    }
    const auto currentIndex = nodepq.popitem();
    nodes[currentIndex].visited = true;
    const auto currentNode = nodes[currentIndex];
    if (traceThisNet) {
      result.popTrace.push_back(currentNode.pos);
      result.popTraceCost.push_back({currentNode.costG, currentNode.costF});
      if (currentNode.parent >= 0) {
        const auto& parent = nodes[static_cast<std::size_t>(currentNode.parent)];
        result.popTraceParent.push_back(parent.pos);
      } else {
        result.popTraceParent.push_back({-1, -1, -1});
      }
      const int popIndex = static_cast<int>(result.popTrace.size());
      const bool inPopRange =
          tracePopStart > 0 && popIndex >= tracePopStart
          && (tracePopEnd <= 0 || popIndex <= tracePopEnd);
      if (inPopRange || tracePopPositions.find(currentNode.pos) != tracePopPositions.end()) {
        writeTracePop(
            std::cerr, net.netName, result.popTrace.size(), currentNode, nodes);
      }
    }
    ++result.visitedNodes;

    if (checkConnection(currentNode.pos, state.endNode, accessPoints)) {
      bool duplicatedCrossing = false;
      const auto crossingNets = collectCrossingNets(nodes, currentIndex, duplicatedCrossing);
      if (strictDrc && duplicatedCrossing) {
        nodes[currentIndex].visited = false;
      } else {
        auto originPath = backTrackOriginPath(nodes, currentIndex);
        auto processedPath = processBendPath(originPath, state);
        if (processedPath.size() < 2) {
          nodes[currentIndex].visited = false;
        } else {
          result.success = true;
          result.finalCostG = currentNode.costG;
          result.finalCostF = currentNode.costF;
          result.originPath = std::move(originPath);
          result.processedPath = std::move(processedPath);
          result.crossingNets = crossingNets;
          if (!strictDrc) {
            result.violatedNets = collectViolatedNets(nodes, currentIndex);
          }

          net.routed = true;
          net.originPath = result.originPath;
          net.routedPath = result.processedPath;
          net.crossingNets = result.crossingNets;
          net.crossingNum = static_cast<int>(result.crossingNets.size());
          net.vioNets = result.violatedNets;
          net.vionets = static_cast<int>(result.violatedNets.size());
          if (traceNet != nullptr && net.netName == traceNet) {
            std::cerr << "TRACE_ROUTE_END"
                      << "\tnet=" << net.netName
                      << "\tattempt=" << traceAttempt
                      << "\tstrict=" << (strictDrc ? 1 : 0)
                      << "\tsuccess=1"
                      << "\tpops=" << result.visitedNodes
                      << "\n";
          }
          writeTracePopFile(true);
          return result;
        }
      }
    }

    if (!nodes[currentIndex].neighborsComputed) {
      std::vector<std::pair<std::size_t, std::string>> neighbors;
      const auto stepsIt = state.nextSteps.find(currentNode.pos[2]);
      if (stepsIt == state.nextSteps.end()) {
        nodes[currentIndex].neighborsComputed = true;
      } else {
        ++result.expandedNodes;
        for (const auto& astarStep : stepsIt->second) {
          bool enablePrediction = true;
          const std::array<int, 3> neighborPosition = {
              currentNode.pos[0] + astarStep.dx,
              currentNode.pos[1] + astarStep.dy,
              astarStep.orientation};
          if (manhattanDH(currentNode.pos, state.endNode) <= state.connectThreshold) {
            enablePrediction = false;
          }
          if (neighborPosition[0] <= state.routingBound[0][0]
              || neighborPosition[0] >= state.routingBound[1][0]
              || neighborPosition[1] <= state.routingBound[0][1]
              || neighborPosition[1] >= state.routingBound[1][1]) {
            continue;
          }
          if (mfotCellCostsActive && config.mfotHardCorridor) {
            bool inMfOtCorridor = true;
            static_cast<void>(mfotCellPenalty(*mfotPlan,
                                              config,
                                              net.netName,
                                              neighborPosition[0],
                                              neighborPosition[1],
                                              &inMfOtCorridor));
            const bool nearStart =
                manhattanDH(neighborPosition, state.startNode)
                <= state.gridRadius + 2;
            const bool nearEnd =
                manhattanDH(neighborPosition, state.endNode)
                <= state.connectThreshold + state.gridRadius + 2;
            if (!inMfOtCorridor && !nearStart && !nearEnd) {
              continue;
            }
          }

          const LidarDrcNode drcNode{
              currentNode.pos,
              currentNode.crossingBudget,
              currentNode.straightCount,
              currentNode.violated,
              currentNode.violatedNets};
          const LidarDrcStep drcStep{
              astarStep.dx, astarStep.dy, astarStep.orientation, astarStep.type};
          const auto ans = drc.bViolateDRC(
              drcNode, drcStep, strictDrc, net.netName, enablePrediction);
          if (tracePositions.find(currentNode.pos) != tracePositions.end()) {
            std::cerr << "TRACE_NEIGHBOR"
                      << "\tnet=" << net.netName
                      << "\tcurrent=" << currentNode.pos[0] << ","
                      << currentNode.pos[1] << "," << currentNode.pos[2]
                      << "\tstep=" << astarStep.dx << "," << astarStep.dy << ","
                      << astarStep.orientation << "," << astarStep.type
                      << "\tviolated=" << (ans.violated ? 1 : 0)
                      << "\tresult=" << ans.resultType;
            if (ans.crossingNeighbor.has_value()) {
              const auto& nb = ans.crossingNeighbor.value();
              std::cerr << "\tcrossing_neighbor=" << nb[0] << "," << nb[1]
                        << "," << nb[2];
            }
            if (ans.crossingNet.has_value()) {
              std::cerr << "\tcrossing_net=" << ans.crossingNet.value();
            }
            if (!ans.violatedNets.empty()) {
              std::cerr << "\tviolated_nets=" << joinSet(ans.violatedNets);
            }
            std::cerr << "\n";
          }

          std::size_t neighborIndex = 0;
          std::string neighborType;
          if (!ans.violated) {
            neighborIndex = getOrCreate(
                neighborPosition, currentNode.crossingBudget, {}, false, {});
            neighborType = astarStep.type;
          } else if (ans.crossingNeighbor.has_value()) {
            neighborIndex = getOrCreate(ans.crossingNeighbor.value(),
                                        currentNode.crossingBudget - 1,
                                        ans.crossingNet.value_or(std::string{}),
                                        false,
                                        {});
            neighborType = ans.resultType;
          } else if (!strictDrc && ans.resultType == "port") {
            // Port bitmap cells are artificial access reservations.  Until the
            // Python ripup policy is fully mirrored, dense fanout/MRR benches
            // need the non-strict fallback to pass these cells to avoid dropped
            // nets; the final physical DRC pass catches any resulting overlap.
            neighborIndex = getOrCreate(neighborPosition,
                                        currentNode.crossingBudget,
                                        {},
                                        false,
                                        {});
            neighborType = astarStep.type;
          } else if (!strictDrc && ans.resultType != "blk"
                     && ans.resultType != "port") {
            neighborIndex = getOrCreate(neighborPosition,
                                        currentNode.crossingBudget,
                                        {},
                                        true,
                                        ans.violatedNets);
            neighborType = astarStep.type;
          } else {
            continue;
          }

          auto duplicate = std::find_if(neighbors.begin(),
                                        neighbors.end(),
                                        [&](const auto& item) {
                                          return item.first == neighborIndex;
                                        });
          if (duplicate == neighbors.end()) {
            neighbors.emplace_back(neighborIndex, neighborType);
          } else {
            duplicate->second = neighborType;
          }
        }
        nodes[currentIndex].neighbors = std::move(neighbors);
        nodes[currentIndex].neighborsComputed = true;
      }
    }

    for (const auto& [neighborIndex, nbType] : nodes[currentIndex].neighbors) {
      auto& neighbor = nodes[neighborIndex];
      if (neighbor.visited) {
        continue;
      }
      double costG = 0.0;
      if (nbType == "crossing_0" || nbType == "crossing_45") {
        costG = currentNode.costG + config.lossCrossing;
      } else {
        costG = currentNode.costG + state.stepG.at(nbType);
      }
      if (mfotCellCostsActive) {
        costG += mfotCellPenalty(*mfotPlan,
                                 config,
                                 net.netName,
                                 neighbor.pos[0],
                                 neighbor.pos[1]);
      }

      if (neighbor.violated) {
        auto unionNets = currentNode.violatedNets;
        unionNets.insert(neighbor.violatedNets.begin(), neighbor.violatedNets.end());
        const auto netNum = static_cast<double>(unionNets.size());
        for (const auto& vioNet : neighbor.violatedNets) {
          if (currentNode.violatedNets.find(vioNet) != currentNode.violatedNets.end()) {
            costG += 0.1 * config.lossCrossing;
          } else {
            costG += netNum * 10.0 * config.lossCrossing;
          }
        }
      }

      if (config.group) {
        const double dis = eulerDist(neighbor.pos, state.endNode) * config.gridResolution;
        if (dis >= state.checkRegion) {
          int count = 0;
          if (state.repel
              && (neighbor.pos[2] + static_cast<int>(std::round(endPort.orientation))) % 180 != 0
              && net.failedCount > 1) {
            if (nbType == "straight_0") {
              count = drc.checkSpacing(neighbor.pos, 5, {});
            } else if (nbType == "straight_45") {
              count = drc.checkSpacing(neighbor.pos, 4, {});
            }
            costG += count * config.lossCongestion;
          } else if (!state.repel) {
            if (nbType == "straight_0" || nbType == "straight_45") {
              count = drc.checkSpacing(neighbor.pos, state.unrouted, groups);
            }
            if (count > 0) {
              costG += count * config.lossCongestion;
            }
          }
        }
      }

      if (neighbor.costG > costG) {
        const std::array<int, 2> neighborPos2 = {neighbor.pos[0], neighbor.pos[1]};
        const std::array<int, 2> endPos2 = {state.endNode[0], state.endNode[1]};
        const double heuristicWeight =
            (mfotPlan != nullptr && mfotPlan->enabled)
                ? mfotSearchWeightForNet(*mfotPlan, config, net.netName)
                : 1.0;
        double costF = costG
                       + heuristicWeight * config.lossPropagation
                             * customH(neighborPos2,
                                       endPos2,
                                       config.lossBending,
                                       config.gridResolution);
        if (strictDrc && historyMap != nullptr) {
          const int x = neighbor.pos[0];
          const int y = neighbor.pos[1];
          if (x >= 0 && x < historyWidth && y >= 0 && y < historyHeight) {
            const auto historyIndex =
                (static_cast<std::size_t>(x) * static_cast<std::size_t>(historyHeight)
                 + static_cast<std::size_t>(y))
                    * 8
                + static_cast<std::size_t>(orientationHistoryIndex(neighbor.pos[2]));
            costF += static_cast<double>((*historyMap)[historyIndex]);
          }
        }
        if (tracePositions.find(currentNode.pos) != tracePositions.end()) {
          const auto oldFlags = std::cerr.flags();
          const auto oldPrecision = std::cerr.precision();
          std::cerr << std::defaultfloat << std::setprecision(17)
                    << "TRACE_COST"
                    << "\tnet=" << net.netName
                    << "\tcurrent=" << currentNode.pos[0] << "," << currentNode.pos[1]
                    << "," << currentNode.pos[2]
                    << "\tneighbor=" << neighbor.pos[0] << "," << neighbor.pos[1]
                    << "," << neighbor.pos[2]
                    << "\ttype=" << nbType
                    << "\tcost_g=" << costG
                    << "\tcost_f=" << costF
                    << "\told_cost_g=" << neighbor.costG
                    << "\n";
          std::cerr.flags(oldFlags);
          std::cerr.precision(oldPrecision);
        }
        if (nbType == "straight_0" || nbType == "straight_45") {
          neighbor.straightCount = currentNode.straightCount + 1;
        } else if (nbType == "bend_45_1" || nbType == "bend_45_2") {
          neighbor.straightCount = 0;
        }
        neighbor.costG = costG;
        neighbor.costF = costF;
        neighbor.parent = static_cast<int>(currentIndex);
        traceSetOperation(neighborIndex, nodepq.contains(neighborIndex));
        nodepq.set(neighborIndex);
      }
    }
  }

  if (traceNet != nullptr && net.netName == traceNet) {
    std::cerr << "TRACE_ROUTE_END"
              << "\tnet=" << net.netName
              << "\tattempt=" << traceAttempt
              << "\tstrict=" << (strictDrc ? 1 : 0)
              << "\tsuccess=0"
              << "\tpops=" << result.visitedNodes
              << "\n";
  }
  writeTracePopFile(false);
  return result;
}

LidarGridRouteFlowResult routeAllNetsGrid(LidarRuntimeView& db,
                                          LidarDrcManager& drc,
                                          const LidarRouteConfig& config)
{
  LidarGridRouteFlowResult flow;
  LidarGridRouter router(db, config);
  router.processNetOrder();
  const auto mfotStart = std::chrono::steady_clock::now();
  auto mfotPlan = buildMfOtPlan(db, drc, config);
  const auto mfotEnd = std::chrono::steady_clock::now();
  applyMfOtGlobalPriorities(db, mfotPlan, config);
  flow.mfotEnabled = mfotPlan.enabled;
  flow.mfotNets = mfotPlan.nets.size();
  flow.mfotCorridorCells = mfotPlan.totalCorridorCells;
  flow.mfotFreeEnergy = mfotPlan.totalFreeEnergy;
  std::cout << std::fixed << std::setprecision(6)
            << "timing_cpp_mfot_plan_s="
            << std::chrono::duration<double>(mfotEnd - mfotStart).count()
            << "\n"
            << "mfot_enabled=" << (mfotPlan.enabled ? 1 : 0) << "\n"
            << "mfot_nets=" << mfotPlan.nets.size() << "\n"
            << "mfot_corridor_cells=" << mfotPlan.totalCorridorCells << "\n"
            << "mfot_free_energy=" << mfotPlan.totalFreeEnergy << "\n"
            << std::flush;

  const int historyWidth = drc.bitmapWidth();
  const int historyHeight = drc.bitmapHeight();
  std::vector<int> historyMap(static_cast<std::size_t>(historyWidth)
                                  * static_cast<std::size_t>(historyHeight) * 8,
                              0);
  seedMfOtHistoryMap(mfotPlan, config, historyMap, historyWidth, historyHeight);

  auto finalizeHistoryStats = [&]() {
    flow.historyNonzero = 0;
    flow.historySum = 0;
    for (const auto value : historyMap) {
      if (value != 0) {
        ++flow.historyNonzero;
        flow.historySum += value;
      }
    }
  };

  auto updateHistoryMap = [&](const LidarAstarRouteResult& result) {
    // Match upstream LiDAR: strict hard_connection does not populate Nets.origin_path.
    if (!result.success || result.strictDrc) {
      return;
    }
    auto netIt = db.netIndex.find(result.netName);
    if (netIt == db.netIndex.end()) {
      return;
    }
    const auto& net = db.nets[netIt->second];
    const auto& port1 = db.ports[net.sourcePortIndex];
    const auto& port2 = db.ports[net.targetPortIndex];
    std::set<std::array<int, 2>> sourceGrids(port1.portGrids.begin(), port1.portGrids.end());
    std::set<std::array<int, 2>> targetGrids(port2.portGrids.begin(), port2.portGrids.end());

    for (const auto& point : result.originPath) {
      const std::array<int, 2> grid = {point[0], point[1]};
      if (sourceGrids.find(grid) != sourceGrids.end()
          || targetGrids.find(grid) != targetGrids.end()) {
        continue;
      }
      const int x = point[0];
      const int y = point[1];
      if (x < 0 || x >= historyWidth || y < 0 || y >= historyHeight) {
        continue;
      }
      const auto historyIndex =
          (static_cast<std::size_t>(x) * static_cast<std::size_t>(historyHeight)
           + static_cast<std::size_t>(y))
              * 8
          + static_cast<std::size_t>(orientationHistoryIndex(point[2]));
      historyMap[historyIndex] += config.historyCost;
    }
  };

  auto registerResult = [&](const LidarAstarRouteResult& result) {
    if (!result.success) {
      return;
    }
    updateHistoryMap(result);
    drc.updatePathBitmap(result.processedPath, result.netName);
    for (const auto& crossingNetName :
         result.crossingNets.pythonIterationOrder()) {
      auto crossingIt = db.netIndex.find(crossingNetName);
      if (crossingIt != db.netIndex.end()) {
        db.nets[crossingIt->second].crossingNets.insert(result.netName);
      }
    }
  };

  auto routeAbnormalFanoutFallbacks = [&](int iteration) {
    std::vector<std::string> abnormalNames(flow.abnormalNets.begin(),
                                           flow.abnormalNets.end());
    for (const auto& netName : abnormalNames) {
      auto netIt = db.netIndex.find(netName);
      if (netIt == db.netIndex.end()) {
        continue;
      }
      auto fallback = tryFanoutTreeFallbackRoute(
          db, drc, db.nets[netIt->second], config);
      if (!fallback.has_value()) {
        continue;
      }
      flow.entries.push_back(
          {iteration, "__fanout_tree_fallback__", fallback.value()});
      registerResult(fallback.value());
      flow.abnormalNets.erase(netName);
      db.abnormalNets.erase(netName);
    }
  };

  auto groupSet = [&](const std::string& groupName) {
    std::set<std::string> groupNetSet;
    auto groupIt = db.groupNets.find(groupName);
    if (groupIt != db.groupNets.end()) {
      groupNetSet.insert(groupIt->second.begin(), groupIt->second.end());
    }
    return groupNetSet;
  };

  auto routeKernel = [&](LidarNet& net,
                         bool strictDrc,
                         int drUnrouted,
                         const std::set<std::string>& groups) {
    return routeSingleNetGrid(db,
                              drc,
                              net,
                              config,
                              strictDrc,
                              drUnrouted,
                              groups,
                              &historyMap,
                              historyWidth,
                              historyHeight,
                              mfotPlan.enabled ? &mfotPlan : nullptr);
  };

  auto routeLoss = [&](const LidarNet& net, const LidarAstarRouteResult& result) {
    return config.lossPropagation * static_cast<double>(net.wirelength) * 1e-4
           + config.ilCross * static_cast<double>(result.crossingNets.size());
  };

  auto ripupLocalNets = [&](const LidarPythonStringSet& crossingNets) {
    for (const auto& crossingNetName : crossingNets.pythonIterationOrder()) {
      auto crossingIt = db.netIndex.find(crossingNetName);
      if (crossingIt == db.netIndex.end()) {
        continue;
      }
      auto& crossingNet = db.nets[crossingIt->second];
      drc.deleteNetFromBitmap(crossingNetName);
      clearRoutedNet(db, crossingNet);
    }
  };

  auto routeWithPythonPolicy = [&](LidarNet& net,
                                   bool strictDrc,
                                   int drUnrouted,
                                   const std::set<std::string>& groups) {
    if (config.netOrder == "topo" || !config.group || !strictDrc) {
      return routeKernel(net, strictDrc, drUnrouted, groups);
    }

    auto crossingResult = routeKernel(net, true, drUnrouted, groups);
    if (!crossingResult.success) {
      return crossingResult;
    }
    if (crossingResult.crossingNets.empty()) {
      return crossingResult;
    }

    bool clear = false;
    for (const auto& crossingNetName :
         crossingResult.crossingNets.pythonIterationOrder()) {
      auto crossingIt = db.netIndex.find(crossingNetName);
      if (crossingIt == db.netIndex.end()) {
        continue;
      }
      const auto& crossingNet = db.nets[crossingIt->second];
      if (net.failedCount == 0
          && crossingNet.eulerDistance > net.eulerDistance + 1000.0) {
        clear = true;
        break;
      }
    }

    if (clear) {
      const auto backupCrossingNets = crossingResult.crossingNets;
      clearRoutedNet(db, net);
      ripupLocalNets(backupCrossingNets);
      net.crossingBudget = 10;
      net.currentBudget = 10;
      return routeKernel(net, true, drUnrouted, groups);
    }

    const double lossWithCrossing = routeLoss(net, crossingResult);
    const int backupFailedCount = net.failedCount;
    const auto backupCrossingNets = crossingResult.crossingNets;
    clearRoutedNet(db, net, false);

    net.crossingBudget = 0;
    net.currentBudget = 0;
    auto detourResult = routeKernel(net, true, drUnrouted, groups);
    if (detourResult.success) {
      const double lossWithoutCrossing = routeLoss(net, detourResult);
      if (lossWithCrossing + 0.2 > lossWithoutCrossing) {
        net.crossingBudget = 10;
        net.currentBudget = 10;
        return detourResult;
      }

      clearRoutedNet(db, net, false);
      net.crossingBudget = 10;
      net.currentBudget = 10;
      return routeKernel(net, true, drUnrouted, groups);
    }

    if (backupFailedCount == 0) {
      ripupLocalNets(backupCrossingNets);
      ++net.failedCount;
      if (net.failedCount > 4) {
        net.enable45 = false;
      }
    }
    clearRoutedNet(db, net, false);
    net.crossingBudget = 10;
    net.currentBudget = 10;
    auto fallbackResult = routeKernel(net, true, drUnrouted, groups);
    net.crossingBudget = 10;
    net.currentBudget = 10;
    return fallbackResult;
  };

  bool routedAny = false;

  std::set<std::string> globalFailedNets;
  int ripupTimes = 0;
  for (int iteration = 1; iteration <= config.maxIteration; ++iteration) {
    PythonNetHeap globalHeap(db);
    for (std::size_t i = 0; i < db.nets.size(); ++i) {
      if (!db.nets[i].routed) {
        globalHeap.push(i);
      }
    }

    while (!globalHeap.empty()) {
      const auto curIndex = globalHeap.pop();
      auto& curNet = db.nets[curIndex];
      if (curNet.routed) {
        continue;
      }

      if (config.group) {
        int groupLen = 0;
        for (const auto& groupName : curNet.groups) {
          auto groupIt = db.groupNets.find(groupName);
          if (groupIt != db.groupNets.end()) {
            groupLen = std::max(groupLen, static_cast<int>(groupIt->second.size()));
          }
        }

        for (const auto& groupName : curNet.groups) {
          auto groupIt = db.groupNets.find(groupName);
          if (groupIt == db.groupNets.end()) {
            continue;
          }

          std::vector<std::size_t> localIndices;
          localIndices.reserve(groupIt->second.size());
          for (std::size_t i = 0; i < groupIt->second.size(); ++i) {
            auto netIt = db.netIndex.find(groupIt->second[i]);
            if (netIt == db.netIndex.end()) {
              continue;
            }
            db.nets[netIt->second].routingOrder = static_cast<int>(i);
            localIndices.push_back(netIt->second);
          }
          if (mfotPlan.enabled && config.mfotPriorityScale > 0.0) {
            std::stable_sort(
                localIndices.begin(),
                localIndices.end(),
                [&](std::size_t lhs, std::size_t rhs) {
                  const double leftPriority =
                      mfotNetPriority(mfotPlan, db.nets[lhs].netName);
                  const double rightPriority =
                      mfotNetPriority(mfotPlan, db.nets[rhs].netName);
                  if (std::abs(leftPriority - rightPriority) > 1e-9) {
                    return leftPriority > rightPriority;
                  }
                  return db.nets[lhs].routingOrder < db.nets[rhs].routingOrder;
                });
            for (std::size_t rank = 0; rank < localIndices.size(); ++rank) {
              db.nets[localIndices[rank]].routingOrder = static_cast<int>(rank);
            }
          }

          const auto localOrder = pythonHeapOrder(db, localIndices);
          const auto groupNets = groupSet(groupName);
          for (const auto localIndex : localOrder) {
            auto& localNet = db.nets[localIndex];
            if (localNet.routed) {
              continue;
            }

            const int drUnrouted = groupLen - localNet.routingOrder + 1;
            auto result = routeWithPythonPolicy(localNet, true, drUnrouted, groupNets);
            if (!result.success) {
              flow.entries.push_back({iteration, groupName, result});
              result = routeWithPythonPolicy(localNet, false, drUnrouted, groupNets);
            }
            flow.entries.push_back({iteration, groupName, result});
            if (result.success) {
              routedAny = true;
              registerResult(result);
              if (!result.strictDrc && !result.violatedNets.empty()) {
                globalFailedNets.insert(localNet.netName);
                globalFailedNets.insert(result.violatedNets.begin(),
                                        result.violatedNets.end());
              }
            } else {
              flow.abnormalNets.insert(localNet.netName);
              db.abnormalNets[localNet.netName] = localNet;
            }
          }
        }
      } else {
        auto result = routeWithPythonPolicy(curNet, true, 2, {});
        if (!result.success) {
          flow.entries.push_back({iteration, {}, result});
          result = routeWithPythonPolicy(curNet, false, 2, {});
        }
        flow.entries.push_back({iteration, {}, result});
        if (result.success) {
          routedAny = true;
          registerResult(result);
          if (!result.strictDrc && !result.violatedNets.empty()) {
            globalFailedNets.insert(curNet.netName);
            globalFailedNets.insert(result.violatedNets.begin(),
                                    result.violatedNets.end());
          }
        } else {
          flow.abnormalNets.insert(curNet.netName);
          db.abnormalNets[curNet.netName] = curNet;
        }
      }
    }

    if (iteration == config.maxIteration && !globalFailedNets.empty()) {
      routeAbnormalFanoutFallbacks(iteration);
      flow.success = false;
      finalizeHistoryStats();
      return flow;
    }
    if (globalFailedNets.empty()) {
      routeAbnormalFanoutFallbacks(iteration);
      flow.success = routedAny && flow.abnormalNets.empty();
      finalizeHistoryStats();
      return flow;
    }

    ++ripupTimes;
    for (const auto& netName : globalFailedNets) {
      auto netIt = db.netIndex.find(netName);
      if (netIt == db.netIndex.end()) {
        continue;
      }
      auto& net = db.nets[netIt->second];
      drc.deleteNetFromBitmap(netName);
      clearRoutedNet(db, net);
      if (ripupTimes == -1) {
        net.crossingBudget = std::max(net.crossingBudget, net.maximumCrossing);
        net.crossingBudget = 100;
      }
      net.maximumCrossing = 0;
    }
    if (ripupTimes == 2) {
      std::fill(historyMap.begin(), historyMap.end(), 0);
    }
    globalFailedNets.clear();
  }

  routeAbnormalFanoutFallbacks(config.maxIteration);
  flow.success = routedAny && flow.abnormalNets.empty();
  finalizeHistoryStats();
  return flow;
}

void writeAstarInitSummary(const LidarRuntimeView& db,
                           const LidarDrcManager& drc,
                           const LidarRouteConfig& config,
                           std::ostream& os)
{
  os << std::fixed << std::setprecision(6);
  for (const auto& net : db.nets) {
    const auto state = buildAstarInitState(db, drc, net, config, 2);
    os << "ASTAR\t" << state.netName
       << "\tconnect_th=" << state.connectThreshold
       << "\trepel=" << (state.repel ? 1 : 0)
       << "\tunrouted=" << state.unrouted
       << "\tcheck_region=" << state.checkRegion
       << "\trouting_bound=" << state.routingBound[0][0] << "," << state.routingBound[0][1]
       << "," << state.routingBound[1][0] << "," << state.routingBound[1][1]
       << "\tcrossing_budget=" << state.crossingBudget
       << "\tradius=" << state.radius
       << "\tgrid_radius=" << state.gridRadius
       << "\tbend45=" << state.bend45Part1 << "," << state.bend45Part2
       << "\tpredict_length=" << state.predictLength
       << "\tstart_port=" << state.startPortName
       << "\tend_port=" << state.endPortName
       << "\tstart=" << state.startNode[0] << "," << state.startNode[1] << "," << state.startNode[2]
       << "\tend=" << state.endNode[0] << "," << state.endNode[1] << "," << state.endNode[2]
       << "\tstart_cost_f=" << state.startCostF
       << "\taccess_points=" << state.accessPoints
       << "\tstepG=straight_0:" << state.stepG.at("straight_0")
       << ",straight_45:" << state.stepG.at("straight_45")
       << ",bend_45_1:" << state.stepG.at("bend_45_1")
       << ",bend_45_2:" << state.stepG.at("bend_45_2")
       << ",bend_90:" << state.stepG.at("bend_90");

    os << "\tnextSteps=";
    bool firstOrientation = true;
    const std::array<int, 8> orientationOrder = {0, 90, 180, 270, 45, 135, 225, 315};
    for (const auto orientation : orientationOrder) {
      if (!firstOrientation) {
        os << ";";
      }
      firstOrientation = false;
      const auto& steps = state.nextSteps.at(orientation);
      os << orientation << ":";
      for (std::size_t i = 0; i < steps.size(); ++i) {
        if (i != 0) {
          os << "|";
        }
        const auto& step = steps[i];
        os << step.dx << "," << step.dy << "," << step.orientation << "," << step.type;
      }
    }
    os << "\n";
  }
}

void writeAstarRouteSummary(LidarRuntimeView& db,
                            LidarDrcManager& drc,
                            const LidarRouteConfig& config,
                            std::ostream& os,
                            bool updateBitmap)
{
  os << std::fixed << std::setprecision(6);
  for (auto& net : db.nets) {
    std::set<std::string> groups;
    for (const auto& groupName : net.groups) {
      auto groupIt = db.groupNets.find(groupName);
      if (groupIt == db.groupNets.end()) {
        continue;
      }
      groups.insert(groupIt->second.begin(), groupIt->second.end());
    }

    auto result = routeSingleNetGrid(db, drc, net, config, true, 2, groups);
    if (!result.success) {
      result = routeSingleNetGrid(db, drc, net, config, false, 2, groups);
    }

    os << "ROUTE\t" << net.netName
       << "\tsuccess=" << (result.success ? 1 : 0)
       << "\tstrict=" << (result.strictDrc ? 1 : 0)
       << "\tshort_sbend=" << (result.shortSbend ? 1 : 0)
       << "\tshort_sbend_length=" << result.shortSbendLength
       << "\tvisited=" << result.visitedNodes
       << "\texpanded=" << result.expandedNodes
       << "\tcost_g=" << result.finalCostG
       << "\tcost_f=" << result.finalCostF
       << "\torigin_len=" << result.originPath.size()
       << "\tprocessed_len=" << result.processedPath.size()
       << "\tpop_trace=" << pathToString(result.popTrace)
       << "\tcrossings=";
    bool first = true;
    for (const auto& crossing : result.crossingNets) {
      if (!first) {
        os << ",";
      }
      first = false;
      os << crossing;
    }
    os << "\tviolations=";
    first = true;
    for (const auto& violation : result.violatedNets) {
      if (!first) {
        os << ",";
      }
      first = false;
      os << violation;
    }
    os << "\torigin_path=" << pathToString(result.originPath)
       << "\tprocessed_path=" << pathToString(result.processedPath)
       << "\n";

    if (updateBitmap && result.success) {
      drc.updatePathBitmap(result.processedPath, net.netName);
    }
  }
}

void writeGridRouteFlowSummary(LidarRuntimeView& db,
                               LidarDrcManager& drc,
                               const LidarRouteConfig& config,
                               std::ostream& os)
{
  const auto flow = routeAllNetsGrid(db, drc, config);
  writeGridRouteFlowSummary(flow, os);
}

void writeGridRouteFlowSummary(const LidarGridRouteFlowResult& flow,
                               std::ostream& os)
{
  os << std::fixed << std::setprecision(6);
  os << "FLOW\tsuccess=" << (flow.success ? 1 : 0)
     << "\tentries=" << flow.entries.size()
     << "\tabnormal=" << joinSet(flow.abnormalNets)
     << "\thistory_nonzero=" << flow.historyNonzero
     << "\thistory_sum=" << flow.historySum
    << "\tmfot_enabled=" << (flow.mfotEnabled ? 1 : 0)
    << "\tmfot_nets=" << flow.mfotNets
    << "\tmfot_corridor_cells=" << flow.mfotCorridorCells
    << "\tmfot_free_energy=" << flow.mfotFreeEnergy
     << "\n";
  for (const auto& entry : flow.entries) {
    const auto& result = entry.route;
    os << "FLOWROUTE\titer=" << entry.iteration
       << "\tgroup=" << entry.groupName
       << "\tnet=" << result.netName
       << "\tsuccess=" << (result.success ? 1 : 0)
       << "\tstrict=" << (result.strictDrc ? 1 : 0)
       << "\tshort_sbend=" << (result.shortSbend ? 1 : 0)
       << "\tshort_sbend_length=" << result.shortSbendLength
       << "\tvisited=" << result.visitedNodes
       << "\texpanded=" << result.expandedNodes
       << "\tcost_g=" << result.finalCostG
       << "\tcost_f=" << result.finalCostF
       << "\torigin_len=" << result.originPath.size()
       << "\tprocessed_len=" << result.processedPath.size()
       << "\tpop_trace=" << pathToString(result.popTrace)
       << "\tcrossings=" << joinSet(result.crossingNets)
       << "\tviolations=" << joinSet(result.violatedNets)
       << "\torigin_path=" << pathToString(result.originPath)
       << "\tprocessed_path=" << pathToString(result.processedPath)
       << "\n";
  }
}

void writeGridRouteResultYml(LidarRuntimeView& db,
                             LidarDrcManager& drc,
                             const LidarRouteConfig& config,
                             std::ostream& os)
{
  const auto flow = routeAllNetsGrid(db, drc, config);
  writeGridRouteResultYml(db, config, flow, os);
}

void writeGridRouteResultYml(const LidarRuntimeView& db,
                             const LidarRouteConfig& config,
                             const LidarGridRouteFlowResult& flow,
                             std::ostream& os)
{
  os << std::fixed << std::setprecision(6);

  os << "schema: picdb_lidar_route_result\n";
  os << "schema_version: 1\n";
  os << "success: " << (flow.success ? "true" : "false") << "\n";
  os << "entries: " << flow.entries.size() << "\n";
  os << "abnormal_nets: ";
  writeYamlStringSeq(os, flow.abnormalNets, 2);
  os << "settings:\n";
  os << "  design: " << db.designName << "\n";
  os << "  grid_resolution: " << config.gridResolution << "\n";
  os << "  bend_radius: " << config.bendRadius << "\n";
  os << "  net_order: " << config.netOrder << "\n";
  os << "  group: " << (config.group ? "true" : "false") << "\n";
  os << "  mfot_enabled: " << (config.enableMfOtRouting ? "true" : "false") << "\n";
  os << "  mfot_iterations: " << config.mfotIterations << "\n";
  os << "  mfot_grid_stride: " << config.mfotGridStride << "\n";
  os << "  mfot_max_samples_per_net: " << config.mfotMaxSamplesPerNet << "\n";
  os << "  mfot_corridor_radius: " << config.mfotCorridorRadius << "\n";
  os << "  success: " << (flow.success ? "true" : "false") << "\n";
  os << "  entries: " << flow.entries.size() << "\n";
  os << "  history_nonzero: " << flow.historyNonzero << "\n";
  os << "  history_sum: " << flow.historySum << "\n";
  os << "  mfot_nets: " << flow.mfotNets << "\n";
  os << "  mfot_corridor_cells: " << flow.mfotCorridorCells << "\n";
  os << "  mfot_free_energy: " << flow.mfotFreeEnergy << "\n";
  os << "  abnormal_nets: ";
  writeYamlStringSeq(os, flow.abnormalNets, 4);

  os << "flow:\n";
  if (flow.entries.empty()) {
    os << "  []\n";
  } else {
    for (const auto& entry : flow.entries) {
      const auto& result = entry.route;
      os << "  - iteration: " << entry.iteration << "\n";
      os << "    group: " << entry.groupName << "\n";
      os << "    net: " << result.netName << "\n";
      os << "    success: " << (result.success ? "true" : "false") << "\n";
      os << "    strict: " << (result.strictDrc ? "true" : "false") << "\n";
      os << "    short_sbend: " << (result.shortSbend ? "true" : "false") << "\n";
      os << "    short_sbend_length: " << result.shortSbendLength << "\n";
      os << "    visited: " << result.visitedNodes << "\n";
      os << "    expanded: " << result.expandedNodes << "\n";
      os << "    cost_g: " << result.finalCostG << "\n";
      os << "    cost_f: " << result.finalCostF << "\n";
      os << "    crossings: ";
      writeYamlStringSeq(os, result.crossingNets, 6);
      os << "    violations: ";
      writeYamlStringSeq(os, result.violatedNets, 6);
      if (!result.popTrace.empty()) {
        os << "    pop_trace_grid: ";
        writeYamlGridPath(os, result.popTrace, 6);
        os << "    pop_trace_cost: ";
        writeYamlCostTrace(os, result.popTraceCost, 6);
      }
      os << "    origin_path_grid: ";
      writeYamlGridPath(os, result.originPath, 6);
      os << "    processed_path_grid: ";
      writeYamlGridPath(os, result.processedPath, 6);
      os << "    processed_path_um: ";
      writeYamlMicronPath(os, result.processedPath, config.gridResolution, 6);
    }
  }

  os << "instances:\n";
  if (db.instances.empty()) {
    os << "  {}\n";
  } else {
    for (const auto& instance : db.instances) {
      os << "  " << instance.name << ":\n";
      os << "    idBlk: " << instance.idBlk << "\n";
      os << "    bbox: [" << instance.bbox.lx << ", " << instance.bbox.ly
         << ", " << instance.bbox.ux << ", " << instance.bbox.uy << "]\n";
      os << "    ports0: " << instance.portsByOrientation[0].size() << "\n";
      os << "    ports90: " << instance.portsByOrientation[1].size() << "\n";
      os << "    ports180: " << instance.portsByOrientation[2].size() << "\n";
      os << "    ports270: " << instance.portsByOrientation[3].size() << "\n";
    }
  }

  os << "nets:\n";
  if (db.nets.empty()) {
    os << "  {}\n";
    return;
  }
  const auto postRoutes = buildPythonPostProcessRoutes(db, config);
  for (const auto& net : db.nets) {
    const auto& sourcePort = db.ports[net.sourcePortIndex];
    const auto& targetPort = db.ports[net.targetPortIndex];
    const auto& routeStartPort =
        (config.group && net.reverse) ? targetPort : sourcePort;
    const auto& routeEndPort =
        (config.group && net.reverse) ? sourcePort : targetPort;
    const auto postPathIt = postRoutes.paths.find(net.netName);
    const std::vector<PostRoutePath> emptyPostPaths;
    const auto& postPaths =
        postPathIt == postRoutes.paths.end() ? emptyPostPaths : postPathIt->second;
    const auto postCrossingIt = postRoutes.crossingSets.find(net.netName);
    const LidarPythonStringSet emptyCrossingSet;
    const auto& postCrossingSet = postCrossingIt == postRoutes.crossingSets.end()
                                      ? emptyCrossingSet
                                      : postCrossingIt->second;
    const int postCrossingNum =
        postPaths.empty() ? 0 : static_cast<int>(postPaths.size()) - 1;
    const auto accessViolations =
        validatePostRouteAccesses(net.netName, postPaths);
    os << "  " << net.netName << ":\n";
    os << "    source: " << net.sourcePortName << "\n";
    os << "    target: " << net.targetPortName << "\n";
    os << "    routed: " << (net.routed ? "true" : "false") << "\n";
    os << "    short_sbend: " << (net.shortSbend ? "true" : "false") << "\n";
    os << "    short_sbend_length: " << net.shortSbendLength << "\n";
    os << "    failed_count: " << net.failedCount << "\n";
    os << "    raw_crossing_num: " << net.crossingNum << "\n";
    os << "    raw_crossing_nets: ";
    writeYamlStringSeq(os, net.crossingNets, 6);
    os << "    crossing_num: " << postCrossingNum << "\n";
    os << "    vionets: " << net.vionets << "\n";
    os << "    crossing_nets: ";
    writeYamlStringSeq(os, postCrossingSet, 6);
    os << "    vio_nets: ";
    writeYamlStringSeq(os, net.vioNets, 6);
    os << "    path_count: " << postPaths.size() << "\n";
    os << "    paths: ";
    writeYamlPostRoutePaths(os, postPaths, 6);
    os << "    postprocess_access_valid: "
       << (accessViolations.empty() ? "true" : "false") << "\n";
    os << "    postprocess_access_violation_count: "
       << accessViolations.size() << "\n";
    os << "    postprocess_access_violations: ";
    writeYamlPostProcessAccessViolations(os, accessViolations, 6);
    os << "    source_port: ";
    writeYamlPort(os, sourcePort, 6);
    os << "    target_port: ";
    writeYamlPort(os, targetPort, 6);
    os << "    route_start_port: ";
    writeYamlPort(os, routeStartPort, 6);
    os << "    route_end_port: ";
    writeYamlPort(os, routeEndPort, 6);
    os << "    origin_path_grid: ";
    writeYamlGridPath(os, net.originPath, 6);
    const auto simplifiedPath = simplifyCollinearPath(net.routedPath);
    os << "    routed_path_grid_raw: ";
    writeYamlGridPath(os, net.routedPath, 6);
    os << "    routed_path_um_raw: ";
    writeYamlMicronPath(os, net.routedPath, config.gridResolution, 6);
    os << "    routed_path_grid: ";
    writeYamlGridPath(os, simplifiedPath, 6);
    os << "    routed_path_um: ";
    auto alignedPath = toMicronPath(simplifiedPath, config.gridResolution);
    applyPythonPortAlignment(alignedPath, routeStartPort, routeEndPort);
    writeYamlMicronPointPath(os, alignedPath, 6);
    os << "    rwguide: ";
    writeYamlGridPointSet(os, net.rwguide, 6);
  }
}

std::shared_ptr<Instance> createCrossingInstanceTemplate()
{
  constexpr double portOffset = 4.0;
  constexpr double width = 0.5;
  CrossSection wg("WG", width);
  Point ref(0, 0);
  double rotation = 0.0;
  std::vector<Point> bbox = {Point(0, 0),
                             Point(2.0 * portOffset, 2.0 * portOffset)};
  auto shape = std::make_shared<Rect>(ref, rotation, bbox);
  auto inst = std::make_shared<Instance>(
      "lidar_crossing", "crossing", shape, InstanceType::CELL);
  inst->setGenerator("crossing");
  inst->setDoc("LiDAR inserted optical crossing");

  auto addPin = [&](const std::string& name,
                    const Point& localPosition,
                    double localRotation) {
    auto pin = std::make_shared<Pin>(
        name, width, localPosition, localRotation, wg);
    pin->setDoc("LiDAR crossing port");
    inst->addPin(pin);
  };
  addPin("o1", Point(0.0, portOffset), 180.0);
  addPin("o2", Point(portOffset, 2.0 * portOffset), 90.0);
  addPin("o3", Point(2.0 * portOffset, portOffset), 0.0);
  addPin("o4", Point(portOffset, 0.0), 270.0);
  return inst;
}

void addPostRouteCrossingCellsToDesign(Design& design,
                                       const PythonPostProcessRoutes& postRoutes)
{
  for (const auto& crossing : postRoutes.crossings) {
    const auto inst = createCrossingInstanceTemplate();
    auto cell = Cell::createCell(crossing.name,
                                 inst,
                                 Point(crossing.center[0],
                                       crossing.center[1]),
                                 crossing.orientation,
                                 false,
                                 Placement::FIXED);
    design.addCell(cell);
  }
}

std::shared_ptr<Pin> resolvePathPortPin(Design& design,
                                        const PathPortDump& port)
{
  if (port.instanceName.empty() || port.pinName.empty()) {
    return nullptr;
  }
  auto& cells = design.getCells();
  const auto cellIt = cells.find(port.instanceName);
  if (cellIt == cells.end() || !cellIt->second) {
    return nullptr;
  }
  return cellIt->second->getPin(port.pinName);
}

std::string splitNetName(const std::string& baseName, std::size_t index)
{
  if (index == 0) {
    return baseName;
  }
  return baseName + "__part_" + std::to_string(index);
}

bool addPhysicalRouteNet(Design& design,
                         const std::string& netName,
                         const PathPortDump& startPort,
                         const PathPortDump& endPort,
                         const std::shared_ptr<Segment>& segment)
{
  auto sourcePin = resolvePathPortPin(design, startPort);
  auto targetPin = resolvePathPortPin(design, endPort);
  if (!sourcePin || !targetPin || !segment) {
    return false;
  }
  auto newNet = std::make_shared<Net>(netName);
  newNet->addPin(sourcePin);
  newNet->addPin(targetPin);
  newNet->addSegment(segment);
  newNet->setRouting(Routing::ROUTED);
  design.addNet(newNet);
  return true;
}

LidarRouteWritebackResult writeRoutedGridToDesign(
    Design& design,
    const LidarRuntimeView& db,
    const LidarRouteConfig& config,
    bool routeSuccess)
{
  LidarRouteWritebackResult result;
  result.success = routeSuccess;

  const auto postRoutes = buildPythonPostProcessRoutes(db, config);
  std::unordered_map<std::string, std::vector<PostProcessAccessViolation>>
      invalidAccessesByNet;
  for (const auto& lidarNet : db.nets) {
    if (!lidarNet.routed || lidarNet.shortSbend) {
      continue;
    }
    const auto postPathIt = postRoutes.paths.find(lidarNet.netName);
    if (postPathIt == postRoutes.paths.end()) {
      continue;
    }
    auto violations =
        validatePostRouteAccesses(lidarNet.netName, postPathIt->second);
    if (violations.empty()) {
      continue;
    }
    result.postprocessInvalidAccesses += violations.size();
    invalidAccessesByNet.emplace(lidarNet.netName, std::move(violations));
  }
  addPostRouteCrossingCellsToDesign(design, postRoutes);
  for (const auto& lidarNet : db.nets) {
    auto netIt = design.getNets().find(lidarNet.designNetName);
    if (netIt == design.getNets().end()) {
      netIt = design.getNets().find(lidarNet.netName);
    }
    if (netIt == design.getNets().end()) {
      ++result.skippedNets;
      continue;
    }

    const std::string designNetName = netIt->first;
    if (!lidarNet.routed) {
      netIt->second->clearSegments();
      netIt->second->setRouting(Routing::UNROUTED);
      continue;
    }
    if (lidarNet.shortSbend) {
      const auto& sourcePort = db.ports[lidarNet.sourcePortIndex];
      const auto& targetPort = db.ports[lidarNet.targetPortIndex];
      const auto& routeStartPort =
          (config.group && lidarNet.reverse) ? targetPort : sourcePort;
      const auto& routeEndPort =
          (config.group && lidarNet.reverse) ? sourcePort : targetPort;
      const auto sampled = shortSbendBezierPoints(routeStartPort, routeEndPort);
      std::vector<Point> points;
      points.reserve(sampled.size());
      for (const auto& point : sampled) {
        points.emplace_back(point[0], point[1]);
      }
      const std::string segmentName = lidarNet.netName + "_sbend_0";
      const std::string xsection =
          lidarNet.material.empty() ? "WG" : lidarNet.material;
      auto segment = Segment::createWaveguide(segmentName,
                                              points,
                                              lidarNet.width,
                                              xsection,
                                              routeStartPort.orientation,
                                              routeEndPort.orientation);
      PathPortDump startDump = pathPortFromLidarPort(routeStartPort);
      PathPortDump endDump = pathPortFromLidarPort(routeEndPort);
      design.removeNet(designNetName);
      if (!addPhysicalRouteNet(
              design, designNetName, startDump, endDump, segment)) {
        ++result.skippedNets;
        continue;
      }
      ++result.waveguideSegments;
      ++result.routedNets;
      continue;
    }

    const auto postPathIt = postRoutes.paths.find(lidarNet.netName);
    if (postPathIt == postRoutes.paths.end() || postPathIt->second.empty()) {
      netIt->second->clearSegments();
      netIt->second->setRouting(Routing::UNROUTED);
      ++result.skippedNets;
      continue;
    }

    bool        canSplitNet = true;
    std::size_t expectedPhysicalNets = 0;
    for (const auto& postPath : postPathIt->second) {
      const auto points = dbWaveguidePointsWithAccess(postPath);
      if (points.size() < 2) {
        continue;
      }
      if (!resolvePathPortPin(design, postPath.startPort)
          || !resolvePathPortPin(design, postPath.endPort)) {
        canSplitNet = false;
        break;
      }
      ++expectedPhysicalNets;
    }
    if (!canSplitNet || expectedPhysicalNets == 0) {
      netIt->second->clearSegments();
      netIt->second->setRouting(Routing::UNROUTED);
      ++result.skippedNets;
      continue;
    }

    design.removeNet(designNetName);
    std::size_t segmentIndex = 0;
    std::size_t physicalNetIndex = 0;
    for (const auto& postPath : postPathIt->second) {
      auto points = dbWaveguidePointsWithAccess(postPath);
      if (points.size() < 2) {
        continue;
      }
      const std::string segmentName =
          lidarNet.netName + "_wg_" + std::to_string(segmentIndex);
      const std::string xsection =
          lidarNet.material.empty() ? "WG" : lidarNet.material;
      auto segment = Segment::createWaveguide(segmentName,
                                              points,
                                              lidarNet.width,
                                              xsection,
                                              postPath.startPort.orientation,
                                              postPath.endPort.orientation);
      if (!addPhysicalRouteNet(design,
                               splitNetName(designNetName, physicalNetIndex),
                               postPath.startPort,
                               postPath.endPort,
                               segment)) {
        continue;
      }
      ++segmentIndex;
      ++physicalNetIndex;
      ++result.waveguideSegments;
    }

    if (segmentIndex == 0 || physicalNetIndex != expectedPhysicalNets) {
      ++result.skippedNets;
      continue;
    }
    ++result.routedNets;
  }

  return result;
}

LidarRouteWritebackResult routeAllNetsGridToDesign(
    Design& design,
    LidarRuntimeView& db,
    LidarDrcManager& drc,
    const LidarRouteConfig& config)
{
  const auto flow = routeAllNetsGrid(db, drc, config);
  return writeRoutedGridToDesign(design, db, config, flow.success);
}

}  // namespace picpr::lidar
