#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <cstdlib>
#include <cctype>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "defyml.h"
#include "lefyml.h"
#include "lidar_astar.h"
#include "db_drc.h"
#include "lidar_drc.h"
#include "log.h"
#include "picdb_lidar_view.h"
#include "routing_rules.h"

namespace
{

void printUsage()
{
  std::cerr
      << "usage:\n"
      << "  pr_lidar_native <lidar_input.yml> <output.gds> [routing options]\n"
      << "  pr_lidar_native <picbench_ref.json> <output.gds> [routing options]\n"
      << "  pr_lidar_native <lef.yml> <def.yml> <out_dir>"
      << " [--net-order=<topo|naive>]"
      << " [--net-default-bound=<int>]"
      << " [--grid-resolution=<float>]"
      << " [--max-iteration=<int>]"
      << " [--route-group|--no-route-group]"
      << " [--enable-45-neighbor|--disable-45-neighbor]"
      << " [--deterministic-order]"
      << " [--no-preserve-net-names]"
      << " [--no-snap-near-integer]"
      << " [--allow-abnormal]\n";
}

struct NativeFlowOptions
{
  RoutingRules                    rules;
  picpr::lidar::LidarViewOptions viewOptions;
  bool                            failOnDbDrc = true;
  bool                            failOnAbnormal = true;
  bool                            requireRoutedNets = true;
};

struct NativeFlowResult
{
  int                   exitCode = 0;
  bool                  routeSuccess = false;
  bool                  dbDrcClean = false;
  std::size_t           routedNets = 0;
  std::size_t           waveguideSegments = 0;
  std::size_t           skippedNets = 0;
  std::size_t           postprocessInvalidAccesses = 0;
  std::size_t           postprocessRejectedNets = 0;
  std::filesystem::path routeResultPath;
  std::filesystem::path flowSummaryPath;
  std::filesystem::path routedDefPath;
  std::filesystem::path dbDrcSummaryPath;
};

bool applyNativeFlowOption(const std::string& arg,
                           NativeFlowOptions& options,
                           bool& allowAbnormal)
{
  if (arg.rfind("--net-order=", 0) == 0) {
    options.rules.netOrder = arg.substr(std::string("--net-order=").size());
  } else if (arg.rfind("--net-default-bound=", 0) == 0) {
    options.rules.netDefaultBound =
        std::stoi(arg.substr(std::string("--net-default-bound=").size()));
  } else if (arg.rfind("--grid-resolution=", 0) == 0) {
    options.rules.gridResolution =
        std::stod(arg.substr(std::string("--grid-resolution=").size()));
  } else if (arg.rfind("--max-iteration=", 0) == 0) {
    options.rules.maxIteration =
        std::stoi(arg.substr(std::string("--max-iteration=").size()));
  } else if (arg == "--route-group") {
    options.rules.group = true;
  } else if (arg == "--no-route-group") {
    options.rules.group = false;
  } else if (arg == "--enable-45-neighbor") {
    options.rules.enable45Neighbor = true;
  } else if (arg == "--disable-45-neighbor") {
    options.rules.enable45Neighbor = false;
  } else if (arg == "--deterministic-order") {
    options.viewOptions.deterministicOrder = true;
  } else if (arg == "--no-preserve-net-names") {
    options.viewOptions.preserveOriginalNetNames = false;
  } else if (arg == "--no-snap-near-integer") {
    options.viewOptions.snapNearIntegerCoordinates = false;
  } else if (arg == "--allow-abnormal") {
    allowAbnormal = true;
  } else {
    return false;
  }
  return true;
}

picpr::lidar::LidarDrcConfig makeDrcConfig(const RoutingRules& rules)
{
  picpr::lidar::LidarDrcConfig config;
  config.gridResolution = rules.gridResolution;
  config.bendRadius     = rules.bendRadius;
  config.maxCrossing    = rules.maxCrossing;
  return config;
}

picpr::lidar::LidarRouteConfig makeRouteConfig(const RoutingRules& rules)
{
  picpr::lidar::LidarRouteConfig config;
  config.netOrder             = rules.netOrder;
  config.group                = rules.group;
  config.maxIteration         = rules.maxIteration;
  config.enable45Neighbor     = rules.enable45Neighbor;
  config.gridResolution       = rules.gridResolution;
  config.bendRadius           = rules.bendRadius;
  config.netBoundScaleFactor  = rules.netBoundScaleFactor;
  config.netDefaultBound      = rules.netDefaultBound;
  config.lossPropagation      = rules.lossPropagation;
  config.lossBending          = rules.lossBending;
  config.lossCrossing         = rules.lossCrossing;
  config.lossCongestion       = rules.lossCongestion;
  config.ilCross              = rules.ilCross;
  config.bendPointsDistance   = rules.bendPointsDistance;
  config.historyCost          = rules.historyCost;
  return config;
}

void writeDbDrcSummary(const Design& design,
                       const std::filesystem::path& path,
                       bool requireRoutedNets)
{
  picpr::drc::DbDrcOptions options;
  options.requireRoutedNets = requireRoutedNets;
  const picpr::drc::DbDrcChecker checker(design, options);
  const auto                      report = checker.check();
  std::ofstream out(path);
  picpr::drc::writeSummary(report, out);
}

bool readCleanDrcSummary(const std::filesystem::path& path)
{
  std::ifstream in(path);
  std::string   line;
  while (std::getline(in, line)) {
    if (line == "clean=1") {
      return true;
    }
    if (line == "clean=0") {
      return false;
    }
  }
  return false;
}

std::string quoteForShell(const std::filesystem::path& path)
{
  std::string value = path.string();
  std::string escaped;
  escaped.reserve(value.size());
  for (const char ch : value) {
    if (ch == '"') {
      escaped += "\\\"";
    } else {
      escaped += ch;
    }
  }
  return "\"" + escaped + "\"";
}

std::string quoteExecutableForShell(const std::filesystem::path& path)
{
  const std::string value = path.string();
  if (value.find_first_of(" \t\"") == std::string::npos) {
    return value;
  }
  return quoteForShell(path);
}

std::string getEnvOrDefault(const char* name, const std::string& fallback)
{
  if (const char* value = std::getenv(name)) {
    if (*value != '\0') {
      return value;
    }
  }
  return fallback;
}

using SteadyClock = std::chrono::steady_clock;

std::string formatSeconds(SteadyClock::duration duration)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(6)
     << std::chrono::duration<double>(duration).count();
  return os.str();
}

void printTiming(const std::string& key, SteadyClock::duration duration)
{
  std::cout << key << "=" << formatSeconds(duration) << "\n";
}

bool isLidarSourceRoot(const std::filesystem::path& path)
{
  return std::filesystem::exists(path / "picroute" / "config" / "comp_LiDAR.yml");
}

std::filesystem::path normalizeIfExists(const std::filesystem::path& path)
{
  if (!std::filesystem::exists(path)) {
    return path;
  }
  return std::filesystem::weakly_canonical(path);
}

std::filesystem::path bundledScriptDir()
{
#ifdef PR_LIDAR_NATIVE_SCRIPT_DIR
  return std::filesystem::path(PR_LIDAR_NATIVE_SCRIPT_DIR);
#else
  return {};
#endif
}

std::filesystem::path bundledPicBenchFlowDir()
{
#ifdef PR_LIDAR_NATIVE_PICBENCH_FLOW_DIR
  return std::filesystem::path(PR_LIDAR_NATIVE_PICBENCH_FLOW_DIR);
#else
  return {};
#endif
}

std::filesystem::path inferLidarSource(const std::filesystem::path& inputYml)
{
  if (const char* env = std::getenv("PICDB_LIDAR_SRC")) {
    const std::filesystem::path path(env);
    if (isLidarSourceRoot(path)) {
      return normalizeIfExists(path);
    }
  }

  for (std::filesystem::path current = normalizeIfExists(inputYml).parent_path();
       !current.empty();
       current = current.parent_path()) {
    if (isLidarSourceRoot(current)) {
      return normalizeIfExists(current);
    }
    if (current == current.parent_path()) {
      break;
    }
  }

  const std::vector<std::filesystem::path> candidates = {
      std::filesystem::current_path() / "LiDAR-external" / "src",
      std::filesystem::current_path() / ".." / "LiDAR-external" / "src",
      bundledScriptDir() / ".." / ".." / ".." / ".." / "LiDAR-external" / "src"};
  for (const auto& candidate : candidates) {
    if (isLidarSourceRoot(candidate)) {
      return normalizeIfExists(candidate);
    }
  }
  return {};
}

int runCommand(const std::string& command)
{
  std::cout << "cmd=" << command << "\n";
  std::cout.flush();
  const int status = std::system(command.c_str());
  return status;
}

NativeFlowResult runNativeFlow(const std::filesystem::path& lefPath,
                               const std::filesystem::path& defPath,
                               const std::filesystem::path& outDir,
                               const NativeFlowOptions& options)
{
  const auto nativeFlowStart = SteadyClock::now();
  NativeFlowResult result;

  std::filesystem::create_directories(outDir);
  std::filesystem::create_directories("logs");
  OPDB::initLogger();

  const auto loadStart = SteadyClock::now();
  LefYml lef(lefPath.string());
  auto   library = lef.toLibrary();
  if (!library) {
    std::cerr << "library load failed\n";
    result.exitCode = 3;
    return result;
  }

  DefYml def(defPath.string());
  auto   design = def.toDesign(library);
  if (!design) {
    std::cerr << "design load failed\n";
    result.exitCode = 4;
    return result;
  }
  design->setRoutingRules(options.rules);
  printTiming("timing_cpp_load_design_s", SteadyClock::now() - loadStart);

  const auto drcConfig   = makeDrcConfig(design->getRoutingRules());
  const auto routeConfig = makeRouteConfig(design->getRoutingRules());

  result.flowSummaryPath = outDir / "lidar_grid_route_flow_summary.txt";
  result.routeResultPath = outDir / "lidar_route_result.yml";

  const auto runtimeInitStart = SteadyClock::now();
  auto runtimeView =
      picpr::lidar::buildRuntimeViewFromDesign(*design, options.viewOptions);
  picpr::lidar::LidarDrcManager runtimeDrc(runtimeView, drcConfig);
  runtimeDrc.initDRC();
  runtimeDrc.initPorts();
  printTiming("timing_cpp_runtime_init_s",
              SteadyClock::now() - runtimeInitStart);

  const auto routeStart = SteadyClock::now();
  const auto flow = picpr::lidar::routeAllNetsGrid(
      runtimeView, runtimeDrc, routeConfig);
  printTiming("timing_cpp_route_core_s", SteadyClock::now() - routeStart);

  const auto writeReportsStart = SteadyClock::now();
  {
    std::ofstream flowSummary(result.flowSummaryPath);
    picpr::lidar::writeGridRouteFlowSummary(flow, flowSummary);
  }
  {
    std::ofstream routeResult(result.routeResultPath);
    picpr::lidar::writeGridRouteResultYml(
        runtimeView, routeConfig, flow, routeResult);
  }
  printTiming("timing_cpp_write_reports_s",
              SteadyClock::now() - writeReportsStart);

  const auto writebackStart = SteadyClock::now();
  const auto writeback = picpr::lidar::writeRoutedGridToDesign(
      *design, runtimeView, routeConfig, flow.success);
  printTiming("timing_cpp_writeback_s", SteadyClock::now() - writebackStart);
  result.routeSuccess = writeback.success;
  result.routedNets = writeback.routedNets;
  result.waveguideSegments = writeback.waveguideSegments;
  result.skippedNets = writeback.skippedNets;
  result.postprocessInvalidAccesses = writeback.postprocessInvalidAccesses;
  result.postprocessRejectedNets = writeback.postprocessRejectedNets;

  result.routedDefPath = outDir / "routed_def.yml";
  const auto writeDefStart = SteadyClock::now();
  DefYml::fromDesignToYml(*design, result.routedDefPath.string());
  printTiming("timing_cpp_write_def_s", SteadyClock::now() - writeDefStart);

  result.dbDrcSummaryPath = outDir / "db_drc_summary.txt";
  const auto dbDrcStart = SteadyClock::now();
  writeDbDrcSummary(*design, result.dbDrcSummaryPath, options.requireRoutedNets);
  result.dbDrcClean = readCleanDrcSummary(result.dbDrcSummaryPath);
  printTiming("timing_cpp_db_drc_s", SteadyClock::now() - dbDrcStart);

  std::cout << "route_success=" << (writeback.success ? "true" : "false")
            << "\n";
  std::cout << "routed_nets=" << writeback.routedNets << "\n";
  std::cout << "waveguide_segments=" << writeback.waveguideSegments << "\n";
  std::cout << "skipped_nets=" << writeback.skippedNets << "\n";
  std::cout << "postprocess_invalid_accesses="
            << writeback.postprocessInvalidAccesses << "\n";
  std::cout << "postprocess_rejected_nets="
            << writeback.postprocessRejectedNets << "\n";
  std::cout << "flow_summary=" << result.flowSummaryPath.string() << "\n";
  std::cout << "route_result=" << result.routeResultPath.string() << "\n";
  std::cout << "routed_def=" << result.routedDefPath.string() << "\n";
  std::cout << "db_drc_summary=" << result.dbDrcSummaryPath.string() << "\n";
  std::cout << "db_drc_clean=" << (result.dbDrcClean ? "true" : "false") << "\n";
  printTiming("timing_cpp_native_flow_s",
              SteadyClock::now() - nativeFlowStart);

  if (!result.dbDrcClean && options.failOnDbDrc) {
    result.exitCode = 11;
    return result;
  }
  if (!writeback.success && options.failOnAbnormal) {
    result.exitCode = 10;
    return result;
  }
  result.exitCode = 0;
  return result;
}

int runLegacyPicDbMode(int argc, char** argv)
{
  if (argc < 4) {
    printUsage();
    return 2;
  }

  const std::filesystem::path lefPath = argv[1];
  const std::filesystem::path defPath = argv[2];
  const std::filesystem::path outDir  = argv[3];

  NativeFlowOptions options;
  options.viewOptions.preserveOriginalNetNames    = true;
  options.viewOptions.snapNearIntegerCoordinates  = true;

  bool allowAbnormal = false;
  for (int i = 4; i < argc; ++i) {
    const std::string arg = argv[i];
    if (!applyNativeFlowOption(arg, options, allowAbnormal)) {
      std::cerr << "unknown option: " << arg << "\n";
      printUsage();
      return 2;
    }
  }
  options.failOnAbnormal = !allowAbnormal;

  return runNativeFlow(lefPath, defPath, outDir, options).exitCode;
}

int runLidarFullFlowMode(const std::filesystem::path& lidarYml,
                         const std::filesystem::path& outGds,
                         const NativeFlowOptions& flowOptions)
{
  const auto fullFlowStart = SteadyClock::now();
  const auto scriptDir = bundledScriptDir();
  const auto converter = scriptDir / "lidar_yml_to_picdb_yml.py";
  const auto renderer = scriptDir / "render_route_result_gds.py";
  if (!std::filesystem::exists(converter) || !std::filesystem::exists(renderer)) {
    std::cerr << "bundled scripts not found under "
              << scriptDir.string() << "\n";
    return 5;
  }

  const auto lidarSrc = inferLidarSource(lidarYml);
  if (lidarSrc.empty()) {
    std::cerr << "could not infer LiDAR source root. Set PICDB_LIDAR_SRC to "
                 "the directory that contains picroute/.\n";
    return 6;
  }

  const auto python = std::filesystem::path(getEnvOrDefault("PICDB_PYTHON", "python"));
  const auto outGdsAbs = std::filesystem::absolute(outGds);
  const auto flowRoot =
      outGdsAbs.parent_path() / (outGdsAbs.stem().string() + "_picdb_flow");
  const auto convertedDir = flowRoot / "converted";
  const auto cppDir = flowRoot / "cpp";
  std::filesystem::create_directories(convertedDir);
  std::filesystem::create_directories(cppDir);
  std::filesystem::create_directories(outGdsAbs.parent_path());

  const auto convertStart = SteadyClock::now();
  const auto convertStatus = runCommand(
      quoteExecutableForShell(python) + " " +
      quoteForShell(converter) + " " +
      quoteForShell(std::filesystem::absolute(lidarYml)) + " " +
      quoteForShell(convertedDir));
  printTiming("timing_lidar_convert_s", SteadyClock::now() - convertStart);
  if (convertStatus != 0) {
    std::cerr << "LiDAR YAML conversion failed with status "
              << convertStatus << "\n";
    return 7;
  }

  NativeFlowOptions options = flowOptions;
  options.failOnAbnormal = false;
  options.failOnDbDrc = false;
  options.requireRoutedNets = true;

  const auto nativeResult = runNativeFlow(convertedDir / "converted_lef.yml",
                                         convertedDir / "converted_def.yml",
                                         cppDir,
                                         options);
  if (nativeResult.exitCode != 0) {
    return nativeResult.exitCode;
  }

  const auto invalidAccessReport = cppDir / "render_invalid_access.txt";
  const auto renderStart = SteadyClock::now();
  const auto renderStatus = runCommand(
      quoteExecutableForShell(python) + " " +
      quoteForShell(renderer) + " " +
      quoteForShell(lidarSrc) + " " +
      quoteForShell(nativeResult.routeResultPath) + " " +
      quoteForShell(outGdsAbs) + " " +
      "--base-lidar-yml " +
      quoteForShell(convertedDir / "converted_lidar.yml") + " " +
      "--skip-invalid-access --skip-abnormal-nets " +
      "--invalid-access-report " + quoteForShell(invalidAccessReport));
  printTiming("timing_lidar_render_s", SteadyClock::now() - renderStart);
  if (renderStatus != 0) {
    std::cerr << "GDS render failed with status " << renderStatus << "\n";
    return 8;
  }

  std::cout << "input_lidar_yml=" << std::filesystem::absolute(lidarYml).string()
            << "\n";
  std::cout << "lidar_src=" << lidarSrc.string() << "\n";
  std::cout << "output_gds=" << outGdsAbs.string() << "\n";
  std::cout << "flow_dir=" << flowRoot.string() << "\n";
  std::cout << "converted_lef=" << (convertedDir / "converted_lef.yml").string()
            << "\n";
  std::cout << "converted_def=" << (convertedDir / "converted_def.yml").string()
            << "\n";
  std::cout << "converted_lidar=" << (convertedDir / "converted_lidar.yml").string()
            << "\n";
  std::cout << "render_invalid_access_report="
            << invalidAccessReport.string() << "\n";
  printTiming("timing_lidar_full_flow_s", SteadyClock::now() - fullFlowStart);
  return 0;
}

std::string lowerExtension(const std::filesystem::path& path)
{
  std::string extension = path.extension().string();
  for (char& ch : extension) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  return extension;
}

std::vector<std::string> translatePicBenchFlowArgs(
    const std::vector<std::string>& args)
{
  std::vector<std::string> translated;
  for (const auto& arg : args) {
    if (arg.rfind("--net-order=", 0) == 0) {
      translated.push_back("--route-net-order="
                           + arg.substr(std::string("--net-order=").size()));
    } else if (arg.rfind("--net-default-bound=", 0) == 0) {
      translated.push_back(
          "--route-net-default-bound="
          + arg.substr(std::string("--net-default-bound=").size()));
    } else if (arg.rfind("--grid-resolution=", 0) == 0) {
      translated.push_back(
          "--route-grid-resolution="
          + arg.substr(std::string("--grid-resolution=").size()));
    } else if (arg.rfind("--max-iteration=", 0) == 0) {
      translated.push_back(
          "--route-max-iterations="
          + arg.substr(std::string("--max-iteration=").size()));
    } else if (arg == "--route-group" || arg == "--no-route-group") {
      translated.push_back(arg);
    } else if (arg == "--enable-45-neighbor") {
      translated.push_back("--route-enable-45-neighbor");
    } else if (arg == "--disable-45-neighbor") {
      translated.push_back("--no-route-enable-45-neighbor");
    } else if (arg == "--deterministic-order"
               || arg == "--no-preserve-net-names"
               || arg == "--no-snap-near-integer"
               || arg == "--allow-abnormal") {
      // These are native-router debug options. The PICBench bridge calls the
      // native router in full-flow mode, where they are not needed.
    } else {
      translated.push_back(arg);
    }
  }
  return translated;
}

int runPicBenchJsonFlowMode(const std::filesystem::path& refJson,
                            const std::filesystem::path& outGds,
                            const std::filesystem::path& nativeExecutable,
                            const std::vector<std::string>& userArgs)
{
  const auto scriptDir = bundledPicBenchFlowDir();
  const auto bridge = scriptDir / "run_picdb_dreamplace_lidar_flow.py";
  if (!std::filesystem::exists(bridge)) {
    std::cerr << "PICBench bridge script not found under "
              << scriptDir.string() << "\n";
    return 9;
  }

  const auto python = std::filesystem::path(getEnvOrDefault("PICDB_PYTHON", "python"));
  const auto outGdsAbs = std::filesystem::absolute(outGds);
  const auto flowRoot =
      outGdsAbs.parent_path() / (outGdsAbs.stem().string() + "_picbench_flow");
  std::filesystem::create_directories(outGdsAbs.parent_path());
  std::filesystem::create_directories(flowRoot);

  std::string command =
      quoteExecutableForShell(python) + " " +
      quoteForShell(bridge) + " " +
      "--ref-json " + quoteForShell(std::filesystem::absolute(refJson)) + " " +
      "--output-gds " + quoteForShell(outGdsAbs) + " " +
      "--output-root " + quoteForShell(flowRoot) + " " +
      "--router cpp " +
      "--native-lidar " + quoteForShell(nativeExecutable);
  for (const auto& arg : translatePicBenchFlowArgs(userArgs)) {
    command += " " + arg;
  }

  const auto status = runCommand(command);
  if (status != 0) {
    std::cerr << "PICBench bridge failed with status " << status << "\n";
    return 9;
  }
  std::cout << "input_picbench_json="
            << std::filesystem::absolute(refJson).string() << "\n";
  std::cout << "output_gds=" << outGdsAbs.string() << "\n";
  std::cout << "picbench_flow_dir=" << flowRoot.string() << "\n";
  return 0;
}

bool isGdsOutputPath(const std::filesystem::path& path)
{
  return lowerExtension(path) == ".gds";
}

bool isJsonInputPath(const std::filesystem::path& path)
{
  return lowerExtension(path) == ".json";
}

bool isYamlPath(const std::filesystem::path& path)
{
  const auto extension = lowerExtension(path);
  return extension == ".yml" || extension == ".yaml";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc >= 3 && isGdsOutputPath(argv[2])) {
    std::vector<std::string> userArgs;
    for (int i = 3; i < argc; ++i) {
      userArgs.emplace_back(argv[i]);
    }
    if (isJsonInputPath(argv[1])) {
      return runPicBenchJsonFlowMode(
          argv[1], argv[2], std::filesystem::absolute(argv[0]), userArgs);
    }

    NativeFlowOptions options;
    options.viewOptions.preserveOriginalNetNames    = true;
    options.viewOptions.snapNearIntegerCoordinates  = true;
    bool allowAbnormal = false;
    for (const auto& arg : userArgs) {
      if (!applyNativeFlowOption(arg, options, allowAbnormal)) {
        std::cerr << "unknown option: " << arg << "\n";
        printUsage();
        return 2;
      }
    }
    return runLidarFullFlowMode(argv[1], argv[2], options);
  }
  if (argc >= 3 && isYamlPath(argv[1]) && isYamlPath(argv[2])
      && std::filesystem::exists(argv[1]) && std::filesystem::exists(argv[2])) {
    std::vector<std::string> legacyArgs = {
        argv[0],
        argv[1],
        argv[2],
        (std::filesystem::current_path()
         / (std::filesystem::path(argv[2]).stem().string()
            + "_native_legacy")).string()};
    for (int i = 3; i < argc; ++i) {
      legacyArgs.emplace_back(argv[i]);
    }
    std::vector<char*> legacyArgv;
    legacyArgv.reserve(legacyArgs.size());
    for (auto& arg : legacyArgs) {
      legacyArgv.push_back(arg.data());
    }
    return runLegacyPicDbMode(static_cast<int>(legacyArgv.size()),
                              legacyArgv.data());
  }
  if (argc >= 4) {
    return runLegacyPicDbMode(argc, argv);
  }
  printUsage();
  return 2;
}
