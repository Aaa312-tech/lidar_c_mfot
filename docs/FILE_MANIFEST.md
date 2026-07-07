# File Manifest

This manifest lists the important files in `lidar_c_inovation_2`.

## Dependency locks

| File | Purpose |
|---|---|
| `.gitignore` | Keeps build outputs, virtual environments, caches, and local standard GDS files out of Git. |
| `requirements-gds-render.txt` | Python packages used by conversion, validation, and final GDS rendering. |
| `requirements-python-lidar-original.txt` | Optional packages for running the original Python LiDAR baseline wrapper. |

## Core C++ module

| File | Purpose |
|---|---|
| `code/src/algorithm/routing/lidar/CMakeLists.txt` | Builds the LiDAR routing library inside PIC-DB. |
| `code/src/algorithm/routing/lidar/include/lidar_astar.h` | A* router interface and route data structures. |
| `code/src/algorithm/routing/lidar/include/lidar_bitmap.h` | Bitmap occupancy model. |
| `code/src/algorithm/routing/lidar/include/lidar_drc.h` | Runtime DRC manager interface. |
| `code/src/algorithm/routing/lidar/include/lidar_mfot.h` | MFOT global plan data structures and API. |
| `code/src/algorithm/routing/lidar/include/lidar_router.h` | Top-level route API. |
| `code/src/algorithm/routing/lidar/include/picdb_lidar_view.h` | PIC-DB to LiDAR runtime view adapter. |
| `code/src/algorithm/routing/lidar/src/lidar_astar.cpp` | A*, crossing-aware routing, rip-up/reroute, post-process, writeback. |
| `code/src/algorithm/routing/lidar/src/lidar_bitmap.cpp` | Bitmap allocation and blockage initialization. |
| `code/src/algorithm/routing/lidar/src/lidar_drc.cpp` | DRC checks, port spreading, bitmap updates. |
| `code/src/algorithm/routing/lidar/src/lidar_mfot.cpp` | MFOT transport/free-energy planning, priority, and search-weight helpers. |
| `code/src/algorithm/routing/lidar/src/lidar_python_set.cpp` | Deterministic Python-like set behavior. |
| `code/src/algorithm/routing/lidar/src/lidar_router.cpp` | Flow summaries and top-level routing orchestration. |
| `code/src/algorithm/routing/lidar/src/picdb_lidar_view.cpp` | Builds runtime view from PIC-DB design. |

## Native executable

| File | Purpose |
|---|---|
| `code/tools/pr_lidar_native/CMakeLists.txt` | Builds `pr_lidar_native.exe`. |
| `code/tools/pr_lidar_native/main.cpp` | CLI entry. Supports LiDAR YAML full-flow and legacy PIC-DB mode. |
| `code/tools/pr_lidar_native/README.md` | Original native tool notes. |

## Python bridge and validation scripts

| File | Purpose |
|---|---|
| `code/tools/pr_lidar_native/scripts/lidar_yml_to_picdb_yml.py` | Converts LiDAR benchmark YAML to PIC-DB intermediate LEF/DEF-style YAML. |
| `code/tools/pr_lidar_native/scripts/render_route_result_gds.py` | Renders C++ route result to GDS using gdsfactory/kfactory. |
| `code/tools/pr_lidar_native/scripts/run_lidar_benchmark_regression.py` | Runs multiple benchmark cases and collects timing/DRC/GDS metrics. |
| `code/tools/pr_lidar_native/scripts/run_python_lidar_original.py` | Runs original Python LiDAR and can dump route traces. |
| `code/tools/pr_lidar_native/scripts/compare_gds_geometry.py` | Compares standard/Python/C++ GDS using layer area and XOR. |
| `code/tools/pr_lidar_native/scripts/compare_lidar_net_metrics.py` | Compares route/net-level metrics. |
| `code/tools/pr_lidar_native/scripts/gdsfactory_adapters.py` | Registers missing/custom gdsfactory adapters used by benchmarks. |

## PICBench flow bridge

| File | Purpose |
|---|---|
| `code/tools/picbench_flow/run_picdb_dreamplace_lidar_flow.py` | DREAMPlace/PICBench to LiDAR flow bridge. |
| `code/tools/picbench_flow/gdsfactory_adapters.py` | Adapter helpers for PICBench gdsfactory cells. |
| `code/tools/picbench_flow/README.md` | Original flow documentation. |

## Configs

| Path | Purpose |
|---|---|
| `code/configs/pr_lidar/route_config/comp_LiDAR.yml` | LiDAR route config. |
| `code/configs/pr_lidar/place_config/*.yml` | Placement config entry points. |
| `code/configs/pr_lidar/place_config/*.json` | Placement JSON configs. |

## Benchmarks

| Path | Purpose |
|---|---|
| `code/benchmarks/picroute/toy_example/` | Small smoke benchmark. |
| `code/benchmarks/picroute/mrr_weight_bank_4x4/` | MRR 4x4 benchmark. |
| `code/benchmarks/picroute/mrr_weight_bank_8x8/` | MRR 8x8 benchmark. |
| `code/benchmarks/picroute/mrr_weight_bank_16x16/` | MRR 16x16 benchmark. |
| `code/benchmarks/picroute/clements_8x8/` | Clements 8x8 benchmark. |
| `code/benchmarks/picroute/clements_16x16/` | Clements 16x16 benchmark. |
| `code/benchmarks/picroute/multiportmmi_8x8/` | Multiport MMI 8x8 benchmark. |
| `code/benchmarks/picroute/multiportmmi_16x16/` | Multiport MMI 16x16 benchmark. |
| `code/benchmarks/picroute/multiportmmi_32x32/` | Multiport MMI 32x32 benchmark. |

These are input benchmarks, not standard routed GDS files.

## Results

| Path | Purpose |
|---|---|
| `results/reference_run/*.gds` | Archived C++ GDS outputs for 9 benchmark cases. |
| `results/reference_run/reference_run.csv` | Archived regression summary. |
| `results/reference_run/reference_run.json` | Archived regression summary in JSON. |
| `results/mfot_only_control_fullcase_20260706_115037/` | Latest MFOT-only control regression outputs and process artifacts. |
| `results/mfot_only_control_fullcase_20260706_115037/mfot_only_control_fullcase_20260706_115037.csv` | Latest MFOT-only 9-case regression summary. |
| `results/reference_gds_compare/gds_pair_summary.csv` | Standard/Python/C++ GDS pair comparisons. |
| `results/reference_gds_compare/gds_layer_xor.csv` | Layer-by-layer XOR. |
| `results/reference_gds_compare/gds_xor_hotspots.csv` | XOR hotspot report. |
| `results/README.md` | Explains archived results and sanitized paths. |
| `standard_gds/README.md` | Placeholder note for external standard GDS files, which are not included. |

## Documentation

| File | Purpose |
|---|---|
| `README.md` | Main usage and current status. |
| `docs/CURRENT_RESULTS.md` | Current included GDS results and standard-GDS comparison. |
| `docs/METHODOLOGY.md` | Methodology for building/converting a high-quality router. |
| `docs/ALGORITHM_CHANGES_AND_INNOVATIONS.md` | Algorithmic changes, engineering innovations, and what was not hardcoded. |
| `docs/EXPERIENCE_AND_TROUBLESHOOTING.md` | Lessons learned and debugging playbook. |
| `docs/ENVIRONMENT.md` | C++ and Python environment dependencies. |
| `docs/TRANSFER_GUIDE.md` | How to move this package into another PIC-DB tree. |

## Helper tools

| File | Purpose |
|---|---|
| `tools/merge_into_picdb.ps1` | Copies packaged code into a PIC-DB source tree. |
| `tools/run_all_cases.ps1` | Runs the default 9-case regression in a PIC-DB tree. |
