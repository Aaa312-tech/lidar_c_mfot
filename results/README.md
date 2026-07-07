# Results

This directory contains archived outputs from the current C++ LiDAR regression.

```text
reference_run/
reference_gds_compare/
mfot_only_control_fullcase_20260706_115037/
```

`reference_run` contains generated C++ GDS files for the default 9 benchmark
cases plus CSV/JSON summaries. The paths inside the summaries are sanitized to
portable relative paths.

`reference_gds_compare` contains geometry comparison reports against external
standard GDS files. The standard files themselves are not included; report paths
use the placeholder `standard_gds/`.

`mfot_only_control_fullcase_20260706_115037` contains the latest synchronized
MFOT-only control run. It preserves the `lidar_c` common routing baseline and
keeps only the MFOT plan plus A* heuristic-weighting changes. The directory
includes CSV/JSON summaries, generated GDS files, stdout/stderr logs, DB DRC
summaries, and per-case `lidar_route_result.yml` process artifacts.

These files are reference artifacts. New exploratory runs should normally write
to a build or checks directory inside the host PIC-DB checkout unless the run is
being intentionally archived with the package.
