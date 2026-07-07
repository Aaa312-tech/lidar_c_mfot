# Current Results

This document records the latest generated C++ LiDAR GDS results included in this package.

The current snapshot is the MFOT-only control version. It keeps the common
router behavior synchronized with `lidar_c` and preserves only the minimal MFOT
planning and A* heuristic-weighting hooks.

## Output directory

```text
results/mfot_only_control_fullcase_20260706_115037/
```

## Generated GDS files

```text
toy_example_gp_cpp.gds
mrr_weight_bank_4x4_cpp.gds
mrr_weight_bank_8x8_cpp.gds
mrr_weight_bank_16x16_cpp.gds
clements_8x8_cpp.gds
clements_16x16_cpp.gds
multiportmmi_8x8_cpp.gds
multiportmmi_16x16_cpp.gds
multiportmmi_32x32_cpp.gds
```

## Regression summary

| case | status | DRC clean | markers | routes | crossings | C++ core route time |
|---|---|---:|---:|---:|---:|---:|
| toy_example_gp | ok | 0 | 1 | 2 | 0 | 0.049993s |
| mrr_weight_bank_4x4 | ok | 1 | 0 | 42 | 6 | 0.811489s |
| mrr_weight_bank_8x8 | ok | 0 | 8 | 105 | 6 | 1.210653s |
| mrr_weight_bank_16x16 | ok | 0 | 90 | 374 | 30 | 7.310558s |
| clements_8x8 | ok | 1 | 0 | 79 | 0 | 0.702268s |
| clements_16x16 | ok | 1 | 0 | 290 | 2 | 5.454614s |
| multiportmmi_8x8 | ok | 1 | 0 | 177 | 33 | 10.190470s |
| multiportmmi_16x16 | ok | 1 | 0 | 349 | 63 | 49.225611s |
| multiportmmi_32x32 | ok | 0 | 1 | 694 | 124 | 113.991247s |

Notes:

```text
toy_example_gp has a known input component overlap marker.
mrr_weight_bank_8x8 and mrr_weight_bank_16x16 still have route geometry markers.
multiportmmi_32x32 has 1 route geometry marker after removing the earlier
extra repair/fallback logic. This is expected for the MFOT-only control run and
confirms that prior repair/fallback changes affected quality.
```

## Comparison to archived original lidar_c reference

| metric | archived lidar_c reference | MFOT-only control |
|---|---:|---:|
| clean cases | 6/9 | 5/9 |
| total markers | 119 | 100 |
| total crossings | 263 | 264 |
| total routes | 2108 | 2112 |
| total routed length | 596848.827 | 596156.786 |
| total C++ route core time | 969.108s | 188.947s |
| total wall time | 1941.114s | 535.443s |

The current MFOT-only effect is mainly A* heuristic weighting based on the
global MFOT plan. Corridor cost, history seeding, hard corridor constraints,
and global priority reordering are disabled by default.

The previous archived reference results remain available in:

```text
results/reference_run/
results/reference_gds_compare/
```

## Standard-GDS comparison

The following comparisons use the three user-provided standard GDS files.

| case | standard XOR | overlap ratio | conclusion |
|---|---:|---:|---|
| clements_8x8 | 0.000000 | 1.000000000 | exact geometry match |
| multiportmmi_8x8 | 5.091752 | 0.999978471 | visually almost identical, tiny crossing-area difference remains |
| multiportmmi_16x16 | 18.680864 | 0.999969043 | visually almost identical, tiny crossing-area difference remains |

Detailed files:

```text
results/reference_gds_compare/gds_pair_summary.csv
results/reference_gds_compare/gds_layer_xor.csv
results/reference_gds_compare/gds_xor_hotspots.csv
```

## Important interpretation

The generated C++ GDS files are not copied from the standard GDS files.

Evidence:

```text
file sizes differ
SHA256 hashes differ
MMI cell counts differ
standard GDS files are only referenced by compare_gds_geometry.py
```

`clements_8x8` is visually identical because it is truly 0 XOR. The two MMI cases look identical in a viewer because their overlap ratios are above 0.99996, but they are not mathematically exact.
