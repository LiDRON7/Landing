# Filter Sensitivity Analysis

## Scope
This report is based on simulation runtime logs from the full LiDAR preprocessing pipeline.
Only configurations with directly provided evidence are included.

## Baseline Test (Full Pipeline)
Observed repeated runtime log line:

- Stage counts | Raw input: 10240 | After NaN removal: 5388 | After Voxel Grid Downsampling: 735 | After ROI Filtering: 735 | After Statistical Outlier: 719

Derived baseline metrics:

- Points in: 10240
- Points out: 719
- Removed: 9521
- Total reduction: 92.98%
- Retention: 7.02%

## Baseline Stage-by-Stage Impact

- Raw -> After NaN: removed 4852 points (47.38% of raw)
- After NaN -> After Voxel: removed 4653 points (86.36% of NaN-cleaned cloud)
- After Voxel -> After ROI: removed 0 points (0.00%)
- After ROI -> After Outlier: removed 16 points (2.18% of ROI output)

Interpretation:

- NaN and Voxel are the dominant reducers in this simulation run.
- ROI is currently non-constraining for this scene.
- Outlier filtering is active but has a small marginal effect.
- Repeated identical counts indicate stable behavior for this fixed scenario.

## Summary Table

| Phase | Configuration | Raw | After NaN | After Voxel | After ROI | After Outlier | Retention (%) |
|---|---|---:|---:|---:|---:|---:|---:|
| Baseline | simulation_full_pipeline_current_config | 10240 | 5388 | 735 | 735 | 719 | 7.02 |
| Individual (NaN) | nan_only_enabled_others_disabled | 10240 | 5388 | 5388 | 5388 | 5388 | 52.62 |
| Individual (NaN) | nan_only_disabled_control | 10240 | 10240 | 10240 | 10240 | 10240 | 100.00 |
| Individual (ROI) | roi_only_default_bounds | 10240 | 10240 | 10240 | 5388 | 5388 | 52.62 |
| Individual (ROI) | roi_only_tight_bounds | 10240 | 10240 | 10240 | 4778 | 4778 | 46.66 |
| Individual (Outlier) | outlier_only_strict | 10240 | 10240 | 10240 | 10240 | 4778 | 46.66 |
| Individual (Outlier) | outlier_only_relaxed | 10240 | 10240 | 10240 | 10240 | 5248 | 51.25 |
| Individual (Voxel) | voxel_only_size_0_10_nan_disabled | 10240 | 10240 | 735 | 735 | 735 | 7.18 |
| Individual (Voxel) | voxel_only_size_0_50_nan_disabled | 10240 | 10240 | 128 | 128 | 128 | 1.25 |

## Individual Filter Evidence

From the provided runtime logs after restarting the node:

- Configuration: NaN filter enabled, Voxel/ROI/Outlier disabled.
- Stage counts: Raw 10240 -> After NaN 5388 -> After Voxel 5388 -> After ROI 5388 -> After Outlier 5388.
- Reduction: 47.38% (retention 52.62%).

- Configuration: NaN filter disabled, Voxel/ROI/Outlier disabled.
- Stage counts: Raw 10240 -> After NaN 10240 -> After Voxel 10240 -> After ROI 10240 -> After Outlier 10240.
- Reduction: 0.00% (retention 100.00%).

- Configuration: ROI filter enabled with default bounds (x/y: -10 to 10, z: -2 to 5), NaN/Voxel/Outlier disabled.
- Stage counts: Raw 10240 -> After NaN 10240 -> After Voxel 10240 -> After ROI 5388 -> After Outlier 5388.
- Reduction: 47.38% (retention 52.62%).

- Configuration: ROI filter enabled with tighter bounds (x/y: -2 to 2, z: -1 to 2), NaN/Voxel/Outlier disabled.
- Stage counts: Raw 10240 -> After NaN 10240 -> After Voxel 10240 -> After ROI 4778 -> After Outlier 4778.
- Reduction: 53.34% (retention 46.66%).

- Configuration: Outlier filter enabled with meanK=20 and threshold=1.0, NaN/Voxel/ROI disabled.
- Stage counts: Raw 10240 -> After NaN 10240 -> After Voxel 10240 -> After ROI 10240 -> After Outlier 4778.
- Reduction: 53.34% (retention 46.66%).

- Configuration: Outlier filter enabled with meanK=50 and threshold=3.0, NaN/Voxel/ROI disabled.
- Stage counts: Raw 10240 -> After NaN 10240 -> After Voxel 10240 -> After ROI 10240 -> After Outlier 5248.
- Reduction: 48.75% (retention 51.25%).

- Configuration: voxel-only with `voxel.size = 0.10`, NaN/ROI/Outlier disabled.
- Stage counts: Raw 10240 -> After NaN 10240 -> After Voxel 735 -> After ROI 735 -> After Outlier 735.
- Reduction: 92.82% (retention 7.18%).

- Configuration: voxel-only with `voxel.size = 0.50`, NaN/ROI/Outlier disabled.
- Stage counts: Raw 10240 -> After NaN 10240 -> After Voxel 128 -> After ROI 128 -> After Outlier 128.
- Reduction: 98.75% (retention 1.25%).

Interpretation:

- Voxel sensitivity is clearly nonlinear in this scene: increasing voxel size from 0.10 to 0.50 drops retained points from 735 to 128.
- A voxel leaf size of 0.50 is overly aggressive for this scene and collapses spatial detail heavily.
- A voxel size near 0.10 retains substantially more structure than 0.50 while still reducing load.
- NaN-only isolation shows the NaN filter alone removes 47.38% of the raw cloud.
- NaN-disabled control confirms the filter is the only source of NaN-stage reduction in this pair.
- ROI-only isolation with default bounds removes 47.38% when applied to the full raw cloud, but contributes 0.00% in the baseline sequence after Voxel due to prior reduction.
- ROI tight-vs-default comparison shows tighter bounds remove an additional 610 points (53.34% vs 47.38%).
- Outlier-only strict settings remove 53.34% of the raw cloud on this scene, which is stronger than the baseline outlier stage after voxelization.
- Outlier-only relaxed settings remove 48.75% of the raw cloud on this scene, which is less aggressive than the strict outlier run and closer to the baseline outlier stage behavior.

## Filters That May Remove Too Many Points

- Total removal is 92.98%, which is very aggressive.
- The largest drops occur at NaN removal and Voxel downsampling.
- For navigation/perception tasks sensitive to point density, this may under-represent small or distant obstacles.
- Voxel-only at size 0.50 removed 98.75%, which is too aggressive for most obstacle-aware tasks.

## Parameter Recommendations

- NaN filter: keep enabled, but validate whether high NaN rate is expected sensor behavior or bridge/format artifact.
- NaN filter: this run shows the filter is doing meaningful work, but it is not excessively aggressive on its own because it preserves 52.62% of the cloud.
- NaN filter: the disabled control preserves 100.00% of the cloud, so the enabled setting is justified if invalid points are expected.
- Voxel Grid: avoid very large leaves like 0.50 for this scenario. Current evidence supports using around 0.10 as a practical starting point, then sweeping 0.05-0.12 for final trade-off tuning.
- ROI filter: default bounds remove 47.38% and tighter bounds remove 53.34% in isolation. Use tighter bounds only when stronger workspace pruning is desired, and tune ROI jointly with Voxel because baseline ROI stage was 0.00% after Voxel.
- Statistical Outlier: strict settings (meanK=20, threshold=1.0) remove 53.34% in isolation, while the relaxed default settings (meanK=50, threshold=3.0) remove 48.75%. Use the relaxed pair as the safer starting point and reserve the strict pair for heavier pruning.

 