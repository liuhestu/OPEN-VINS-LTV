# EuRoC launch batch results

Batch: `20261105T211124`. 50/50 completed successfully; 0/50 failed.

`MH_04_difficult` was intentionally skipped. All other 10 sequences were attempted in OFF, G, V, L, GVL order.

Use only rows marked `passed` in `summary.csv` as complete trajectories. Failed runs may contain partial TUM files retained for diagnosis.

`trajectory.tum` uses eight TUM columns: `timestamp tx ty tz qx qy qz qw`. Original covariance-bearing recordings are retained in `trajectory_raw.txt`.

Playback rate: 1.0; offset: 0; three isolated ROS domains. Weights and launch behavior were kept unchanged. Retried runs use the repaired pause lifecycle; earlier successful runs retain their original metadata. No APE/RPE evaluation was performed.

| Sequence | OFF | G | V | L | GVL |
|---|---|---|---|---|---|
| MH_01_easy | [passed](MH_01_easy/OFF/trajectory.tum) | [passed](MH_01_easy/G/trajectory.tum) | [passed](MH_01_easy/V/trajectory.tum) | [passed](MH_01_easy/L/trajectory.tum) | [passed](MH_01_easy/GVL/trajectory.tum) |
| MH_02_easy | [passed](MH_02_easy/OFF/trajectory.tum) | [passed](MH_02_easy/G/trajectory.tum) | [passed](MH_02_easy/V/trajectory.tum) | [passed](MH_02_easy/L/trajectory.tum) | [passed](MH_02_easy/GVL/trajectory.tum) |
| MH_03_medium | [passed](MH_03_medium/OFF/trajectory.tum) | [passed](MH_03_medium/G/trajectory.tum) | [passed](MH_03_medium/V/trajectory.tum) | [passed](MH_03_medium/L/trajectory.tum) | [passed](MH_03_medium/GVL/trajectory.tum) |
| MH_05_difficult | [passed](MH_05_difficult/OFF/trajectory.tum) | [passed](MH_05_difficult/G/trajectory.tum) | [passed](MH_05_difficult/V/trajectory.tum) | [passed](MH_05_difficult/L/trajectory.tum) | [passed](MH_05_difficult/GVL/trajectory.tum) |
| V1_01_easy | [passed](V1_01_easy/OFF/trajectory.tum) | [passed](V1_01_easy/G/trajectory.tum) | [passed](V1_01_easy/V/trajectory.tum) | [passed](V1_01_easy/L/trajectory.tum) | [passed](V1_01_easy/GVL/trajectory.tum) |
| V1_02_medium | [passed](V1_02_medium/OFF/trajectory.tum) | [passed](V1_02_medium/G/trajectory.tum) | [passed](V1_02_medium/V/trajectory.tum) | [passed](V1_02_medium/L/trajectory.tum) | [passed](V1_02_medium/GVL/trajectory.tum) |
| V1_03_difficult | [passed](V1_03_difficult/OFF/trajectory.tum) | [passed](V1_03_difficult/G/trajectory.tum) | [passed](V1_03_difficult/V/trajectory.tum) | [passed](V1_03_difficult/L/trajectory.tum) | [passed](V1_03_difficult/GVL/trajectory.tum) |
| V2_01_easy | [passed](V2_01_easy/OFF/trajectory.tum) | [passed](V2_01_easy/G/trajectory.tum) | [passed](V2_01_easy/V/trajectory.tum) | [passed](V2_01_easy/L/trajectory.tum) | [passed](V2_01_easy/GVL/trajectory.tum) |
| V2_02_medium | [passed](V2_02_medium/OFF/trajectory.tum) | [passed](V2_02_medium/G/trajectory.tum) | [passed](V2_02_medium/V/trajectory.tum) | [passed](V2_02_medium/L/trajectory.tum) | [passed](V2_02_medium/GVL/trajectory.tum) |
| V2_03_difficult | [passed](V2_03_difficult/OFF/trajectory.tum) | [passed](V2_03_difficult/G/trajectory.tum) | [passed](V2_03_difficult/V/trajectory.tum) | [passed](V2_03_difficult/L/trajectory.tum) | [passed](V2_03_difficult/GVL/trajectory.tum) |

Reused successful runs from the original batch: 25. Earlier failed outputs were overwritten as requested.

Failure causes:


Configuration snapshots, checksums, batch runner, status and independent audit: `_batch/20261105T211124/`.

Independent audit errors: 0.
