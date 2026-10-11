# 实验过程与归档资料

`docs/` 顶层保留四份结果报告。本目录集中保存支撑报告的配置、逐项数据、验证记录、研究工具和历史材料。

| 目录/文件 | 功能 |
|---|---|
| `euroc_final_results_data/` | 最终冻结配置的50组精度、50组串行开销、局部窗口、配置、审计及哈希 |
| `euroc_results_data/` | 早期EuRoC模式矩阵、运行身份、输入与完整性记录 |
| `euroc_tune_results/` | G/V权重扫描、L档位扫描、组合实验和选参依据 |
| `uzhfpv_results_data/` | UZH-FPV输入、标定、模式结果与运行核验 |
| `ltv/` | 分阶段接入、观测器验证、特征管理、相关性及工程验收研究记录 |
| `ltv_handover/` | 原观测器移植、公式与数值对照、交接材料 |
| `experiments/` | 回放/分析工具、实验配置、重构验证和已退出主构建的研究源码 |
| `Pose_Velocity_and_Landmark_Position_Estimation_Using_IMU_and_Bearing_Measurements.pdf` | LTV观测器参考论文 |

当前手动回放入口：

```bash
python3 docs/experiment_process/experiments/tools/ltv_gain_scan/run_manual_euroc.py
```

选择文件仍为仓库的 `config/euroc_final/manual_run.yaml`。历史manifest及JSON中的源码版本、原始输出路径和历史命令属于溯源证据，不因目录迁移而改写；本目录中的旧研究脚本不保证能直接作用于删减后的定稿源码。重现旧研究请使用其中记录的源码版本及冻结二进制。
