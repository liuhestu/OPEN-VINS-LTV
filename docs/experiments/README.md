# LTV 实验入口与归档

`experiment` 保存 A/B/C 整合后的全部实验代码与历史工具；`main` 从相同代码起点整理目录和入口。所有算法参数、门控、残差、Jacobian、数值保护与更新顺序保持原样，默认融合 OFF。本轮结构整理不改变此前预测 FAIL、工程 STOP 和统计证据缺口，也不授予组合资格。

## 当前入口

研究工具不进入默认构建，ROS 2 使用：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ov_msckf --cmake-args -DBUILD_LTV_PHASE0_TESTS=ON -DBUILD_LTV_EXPERIMENTS=ON
source install/local_setup.bash
ros2 run ov_msckf run_ltv_gv_production /absolute/path/to/docs/experiments/configs/euroc_c0/estimator_config.yaml /absolute/path/to/ASL/sequence/mav0 /unique/output OFF
```

- `run_ltv_gv_production`：原生 ASL 标量回放。明确实验模式 OFF/G/V/GV/L/L_GV；ON 仅用于复现失败候选。
- `run_ltv_gv_evaluation`：同一输入与算法路径的完整矩阵/缓存诊断回放，不用于生产性能判定。
- `replay_ltv_feature_cache`：重放带版本的原有缓存；`replay_ltv_landmark_shadow` 保留其独立预测用途，未冒充主估计 ON。
- `tools/ltv_three_track/evaluate_runs.py` 与 `assess_metrics.py`：A/B 共用冻结 OFF 支持、局部窗口、空支持和验收参数；阈值来自原合同。

其余工具按原研究包组织在 tools 中，不作为默认构建或统一实验的入口。已退出当前构建的旧 euroc/value/passive 驱动源码仅用于历史研究；准确复现旧命令请使用对应归档标签。历史 source/manifest/hash 不改写为当前目录。

## 配置与结果

[共享 C0 配置](configs/euroc_c0/estimator_config.yaml) 与原 A/B YAML 字节相同，配套两标定文件留在同一目录。已有生产数据集 config 路径和值不变。

[三线最终结论](../ltv/three_track_validation/final_usability_and_integration.md)、[115 次尝试](../ltv/three_track_validation/run_manifest.csv) 和既有 docs/ltv 下的其他报告保留在原位置，避免破坏冻结哈希和链接。

## 恢复历史分支

[原分支→归档标签/OID](archive/branches.json) 包含本地及 origin 的原始引用。例：

```bash
git fetch origin --tags
git switch --detach archive/consolidation-20261008T153807/origin/experiment/ltv-three-track-b-20261008
```

仓库外 bundle/未提交文件备份与原始大结果的稳定位置见 branches.json。它们是本机数据路径，不是公开下载地址。归档标签已在远端逐个核验；分支最终删除前还会核对原 tip，拒绝覆盖并发新增提交。

[结构整理与数值回归](consolidation_report.md) 记录实际构建、回放、两分支 OID 和清理结果。

六模式 schema 2 中 OFF 为纯 OpenVINS（Observer 也关闭），与归档 schema 1 的 managed-Observer OFF 定义不同；历史结果不改写。L 同时启用 landmark observer/shadow 历史与 landmark_approx，L_GV 加启 gravity/velocity。旧 L_ON 是 L 别名，旧 L_OFF/V10 仅为历史工具保留，不参与六模式矩阵。

## EuRoC 六模式全量矩阵

[最终 ATE RMSE](../euroc_results.md) 收录 11 序列 × 6 模式的 66 次新完整回放、实际融合计数、异常序列和验收边界。所有组固定 C0、max_slam=0。

批次入口为 `tools/ltv_three_track/run_euroc_six.py`；ATE 入口为 `tools/ltv_three_track/evaluate_euroc_six.py`。准确命令、独立构建来源及完整 manifest 见结果文档和 `../euroc_results_data/`。

## UZH-FPV 原配置与增益扫描

[UZH 原配置六模式对照](../uzhfpv_results.md) 使用公开 GT 的 Snapdragon 双目和 IMU，保留各类 UZH 标定、噪声与初始化；[增益扫描](../euroc_tune_results/report.md) 先在代表序列筛选，只有相对 OFF 及同模式 1× 的 ATE 增幅均不超过 10% 才扩展。

入口与隔离约束见 [ltv_gain_scan 使用说明](tools/ltv_gain_scan/README.md)。最多四个隔离回放进程；原始数据和日志放在仓库外。indoor_forward_7 因实际融合次数全部为零，按用户要求从后续增益测试和汇总中排除，已完成记录保留。

## EuRoC 独立 G/V 权重

[统一结果与分析](../euroc_tune_results.md) 收录七档 G/V 单分支、非对称 GV 网格，以及另行追加的 10×/16×单分支。历史档位复用；入口及隔离约束见 [独立扫描说明](tools/ltv_gain_scan/README_independent.md)。
