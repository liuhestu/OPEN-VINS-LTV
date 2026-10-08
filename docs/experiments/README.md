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

- `run_ltv_gv_production`：原生 ASL 标量回放。原 CLI 模式 OFF/G/V/GV/L_OFF/L_ON 保持不变；ON 仅用于复现失败候选。
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
