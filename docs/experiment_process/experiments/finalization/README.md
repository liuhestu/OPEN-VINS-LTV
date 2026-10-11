# 定稿精简与可读架构重构

本目录记录2026-10-11的诊断删减、目录整理、可读架构重构及保护逻辑审查。HEAD为 `231b9c36035b651ffa1a6346476892e1f5eb1623`，重构工作区尚未提交，实际代码身份以closeout.json中的文件哈希为准。

## 当前结构

LTV核心头文件/源文件从最初45个收敛为20个：observer 3、features 7、runtime 6、fusion 3、config 1。原landmark_adapter目录取消，数学Observer不再承担OpenVINS输入、特征几何、生命周期或可用性策略。Updater仍属于fusion，因为它更新OpenVINS主State/P；Observer更新自己的x/P。

类的主链：VioManager → makeLtvOpenVinsInput → LtvRuntime → LtvFeatureFrontend/LtvFeatureManager + LtvObserver + LtvAvailabilityPolicy → UpdaterLTV/UpdaterLandmarkApprox → UpdaterMSCKF → StateHelper。

研究诊断退出主构建；源码与专用测试位于 `../archive/ltv_diagnostics_231b9c3/`。旧字段/别名清理、配置校验集中、NIS分解复用和事务副本精简的依据见 [保护逻辑审查](guard_review.md)。数值处理、时间契约、算法门控与完整提交机制保留。

## 验证记录

| 记录 | 范围与结果 |
|---|---|
| [unit_tests.json](unit_tests.json) | 诊断删减阶段：25项功能测试及1个预期失败负对照 |
| [regression.json](regression.json) | 诊断删减阶段：两条完整序列、五种模式，共10组 |
| [architecture_unit_tests.json](architecture_unit_tests.json) | 最终架构：25项功能测试全部通过，含非C0合法配置和解析NIS=7；负对照返回预期1 |
| [architecture_regression.json](architecture_regression.json) | MH_05_difficult、V2_02_medium的OFF/G/V/L/GVL，共10组全部通过；轨迹、完整主状态/P审计、Observer摘要、融合收据与冻结结果逐字节一致 |
| [architecture_inventory.json](architecture_inventory.json) | 当前模块、文件与行数 |
| [docs_relocation_validation.json](docs_relocation_validation.json) | 四份报告的本地链接、docs顶层布局和80个冻结数据文件哈希检查 |
| [closeout.json](closeout.json) | 当前源码/构建/安装身份与收尾检查 |

这是结构重构和固定输入的回归证据，不是独立精度验证；历史开销FAIL和统计一致性缺口不改变。其他序列未在本轮重构后重新全量回放。测试原始日志及新回放输出存放在仓库外的 `/home/he/output/`，本目录保留结构化索引。

## 当前启动方式

```bash
python3 docs/experiment_process/experiments/tools/ltv_gain_scan/run_manual_euroc.py
```

选择文件仍是 `config/euroc_final/manual_run.yaml`，默认使用新安装 `/home/he/open_vins_ltv_ws/install_ltv_readable`。`install_root: null` 可选择历史冻结二进制。修改序列、模式和G/V/L强度无需重新编译。

若需要重建当前ROS包与离线入口，可在工作区根目录使用：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ov_msckf --cmake-args -DBUILD_LTV_EXPERIMENTS=ON
```

这会写入常规install目录，手动选择文件中的install_root需相应设置为该目录。实际本轮隔离构建使用 `/tmp/ltv_finalize_build` 和 `/tmp/ltv_finalize_install`，随后将安装前缀迁到持久目录，并核对二进制/库字节和动态库解析。
