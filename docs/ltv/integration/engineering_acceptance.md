# OpenVINS–LTV 工程验收（冻结版本 c7fc29b）

Phase 1–4 已通过。效果结论必须等待正式11序列及复测，不能由本文件推出。
运行根为 `/home/he/output/openvins_ltv_autonomous_20261002`；完整运行配置、命令、源码/库/二进制SHA与日志按run保留。

## 实现边界

- `LtvObserver` 独立持久化内部路标、速度、重力与Riccati矩阵。目标核心仅移植skew依赖及已批准的两个完整RHS `.eval()`；只读源与golden均未改。
- `LtvAdapter` 只在IMU回调复制缓存，相机状态owner线程按物理端点插值；使用该区间传播前bias，校正顺序调用原生Dm/Tg。camera0 bearing是值副本，size_t ID有稳定int映射。
- 暂停、初始化等待、ZUPT、坏时间与重置使旧快照失效；恢复重启observer并warmup，不修改主VIO重置策略。不得外推、重复积分或同帧重试已拒快照。
- `UpdaterLTV` 固定同一prior/current/FEJ/P/clones，分别门控G/V。V不依赖gravity_valid。测量噪声与Riccati矩阵分离，sigma平方一次、degree转换一次。
- `UpdaterMSCKF` 保留原视觉构造、门控与压缩，在原提交点合并G/V。列布局保持原visual order并追加缺失q/v，重叠父子变量拒绝；空视觉也只进入同一提交入口。每token最多一次native EKF。
- 原 `Propagator`、`State`、`StateHelper`、`IMU`、`JPLQuat`、`UpdaterHelper` 与起点源码字节一致。完整P及全部交叉项由原生更新核处理。
- 默认G/V OFF；EuRoC固定标定、MSCKF-only、ArUco/ZUPT关闭是五模式共同配置，不降低单独B的预算。

## 已执行证据

| 测试/关卡 | 范围 | 证据 |
|---|---|---|
| 源及目标174事件parity | 完整s/P、slot/lifecycle、reset/warmup/substeps、原atol/rtol | `evidence/phase0_recheck_20261003` |
| Alias | 原表达式反例、真实核心正向、旧核心负对照 | 同上及独立alias_fix.diff |
| 原生T1/T2 | 500组×3 eps、15列、current/不同FEJ、原生JPL retraction、固定标定/重力符号 | `phase0_recheck.md`，最大绝对误差6.67e-9 < 1e-6 |
| 原生T3/T5 | full P/Joseph、singular clone、真Phi/clone/视觉FEJ | 同上；原生Q仅限定输入行为验证，没有顺便修改预测 |
| T6 adapter | 插值/offset/去重/pause/reset/漏检/ID/持久P/bias校正 | OUT `phase3_tests/test_ltv_timing.*` |
| 实际joint布局/门控 | G/V四组合、空视觉、stale IMU/clone、非法R/S、q符号、FEJ current residual | OUT `phase3_tests/test_ltv_joint_layout.*` |
| joint更新代数 | 50组完整native mean/P vs dense/Joseph，正确sequential创新修正 | OUT `phase3_tests/test_ltv_joint_covariance.*` |
| 真实S数值fixture | 消减导致双三角差异，采用native Upper约定后更新正确；不放宽阈值 | `test_ltv_innovation` 与D005 |
| 配置实际生效 | 缺省OFF、显式相关性确认、sigma×2导致R×4、非法设置拒绝 | OUT `phase3_tests/test_ltv_options.*` |
| T4额外边界 | 真triangulation/refinement后chi2全拒绝仍一次辅助；原生空压缩+实际提交；sigma信息PSD序 | OUT `phase4_edge.*` |
| 原版/OFF/Passive | 400包audit/trajectory字节一致，完整V1_01 B/P字节一致；同步器修复后再查400包 | Phase1证据、OUT `phase4_sync_V1_01_easy_*` |
| Phase2 G完整 | 2912包、29120 IMU、2800输出、2757次实际G | OUT `phase2_full_audit.json` |
| Phase3 V完整 | 同样完整输入/输出、2781次实际V、G行0 | `evidence/phase3/phase3_full_audit.json` |
| Phase4 GV三开发集 | V1_01/V2_02/V2_03分别2757/2183/1380次同时G+V，每包<=1次EKF | `evidence/phase4/*_audit.json` |
| 构建 | 全新隔离ROS2三包及五个Phase0安装测试通过；后续增量安装构建通过 | Phase0复核与OUT `phase4_sync_build.*` |

空压缩说明：正常特征有clone列且ct_meas>0时，原生压缩行数=min(rows,cols)>0，不能自然得到零行。
测试调用真实compression的2×0边界，再调用实际辅助提交；不声称在自然视觉数据触达数学上不可达分支。
生产三处提前返回均调用同一auxiliary_only路径，没有修改原压缩算法。

## 输入同步修订

V2_03原cam0有1922消息、cam1有2336消息，ROS2 bag metadata也一致；1921对header时间戳完全相同。
离线runner最初按同索引配对导致完整运行在输入检查失败。D007记录修复：按原始int64精确时间相等配对，逐条记录未匹配1左/415右，保留完整时间范围和全部IMU。
这不是按估计结果丢帧，不改原CSV/图像/GT，不把异步消息近邻强配。所有模式使用相同规则。
最终覆盖仍按原cam0初始化后支持检查99%，evaluator从未因此放宽。

## 完成审计补充

冻结后无生产代码或参数变化。链接实际冻结动态库的独立测试 `evidence/final/test_ltv_bias_epoch.cpp` 验证新bias仅用于下一积分区间、旧snapshot保持值拷贝、未来缓存不提前积分、Unix大时间戳和version/canonical key拒绝。state/P对照误差分别5.55e-17 / 4.96e-10，通过原adapter测试容差；实际库路径有ldd证据。

## 冻结与解释限制

唯一配置为规范初始保守值：G=10deg、V=1m/s、99% NIS门限、额外quality gate关闭、observer warmup20。
额外参数候选0，没有以ATE为反馈选参。代码/二进制/库/配置/输入/evaluator身份见 `frozen_configuration.json`。
正式运行发生身份漂移会在启动前拒绝；失败目录及旧版本开发运行均保留。

联合更新只落实独立噪声工作模型，并未显式建模共享IMU、bearing、bias带来的交叉相关。
Jacobian、FEJ几何或Joseph测试通过，均不证明全系统统计一致性，不继承原论文GES/AGAS保证。
