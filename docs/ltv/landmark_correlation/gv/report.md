# G/V 历史作用与不退化复核

G/V/GV 在已冻结 V2_02_medium、V2_03_difficult 配置上，**全部重算精度指标满足本轮参考文档的工程容限**。统计一致性仍未成立，融合收益没有可重复的独立证据，性能验收缺失。生产 G/V 保持关闭，不因本报告调噪声或确认门限。

本报告只复核历史完整运行，不验证本轮新增 shadow 源码。G/V 历史源码为 `16f02e30a4ba8308e112eb2c96d30046672007ec`，在线构建身份以 `docs/ltv/gv_fusion_evaluation/evidence/compiled_source.json` 和各运行 `identity.json` 为准；不能将当前 HEAD 直接贴到旧结果上。

## 身份与可复核范围

原始根目录 `/home/he/output/ltv_gv_fusion_evaluation_20261005`。选择实际 runs 目录中的两序列 OFF/G/V 的 `_03` 和 GV 的 `_01`。历史 `final_runs` 仍指向已经不存在的 `/tmp/ltv_gv_evaluation`，本轮未修补或替换历史软链接，分析器直接读取原始 `runs`。

[raw_identity.json](raw_identity.json) 重新计算并逐项核对全部 24 份 features/fusion/matrices 日志、两份发布 GT、8 组 trajectory/effective_options/identity/replay，以及各运行记录的 YAML 和传感器 CSV 身份。全部相符，缺失或不符会使程序停止，不使用摘要替代原始证据。完整图像输入的历史聚合 hash 由主 manifest 复核，本文不把 CSV 校验扩大为图片重新校验。

每次 replay.json 的 complete、input_camera_packets、input_imu_samples 与实际事件数量在新增完整性文件中复核。历史完整矩阵 Joseph 与均值注入检查见 `docs/ltv/gv_fusion_evaluation/matrix_audit.json`，其 9144 次包括重复 GV；本轮额外对六个唯一 ON 运行恢复原生15维 IMU全部修正分块与实际联合 P 变化，见 [matrix_blocks.json](matrix_blocks.json)。本轮恢复共6832次唯一实际联合更新（未把确定性重复计入独立样本）。该核对验证块对角近似声明下的实现，不验证缺失的主/LTV/视觉相关性。

## 冻结评价与结果

[evaluation_contract.json](evaluation_contract.json) 记录沿用参考默认容限与旧 OFF 分层协议 SHA256。[frozen_intervals.json](frozen_intervals.json) 保存实际时间、精确帧索引及视觉弱阈值。区间仅由 OFF 生成：全程、弱视觉、G/V 高误差且较1秒前上升、G/V readiness 恢复中断。高误差采样掩码保留不连续性，不把稀疏点重新解释为完整连续时间窗口。旧局部定义来自 `docs/ltv/gv_no_benefit_diagnosis/protocol.md`，不是从 ON 曲线选优势区间。

这是历史 ON 的**回顾性默认容限复核**，旧 ON 已存在，因此不声称本轮新容限在旧 ON 之前冻结。新 ON 的正式冻结由主 acceptance 合同控制。本文没有产生新 ON 或新参数组合。

轨迹遵循历史评价器：同一 OFF 初始化后相机支持，GT 最近邻20ms，输出1μs，SE(3)无尺度对齐，每种模式全程对齐一次；局部沿用该全程变换，不局部重拟合。RPE 两端都要属于固定掩码，端点时间匹配25ms。速度直接使用 EuRoC 发布 GT 世界速度和 SLERP 姿态转到 body，括号最大10ms，没有差分轨迹或重建伪真值。GT仅离线。

[metrics.csv](metrics.csv) 保存 2 序列 × 4 模式 × 6 区间 × 6 指标，共288行；所有行都有有效样本且全部满足容限。ATE、1s/5s RPE全程结果与历史指标逐项相差小于1e-12；速度为本轮独立从原始轨迹和发布GT重算。不得将局部RPE稀疏配对通过扩展为整个恢复过程统计一致性通过。

| 序列 | 模式 | ATE m | body速度 RMSE m/s | ATE相对OFF差 m |
|---|---|---:|---:|---:|
| V2_02_medium | OFF | 0.051975209571 | 0.038928815435 | 0 |
| V2_02_medium | G | 0.051978424684 | 0.038929387858 | +0.000003215114 |
| V2_02_medium | V | 0.051973334269 | 0.038928427215 | −0.000001875302 |
| V2_02_medium | GV | 0.051977001579 | 0.038928993008 | +0.000001792008 |
| V2_03_difficult | OFF | 0.207427615291 | 0.063455951075 | 0 |
| V2_03_difficult | G | 0.207245140697 | 0.063287356657 | −0.000182474595 |
| V2_03_difficult | V | 0.207234970408 | 0.063284869286 | −0.000192644883 |
| V2_03_difficult | GV | 0.207243631872 | 0.063286471297 | −0.000183983420 |

## 实际作用、覆盖与拒绝

[events.csv](events.csv) 在同一 OFF 掩码上逐项重算 available、observer_valid、ready、requested、门控尝试、accepted、applied 和逐类拒绝。applied 严格要求 receipt消费、分支有效行、joint_applied且一次 EKF；ready和accepted不当作applied。旧日志没有独立 eligible 布尔字段，不能将requested伪称eligible；可利用 ready/receipt/拒绝明细审计资格边界。

完整事件数：V2_02 G-only 1606次G，V-only 1571次V，GV 1606次G/1571次V；V2_03分别706次G和637次V。主状态/P首次分歧在各模式均对应真实辅助更新，旧 `events.json` 的首次事件由新增完整性核对重新检查。共同GT支持上的更新数与全输入事件数区别保留（V2_02边界排除14帧，V2_03排除7帧）。

此前诊断的关键解释仍成立：V2_02 G就绪输出整体没有稳定精度优势；V2_03 V部分高且上升误差段优于主状态，但大量这些输出不满足readiness。已实际更新的辅助贡献约微度、微米和微m/s。共同支持GV分支贡献P95：V2_02 G姿态6.55e-6°/速度2.79e-7m/s，V为8.96e-6°/1.87e-6m/s；V2_03 G为1.44e-5°/6.01e-7m/s，V为2.18e-5°/2.97e-6m/s。原R主导投影先验（约1e-5比例），小NIS不能证明统计一致性。

本轮原始矩阵恢复的GV gyro-bias贡献P95：V2_02 G/V为8.44e-9/7.87e-9 rad/s，V2_03为2.58e-8/3.61e-8 rad/s；accel-bias贡献P95分别为8.18e-7/1.08e-6和1.38e-6/1.47e-6 m/s²。所有15维IMU分块Kr及bias变化从原始联合K列恢复；P变化为同一次视觉+辅助联合更新实际后验减先验，不能把整个P收缩归因于辅助分支。既有分支单独注入的方向核对只是一阶作用诊断，不能当另一次EKF或因果收益。误差增大段各模式实际覆盖及中断没有更新已逐行保存。

恢复时延与供给中断的完整旧只读诊断见 `docs/ltv/gv_no_benefit_diagnosis/supply.json`；本轮未改确认20。此前 V-readiness 实验确认变动后ATE未改善（`docs/ltv/v_readiness_recovery/report.md`），因此不能把确认等待视为唯一瓶颈，也不据本轮结果开启参数扫描。

## 判定边界与最小缺口

- G-only、V-only、GV **精度子合同**：PASS_WITHIN_SCOPE，仅限上述两条序列和实际冻结配置。
- 性能子合同：NOT_EVALUATED。重诊断矩阵I/O和谱检查、历史环境与资源争用不足以重建匹配生产路径p95增长≤5%、RSS增长≤10%、相机积压/丢帧变化。最小补充为同机器同输入匹配OFF/ON顺序回放，生产诊断频度一致，并记录每帧主处理时延、RSS峰值与输入队列。
- 数值/崩溃/完整输入：历史运行可复核，无新增运行故无本轮新失效；新shadow构建由主运行矩阵单独验收。
- OFF工程精确回归：历史parity/repeat可复用，但本报告不替代新源码的OFF回归。
- 统计一致性：NOT_ACHIEVED；现有块对角噪声没有 G/V互相关、LTV-main或LTV-visual交叉块，低NIS和重复确定性回放都不闭合该缺口。
- 融合收益：NOT_DEMONSTRATED；V2_02极微小恶化/改善混合，V2_03微改善，确定性重复不是独立随机试验。
- 泛化：其他序列未评测，不给扩大范围通行结论。

保持生产默认关闭和确认20。无明显接入错误，停止G权重试探；V可保留独立实验能力。下一步首先完成主团队LC相关性与增量信息审计；不因本轮轨迹容限通过让landmark取得ON资格。

## 复现

```bash
OPENBLAS_NUM_THREADS=1 python3 scripts/ltv_landmark_correlation/gv/audit.py
OPENBLAS_NUM_THREADS=1 python3 scripts/ltv_landmark_correlation/gv/matrix_blocks.py
```

原始外部目录不可用时程序应失败，不能由本报告数字补造成功结果。新增工具不改变在线源码、配置、输入或生产开关。
