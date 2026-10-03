# 自主执行进度

> 以下为按阶段保留的历史进度；早期“运行中”和“下一步”不是当前任务状态。最终状态见本文末尾与 final_report.md。

- 当前：Phase 1–5完成，最终40项证据一致性核验通过；55正式+16复测+1原版诊断均结束，无活动回放。
- 输出根：`/home/he/output/openvins_ltv_autonomous_20261002`。
- 起点、diff、预算与恢复状态：输出根的 experiment_manifest.json / checkpoint.json / runs.jsonl。
- 最新附件优先：完整回放300次上限；最多6组额外候选；开发仅V1_01、V2_02、V2_03。
- 执行顺序：Passive/options/adapter → 合成时序与原版/OFF/P逐事件回归 → V1_01完整B/P → G → V → GV → 开发集验收 → 唯一参数冻结 → 全11×5 → 关键序列复测 → 报告。
- 当前无外部阻断；本地全11序列ASL与ROS2 bag可见，待输入清单校验。
- 不覆盖已有Phase 0未提交工作；不修改原VINS/GT/核心原生滤波文件。

## Phase 1 增量
- 新增 LtvOptions/LtvAdapter，manager Passive调度与ROS2原始双目header同步检查。
- 时序单元测试通过，日志：输出根 phase1_timing.*。
- 原版/OFF/P短段400包严格一致：phase1_short_comparison.json；不是ATE近似比较。
- 正在完整V1_01 B/P验收，查看checkpoint和runs.jsonl确定运行状态，勿重复启动活跃PID。
- 生产G/V尚未接线，不运行G/V模式。当前完成完整回放0（下一次启动后以runs.jsonl为准）。

## Phase 1 PASS / Phase 2 开始
- 完整 V1_01 B/P：2912 camera、29120 IMU、2800输出，audit与trajectory逐字节一致。
- P有效2781帧，G方向健康2757帧；未注入任何辅助。
- 证据：docs/ltv/evidence/phase1 与输出根完整run目录。
- 完整回放已启动2/300；无活动回放。
- 下一步：实际UpdaterLTV、列布局、一次联合提交；替换参考公式测试为实际代码；通过后仅跑G smoke。

## Phase 2 进展
- 实际UpdaterLTV G/V纯模型、独立门控、合并工具和MSCKF三个early return已接线；仍按阶段先验收G，再验收V/GV。
- 原生Jacobian/FEJ/协方差/时序/列布局、50组joint/Joseph/linear sequential测试均通过。
- 新版B/P 400包仍与保存原版digest/trajectory字节一致。
- G短段发现联合S双三角舍入误拒，已通过实际捕获fixture确认并按native Upper约定修复（D005），回归通过；未改变阈值/参数。
- 完整V1_01 G运行中：checkpoint/runs.jsonl记录PID及完整回放计数（本次是第3次）。
- evaluator已在任何真实ATE计算前冻结：输出根 evaluator_protocol.json；全11输入/GT哈希 input_manifest.json 已生成。未进入正式全11评估。

## Phase 2 PASS / Phase 3 開始
- 完整V1_01 G：全输入消费、2800输出、2757次G联合提交、每包EKF最多1次；无asymmetric_S误拒。
- 主P最大非对称1.36e-19，最小谱舍入-2.76e-17；原生生产内核未修改。
- 完整回放3/300，参数仍初始保守值，候选调整0。
- 下一步V-only；先补充stale-clone/配置读取等边界与G/V gain分量诊断。所有效果相关参数未改。

## Phase 3 PASS / Phase 4 开始
- 2026-10-03：已重新读取目标附件；此前Phase 0复核有新增可追溯证据，本目标继续Phase 1–5。
- Phase 3配置/时序/过期clone/联合协方差/实际数值fixture测试全通过；完整Phase 0原生测试也已再次通过。
- V1_01 V完整回放：2912 camera、29120 IMU、2800输出，2781次V联合更新，G行0，每包最多1次EKF。
- V NIS均值0.25153、最大4.61382；V修正均值1.23e-5、最大9.44e-5；P非对称0、谱负舍入约3e-17；单epoch、无重复attempt。
- 证据：输出根phase3_tests、phase3_full_audit.json及对应run目录。未计算真实ATE、未改参数，额外候选0。
- 完整回放4/300；下一步补充实际chi2全拒绝、原生空压缩、sigma信息序测试，再GV三开发序列。

## Phase 4 PASS / 冻结准备
- 三开发序列GV完整运行：V1_01为2757次GV，V2_02为2183次GV，V2_03为1380次GV；每帧EKF<=1，有限状态/P，单epoch，无重复attempt。
- V2_03首轮因离线同步器按数组索引配对失败（计入预算），D007记录原因；改为原始int64精确时间配对后全部1921同步包/23370 IMU消费，未匹配1左/415右逐条保留。
- 修复后V1_01 B/P 400包与保存原版audit/trajectory逐字节一致；实际配置JSON核对通过。原始输入/GT全11哈希再次匹配。
- 完整回放8/300（含1次输入失败），未计算真实ATE；额外候选0，保留sigmaG=10deg、sigmaV=1m/s、原NIS/质量/warmup全部初始值。
- 下一步本地代码commit，冻结代码/二进制/库/基础配置/evaluator/input identity；串行55正式组合，随后统一评估与三关键序列B/G/V/GV复测。

## Phase 5 正式运行
- 冻结代码：c7fc29b11352da3104c80462bcdd32553e785085；基础配置/源码/二进制/库/evaluator/input身份见frozen_configuration.json及输出根experiment_manifest.json。
- 驱动命令：`python3 scripts/ltv/run_final_matrix.py`；日志为输出根final_matrix_driver.log，进程/计数以checkpoint和/proc为准。
- 驱动支持已完成且身份一致的正式运行复用；首次失败停止诊断，不覆盖记录，不在正式集上选参。
- 完成后：`python3 scripts/ltv/summarize_final.py` → `python3 scripts/ltv/run_repeats.py` → 统一复测汇总、最终报告与逐项完成审计。
- 本轮未完成最终目标，Goal保持active。

## 最终结果与完成核验
- 冻结代码c7fc29b、候选0，80/300 full replay（开发8含1失败、正式55、原版诊断1、复测16）。
- 全11 B/P字节一致。四条关键序列16次复测字节一致，ATE范围0。输入/GT最终SHA匹配，evaluator及估计器版本未变。
- MH_04五模式定位FAIL：巨大ATE保留；原版完整baseline与冻结B字节一致，不能宣称发散减少即定位收益。
- 共同有效10/11：B0.131648m，G0.131628m，V0.129939m，GV0.129962m；G基本持平，V/GV约1.3%小幅且场景依赖。
- final_report.md、euroc_gv_results.md、requirements_audit.md及evidence/final已生成；manifest/哈希/日志/链接最终核验已通过，见evidence/final/completion_audit.json。
