# R4 正式验证收尾

本轮状态：**NOT_ACHIEVED_WITH_EVIDENCE**。轻量 Active Consistency 已实现，冻结执行矩阵已完成，能在校正之前阻断已知坏活动路标；有效真实序列的输出合同通过，但恢复总合同及部分合成因果对照证据尚未闭合，不能宣布整体 `PASSIVE_READY_EUROC`。

基线258b0b6，C02在正式确认前冻结为history/holdout均0.02 rad、过去至少5帧/0.20 s、两次可评价失败退休。拟合使用现有SeedEstimator，当前及未来观测不进入past fit。首次失败保slot/交叉P并跳过实际观测；不可评价保持计数及原观测行为。生产核心方程、Riccati、Q/V/P0、主VIO、readiness合同与G/V融合未改，注入为零。没有新增backend、gain搜索或种子Gate。

用户授权将完整真实/缓存预算扩大至100，旧尝试仍计数；本次实际使用real38/100、synthetic12/12、short_real3/16、candidate2/4。正式矩阵为11 ON、9匹配R3B OFF、7新B、2完整重复，加1日志OFF；4旧B及2旧OFF按明确身份复用。8项冻结合成运行全部退出0。所有失败、工具修复与重试保留在新账本，没有更改前三轮账本。

## 五项结论

1. **27537确实能在进入校正前被拒绝。** 同一完整x/P起点的实际核心局部回放，速度误差为prediction0.106985、旧posterior0.251884、新posterior0.090604 m/s；首次skip保留slot。这是实际重积分，不能当作连续轨迹的同状态反事实。C02连续运行中27537于74.30 s因两次失败退休，76.75 s被身份保护拒绝再准入。[局部因果证据](event_76_75.md)、[C02连续生命周期](full_C02_target_lifecycles.md)。
2. **29636历史不一致能够触发skip/retire，但必须区分候选。** 局部实际核心回放，速度误差旧0.119713→新0.108943 m/s；新重力误差仍略劣于prediction。C01连续轨迹78.55 s skip、78.60 s retire；所选C02中它从未准入，受容量及原seed检查限制，不可把C01的退休因果套给C02。[局部证据](event_78_90.md)、[C01生命周期](full_C01_target_lifecycles.md)。
3. **全序列存在收益，但不是所有尖峰或所有场景改善。** V2_03共同支持raw速度RMSE0.334029→0.329387 m/s；共同1436有效诊断包上描述性大速度修正180→110、重力修正68→59，启动峰值未降低，部分瞬态指标略恶化。修正率不是误差验收门槛。MH_04共同支持raw速度9.1163→13.7788、eta5.3993→6.6340，仍保留真实回归代价；不能称“只发生高频抵消”。[完整实测表和曲线](evidence/formal_real_results.md)、[MH_04诊断](drafts/MH04_confirmation_diagnosis.md)。
4. **有效真实序列覆盖与持续ready通过；短寿命合成仍有限制。** V2_03 G/V覆盖39.09/35.27%、最长joint6.30 s；正常V2_02为70.87/69.33%、12.80 s。4个指定正常初始化来源合同通过，可靠10%相对误差比例约99.05–99.51%，但更严reliable5仅84.51–93.74%。FAST合成长joint只有3.55/2.25/2.20 s，纯时间FAST速度覆盖19.8168%。这些是完整负诊断，不放宽门槛、不延长warmup。没有同输入OFF，不能把这些合成数值归因成新机制改善。[合成结果及源分层分母](evidence/formal_synthetic_results.md)。
5. **真实输出合同与整体成功必须分开。** 原冻结评价器保留11技术有效记录、10通过及MH_04失败，聚合NOT_MET。独立B-only复核证明MH_04在R4前已声明定位失败，本轮B完整轨迹逐字节重现原失败，输入、主参数及校准相同；按既有基线失败条款另列BASELINE_INVALID，10个定位有效序列的输出合同10/10通过，满足至少8条要求。这不是改写原metrics或按NEW误差删序列。整体仍不通过：dropout恢复评价第一段30–31.5 s供给中断，第二段31.7–60 s通过、宽档joint33.55–60 s，但总pass=false；不能删首段宣布恢复成功。新201/202案例没有同身份OFF，也未覆盖每条件两种子的成对确认。[两种原样保留的聚合](evidence/formal_B_only_contract_assessment.json)、[B-only证据](drafts/mh04_predeclared_baseline_failure_review.md)。

## 工程与证据

独立包构建、八项集成测试及数学/无未来泄漏/生命周期审查通过。默认OFF完整缓存、C01/C02完整缓存实际core/adapter逐值一致。正式ON/OFF/B全部完整消费输入，主状态/P/FEJ/clone/visual/轨迹旁路检查通过且零注入。V1_03、V2_03两次完整native重复的全部cache、audit、trajectory SHA一致；V2_03仅关闭ltv_log_enabled的完整native对照同样三文件精确一致，补齐了固定输入缓存重放未覆盖的主流程验证。[重复证据](evidence/formal_repeats_exact.json)、[日志开关证据](evidence/formal_logging_ablation.json)。

600 s独立资源试验12001相机/120002 IMU、零不变量错误；整条分支mean13.474 ms/P9514.344 ms，相机周期50 ms，满足原50/100 ms门槛。首/末分钟RSS中位108.39/109.32 MB。这是有限压力结果，不是全场景内存证明。完整性能与源可靠性开发证据继续保留，不冒充8项新种子的配对结果。

PRESSURE负对照G/V ready为零，严重比例为null，不能把评价器false解释成实际坏ready比例100%，也不能解释成可用性通过。其他7项合成具有非空ready支持、严重比例为零；全时域raw峰值与缺失仍保留。

完整状态/P、原始日志、误差数组与命令/退出码在 `/home/he/output/ltv_active_landmark_consistency_r4/`；仓库保存19张完整时间轴图、结果表、冻结身份、账本摘要和调度脚本。[逐项验收](completion_audit.md)、[科学聚合独立审查](drafts/formal_scientific_aggregation_review.md)、[合成运行身份](evidence/formal_synthetic_summary.json)。预算扩充前报告逐字节归档于[evidence/pre_budget_expansion](evidence/pre_budget_expansion/final_report.md)。

本轮到此停止。下一项最小证据工作是提供连续≥30 s充分供给的恢复输入及同输入OFF对照，并把评价工具的“运行完成”与B定位有效分开；本轮不新增运行、不救MH_04基线、不修改算法或选参，也不自动进入下一轮或G/V闭环。
