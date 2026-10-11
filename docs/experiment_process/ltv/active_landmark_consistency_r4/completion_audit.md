# R4 最终验收矩阵

冻结执行矩阵已完成，未闭合项如下。整体结果 **NOT_ACHIEVED_WITH_EVIDENCE**。原冻结数学/误差阈值不变；“运行完成”和“科学通过”分别记录。扩充前矩阵归档于 evidence/pre_budget_expansion/completion_audit.md。

| 要求 | 证据 | 最终判断 |
|---|---|---|
| 薄层复用history/SeedEstimator，不修改核心/gain/主VIO | 实现、构建21、独立审查 | PASS |
| past-only/current holdout、无未来泄漏、Gauge一致 | 纯测试及独立测试17/18/29 | PASS |
| skip实际校正且保slot、两次失败retire、neutral计数不变 | 生命周期/集成29、实际core事件24/25、native live | PASS |
| default OFF完整x/P/slot/event/ready一致 | 历史完整缓存30；正式9 OFF工程与三方主旁路 | PASS，限定已测身份 |
| 27537局部因果；C02连续生命周期 | event_76_75、full_C02_target_lifecycles | PASS；C02目标前已退休 |
| 29636局部因果与真正退役 | event_78_90、C01/C02生命周期 | 局部/C01 PASS；C02未准入，不误称active retire |
| C02正式冻结及依赖SHA | frozen_config、运行守护、独立审查 | PASS；无正式结果驱动调参 |
| 11完整native ON、9匹配OFF、7新B及合法历史复用 | formal ON/CONTROLS审查、B_reuse_manifest | PASS，未按NEW误差隐藏运行 |
| 原冻结工具全11聚合 | formal_all11_assessment | NOT_MET：MH_04；原metrics不改 |
| 原B-only有效性契约 | MH_04历史预声明及完整B轨迹同SHA | BASELINE_INVALID(MH_04)；其余10/10输出通过，非全任务PASS |
| 主状态/P/FEJ/clone/visual/trajectory、不注入、全输入 | 11科学metrics工程项+完整native audit | PASS |
| ≥2确定性完整重复 | formal_repeats_exact | PASS，两条cache/audit/trajectory SHA精确一致 |
| logging ON/OFF完整native数值 | formal_logging_ablation | PASS，仅改log_enabled；不以缓存重放代替主流程 |
| 8冻结合成完整身份/消费/退出码 | formal_synthetic_jobs、formal_synthetic_summary | PASS，8×60 s；未追加OFF |
| 指定正常来源样本/机会/reliable10 | formal_synthetic_results | 4指定来源PASS；fallback单独报告 |
| 正常合成精度/覆盖/5s诊断 | 8原metrics及全时间轴图 | FAST连续段短、TEMP V覆盖19.8168%；如实保留，未扩张真实合同适用范围 |
| 合成严重误ready | 原metrics及适用分母解释 | 7项非空ready严重比例0；PRESSURE空ready为null，无可用性成功证据 |
| 合成同输入OFF因果/回归 | 201/202完整身份搜索 | UNVERIFIED，没有同SHA OFF，不跨种子复用 |
| 恢复总合同 | 开发及正式dropout原区间 | 未闭合：首段供给中断false，后长段PASS；未删区间或改标签 |
| ≥600s性能/有限资源边界 | 48运行/54评价 | PASS已测试身份；非普遍证明 |
| 同条件成对两种子泛化 | 冻结8案矩阵 | 未完成全条件配对，不宣称覆盖 |
| 旧core/config/golden/历史证据/旧账本只读 | final_delivery_audit | 最终逐字节核对；用户原未提交文件不入commit |
| 交付与停止边界 | final_report、决策/预算及最终commit | 保存完整证据；不继续G/V闭环/下一轮 |

real38/100、synthetic12/12、short_real3/16、candidate2/4；分析/构建/测试失败均计入原样账本，没有退费。无需再运行来消耗剩余真实预算。未执行合成OFF、全条件配对种子或额外恢复输入；它们是明确的证据缺口，不是隐含成功。
