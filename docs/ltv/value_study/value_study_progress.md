# 本轮进度：完成并停止

执行合同：[value_study_goal.md](value_study_goal.md)；协议：[value_study_protocol.md](value_study_protocol.md)。

- A/B/C/D/E全部完成。57/80次完整回放（九条×五模式45次 + 三条×四模式复测12次），12/20次短段；未运行MH_04，未扩展全量UZH。
- 9条B/P轨迹及完整状态/P/FEJ/clone/视觉集合审计一致；12个复测均字节一致。诊断旁路及4个计时插桩短段一致。
- 隔离构建、9个原生测试、Python参考/统计/资格/evaluator验收通过；原始输入、GT、冻结源码/库/二进制、只读旧交接与报告核验通过。
- G/V开发资格均为否；C0保持G10度/V1m/s，额外权重候选0；observer及原生预测/EKF未改。
- UZH独立验证平均ATE：G改善1.211%、V退化0.053%、GV改善0.871%；直接G/V质量明显落后prior，单步收益极小。主要小幅ATE收益来自GT支持有限的outdoor_forward_5，不能据此宣称稳定增量信息。

交付：[最终报告](value_study_report.md)、[实现与验收](value_study_validation.md)、[完成核验](../evidence/value_study/completion_audit.json)。图表与原始结果索引均由报告链接。

输出根：`/home/he/output/openvins_ltv_value_study_20261003`，最终checkpoint无运行中进程。成本统计曾误调用无GT短段参考插值，保留失败日志后改为GT无关读取，复用成功回放；正式数值参考脚本未改。

本Goal结束，不自动启动observer调参、数学架构修改、全量数据集或下一阶段实验。
