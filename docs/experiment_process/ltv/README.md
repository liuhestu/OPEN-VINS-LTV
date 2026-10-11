# LTV 任务索引

按任务阅读，各任务的“完成/停止”只适用于其自身；历史Goal和预算不会自动恢复。

|任务|入口|状态|
|---|---|---|
|Phase 0 移植、alias与原生数学|[复核](phase0/phase0_recheck.md)、[测试矩阵](phase0/test_matrix.md)|完成|
|Phase 1–4 adapter与G/V联合接入|[工程验收](integration/engineering_acceptance.md)、[决策记录](integration/decision_log.md)|完成|
|首轮EuRoC全量效果评估|[最终报告](euroc_evaluation/final_report.md)、[逐序列结果](euroc_evaluation/euroc_gv_results.md)|完成，保留MH_04失败|
|EuRoC/UZH信息价值验证|[合同](value_study/value_study_goal.md)、[报告](value_study/value_study_report.md)、[验收](value_study/value_study_validation.md)|完成：57次完整、12次短段|
|VINS差异与observer根因排查|[计划](observer_diagnosis/plan.md)、[排查报告](observer_diagnosis/report.md)|本轮完成：10组核心对照、2次插桩回放|

鲁棒融合任务：[计划](robust_fusion/plan.md)、[全 EuRoC 有限调参结果](robust_fusion/report.md)：80次完整回放完成；非线性滑窗未实施。

`evidence/`是只读历史证据归档，其已有phase/value_study子目录、文件字节及绝对历史路径保持原样；大型日志和数据仍在各任务记录的外部输出根。`../ltv_handover/`也保持只读。文档迁移映射和迁移前后SHA见[document_paths.json](document_paths.json)；正文只修订导航链接，历史命令/manifest内原路径不批量改写。

上一轮信息价值验证的完整提交为`02ef83c`。严格冻结身份检查应在对应历史版本执行；新任务新增测试/源码不表示旧实验失效，也不得把旧PASS解释成新代码已验收。
