# LTV 融合权重实验

[最终增益报告](report.md) 包含代表序列筛选、1×/2×/4×全量结果及相对 OFF/原配置的变化，改善加粗。[原配置 UZH-FPV 对照](../uzhfpv_results.md) 保留最初六序列的完整结果。

增益比较排除 MH_04 和 indoor_forward_7，覆盖十条 EuRoC 和五条 UZH。indoor_forward_7 已完成的 16 个原始配置保留在 [排除结果](excluded_results.json)，不参与收益汇总。

[实验入口与复现](../experiments/tools/ltv_gain_scan/README.md) · [独立审计](independent_audit.json) · [实际线程设置](runtime_threading.json) · [描述性汇总](summary.json)。筛选 PASS 不是工程验收通过；历史预测 FAIL 与工程结论保留。
