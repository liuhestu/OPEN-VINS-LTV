# OpenVINS + LTV 不退化与landmark判定

结果为 **landmark STOP_WITH_REASON**。本轮实际完成构建、相关检查、历史G/V逐序列与固定局部精度复核、LC01–08实验及其明确的BLOCKED/INCONCLUSIVE范围、六次完整匹配OFF/shadow回放。保持确认20、G/V生产关闭、验证max_slam=0；不合并主分支、不启用landmark融合。任务交付与统计/融合验收是不同结论。

- [最终报告](report.md)：判定、证据、开销、限制和最小后续工作。
- [验收合同](acceptance.yaml)、[身份清单](manifest.json)、[运行矩阵](run_matrix.csv)、[证据索引](evidence_index.csv)、[指标](metrics.csv)。
- [G/V作用](gv_audit.md)、[联合模型](covariance_design.md)、[GO/STOP](landmark_go_stop.md)。
- [LC01–08规定产物](landmark_correlation/report.md)、[实验矩阵](landmark_correlation/experiment_matrix.csv)。

原始矩阵/日志位于manifest列出的 `/home/he/output/ltv_nondegradation_correlation_20261008`；受控Observer原始数据另外保存在ignored results。所有大文件通过路径/SHA索引引用，不加入Git。用户原始 `.md.md` 参考文件保留；规范路径副本内容相同。
