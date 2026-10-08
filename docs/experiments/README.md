# 完整实验分支

本分支汇集A/B/C全部代码、历史工具和报告，保留原始研究布局。main从此整合起点做结构整理；新标准入口位于main的docs/experiments，数值等价检查见main的consolidation_report.md。

此分支重放历史/完整诊断：BUILD_LTV_PHASE0_TESTS=ON。原生驱动位于ov_msckf/tests/ltv/tools，Python入口在scripts/。原ARGV、C0、实验开关默认值不变。main仅调整组织，不授予此前失败实验任何新资格。

三线最终结论仍见docs/ltv/three_track_validation/final_usability_and_integration.md；A/B工程STOP，C仅有限条件模型。归档分支与标签映射见archive/branches.json，Git bundle和未提交内容备份在其中指定的仓库外路径。大结果不删除，也不冒充公开下载。
