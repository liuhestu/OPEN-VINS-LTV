# 条件信息与未消费观测

[LC08原始复核](../../landmark_correlation/holdout/report.md)核对时间、身份、历史geometry及共同样本：当前bearing在预测时尚未消费，之后允许消费，属于滚动一步留出而非永久holdout。历史geometry严格使用past，主预测原字段精确复现。V2_02三方共同样本86102/87075，geometry对照frame-RMS改善0.484262、CI[-0.035071,0.604226]；V2_03为55584/55908、改善0.126036、CI[-0.171455,0.380810]。正常/恢复组也未形成稳健优势，判INCONCLUSIVE。

线性参考的完全重复信息与非零视觉重复反例均验证给定视觉之后信息秩0。置零跨块会制造假后验收缩，见approximation_comparison.csv。真实main/LTV/visual joint缺失，真实条件信息秩保持BLOCKED；条件unit-source rank不是有效新信息秩。预测改善不能替代ON轨迹收益，本轮没有landmark融合。
