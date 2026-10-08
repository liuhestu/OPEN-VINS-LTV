# LC01–08执行结果与阶段D

阶段D为STOP_WITH_REASON。逐项状态、已执行范围、剩余依赖见 [experiment_matrix.csv](experiment_matrix.csv)；真实统计校准NOT_EVALUATED，融合收益NOT_DEMONSTRATED。实验执行完成允许得到STOP，不能被写成全部统计/融合验收通过。

LC01–03的实际映射、只读共同源传播、滚动anchor与生命周期已落实；它们没有消除main/历史噪声/visual/gain的未知块。LC04–06在明确参考/受控范围完成；Monte Carlo抽样合同、反例功效与非线性失效边界保留。LC07两序列各一次匹配OFF与两次完整shadow，主状态、完整P、Observer x/P_R、生命周期与旧基线精确一致；新增source/matrix流逐字节重复。独立reader检查285250项B/Gram/anchor恒等式、107346项prior-snapshot身份，max相对差约1.13e-16；它不验证所有完整F递推或物理噪声模型。真实有效完整约束为0，只能判模型/覆盖不足，不能判可融合。

重日志shadow p95增长约7.35%/11.18%，不满足5%预算，单独记录于shadow_summary.csv；这是诊断模式而非候选生产路径。LC08的geometry优势置信区间跨零，真实条件信息仍未知。所有GO前提均需真实源模型闭合和信息增量证据；[STOP理由及最小扩展](../landmark_go_stop.md)给出具体缺失矩阵。

LC05补充：协方差兼容不等于固定r可解。原7场景14对照中的4处固定r不在S有效子空间，见 [residual_support.csv](../../landmark_correlation/reference/residual_support.csv)，已明确拒绝作为物理候选，原Kr/P只解释为形式离线诊断。没有用投影r静默绕过rank问题。
