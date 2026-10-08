# Landmark：STOP_WITH_REASON

保持 shadow，landmark ON 不实施，G/V生产默认关闭、确认20、验证 max_slam=0。此决定来自未闭合的真实联合统计和未识别的增量信息，不是回放预算或参数强度不足。

| 编号 | 总项状态 | 已执行证据 | 必须补齐的最小依赖 |
|---|---|---|---|
| LC-01 | BLOCKED | 实际 seed 原生JPL有限差分、完整受控源联合映射、三幅度36000抽样；边缘和C_x_seed直接源参考等价 | 主clone/历史bearing后验交叉块、复用bearing间相关、标定/time依赖；在来源首次消费前登记并沿主更新保留 |
| LC-02 | BLOCKED | 实际Euler F/B；共享input offset贯穿IMU/camera；同帧bearing各Euler子步使用同源合成B；含D的通用联合传播与独立增广参考 | main/Observer/visual交叉、插值IMU端点来源与bias/reset映射、Riccati/gain敏感度；真实完整主联合传播尚未实现 |
| LC-03 | BLOCKED | 实际slot M、只读上一camera条件anchor、物理时间、跨时刻/跨点源贡献；birth/retire/reset/epoch/anchor替换核对 | 全seed初始化联合块、C_xa/C_xt、clone扩增/边缘化及主视觉更新；将受控rolling anchor连接到成熟候选primary anchor |
| LC-04 | PASS（线性参考限定） | 7场景直接源残差与R/N/R_L_visual/S消元、独立SVD、广义Joseph；零及非零视觉完全重复信息反例 | 真实消元依赖LC01–03；真实R/N/S仍BLOCKED，不由此参考颁发ON资格 |
| LC-05 | PASS（离线对照执行） | 同prior/r，完整和明确置零模型谱/K/Kr/P/条件信息；部分置零非PSD反例明确拒绝 | 真实prior联合块未知，真实候选K禁止计算；不使用置零结果作为fallback或资格 |
| LC-06 | PASS（受控参考限定） | 七种12000 IID线性参考、同源破坏反例、功效/同时区间、多个非线性幅度；实际seed与Observer局部输入核对 | 完整实际gain/main/seed联合误差校准未闭合；没有独立landmark真值，真实Sigma_L校准NOT_EVALUATED |
| LC-07 | BLOCKED（模型资格） | 新增逐IMU/camera/lifecycle/anchor矩阵流；真实完整运行、OFF精确回归及开销结果另见report/run_matrix，不能由设计推定 | 全部真实valid_unconditional=false；所记录仅fixed-seed/fixed-gain单位源贡献，零有效真实约束不能称可融合 |
| LC-08 | INCONCLUSIVE | 未消费当前bearing、历史几何对照、三预测共同支持重算；两序列全程/正常/恢复区间的改善CI跨零 | 真实条件信息秩需联合块；新的独立传感信息或可检验的保守相关性模型，保留相同共同样本评价 |

缺失完整 Sigma_tt/aa/at、C_xt/xa 和 LTV-visual cross 时，不能把条件源 Gram 拼成完整联合分布。本轮已实际增加只读传播、anchor快照和可重算矩阵；这些贡献确实可产生，但真实缺块仍为 UNKNOWN。初值固定、gain固定是控制实验的条件，不是未知真实相关性置零的借口。

完全重复信息反例在完整模型下 prior/posterior trace 同为1.10、信息秩0；置零模型却得到 posterior trace=0.78455246、假信息trace=0.31544754、秩3。近秩亏按冻结有效子空间处理，无ridge、负谱裁剪或调R修复。生产Riccati旧谱处理未改变，但不能将它解释为联合误差PSD修复。

真实点误差没有独立truth。EuRoC位姿GT只能验证位姿/速度，历史bearing重三角化不构成独立landmark GT。因此真实NEES/NIS统计校准不授予通过；合成或受控实现核对也不能替代它。

最小后续扩展按依赖顺序：登记原始IMU端点/像素来源及校准雅可比；在主视觉nullspace/compression之前保留来源系数，沿主传播、clone、视觉更新和原生JPL注入/reset传播；对实际seed建立联合初始化；把Observer完整均值/P_R gain敏感度及相同来源纳入只读联合误差系统；连接成熟primary anchor并处理真实生命周期；在独立仿真point truth或可观测的受控场景校准；再用未消费bearing和条件信息检验有效增量。没有上述新证据，不扫描参数、不增强融合力度、不进入landmark ON。

G/V已有精度容限满足的证据、OFF工程精确回归、统计一致性与融合收益分别记录，任何一项都不能替代这里的GO条件。R4原未闭合项保持NOT_ACHIEVED。
