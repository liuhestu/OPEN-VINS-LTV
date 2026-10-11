# OpenVINS + LTV 本轮可复核判定

本轮交付判定为 **landmark STOP_WITH_REASON**。工程等价、精度容限、统计一致性、融合收益分别判定。生产融合保持关闭，确认20；实际验证配置全部max_slam=0，主算法、seed均值及管理算法未改，未扫描噪声或门限。

| 项目 | 结论 | 范围 |
|---|---|---|
| OFF工程等价 | PASS | 两条序列冻结同步输入合同；主状态/完整P、Observer完整x/P_R、生命周期/readiness、receipt精确复现 |
| G-only、V-only、GV精度 | PASS_WITHIN_SCOPE | 历史两序列全程与六个OFF固定分层；288指标全部满足默认容限 |
| G/V完整工程不退化合同 | NOT_EVALUATED | 生产p95、RSS及ROS队列/丢帧尚无匹配证据；不以精度子合同替代 |
| LC01–03 / LC07真实模型资格 | BLOCKED | 已实现只读传播/真实完整shadow；full joint所需块仍UNKNOWN，有效完整约束0 |
| LC04–06 | PASS（明确参考/受控范围） | 消元/重复信息/置零对照及Gaussian参考；真实非线性片段描述性，不授予真实统计校准 |
| LC08 | INCONCLUSIVE | 未消费bearing与历史geometry共同样本改善CI跨零，真实条件信息秩未知 |
| Landmark前提/ON | STOP_WITH_REASON / NOT_IMPLEMENTED | 无真实模型和信息资格，未实现或运行融合ON |
| 系统统计一致性 | NOT_ACHIEVED | P_R不是经证明的Sigma_L；共享输入、seed、main/visual cross及gain sensitivity未闭合 |
| 真实landmark统计校准 | NOT_EVALUATED | 无独立landmark真值；EuRoC pose GT及共享bearing三角化不能替代 |
| 融合收益 | NOT_DEMONSTRATED | 微小确定性轨迹变化和预测优势均非独立融合收益证据 |

## 工程执行与身份

初始源码1b0bf91，独立实验分支experiment/ltv-nondegradation-correlation-20261008；历史G/V源码16f02e3。验收在新运行前单点冻结，旧ON的本轮精度复核明确为回顾性检查。106项G/V原始文件/配置/GT/输入身份重核，全部图像聚合hash重新读取一致，所有历史几何/留出原始缓存身份核对。详细身份与脚本/binary SHA见manifest及evidence_index，不把历史源码标成当前HEAD。

真实ROS Humble colcon依赖构建成功，最终ov_msckf clean-first构建成功。29个C++可执行检查与2个精确比较全部退出0；另6项联合参考、7项landmark分析单元测试及评价器rigid/no-scale/缺输出脚本检查通过。新增C++14 standalone真实seed/Observer检查已执行；ROS2沿用仓库既有C++17配置。core比较对匹配历史native输出为零容差，174事件design matrices对原fixture亦零容差；旧prototype输出对历史binary自身存在5.7e-14差异，原fixture容限没有改，见core_reference_scope。

曾有一次test_ltv_timing崩溃：集成期间完成的旧对象没有新header依赖，导致混用类布局。gdb定位构造/configure赋值，旧binary/library、失败ledger保留。源码冻结后clean-first重建修复，完整检查与回放全部通过；不把开发失败删除或计为一个成功运行。详见development_failures.json。

每条序列实际运行一次匹配OFF、两次完整SHADOW_OFF，顺序执行。无短时截断、每个冻结输入IMU样本和同步camera包均消费，原unmatched camera日志精确复现。新matrix/source CSV、lossless bin/jsonl和成功post-frame identity receipt两次逐字节相同。OFF完整main矩阵、轨迹、audit、缓存bytes及Observer全x/P_R精确复现；仅排除features.compute_time_ms，新timer/shadow文件单独索引。

| 序列 | 同步camera包 | IMU样本 | OFF → shadow camera p95 | OFF → shadow峰值RSS |
|---|---:|---:|---:|---:|
| V2_02_medium | 2348 | 23490 | 30.864 → 33.132 ms | 129.23 → 133.23 MiB |
| V2_03_difficult | 1921 | 23370 | 26.976 → 29.992 ms | 131.38 → 126.85 MiB |

p95计feed_measurement_camera，包含同步shadow矩阵记录，不包含外围大矩阵序列化；各模式字段与线程/输入合同相同。重诊断shadow的p95增长7.35%/11.18%，均超5%预算；RSS变化+3.10%/−3.45%，绝对值如表。该结果只描述诊断路径，默认关闭。没有测量生产G/V ON开销或ROS transport queue，不能将单次RSS降低解读成优化收益。

V2_03原始清单cam0=1922、cam1=2336，其中1/415张无严格同步配对，416行unmatched与旧基线一致。完整回放指既有冻结同步输入合同的全部1921包，并非声称所有单侧原始图像均入估计器或其时间都获得位姿评价；GT/精度支持及边界见metrics、frozen_intervals。没有掩盖新丢帧或将缺支持计成成功。

## G/V作用与局部退化

6832次唯一实际联合更新的15维IMU姿态/位置/速度/两bias Kr分块与P变化从原始矩阵重算，首个main状态/P差异均对应真实辅助更新。作用约微度、微米、微m/s；V2_03部分V优势处无readiness更新，确认10旧试验也未改善ATE。G没有稳定新优势；不调信任制造轨迹变化。[完整G/V报告](../landmark_correlation/gv/report.md)给出全程/局部定义、每序列每模式指标及拒绝分布。P变化含同次视觉更新，不能全归因辅助分支；重复确定性回放不是独立随机样本。

本地11条EuRoC均存在，没有以缺数据解释扩展未执行。当前没有完整生产性能/统计信息资格和有收益候选，本轮停止扩大ON矩阵，其他序列泛化NOT_EVALUATED。若后续候选资格闭合，再按合同先冻结OFF与区间，补常规/困难至少两条匹配序列。

## LC01–08及停止依据

[规定目录的实验矩阵](landmark_correlation/experiment_matrix.csv)及[LC报告](landmark_correlation/report.md)列每项运行、作用范围、缺块与依赖。实际seed native JPL Jacobian最大差6.97e-7；受控完整源Sigma_seed/C_x_seed与显式联合映射等价；3x12000真实seed输入抽样全部幅度保留。实际Observer同源two-Euler+camera三幅度各4000抽样，完整gain/P_R更新参与；full camera Jacobian与fixed-gain shadow差约2.94%，不是将差异删掉后标校准PASS。

只读Observer在真实调用中传播Euler F/B、持久offset、同帧bearing合成B、slot M、上一camera条件anchor及跨时刻/跨点块。独立reader验证285250项B/Gram/anchor恒等式，107346项prior-snapshot身份，最大相对差1.13e-16；所有4072 camera attempts唯一匹配成功receipt，未混淆applied。该reader没有独立重算全部F/完整source递推；不扩大其证明范围。真实unconditional有效率0，模型不足而非融合资格通过。

Gaussian参考七场景各12000IID，合成正确模型同时区间通过，故意破坏共享来源被拒绝，预先登记功效满足要求；非线性高幅stereo-depth偏差保留。重复信息完整模型trace保持1.10、信息秩0，置零却得到0.78455246的假缩减。近秩亏按冻结有效子空间处理，不加ridge/裁谱修复真实联合模型；置零仅离线诊断。额外有效子空间审计识别4处固定r不在S支持空间，相关Kr只作形式诊断，不作为可提交候选。

未消费bearing采用滚动pre-consumption预测，不是永久holdout。三方共同支持保留历史geometry：V2_02改善CI[-0.035071,0.604226]、V2_03[-0.171455,0.380810]，正常/恢复CI也跨0。预测价值、有效增量和ON轨迹收益没有混为一谈。

[STOP与最小后续扩展](landmark_go_stop.md)：保留raw IMU端点/像素来源，沿主propagation、visual nullspace/compression、EKF/JPL注入、clone扩增/边缘化保留交叉来源；建立actual seed联合初始化及完整Observer gain敏感度，连接成熟primary anchor；用独立point truth或受控场景校准，再检验条件增量。只读诊断是opt-in；其I/O/身份异常采用fail-fast保留失败，未来生产候选须补统计模块失败隔离及transaction commit/abort lineage。R4原未闭合项仍NOT_ACHIEVED。

## 复核入口与交付

脚本位于scripts/ltv_landmark_correlation。run.py记录append-only尝试、禁止覆盖replay目录，完整匹配与重复检查；audit_shadow.py独立解码/receipt核对；closeout.py核查冻结源码SHA及完成状态；assemble_bundle.py汇编规定9个LC产物。真实构建命令、逐运行argv/环境、全部原始文件路径/SHA和失败重跑见manifest、run_matrix、evidence_index。大日志/矩阵留在output/results，不提交Git。

分支提交/推送及远端OID验证见delivery.json；不合并主分支、不启用生产融合。用户未提交修改保留。此报告交付一个有证据的STOP，不颁发完整工程/统计/融合收益通行证。
