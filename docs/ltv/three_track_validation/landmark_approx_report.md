# Landmark 工程近似：真实 ON 已检验，最终 STOP

真实 ON 完整作用于 V2_02/V2_03：分别1570/627帧实际消费3140/1244点，最多2点/帧、一次联合 EKF。完整 OFF 精确回归、JPL/FEJ Jacobian、原生布局和短程校正分块重建通过；生产独立开关默认 false。

完整匹配 DEV 的504指标行有1 FAIL、12空支持 NOT_RUN。V2_03 原冻结三点尾窗 ATE 由0.1889666923增至0.1947820938m，恶化5.8154mm，超过3.7793mm限值。全程改善0.1862mm不抵销它。固定源码/参数 OFF/ON 完整重复的11必要身份文件逐字节相同；独立 Horn 拟合确认失败，共同 OFF 对齐仍恶化5.5422mm。五个真实矩阵单步空/零信息控制未发现超舍入实现错误，不证明长非线性反馈稳定。

因此工程 **FAIL → STOP**，实际可用收益未证明，生产性能/RSS/留出/全可用集/组合扩展 NOT_RUN。这个 STOP 的原因是实际工程局部退化，与冻结的5%预测 FAIL独立；未调噪声或删窗口追求 PASS。噪声是明示独立近似，缺失的 main/Observer/seed/anchor/visual 交叉与均值没有被证明为零，统计一致性未成立。

完整实现、证据和全部49次尝试见 [B 最终报告](task_b/final_report.md)、[近似实现报告](task_b/landmark_approx_report.md) 与 [manifest](task_b/run_manifest.csv)。配置与入口见 `config/ltv_three_track/task_b/README.md`，仅供复现失败实验；源码分支 `experiment/ltv-three-track-b-20261008`，运行源码 `5cdedc087891430369a7fa88db12cdbba58bd05d`，最终提交 `c04d8c63e1f4cc562589a93259f2efc70524aa8f`。
