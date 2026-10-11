# A–B：已完成的原参数对照

原参数对照与工具验收已完成；增益、生命周期及初始化尚待执行，本文件不是最终研究结论。

|场景 / 实现|50–60 s V RMSE m/s|η RMSE m/s²|宽档 t_VG（全部检查）|
|---|---:|---:|---|
|REGULAR / CONT_REF|0.30990862|0.1835157|NOT_OBSERVED_WITHIN_HORIZON|
|REGULAR / SPLIT_REF|0.31010702|0.18353037|NOT_OBSERVED_WITHIN_HORIZON|
|REGULAR / CORE|0.29969054|0.17815606|NOT_OBSERVED_WITHIN_HORIZON|
|FAST / CONT_REF|0.014272851|0.0084124084|40.775|
|FAST / SPLIT_REF|0.01428137|0.008413153|40.805|
|FAST / CORE|0.01449644|0.0090425368|37.855|
|STATIC / CONT_REF|1.5699464e-14|4.2787828e-14|4.85|
|REGULAR / EXPERIMENTAL|0.29969054|0.17815606|NOT_OBSERVED_WITHIN_HORIZON|

REGULAR 的末段误差主要在连续参考中就存在。CORE−SPLIT 的全时域速度差 RMSE 为 0.0226804 m/s，SPLIT−CONT 为 0.0242180 m/s；不能把这些不同来源合并归因。提高 IMU 至 1000 Hz 或相机至 100 Hz 后，末段速度 RMSE 仍约 0.30–0.31 m/s，未消除误差。没有观察到明确公式/旧值求导缺陷，本轮没有修订离散算法。

准确全状态初值（10 s）：连续速度 RMSE 1.16946e-14 m/s，实验副本 0.00610436 m/s。副本关闭 hook 的完整 60 s 状态（200 Hz）、slot、相机子步/重置事件及整秒完整 P 与生产 CORE 逐值一致，谱 floor 触发次数为零。

静止负对照：末段 v/η 数值正确，路标 RMSE 18.8190 m、负 ray 比例 100%、投影残差 1.22e-15。不能用低投影残差或正确 v/g 证明深度正确。

S_PAPER：连续宽档 t_VG 11.45 s，CORE 在包含校正前样本后为 10.555 s。真实相机 Z 为负的比例约 49.63%，仍使用有符号理想 bearing；不作为真实相机全程可见的验证。

三场景加严参考的归一化状态及完整 P 差均 <6e-14，满足 1e-6 合同。所有数字来自本轮仿真，不复用旧 probe 汇总值。完整 120 s 连续延长正在单独执行，保留原 0–60 s 指标。

执行偏差（SHA 范围和短测并发）如实记录在 [decision_log.md](decision_log.md)。生产源码、主配置、论文、执行文档、交接副本和历史证据当前 588 项复核未变；`ov_data` 已从后续字节复核排除。
