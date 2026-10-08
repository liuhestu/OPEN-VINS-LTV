# EuRoC 六模式 ATE RMSE（运行准备中）

OFF/G/V/GV/L/L_GV，11条序列，66次新完整回放；全部max_slam=0、冻结C0。OFF是纯OpenVINS，L_GV同时开启三种融合。

构建、短程接口检查通过后执行全量；没有数据的项不填0或PASS。最终报告将包含逐序列ATE(m)、初始化/覆盖、真实融合次数与失败。

[冻结协议](euroc_results_data/protocol.json)
