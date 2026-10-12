# OpenVINS-LTV 源码阅读顺序

当前入口为 LtvObserver、LtvFeatureManager、UpdaterLTV。LTV 自身保存数学状态及额外特征信息，运行调度直接接入 OpenVINS。

1. **数学**：阅读 `ov_msckf/src/ltv/LtvObserver.h/.cpp` 的 synchronize、propagateImu、correct。跟踪动态 `[路标…,v_B,g_B]` 的索引重排、P 交叉块、F 传播和 Joseph 修正。数学输出不包含宿主时钟和准入状态。
2. **特征**：阅读 `ltv/features/LtvFeatureManager` 的 prepare、commit。沿 SeedGeometry → FeaturePool → Observer 输入理解初始化、保留和退役；FeaturePool 是内部生命周期实现，Manager 是对外入口。LandmarkHistory 在修正前取过去候选，修正后保存锚点。
3. **输入和调度**：阅读原生 `state/Propagator` 的 imu_interval、correct_imu，再读 `core/VioManagerLTV.cpp`。关注共享缓存的读取边界、传播前 bias、特征/Observer 的完整提交，以及启动和故障恢复。
4. **融合**：阅读 `update/UpdaterLTV` 的 update_health 与 build。前者每事件推进一次融合确认，后者只读当前原生先验；G/V/L 的 FEJ Jacobian 和启发式噪声在此构造。
5. **原生更新**：阅读 `update/UpdaterHelper` 的 MeasurementBlock、merge_measurements、validate_measurement，以及 UpdaterMSCKF::update_joint。类型列映射与状态归属检查通用化，最终只有 StateHelper::EKFUpdate 更新宿主状态。
6. **实际启动**：阅读 `ltv/LtvOptions.h/.cpp`、`config/euroc_ltv/ltv.yaml` 和 subscribe.launch.py。模式/权重在 C++ 中解析；启动方式见配置 README。对齐与保存分别由 ov_eval/live_align_trajectory、pose_to_file 实现，launch 与 ROS 1 共用 dolivetraj、alignment_type、dosave、dotime 等命名。

专用 `ov_msckf/tests/ltv` 及构建入口已移除，历史测试复现使用相应 Git 提交。

Observer 的 P 为设计矩阵；共享观测源的相关性近似仍存在。结构整理与基础测试不替代精度和统计一致性评估，本轮仅做必要测试及 OFF/GVL 短段回放。
