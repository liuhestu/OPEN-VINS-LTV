# LTV：三个阅读入口

LTV 自身负责观测器数学与输入特征管理；原生 OpenVINS 负责输入处理、调度及 EKF 更新。

| 入口 | 职责 |
|---|---|
| `LtvObserver.h/.cpp` | 动态路标/v/g 状态、设计矩阵、IMU 传播、一次相机修正 |
| `features/LtvFeatureManager.h/.cpp` | 特征几何、生命周期、ID 映射、观测组装、L 锚点和候选 |
| `../update/UpdaterLTV.h/.cpp` | G/V/L 残差、FEJ Jacobian、噪声、门控、融合确认与宽限 |

FeatureManager 内部使用 FeaturePool 实现历史、身份保护和 birth/retain/retire 规则；SeedGeometry 实现初始化几何；LandmarkHistory 维护过去锚点。它们不另设对外协调入口。

一次相机事件：

1. `VioManager::process_ltv` 从 `Propagator::imu_interval` 读取完整区间。Propagator 是唯一 IMU 缓存所有者；LTV 读取不会消费数据，不允许末端外推。
2. 使用传播前 bias，经 `Propagator::correct_imu` 校正端点，取均值推进 Observer。
3. 在特征管理副本中准备输入，并在当前修正前选 L 候选。
4. 在 Observer 副本上同步路标并修正；成功后提交特征及数学状态，再保存当前锚点。
5. 原生 VioManager 处理启动、休眠、恢复与 epoch。UpdaterLTV 每个有效事件只推进一次融合确认计数，量测 build 不改变计数。
6. UpdaterMSCKF 在最终更新作用域调用 UpdaterLTV，借助 UpdaterHelper 的通用量测块合并视觉与辅助量测，调用一次原生 EKFUpdate。辅助拒绝保留视觉更新，无视觉可只提交辅助量测。

Observer 不持有宿主 State、时钟或就绪状态。数学输出和同步接口声明在 LtvObserver.h；宿主事件元数据在 UpdaterLTV.h。原 Runtime、独立 Readiness 和 LtvTypes 文件已移除。

数学保持 compact_v2：`F=I+dt*A`、`P=F*P*Fᵀ+dt*V`；相机 `Rc=I/(q_landmark*dt)`，LLT 求解与 Joseph 更新。首次相机事件和空观测跳过修正。P 为观测器设计矩阵，不能作为辅助噪声 R。

配置与启动见 [euroc_ltv](../../../config/euroc_ltv/README.md)。专用 LTV 测试目录及构建入口已移除。历史实验使用相应 Git 版本重现。
