# OpenVINS-LTV 源码课程大纲

目标：沿一次IMU/相机事件看懂状态归属、Observer数学递推、特征准入和G/V/L融合。当前基于HEAD `231b9c36035b651ffa1a6346476892e1f5eb1623` 上的compact_v2实现，工作区修改尚未提交；旧实验身份和数据保持原样。

前置知识：坐标变换、线性时变系统/Riccati方程、EKF残差与Jacobian、MSCKF克隆状态。范围：当前生产源码与手动离线入口；暂不展开论文收敛证明、统计一致性及旧研究诊断。

## 先建立全貌

`ov_msckf/src/ltv/` 按五类职责组织，分为observer、features、runtime、fusion、config。observer只保存数学状态；features只管理几何和生命周期；runtime连接输入并提交事务；fusion只构造量测，MSCKF通过原生EKFUpdate写回主估计器。

主链：VioManager接收输入 → makeLtvOpenVinsInput → LtvRuntime::processManaged → LtvFeatureFrontend与LtvObserver → 可用快照 → G/V/L量测 → MSCKF联合更新。

详细模块导图见 [ltv/README](ov_msckf/src/ltv/README.md)。本文件按流程分课，不按文件数量分课。

## C01：一帧数据怎样进入两套状态系统

状态：大纲。读完能回答：谁拥有主State，谁拥有Observer的x/P，数据怎样传给它们？

主链：IMU/相机触发 → VioManager → 输入转换 → Runtime → 快照 → 辅助融合。

1. `ov_msckf/src/core/VioManager.cpp::feed_measurement_imu`：同一IMU进入原生传播器与Runtime队列。
2. 同文件 `do_feature_propagate_update`：捕获bias，传播/克隆后处理相机事件。
3. `ov_msckf/src/ltv/runtime/LtvOpenVinsInput.cpp::makeLtvOpenVinsInput`：将State、标定及当前特征转成值快照，不保留源指针。
4. `runtime/LtvRuntime.h`：辨认其拥有的Observer、FeatureFrontend、AvailabilityPolicy。

选读验证：实际入口的effective_options.json、replay.json。暂缓数学公式（C02）和量测合并（C05）。

## C02：Observer怎样递推路标、速度与重力

状态：大纲。读完能回答：x怎样排列，IMU怎样产生A/B，bearing怎样产生观测修正？

主链：configure/reset → synchronize → IMU传播 → 相机观测矩阵 → LLT/Joseph一次修正 → 数值检查 → 数学输出；失败由Runtime重置。

1. `observer/LtvTypes.h`：LtvConfig、LtvSnapshot、受控birth/retain命令。
2. `observer/LtvObserver.cpp`：reset、velocityOffset、gravityOffset，辨认路标/v/g分块。
3. 同文件 propagateImu：构造A与F=I+ΔtA，推进state以及P=FPFᵀ+ΔtV。
4. 同文件 correct：构造C/y，使用R_c=I/(q·Δt)、LLT求K并执行Joseph更新；首次相机事件跳过修正。
5. 同文件 snapshot、reset：读数学输出；有效性与时钟由Runtime管理。

以上路径均相对于 `ov_msckf/src/ltv/`。选读验证：test_ltv_compact_math。暂缓路标准入（C03）及可用性（C04）。

## C03：一个视觉特征怎样成为Observer路标

状态：大纲。读完能回答：初始化来源怎样选择，质量怎样判断，为什么同一ID不能重复birth？

主链：当前/历史bearing → seed几何及风险 → 准入 → active/coasting/retired → 受控命令 → Observer状态重排。

1. `features/LtvFeatureTypes.h`：按输入、seed、历史、生命周期、事件结果几个段认识值类型。
2. `features/LtvFeatureFrontend.cpp::process`：组织双目/时序候选与历史一致性检查。
3. `features/LtvSeedGeometry.cpp`：LtvSeedEstimator::estimate与LtvSeedUncertainty::propagate，得到均值和风险。
4. `features/LtvFeatureManager.cpp`：先读LtvFeatureQuality::evaluate，再读LtvFeatureManager::step；历史与ID保护辅助代码在同一模块。
5. `observer/LtvObserver.cpp::synchronize`、rebuildState：安装一次birth，保留已有状态/协方差块。

选读验证：test_ltv_feature_manager、test_identity_guard、test_ltv_active_lifecycle。暂缓ready策略（C04）和辅助量测（C05）。

## C04：一次相机事务怎样暂停、恢复和完整提交

状态：大纲。读完能回答：为什么有数值输出仍不ready，为什么不能只提交一半的路标或x/P？

主链：相机时刻 → IMU区间 → epoch边界 → 传播 → 暂存特征 → 暂存修正 → 全部提交 → 可用性结果 → claim。

1. `runtime/LtvRuntime.cpp::process`：compact_v2只进入processManaged，要求受控FeatureManager。
2. 同文件processManaged的六个Stage：沿原有执行顺序读取，不先跳进内部工具函数。
3. `runtime/LtvAvailabilityPolicy.cpp::update`、acknowledgeBootstrap、commit：读取预热、ready/grace及恢复计数。
4. `runtime/LtvRuntime.cpp::pauseManaged`、reset、claim：读取中断、epoch与一次提交保护。

选读验证：test_ltv_availability相关行为保留在当前名为test_ltv_readiness的测试中；另读test_ltv_adapter_hardened。暂缓日志格式与旧兼容路径的细节。

## C05：G/V/L怎样改变OpenVINS主State

状态：大纲。读完能回答：三类残差各是什么，权重在哪里生效，视觉与辅助量测在哪里只提交一次？

主链：Runtime值输出 → 视觉压缩块 → MSCKF作用域内构造G/V/L并门控 → 合并 → 原生EKFUpdate → 更新诊断。

1. `fusion/UpdaterLTV.h/.cpp`：gravity、velocity、build，读残差/H/R及门控。
2. `runtime/LtvLandmarkHistory.h/.cpp`：before、after，读修正前候选和修正后锚点；`fusion/UpdaterLTV.cpp::landmark`读FEJ模型与信息上限。
3. `ov_msckf/src/update/UpdaterMSCKF.cpp::update`：读视觉为空、辅助为空和合并提交分支。
4. `ov_msckf/src/update/UpdaterMSCKF.cpp::update_joint` 与 `ov_msckf/src/state/StateHelper.cpp::EKFUpdate`：追到主State与P写入。
5. `config/euroc_final/manual_run.yaml` 及手动入口：核对模式、强度换算、实际参数与更新次数。

选读验证：test_ltv_gv_jacobian、test_ltv_compact_native及完整序列回归记录。暂缓历史研究诊断；其源码/结果在docs/experiment_process中归档，不是当前学习前置。

## 自学与验证

一次只展开一课，以对应问题为目标。文件中的类是边界，函数中的阶段是阅读顺序；合并文件不等于合并职责。若某一段数学或事务流程看不懂，再针对该课展开讲解。

重构验证需分别报告：编译、数学/时序/生命周期测试、固定输入的主状态/P、Observer x/P和融合收据对照。本轮允许ON轨迹改变，OFF须保持原生更新；结果见docs/experiment_process/experiments/compact_v2，不覆盖历史结论。
