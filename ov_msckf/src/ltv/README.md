# LTV：按一次相机事件阅读

这份代码按五个职责组织。先读输入到输出的主链，再展开某个模块；不用按头文件顺序逐个阅读。

```text
ltv/
  observer/       数学：独立状态 x、Riccati 矩阵 P、IMU传播、bearing修正
  features/       特征：seed几何、历史、准入、保留、退役
  runtime/        运行：OpenVINS值转换、IMU/相机时序、事务、可用性
  fusion/         融合：G/V/L辅助量测与主估计器的联合EKF更新
  config/         参数：读取与检查现有YAML
```

## 谁拥有数据

```text
VioManager（拥有OpenVINS主State和原生前端）
  └─ LtvRuntime
       ├─ LtvObserver：拥有独立x/P
       ├─ LtvFeatureFrontend
       │    └─ LtvFeatureManager：拥有历史、ID保护和生命周期
       └─ LtvAvailabilityPolicy：拥有预热、恢复、ready/grace计数

LtvRuntime只输出快照；UpdaterMSCKF在最终更新作用域内调用UpdaterLTV构造G/V/L量测，
UpdaterMSCKF合并视觉与辅助量测，StateHelper::EKFUpdate写回主State/P。
```

LTV独立保存状态，但输入仍使用OpenVINS的bias、标定和路标初始化几何；状态存储独立不代表噪声统计独立。

## 推荐的第一遍阅读顺序

| 顺序 | 文件与入口 | 阅读目的 |
|---|---|---|
| 1 | ../core/VioManager.cpp：feed_measurement_imu、do_feature_propagate_update | 找到IMU队列入口和相机事件入口 |
| 2 | runtime/LtvOpenVinsInput.cpp：makeLtvOpenVinsInput | 看原生State/Feature如何转换成值快照 |
| 3 | runtime/LtvRuntime.cpp：process、processManaged | 顺着标出的六个阶段看时序与完整事务 |
| 4 | features/LtvFeatureFrontend.cpp：process | 看seed来源选择、几何检查及生命周期提交 |
| 5 | observer/LtvObserver.cpp：synchronize、propagateImu、correct | 看数学状态与Riccati矩阵递推 |
| 6 | fusion/UpdaterLTV.cpp：build、landmark；runtime/LtvLandmarkHistory.cpp：before、after | 看快照如何成为辅助量测 |
| 7 | ../update/UpdaterMSCKF.cpp：update | 看视觉与辅助量测如何合并并写回主估计器 |

compact_v2只保留受控特征路径；ON配置必须启用feature_readiness_enabled。OFF不创建Runtime，保持原生视觉更新。

## 各模块内部怎样读

- observer/只保留LtvObserver.h/.cpp和LtvTypes.h。状态排列是每个路标的三维机体系位置，随后是机体系速度和重力。IMU构造A(t)、B并传播x/P；相机以bearing投影构造观测矩阵，首次事件或Δt=0跳过修正；其余相机事件执行一次LLT求解和Joseph更新。IMU使用F=I+ΔtA、P=FPFᵀ+ΔtV。
- features/LtvFeatureTypes.h集中存放输入、seed、历史、生命周期和事件结果的值类型。它不执行估计器更新。
- features/LtvSeedGeometry.h/.cpp放seed均值、Jacobian和不确定性传播；features/LtvFeatureManager.h/.cpp放历史、质量判断、ID保护和准入/退役；features/LtvFeatureFrontend.h/.cpp组织几何检查和manager事务。
- runtime/LtvRuntime.h/.cpp负责一次相机事件的六阶段流程：采样IMU区间、epoch边界、Observer传播、暂存特征事务、暂存相机修正、完整提交。LtvAvailabilityPolicy负责预热与ready条件，不直接修改Observer。
- config/LtvOptions.h保留已有YAML参数名；Observer内部Q/V/P0与融合G/V/L权重是两类参数。通用API校验合法范围，实验入口单独检查冻结C0。

Runtime拥有LtvLandmarkHistory：相机修正前选择候选，修正后保存锚点，输出纯值候选。融合模型只读取候选与当前原生State，构造FEJ Jacobian、噪声和信息上限。UpdaterMSCKF通过merge_measurements合并列，且只调用一次原生EKFUpdate；辅助拒绝时保留视觉量测。Observer不保存时钟、epoch、预热或准入历史，数学失败返回false，由Runtime重置。

## 启动

编辑config/euroc_final/manual_run.yaml，然后在仓库根目录执行：

```bash
python3 docs/experiment_process/experiments/tools/ltv_gain_scan/run_manual_euroc.py
```

手动YAML默认使用install_compact_v2；五种模式与冻结融合权重继续保留。日志仅记录实际更新次数、拒绝原因、NIS和耗时，旧分支K/Kr字段已删除。当前Observer的P是设计矩阵，不是辅助量测R。历史诊断和旧接口的源码在docs/experiment_process/experiments/archive/，应使用其中记录的版本重现。

保护逻辑审查见 [guard_review.md](../../../docs/experiment_process/experiments/finalization/guard_review.md)。

本轮结果与重现入口见 [compact_v2](../../../docs/experiment_process/experiments/compact_v2/README.md)。
