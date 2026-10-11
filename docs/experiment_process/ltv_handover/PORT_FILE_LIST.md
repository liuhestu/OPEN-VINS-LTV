# 移植文件清单与依赖边界

本地源HEAD与SHA在 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。`source_core/`仅含四个字节一致、chmod444的原样文件；没有删除copyright或改include，没有打包VINS/ROS工程。复制权限只防止普通编辑，SHA才是内容核验依据。

## 必须复制：原样复现第一层的最小集合

| 原路径 | 必要性与传递include |
|---|---|
| `vins/src/ltv/ltv_observer.cpp` | 唯一observer实现，includes observer.h、`../utility/utility.h`及标准库algorithm/chrono/cmath/limits/set |
| `vins/src/ltv/ltv_observer.h` | 对外API、持久state/P与maps，includes types.h、unordered_map、vector |
| `vins/src/ltv/ltv_types.h` | config/feature/reset/snapshot值类型，includes Eigen Dense和string；包含暂时不用的VINS factor诊断字段，可先留以保持原样 |
| `vins/src/utility/utility.h` | observer只用头内模板`Utility::skewSymmetric`；传递includes仅标准库和Eigen，无ROS/Ceres/OpenCV。保留完整原头做parity，之后可剥离必要数学函数 |

依赖：C++14标准库 + Eigen3（本次3.4.0）；observer需要Dense、Geometry由Dense的实际include和utility头提供。`utility.cpp`实现g2R，observer从不调用，**不需要链接**。本次独立harness仅多加nlohmann/json作为JSON读写依赖，nlohmann不是observer依赖。gtest/Ceres只用于已有测试/factor参考验证，不属于最小核心。

## 仅供参考：通过目标适配层重写

| 文件/模块 | 参考内容 | 不可原样带入的框架语义 |
|---|---|---|
| `vins/src/estimator/estimator.h/.cpp` | bias/extrinsic/td读取、buffer区间、先snapshot再gate/solve、reset调用 | VINS窗口索引、预积分、camera首末样本处理与并发锁需新适配；不复制Estimator类 |
| `vins/src/estimator/parameters.h/.cpp` | YAML key→field、默认、clamp/disable规则 | OpenCV FileStorage、ROS日志、VINS全局变量与输出截断不是目标参数系统 |
| `vins/src/ltv/ltv_quality_gate.*` | G质量条件/原因位 | base资格/世界g/cooldown与失败策略由目标重新定义；无安全保证 |
| `vins/src/ltv/ltv_velocity_quality_gate.*` | V创新/disagreement/冷却、冲突fail closed | 需要同刻目标速度与姿态；不能拿clone姿态和current速度混算 |
| `vins/src/ltv/ltv_csv_logger.*` | 列名、max_digits10、单位、逐帧记录 | 原logger重开会trunc，需独立epoch日志/非破坏路径；wall time不参与parity |
| `vins/src/factor/ltv_gravity_factor.*` | 方向残差、归一化、符号、Huber前尺度 | Ceres接口与3×7 ambient伪布局不是OpenVINS H |
| `vins/src/factor/ltv_velocity_factor.*` | `R_WB^T V_W-v_LTV`、速度sigma | Ceres3×9 `[V,Ba,Bg]`布局不同于目标IMU `[q,p,v,bg,ba]` |
| `vins/src/factor/pose_local_parameterization.*` | Hamilton右乘小量与人工PlusJacobian | 不可当JPL左乘误差state或真实四元数ambient导数 |
| `vins/test/test_ltv_*.cpp` | 既有API/公式/lifecycle/window/gate/factor测试 | window和Ceres测试是参考，用目标类型重新验证导数 |
| `vins/scripts/run_stage6_joint.py`、`run_stage7_tuning.py`、对应evaluator/test | 覆盖顺序、身份SHA、评价口径，确认历史有效配置 | 不把旧runner默认/CLI直接接到目标，不启动全数据集 |
| `config/euroc/euroc_stereo_imu_ltv_config.yaml`、`config/tuning/*`、本包`evidence/` | 基础/最终参数与原始manifest；cam0外参方向例子 | 发布参数非OpenVINS最佳参数/噪声模型；freq在源生产代码未消费 |

这些仅供参考文件未塞进source_core，完整源路径/哈希在manifest，关键外部实验配置/manifest在evidence。移动包离开源仓库后，若需要因素参考源码，可按manifest从明确HEAD获取并验证SHA；最小observer与fixture不依赖这些参考文件即可运行。

## 禁止迁移到目标更新链

- `ltv_velocity_oracle_gate.*`、`build_stage5_oracle_mask.py`以及GT驱动的mask：离线诊断Oracle，不是在线传感器。
- `ltv_snapshot_window.h`的VINS滑窗重排与快照重复求解；目标应通过物理时刻/epoch管理一次辅助更新，不复制容量11或MARGIN_OLD/SECOND_NEW逻辑。
- Ceres factor实例、HuberLoss、PoseLocalParameterization、MarginalizationInfo/VINS prior/重投影/预积分/FeatureManager；只读其物理量定义，不导入估计器算法。
- `stage5Replay.cpp`、旧ROS replay/调参/evaluator入口，以及loop_fusion/global_fusion：不属于第一层核心，不能替换OpenVINS原预测或frontend。
- 论文(20)及后续pose observer、已知世界landmark输入或新增OpenVINS landmark update：本次原型范围禁止。

## License/header原样保留

`utility.h:1`原GPLv3/HKUST头已保留。observer/types当前没有独立版权/许可头；本地未找到LICENSE/COPYING，package.xml许可仍TODO。manifest如实记录，没有创建声称来自上游的LICENSE。复制文件与向第三方再分发/合入目标项目的许可是否适合，是目标移植前的待确认项；不凭“无ROS include”推断许可独立或全部算法纯新写。
