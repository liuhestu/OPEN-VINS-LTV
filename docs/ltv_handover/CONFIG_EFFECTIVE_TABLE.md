# 最终配置逐字段审计

`[代码事实]` 本表来自实际本地LtvConfig默认、参数解析和已复制的effective YAML；不把base等同replay。当前列明确指Stage7 T2/V1_01_easy主trial（不是repeat/live、不是所有实验）。notune列为统一批次同序列joint_v_gate；baseline的factor/gate开关另见下面模式表。key全名解析在`vins/src/estimator/parameters.cpp:72 + readLtvConfig`，使用位置简写相对`vins/src/ltv/`，注明estimator例外。

原始文件及SHA：

- base：`config/euroc/euroc_stereo_imu_ltv_config.yaml`。
- notune：`evidence/notune_V1_01_easy_joint_v_gate_effective_config.yaml`。
- T2发布：`evidence/stage7_T2_effective_config.yaml`；真正运行内容见`stage7_T2_runtime_config.yaml`与`stage7_T2_input_manifest.json`。
- live启动：`evidence/stage7_live_effective_config.yaml`，算法字段与T2相同、输出路径不同。
- 本包SOURCE_MANIFEST逐文件保存原路径、源SHA及副本SHA；这里的默认是源C++构造默认，不是runner默认。

| key | 类型 | 单位/含义 | C++默认 | 基础YAML | notune实际Joint | Stage7 T2实际Joint | T2覆盖来源 | 使用位置 |
|---|---|---|---|---|---|---|---|---|
| ltv_enable | bool | 布尔 | false | 1 | 1 | 1 | FROZEN_SETTINGS/MODES | observer.cpp:79 enabled；Estimator::setParameter |
| ltv_log_debug | bool | 布尔 | false | 1 | 1 | 1 | FROZEN_SETTINGS/MODES | estimator.cpp:198 + csv_logger::configure |
| ltv_enable_gravity_factor | bool | 布尔 | false | 0 | 1 | 1 | FROZEN_SETTINGS/MODES | estimator.cpp:614/1422 + optimization G |
| ltv_enable_velocity_factor | bool | 布尔 | false | 0 | 1 | 1 | FROZEN_SETTINGS/MODES | estimator.cpp:623/1454 + optimization V |
| ltv_enable_gravity_quality_gate | bool | 布尔 | false | 0 | 1 | 1 | FROZEN_SETTINGS/MODES | estimator.cpp:166/619 + G eligibility |
| ltv_enable_velocity_quality_gate | bool | 布尔 | false | 0 | 1 | 1 | FROZEN_SETTINGS/MODES | velocity_quality_gate.cpp:10 + configure；estimator.cpp:652 |
| ltv_enable_velocity_oracle_gate | bool | 布尔 | false | 0 | 0 | 0 | FROZEN_SETTINGS/MODES | estimator.cpp:188 + Oracle::configure/evaluate |
| ltv_max_features | int | 点数 | 30 | 30 | 30 | 30 | 基础YAML（未覆盖） | observer.cpp:161 + updateFeatureLifecycle |
| ltv_min_features | int | 点数 | 15 | 15 | 15 | 15 | 基础YAML（未覆盖） | observer.cpp:421/485 + warmup/snapshot |
| ltv_feature_max_missed_frames | int | 帧数 | 2 | 2 | 2 | 2 | 基础YAML（未覆盖） | observer.cpp:161 + updateFeatureLifecycle |
| ltv_warmup_camera_updates | int | 帧数 | 20 | 20 | 20 | 20 | 基础YAML（未覆盖） | observer.cpp:421/485 + warmup/snapshot |
| ltv_max_imu_dt | double | s | 0.05 | 0.05 | 0.05 | 0.05 | 基础YAML（未覆盖） | observer.cpp:122 + propagateImu |
| ltv_reset_gap | double | s | 0.2 | 0.2 | 0.2 | 0.2 | 基础YAML（未覆盖） | observer.cpp:309 + updateFeatures time checks |
| ltv_timestamp_tolerance | double | s | 0.005 | 0.005 | 0.005 | 0.005 | 基础YAML（未覆盖） | observer.cpp:309 + updateFeatures time checks |
| ltv_q_landmark | double | Q设计尺度 | 1e-4 | 1.0e-4 | 1.0e-4 | 1.0e-4 | 基础YAML（未覆盖） | observer.cpp:365/394 + updateFeatures |
| ltv_v_landmark | double | V设计尺度 | 1e6 | 1.0e6 | 1.0e6 | 1.0e6 | 基础YAML（未覆盖） | observer.cpp:99 + processNoise |
| ltv_v_velocity | double | V设计尺度 | 1e6 | 1.0e6 | 1.0e6 | 1.0e6 | 基础YAML（未覆盖） | observer.cpp:99 + processNoise |
| ltv_v_gravity | double | V设计尺度 | 1e6 | 1.0e6 | 1.0e6 | 1.0e6 | 基础YAML（未覆盖） | observer.cpp:99 + processNoise |
| ltv_initial_p_landmark | double | P0设计尺度 | 1.0 | 1.0 | 1.0 | 1.0 | 基础YAML（未覆盖） | observer.cpp:275 + rebuildState |
| ltv_initial_p_velocity | double | P0设计尺度 | 1.0 | 1.0 | 1.0 | 1.0 | 基础YAML（未覆盖） | observer.cpp:59 + initializeBaseState |
| ltv_initial_p_gravity | double | P0设计尺度 | 1.0 | 1.0 | 1.0 | 1.0 | 基础YAML（未覆盖） | observer.cpp:59 + initializeBaseState |
| ltv_covariance_floor | double | 混合状态P设计尺度 | 1e-9 | 1.0e-9 | 1.0e-9 | 1.0e-9 | 基础YAML（未覆盖） | observer.cpp:437 + sanitizeCovariance |
| ltv_covariance_failure_threshold | double | 相对谱容差 | -1e-3 | -1.0e-3 | -1.0e-3 | -1.0e-3 | 基础YAML（未覆盖） | observer.cpp:437 + sanitizeCovariance |
| ltv_camera_euler_safety | double | 设计无量纲 | 0.5 | 0.5 | 0.5 | 0.5 | 基础YAML（未覆盖） | observer.cpp:365/394 + updateFeatures |
| ltv_max_camera_substeps | int | 数值积分子步数 | 100 | 100 | 100 | 100 | 基础YAML（未覆盖） | observer.cpp:365/394 + updateFeatures |
| ltv_gravity_norm_min | double | m/s² | 7.0 | 7.0 | 7.0 | 7.0 | 基础YAML（未覆盖） | observer.cpp:485 + snapshot；estimator.cpp:599 |
| ltv_gravity_norm_max | double | m/s² | 12.0 | 12.0 | 12.0 | 12.0 | 基础YAML（未覆盖） | observer.cpp:485 + snapshot；estimator.cpp:599 |
| ltv_gravity_sigma_deg | double | deg（内部rad） | 10.0 | 10.0 | 10.0 | 10.0 | Stage7六维候选T2→frozen数值 | estimator.cpp:614/1422 + optimization G |
| ltv_gravity_huber_delta | double | 加权残差norm | 2.0 | 2.0 | 2.0 | 2.0 | FROZEN_SETTINGS/MODES | estimator.cpp:614/1422 + optimization G |
| ltv_gravity_gate_min_features | int | 点数 | 15 | 15 | 15 | 15 | FROZEN_SETTINGS/MODES | quality_gate.cpp:9 + evaluateGravityQualityGate |
| ltv_gravity_gate_max_eta_norm_error | double | m/s² | 0.5 | 0.2 | 0.2 | 0.2 | Stage7六维候选T2→frozen数值 | quality_gate.cpp:9 + evaluateGravityQualityGate |
| ltv_gravity_gate_max_normalized_innovation | double | 投影m，非NIS | -1.0 | 0.05 | 0.05 | 0.03 | Stage7六维候选T2→frozen数值 | quality_gate.cpp:9 + evaluateGravityQualityGate |
| ltv_gravity_gate_reset_cooldown_frames | int | 帧数 | 0 | 0 | 0 | 0 | FROZEN_SETTINGS/MODES | estimator.cpp:663/678 + G cooldown |
| ltv_velocity_sigma_mps | double | m/s | 1.0 | 1.0 | 1.0 | 1.0 | Stage7六维候选T2→frozen数值 | estimator.cpp:623/1454 + optimization V |
| ltv_velocity_huber_delta | double | 加权残差norm | 2.0 | 2.0 | 2.0 | 2.0 | FROZEN_SETTINGS/MODES | estimator.cpp:623/1454 + optimization V |
| ltv_velocity_gate_min_features | int | 点数 | 25 | 25 | 25 | 25 | FROZEN_SETTINGS/MODES | velocity_quality_gate.cpp:25 + evaluate |
| ltv_velocity_gate_max_normalized_innovation | double | 投影m，非NIS | 0.03 | 0.03 | 0.03 | 0.03 | Stage7六维候选T2→frozen数值 | velocity_quality_gate.cpp:25 + evaluate |
| ltv_velocity_gate_max_disagreement_mps | double | m/s | 0.5 | 0.5 | 0.5 | 0.5 | Stage7六维候选T2→frozen数值 | velocity_quality_gate.cpp:25 + evaluate |
| ltv_velocity_gate_reset_cooldown_frames | int | 帧数 | 10 | 10 | 10 | 10 | FROZEN_SETTINGS/MODES | estimator.cpp:720/735 + V cooldown |
| ltv_snapshot_max_time_error | double | s | 0.005 | 0.005 | 0.005 | 0.005 | 基础YAML（未覆盖） | estimator.cpp:592/623 + base eligibility |
| ltv_velocity_oracle_mask_path | std::string | 路径/列名 |  | "" | "" | "" | FROZEN_SETTINGS/MODES | estimator.cpp:188 + Oracle::configure/evaluate |
| ltv_velocity_oracle_mask_column | std::string | 路径/列名 | "oracle_0" | "oracle_0" | "oracle_0" | "oracle_0" | 基础YAML（未覆盖） | estimator.cpp:188 + Oracle::configure/evaluate |
| ltv_debug_csv_path | std::string | 路径/列名 |  | "~/output/ltv_debug.csv" | "/home/he/output/ltv_euroc_final_unified.partial-ba8323ce40c045bdaea2cce15bfac063/V1_01_easy/joint_v_gate.partial-7317660f45774f52ad5a1f9f290430e6/ltv_debug.csv" | "/home/he/output/ltv_stage7_tuning/all11_20261001/trials/V1_01_easy/5cf0514d06eb3235b0590f723d4ef8fa8d665c0c43c2ea2cc689126b767d6cc0/ltv_debug.csv" | effective_config输出路径；runtime为partial | estimator.cpp:198 + csv_logger::configure |

## 其他运行约定（不要漏看freq未消费）

| key | 类型/单位 | parser缺省或定义 | base | notune/T2文件值 | 覆盖/实际使用 |
|---|---|---|---|---|---|
| freq | YAML整数，名义Hz | C++没有对应读取/运行变量 | 10 | 20 | runner FROZEN写20；`rg`及完整readParameters/inputImage/trackImage未发现消费；**不是内核或frontend限频依据** |
| multiple_thread | int布尔 | YAML必需项 | 1 | 0 | frozen；stage5Replay.cpp:147额外强制0；inputImage在1时每两帧入队、0时每帧 |
| imu / num_of_cam | int | YAML必需项 | 1 / 2 | 1 / 2 | 不覆写，Estimator没有IMU时disable LTV |
| td / estimate_td | double s / int布尔 | YAML | 0 / 0 | 0 / 0 | base；t_imu=t_header+td，optimization可能更新td（此批固定） |
| estimate_extrinsic | int enum | YAML | 0 | 0 | 此批fixed；其他配置可优化ric/tic，下帧反馈LTV |
| g_norm | double m/s² | YAML | 9.81007 | 9.81007 | VINS G.z读取；LTV eta不强制设此值，G factor用-g |
| max_solver_time | double s | YAML | .04 | .04 | frozen外不变；MARGIN_OLD用80%，其他用100% |
| max_num_iterations | int | YAML | 8 | 8 | Ceres迭代上限，非LTV Euler子步 |
| max_cnt / min_dist | int点数 / pixel | YAML | 150 / 30 | 150 / 30 | VINS frontend约束，非LTV max_features=30 |
| show_track/save_image/load_previous_pose_graph | int布尔 | YAML | 1/1/0 | 0/0/0 | FROZEN_SETTINGS；save/graph不属observer核心 |
| acc_n,gyr_n,acc_w,gyr_w | double，宿主噪声参数 | YAML | .1/.01/.001/.0001 | 同base | VINS预积分，**不是LTV V/Q** |
| body_T_cam0 | OpenCV 4×4 | YAML | 实际EuRoC标定 | 同base | R_BC与p_BC，完整矩阵保存在上述YAML，不与camera1混用 |
| output_path | string路径 | YAML | ~/output/ | 实验最终目录 | runner最后覆写；runtime为partial，effective为published |

## 覆盖顺序与模式

`run_stage6_joint.py:87 + effective_config`：base → frozen → MODES → output/log路径。Stage7 `run_stage7_tuning.py:49 + config`先在frozen上覆盖六个白名单数值，再执行同一模式规则；MODE开关不可由六维候选绕过。CLI仅config/cache/replay/output/dry-run选择，没有任意LTV数值CLI。Stage6 `--preserve-config-frequency`移除freq覆盖，源C++仍没有消费freq。

| 模式 | observer | G factor | G gate | V factor | V gate | Oracle |
|---|---|---|---|---|---|---|
| baseline B | 1 | 0 | 0 | 0 | 0 | 0 |
| gravity_only G | 1 | 1 | 1 | 0 | 0 | 0 |
| velocity_only | 1 | 0 | 0 | 1 | 0 | 0 |
| joint | 1 | 1 | 1 | 1 | 0 | 0 |
| joint_v_gate W0/T2 | 1 | 1 | 1 | 1 | 1 | 0 |

旧notune说明的“固定Orcal gate”与上表/actual YAML冲突：V_gate实际为在线heuristic gate，Oracle关。W0→T2只变G normalized innovation .05→.03；q、V、P0、Euler floor/warmup/feature count都不变。

## 文件值到实际配置的二次处理

- `LtvObserver::configure:28`：max_features≥1；min_features∈[1,max]；missed≥0；warmup≥1；safety≥1e-3；substep cap≥1。仅observer内部clamp其config副本，Estimator gate另存config，所以异常max_features可能产生两个副本不一致；当前配置为合法值，不触发。
- `Estimator::setParameter:139`：无IMU/camera时关LTV；LTV关时关factor/V门控/Oracle；非法G sigma/Huber/snapshot误差关G factor；G feature门槛clamp、cooldown≥0；非法G门限关G gate，可能退回无gate资格。
- V sigma/Huber由每次base eligibility检查；V quality gate的invalid/conflict使pass为false。Oracle加载失败fail closed；本批Oracle关闭。
- Q/V/P0/reset_gap/IMU_dt等未全部校验finite/正性；不要把可解析配置等同论文可接受配置。
- 空debug_csv_path时parser补`OUTPUT_FOLDER/ltv_debug.csv`；原logger支持`~/`且会trunc，别用旧实验路径配置新的验证。
- T2 runtime/published的算法标量已逐项比较；路径差是publish动作，不是不同算法。文件SHA与SOURCE_MANIFEST一致不等于当时历史源码能由当前HEAD完整重建。
