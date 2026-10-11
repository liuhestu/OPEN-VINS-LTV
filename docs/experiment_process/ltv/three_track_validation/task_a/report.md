# 任务 A：固定 G/V 四模式验收

本轮冻结候选在 V2_03 的预先登记末尾 10 秒窗出现局部位置误差超限，G、V、GV 均为 **FAIL**。全程误差与首轮生产诊断开销处于容限内不能抵消该失败。参数保持固定；不开展权重扫描。第二次完整匹配重复的11个必要原始文件×4模式逐字节相同，3点局部 FAIL 精确复现；最终 **STOP** 三模式扩展，留出未消耗。

## 冻结范围与实际交付

主冻结点 `2417d036002fbfd542ae71d96d0fc734cd6d47e8`；实际运行候选源码 `939c689e11aa1ef30adbf60846ef7e5fdbc55290`，独立分支 `experiment/ltv-three-track-a-20261008`。算法源码与 ABI 对应依赖库源未变，验证两个 runner 在全新对象中编译；同任务已冻结三依赖库逐源 SHA、ABI 与库字节核对后复制到单一新 install，修复 prefix hooks，实际 ldd 全部 ov_* 位于新根。完整父子构建证明见 [runner_build_identity_v3.json](evidence/runner_build_identity_v3.json)。没有共享其他任务 overlay、对象或临时库。

C0 配置见 `config/ltv_three_track/task_a/fixed_c0/`，三 YAML 字节与历史相同。确认20、30/15容量、q=1e-4、V=1e6、P0=1、sigma G=10°、V=1m/s、质量门/Huber关闭、NIS原阈值、max_slam=0。生产默认未改。四模式只改变明确的实验 G/V flags，correlation_model 为 independence_approximation，未知 main/LTV/visual 和 G/V 交叉项没有被证明为零。

所有真实运行使用原始 EuRoC ASL 图像和 IMU，调用真实 VioManager；同一同步 camera 规则与全部 IMU 输入，未使用 GT 初始化或将 GT 传入估计器。这是同步离线完整真实数据回放，**不是 ROS transport/queue 回放**。p95 计真实 feed_measurement_camera，同一 scalar_ltv_csv 级别，缓存/大矩阵研究序列化关闭，原算法数值保护仍开启；图像解码及计时外回归 digest 不计入 feed p95。四模式 OFF 指 managed Observer 仍运行而 G/V 融合关闭，不能当作纯 OpenVINS 总接入成本。

共享输入 publisher v1 对 V2_02、V2_03、V1_01、V1_03 所有传感器 CSV、图像及 GT 重新读取 SHA；两开发集精确旧 aggregate，相应旧 OFF 16 必要原始文件全部可访问且 SHA 相符。见 [data_identity_version.json](data_identity_version.json)。留出输入已冻结但未用于设计或调参；扩大验收前必须先获得开发集候选资格。

## 真实构建、OFF 与失败保留

普通 ROS 依赖 build 和显式 `BUILD_LTV_PHASE0_TESTS=ON` 的完整 probes build 均 EXIT0。13 个相关 native tests、core harness 和完整 matrices 零容差检查全部通过；第二次短程序对照通过 child pre-exec 登记 PID/PGID/start_ticks，避免采样漏掉短命进程。源码是 C++14 兼容写法，沿用仓库 ROS2 C++17 构建链；原优化数值 flags 保留，仅末尾 -g0 控制调试信息体积。

两开发序列新 diag 对历史 OFF 主状态/完整 P、完整 Observer x/P_R/cache、lifecycle/readiness/receipt 等既定字段零容差通过。新 diag/scalar 的纯实际 visual 输出、trajectory、lifecycle、同步 roster 字节相同；全帧 main state/P、tracker、Observer x/P_R 精确通过。V2_02 为2348包/23490 IMU，V2_03 为1921同步包/23370 IMU，原416 unmatched 明确保留。

[development_failures.json](development_failures.json) 保留：漏开 probes 构建 flag 的预检失败（没有启动 tests）；scalar logging receipt 未消费的实现缺陷；跨诊断级别旧 mixed visual_digest 不可比。旧 digest 混入 passive-only 容器，批准关闭 audit 后必异；原 FAILED 未删除，改为两个新 runner 同时输出纯 good_features_MSCKF 的 algorithm_visual_identity 新文件，旧 diag→历史硬合同保持原样，新对象和完整配对全部重验。没有改变数值容限、删除算法输出或事后挑选样本。

## 新四模式完整结果

以下均来自本轮生产级第一轮真实完整运行，不是历史复用。全程 SE3 无尺度对齐，各模式只拟合一次，局部窗复用全程变换。旧 OFF 弱视觉/上升高误差/中断 mask 与固定10s bins 在 ON 之前定义；GT pose 最近20ms、trajectory匹配1us，RPE端点25ms；body速度使用发布 GT 速度与 quaternion SLERP，括号≤10ms。

| 序列 | 模式 | ATE m | body速度 RMSE m/s | 姿态 RMSE ° | 实际 G/V 次数 | p95 ms（增量） | peak RSS KiB（增量） |
|---|---|---:|---:|---:|---:|---:|---:|
| V2_02 | OFF | .051975210 | .038928815 | 1.363987665 | 0/0 | 24.651290 | 114152 |
| V2_02 | G | .051978425 | .038929388 | 1.363965060 | 1606/0 | 25.607688 (+3.880%) | 116568 (+2.116%) |
| V2_02 | V | .051973334 | .038928427 | 1.364076067 | 0/1571 | 25.833321 (+4.795%) | 115296 (+1.002%) |
| V2_02 | GV | .051977002 | .038928993 | 1.364060521 | 1606/1571 | 25.644869 (+4.031%) | 114728 (+.505%) |
| V2_03 | OFF | .207427615 | .063455951 | 1.436063353 | 0/0 | 24.351917 | 111848 |
| V2_03 | G | .207245141 | .063287357 | 1.442811743 | 706/0 | 25.159726 (+3.317%) | 111928 (+.072%) |
| V2_03 | V | .207234970 | .063284869 | 1.442664006 | 0/637 | 25.143025 (+3.249%) | 115456 (+3.226%) |
| V2_03 | GV | .207243632 | .063286471 | 1.442706179 | 706/637 | 25.258145 (+3.721%) | 110676 (−1.048%) |

没有新增初始化延迟或输出缺失，V2_02 全模式2266输出，V2_03 全模式1806输出。首轮未见 NaN/异常退出或明确资源/数据污染。peak RSS 子项处于10%门槛内；有限序列25–34个 estimator RSS时序样本显示启动分配和数 MiB 回落/增长，不能证明无限运行无增长，长期趋势资格 **NOT_EVALUATED**。V2_02 V-only p95仅距5%限0.205个百分点，单次首轮不能授予稳定生产资格；未择优重测。

1008 个全程、10s、原固定mask指标中3个 FAIL，24个无配对支持 local RPE 为 NOT_RUN_EMPTY_SUPPORT；没有把空窗计 PASS。全程 RPE1/5秒平移/旋转、姿态/速度和所有可用局部指标保留于 [metrics_first_round.csv](evidence/metrics_first_round.csv)，拒绝、bias估计首末演化和1s时间块2000次 paired bootstrap见 [events_first_round.json](evidence/events_first_round.json)。bootstrap仅描述给定轨迹效应，不将每帧或确定性重复当作独立实验。

## 实际作用与局部反例

Receipt严格要求 consumed/joint_applied/有效 branch 行/一次 EKF。更新不是仅 enabled/ready：本轮 G、V 有实际主状态作用，第一次完整状态/P分歧均对应真实辅助联合更新。branch Kr 全状态混合范数 p95：V2_02 G≈1.59e-6、V≈4.04e-6，V2_03 G≈3.70e-6、V≈7.83e-6。该范数混合 rad/m/m/s/bias等单位，不冒充单独物理 bias 校正量。真实 bias 精度无可信独立参考，NOT_RUN；其估计演化保留。极小直接校正不等于全轨迹变化只能为微米，后续 covariance/视觉反馈仍能放大影响。

冻结 V2_03 fixed_10s_11 末窗有3个 GT支持时刻，相对 OFF初始化110.05/110.15/110.25秒。OFF局部ATE .188966692m，G .194793127、V .194757039、GV .194776066，分别恶化5.826/5.790/5.809mm，超过 max(2%×OFF,2mm)=3.779mm。三模式 **FAIL**；支持只有3点不提供删除该窗的授权，也不被全程微改善抵消。

独立 Horn 四元数4×4特征 SE3 计算（不同于原 SVD实现）复现上述 RMSE到约1e-15。原生未对齐尾部位置差约13.3–13.7mm，全程最大约73mm；full alignment旋转差约.009°、平移差约.55mm，对齐后尾部差仍约13mm。因此不是仅对齐重分配。全程 tracker digest没有改变，纯视觉实际点首次改变在第251帧、首次 aux 更新下一帧，当帧 visual_rows=72，正确 joint_applied；不能据此把全部后续偏差归因某一个直接 Kr分块。详见 [因果分解](evidence/tail_v203_causal_audit.json) 和 [独立公式](evidence/tail_v203_independent_formula.json)。

## 分开裁决与接入逻辑

| 项目 | 结论 |
|---|---|
| 实现/坐标、局部Jacobian与数值保护 | relevant native tests与新OFF硬精确检查通过；不是全统计模型证明 |
| 真实有效更新 | G/V/GV均有真实作用，直接校正很弱 |
| 全程精度/首轮生产p95/peak RSS子项 | 两开发序列处于冻结容限内，仅首轮所测同步接口范围 |
| 完整逐序列工程不退化 | G/V/GV在V2_03固定末窗 FAIL，不能授予跨序列候选资格 |
| 实际收益 | NOT_DEMONSTRATED；微小全程改变与局部反例同时保留 |
| 统计一致性 | NOT_ACHIEVED；独立块近似、缺交叉项和真实校准不能由PSD或determinism补证 |
| 留出/全部可用集泛化 | NOT_RUN；失败开发候选未扩大，留出未消耗作调参 |
| A/B组合 | A不授予组合资格；B工程路径独立，集成不得继承单模块资格 |

保留 G/V 实验独立开关，生产仍 OFF、确认20、max_slam=0。第二次 V2_03 OFF/G/V/GV 在同一冻结源码/参数下完整回放，11个原始文件每模式逐字节相同：trajectory、完整state/P audit、Observer x/P_R、lifecycle/readiness、scalar receipts/fusion、同步 roster、effective/replay/instrumentation。相同3点尾窗 RMSE和 FAIL 原样复现，见 [repeat_determinism_v203.json](evidence/repeat_determinism_v203.json)。该重复是非性能批，未择优替换首轮 p95；确定性不是独立统计样本。

因此停止三模式scope扩展和参数扫描。最小下一步是基于源相关性/视觉反馈的受控模型修正及重新完整验收，不能靠放宽局部阈值或删除末窗获得资格。可以先设计保留原视觉单独update数值路径的受控 counterfactual 与相关性诊断，区分弱辅助校正和后续 covariance 反馈；本轮证据尚不能把73mm轨迹差归因单一机制，不据此猜测扫描权重。

## 可复核原始 manifest 与重跑入口

[run_manifest.csv](run_manifest.csv) 与 [attempts_index.json](attempts_index.json) 包含全部成功、队列/预检失败、修正重跑、源码/父依赖 OID、数据/config/binary/library SHA、实际 namespace/env/locks、PID/PGID/start身份、输出路径及小证据文件 hash。每项 EXIT_ZERO 只是执行状态，科学资格单独见本报告。完整大 cache/matrices 留在稳定 taskroot/results，以 manifest 的绝对路径和SHA定位；这些本机路径不是已公开下载链接。历史数据仅作为 HISTORICAL_REUSED 零容差对照，不冒充新ON。

可从本分支源码及独立 build/install 重跑；普通 colcon 必须加 -DBUILD_LTV_PHASE0_TESTS=ON 才安装验证目标，完整 argv/flags/source snapshot 和同任务依赖复用关系见各run manifest与runner_build_identity。统一协调工具使用root发布的launcher_v2，所有build/clean/run共享host_heavy与task_artifact，全局锁先于任务锁；performance批额外独占gate，其他任务明确暂停numeric。没有运行中重建，不修改原生产配置。
