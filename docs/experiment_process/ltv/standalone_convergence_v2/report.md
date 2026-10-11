# LTV 第一层独立验证 v2

状态：**COMPLETED**。完整仿真尝试 58/64；成功 58。大数组、完整矩阵、每次命令与日志：`/home/he/output/ltv_standalone_convergence_v2`。

基线为 `43215b84e90c18d79d55528c811d85ab22fc04a8`。本轮只使用解析合成输入；没有启动 ROS、OpenVINS、VINS-Fusion，没有使用真实序列进行仿真或评价 ATE。首次 SHA 冻结曾意外读取仓库 `ov_data` 参考轨迹的字节用于哈希，没有解析或用于科学评价；后续已排除，详见 [decision_log.md](decision_log.md)。冻结清单含论文、执行文档、配置、生产源码、交接副本及历史证据；末尾复核生产、配置、论文、执行文档、交接副本及历史证据未变；数据目录不再读取复核。

模型依据为冻结的[本地 2025 ACC 论文](../../Pose_Velocity_and_Landmark_Position_Estimation_Using_IMU_and_Bearing_Measurements.pdf)式 (10)/(14)、(15)–(17) 与 (19)。CONT_REF 使用原始、未缩放的完整 Riccati 方程与 DOP853。SPLIT_REF 的冻结 IMU 子流使用解析矩阵指数/多项式噪声积分，冻结观测子流使用精确信息更新的 Woodbury 等价式。CORE 直接编译生产源码；EXPERIMENTAL 是同一核心的均值 hook/数值计数副本，未修订数值算法。参数、寿命及初始化阶段以已通过完整时域 parity 的 EXPERIMENTAL 为离散目标。首次相机事件校正长度为零。共同 x(0)/P(0) 定义在首帧建槽及规定初始化之后；准确全状态初值的 t=0 建槽前六维记录只作事件审计，不冒充初始化后的估计值。

所有误差保留启动瞬态，共同物理采样率 200 Hz；相机前后另存。宽档为速度 0.10 m/s、重力方向 1°、模长 0.20 m/s²、路标相对三维误差 5%；严档为 0.05、0.5°、0.10、2%。固定点表格与选参的 settling 合并共同 200 Hz 检查及相机前/后全部样本，要求持续至结束且余下至少 5 s；未达标仅表示当前时域内未观测到。

## 1. 固定点质量

|运行/场景/实现|全程 V RMSE|全程 η RMSE|全程路标 RMSE|末段 V RMSE|末段 η RMSE|末段路标 RMSE|宽/严 t_VG|
|---|---:|---:|---:|---:|---:|---:|---|
|01_REGULAR_CONT_REF_C0 60s ZERO |1.58986|1.74797|11.251|0.309909|0.183516|3.02894|未在时域内观测到 / 未在时域内观测到|
|02_REGULAR_CONT_REF_C0 60s ZERO tight|1.58986|1.74797|11.251|0.309909|0.183516|3.02894|未在时域内观测到 / 未在时域内观测到|
|03_REGULAR_SPLIT_REF_C0 60s ZERO |1.59121|1.74835|11.2517|0.310107|0.18353|3.02917|未在时域内观测到 / 未在时域内观测到|
|04_REGULAR_CORE_C0 60s ZERO |1.57398|1.74347|11.2251|0.299691|0.178156|2.95153|未在时域内观测到 / 未在时域内观测到|
|05_FAST_CONT_REF_C0 60s ZERO |2.43977|2.0243|7.4146|0.0142729|0.00841241|0.0478609|40.775 / 46.45|
|06_FAST_CONT_REF_C0 60s ZERO tight|2.43977|2.0243|7.4146|0.0142729|0.00841241|0.0478609|40.775 / 46.45|
|07_FAST_SPLIT_REF_C0 60s ZERO |2.44128|2.02463|7.41593|0.0142814|0.00841315|0.0478904|40.805 / 46.455|
|08_FAST_CORE_C0 60s ZERO |2.42169|2.02081|7.38958|0.0144964|0.00904254|0.0147129|37.855 / 43.555|
|09_PAPER_CONT_REF_C0 20s ZERO |1.30635|2.59008|0.946562|0.262086|0.174907|0.348781|11.45 / 13.53|
|10_PAPER_CONT_REF_C0 20s ZERO tight|1.30635|2.59008|0.946562|0.262086|0.174907|0.348781|11.45 / 13.53|
|11_PAPER_CORE_C0 20s ZERO |1.27508|2.57553|0.93726|0.248804|0.181264|0.329058|10.555 / 未在时域内观测到|
|12_REGULAR_CONT_REF_C0 10s EXACT |1.16946e-14|1.54496e-14|1.82016e-14|1.16946e-14|1.54496e-14|1.82016e-14|0 / 0|
|13_REGULAR_EXPERIMENTAL_C0 10s EXACT |0.00610436|0.00560822|0.0221362|0.00610436|0.00560822|0.0221362|0 / 0|
|14_STATIC_CONT_REF_C0 60s ZERO |1.18613|1.61583|18.7967|1.56995e-14|4.27878e-14|18.819|4.85 / 5.785|
|15_REGULAR_EXPERIMENTAL_C0 60s ZERO |1.57398|1.74347|11.2251|0.299691|0.178156|2.95153|未在时域内观测到 / 未在时域内观测到|
|20_REGULAR_CONT_REF_C0 120s ZERO |1.58986|1.74797|11.251|0.021272|0.0129685|0.208963|85.975 / 97.605|

参考加严验收（逐个 200 Hz 时刻、完整状态和完整 P）：

|普通 / 加严|最大归一化状态差|最大完整 P 相对差|≤1e−6|
|---|---:|---:|---|
|01_REGULAR_CONT_REF_C0 / 02_REGULAR_CONT_REF_C0|2.39649e-14|2.16311e-14|True|
|05_FAST_CONT_REF_C0 / 06_FAST_CONT_REF_C0|1.24843e-14|2.12254e-14|True|
|09_PAPER_CONT_REF_C0 / 10_PAPER_CONT_REF_C0|3.61546e-14|5.90328e-14|True|
|31_REGULAR_CONT_REF_C6 / 35_REGULAR_CONT_REF_C6|2.41978e-14|2.36969e-14|True|

末段列：60 s 运行取 50–60 s，PAPER 取规定的 5–20 s，准确初值短测取 0–10 s。120 s 延长行的全程列保留原 0–60 s，末段列为 110–120 s，完整 120 s 曲线另存。每次运行的全部规定区间、三轴误差、P95/max、速度范数 RMS 比、角度有效分母、模长误差、逐点严格/宽档 settling 及相机前后统计见 `evidence/summary.json` 和外部 `trace.npz` / `camera.npz`。角度在 η≈0 时为无定义，不计为零。

## 2. 离散来源

|场景 / A−B|速度差 RMSE|η 差 RMSE|最大归一化 x 差|整秒 P 最大相对差|
|---|---:|---:|---:|---:|
|REGULAR CORE−SPLIT_REF|0.0226804|0.015511|0.0794896|0.0222785|
|REGULAR SPLIT_REF−CONT_REF|0.024218|0.013608|0.0779535|0.0265145|
|FAST CORE−SPLIT_REF|0.0342464|0.0212062|0.0805039|0.0233796|
|FAST SPLIT_REF−CONT_REF|0.0309218|0.0182004|0.0784022|0.0286432|

|实现 / IMU / camera Hz|末段 V RMSE|末段 η RMSE|最大相机子步|
|---|---:|---:|---:|
|SPLIT_REF 200 / 20|0.310107|0.18353|0|
|CORE 200 / 20|0.299691|0.178156|2|
|CORE 1000 / 20|0.306423|0.182453|2|
|CORE 200 / 100|0.300335|0.178137|1|
|CORE 1000 / 100|0.307091|0.182435|1|
|SPLIT_REF 200 / 100|0.309575|0.18352|0|

CORE−SPLIT_REF 隔离子流积分、谱保护和实现；SPLIT_REF−CONT_REF 包含采样、bearing 冻结与分步组织。相机校正时间 τ 不推进物理时间。floor 计数只统计特征值被抬至下限的实际动作，不是谱检查/重构的总调用次数。生产核心未暴露谱保护计数；只有 EXPERIMENTAL 的计数是真实插桩值，不能把 CORE 日志的占位零解释为未触发保护。

## 3. 增益对照

冻结诊断选择：**C6**（SELECTED_DIAGNOSTIC）。排序只用宽档 t_VG，未达标或分辨不了时用 50–60 s 的 J_tail。严档及其他区间不参与事后改换目标。允许诊断候选增加瞬态峰值，不宣称全面改善。

|候选 / 实现|Q / V / P0|宽 t_VG|J_tail|200 Hz 速度峰值|全程 V RMSE|耗时 s|最大相机子步|floor 次数 / 累计谱抬升|
|---|---|---|---:|---:|---:|---:|---:|---|
|C0 CONT_REF|0.0001 / 1e+06 / 1|未在时域内观测到|4.01666|6.77949|1.58986|304.543|—|0 / 0|
|C0 EXPERIMENTAL|0.0001 / 1e+06 / 1|未在时域内观测到|3.88769|6.8857|1.57398|29.3434|2|0 / 0|
|C1 CONT_REF|1e-05 / 1e+06 / 1|未在时域内观测到|4.2107|8.17931|1.80859|297.81|—|0 / 0|
|C1 EXPERIMENTAL|1e-05 / 1e+06 / 1|未在时域内观测到|4.04697|8.19393|1.77195|28.5572|1|0 / 0|
|C2 CONT_REF|0.001 / 1e+06 / 1|未在时域内观测到|3.93949|6.25923|1.51057|325.31|—|0 / 0|
|C2 EXPERIMENTAL|0.001 / 1e+06 / 1|未在时域内观测到|3.83682|6.49404|1.51601|35.8653|9|0 / 0|
|C3 CONT_REF|0.0001 / 100000 / 1|未在时域内观测到|4.21069|8.17927|1.80858|298.062|—|0 / 0|
|C3 EXPERIMENTAL|0.0001 / 100000 / 1|未在时域内观测到|4.04695|8.19389|1.77195|28.6671|1|0 / 0|
|C4 CONT_REF|0.0001 / 1e+07 / 1|未在时域内观测到|3.93949|6.25924|1.51057|328.507|—|0 / 0|
|C4 EXPERIMENTAL|0.0001 / 1e+07 / 1|未在时域内观测到|3.83682|6.49404|1.51601|36.0642|9|0 / 0|
|C5 CONT_REF|0.0001 / 1e+06 / 100|未在时域内观测到|4.01652|6.7791|1.58978|302.978|—|0 / 0|
|C5 EXPERIMENTAL|0.0001 / 1e+06 / 100|未在时域内观测到|3.88754|6.88531|1.57391|29.4465|2|0 / 0|
|C6 CONT_REF|0.0001 / 1e+06 / 10000|未在时域内观测到|4.00184|6.74043|1.58254|299.566|—|0 / 0|
|C6 EXPERIMENTAL|0.0001 / 1e+06 / 10000|未在时域内观测到|3.87275|6.84633|1.56664|29.4936|2|0 / 0|

C* 相对 C0 的瞬态：`{"v": {"base_peak": 6.927056918772952, "candidate_peak": 6.887583049652526, "peak_increase": -0.039473869120426386, "peak_ratio": 0.994301494908545, "duration_above_C0_peak": 0.0, "duration_above_C0_peak_after_10s": 0.0, "C0_peak_after_10s": 2.115510725036533, "candidate_peak_after_10s": 2.1073821788399414, "duration_above_late_C0_peak_after_10s": 0.0, "rmse_full": 1.5666359074306373, "rmse_tail": 0.2985404437713512}, "eta": {"base_peak": 9.81, "candidate_peak": 9.81, "peak_increase": 0.0, "peak_ratio": 1.0, "duration_above_C0_peak": 0.0, "duration_above_C0_peak_after_10s": 0.0, "C0_peak_after_10s": 1.234908084155495, "candidate_peak_after_10s": 1.2300771370001227, "duration_above_late_C0_peak_after_10s": 0.0, "rmse_full": 1.7403044054576524, "rmse_tail": 0.17746971644897142}}`。峰值使用共同 200 Hz 与相机前/后两侧的最大值；超过该 C0 峰值的持续时间按共同 200 Hz 完整时间轴梯形积分（0.005 s 时间分辨率），全部种类峰值另见 `peaks_all_samples.json`；10 s 后的值另列。

|噪声候选 / seed|J_tail|末段 V RMSE|末段 η RMSE|宽档目标内时间比例|
|---|---:|---:|---:|---:|
|C0 / 42|12.1265|0.936968|0.551371|0|
|C0 / 43|12.0322|0.929454|0.547523|0|
|C6 / 42|12.1228|0.936678|0.551196|0|
|C6 / 43|12.0283|0.929155|0.547344|0|

噪声只做两种子的敏感性检查，不是充分统计验证。`evidence/noise_response.json` 另报同候选带噪输出相对无噪输出的扰动 RMSE，以免收敛偏差改善掩盖噪声放大。IMU 与 bearing 噪声均为冻结的每样本噪声，C0/C* 同种子完全共用；不得将不同采样率下的每样本噪声混为同一连续谱密度。`gain_diagnostics.npy` 保存 K 范数、三类实际 K r、P_vL/P_etaL 及各块尺度；`gain_landmark_contributions.npy` 保存逐点 K_i r。离散诊断取相机事件前的已驻留状态与当前可见观测，连续诊断取整秒。`camera_correction_deltas.npz` 从实际相机前后状态中扣除 slot 重排与一次均值写入，给出真实净校正量，包含全部相机子步的最终作用；不把它冒充每个子步内部的瞬时 K r。

## 4. 寿命对照

|场景 / 实现 / 候选 / 寿命 s|physical bearing SHA 前12位|驻留中位数 s|完整观测轨迹最后可见前路标宽档达标率|末段 V RMSE|末段 η RMSE|
|---|---|---:|---:|---:|---:|
|REGULAR EXPERIMENTAL C0 0.5|a667d51ac212|0.5|0|1.58206|0.912797|
|REGULAR EXPERIMENTAL C0 1|a667d51ac212|1|0|1.53346|0.908667|
|REGULAR EXPERIMENTAL C0 2|a667d51ac212|2|0|1.52714|0.908249|
|REGULAR EXPERIMENTAL C0 5|a667d51ac212|5|0|1.52442|0.904061|
|FAST EXPERIMENTAL C0 0.5|5ced0931f68f|0.5|0|4.71139|2.71023|
|FAST EXPERIMENTAL C0 1|5ced0931f68f|1|0|4.70685|2.73232|
|FAST EXPERIMENTAL C0 2|5ced0931f68f|2|0|4.71701|2.74092|
|FAST EXPERIMENTAL C0 5|5ced0931f68f|5|0|4.68387|2.71088|
|REGULAR SPLIT_REF C0 1|a667d51ac212|1|0|1.54105|0.907343|
|REGULAR EXPERIMENTAL C6 1|a667d51ac212|1|0|1.53248|0.907323|
|FAST EXPERIMENTAL C6 1|5ced0931f68f|1|0|4.7104|2.73126|

最后可见与删除分别记在 `observations.json` / `lifecycle.json`，成功截止于最后可见，要求先连续保持 0.10 s 并在最后可见采样仍达标。首次保持段的起点年龄与确认年龄（至少再等待 0.10 s）分别保存，不把起点当作在线确认时刻。末端仍存活点与观测仍延续点分别标记右删失。表中成功率的分母只含观测已结束的入槽轨迹；所有轨迹截至时域末的达标比例、右删失分母、v/g/路标联合达标率，以及更保守的“最后可见前连续 0.10 s 始终达标”比例另见 `evidence/track_outcomes.json`，没有将右删失当作必然失败。未入槽观测、观测出生、驻留出生均有记录；实际首次非零校正在 `events_verified.json`，其中剔除了 t=0 后未再可见、实际没有被校正的启动轨迹。原始 `observations.json` 的该字段在这些点上是下一相机时刻的计划值，不能当成发生过的校正。驻留中位数包含末端截断样本，右删失数量另列，不能当作无偏寿命估计。稳定 ID 只是理想身份连续参照。

## 5. 一次初始化

|场景 / 模式|接受数 / 驻留数|观测延迟中位数 s|种子误差 RMSE m|完整观测轨迹最后可见前路标宽档达标率|末段 V RMSE|末段 η RMSE|末段路标 RMSE|
|---|---|---:|---:|---:|---:|---:|---:|
|REGULAR ZERO|— / 1827|—|—|0|1.53346|0.908667|14.8114|
|FAST ZERO|— / 1827|—|—|0|4.70685|2.73232|15.583|
|REGULAR GT_SEED|1827 / 1827|0.1|0|0.941111|0.00682675|0.0061673|0.00341206|
|REGULAR GEOM_IDEAL|1824 / 1827|0.15|1.4406e-10|0.938333|0.0429127|0.0297004|3.0153|
|REGULAR GT_MATCHED|1824 / 1827|0.15|0|0.938333|0.0429127|0.0297004|3.0153|
|REGULAR GEOM_NOISY|1823 / 1827|0.1|4.08615|0.115|0.371293|0.21344|5.08465|
|FAST GT_SEED|1827 / 1827|0.1|0|0.941111|0.0140098|0.00972821|0.00417336|
|FAST GEOM_IDEAL|1824 / 1827|0.1|5.93062e-11|0.938889|0.0137532|0.00974962|0.481736|
|FAST GT_MATCHED|1824 / 1827|0.1|0|0.938889|0.0137532|0.00974962|0.481736|
|FAST GEOM_NOISY|1824 / 1827|0.1|2.07581|0.326111|0.415236|0.20889|2.07855|

GT_SEED 是入槽真值诊断；GEOM_IDEAL 使用同 ID 过去/当前 bearing 与精确模拟相机位姿；GT_MATCHED 复用完全相同触发规则、ID 和时刻；GEOM_NOISY 使用 seed42 的位置/姿态噪声，三角化和执行时的机体系变换均使用带噪先验。至少三帧且跨度 0.10 s，条件数≤1e8、最大视线夹角≥0.5°、残差≤0.5°、正射线，最多 1 s 历史。每点最多写一次，只改该点均值，P 和 v/η 均值规则保持原样。初始化瞬间的完整 x/P 由已保存的相机事件前状态、实际 slot 重排及冻结种子输入精确重建，另存 `initialization_snapshots_reconstructed.npz`；该文件明确标记重建来源，不冒充额外现场采样，P 为均值写入前后共用矩阵。全部保存的协方差快照与同场景零种子运行逐值相同，见 `evidence/initialization_covariance_parity.json`。这是额外位姿先验诊断，不是已完成在线融合。

## 6. 几何与完整证据

有限窗口指标不乘 Q，按物理点/连续 ID 的世界 bearing 计算；固定 ID 的 1/5/10 s 完整窗口见 `evidence/geometry.json`，窗口不完整为缺失。这些有限窗口不能证明全时间 uniform PE。S_PAPER 使用文档假设的 4×4 地面网格、P0=I、R(t)=Exp(t[−1,3,0]×) R_y(−2t) 与有符号理想 bearing；相机 Z 负比例独立记录，未翻轴，未裁剪后方点，也未按 Fig.3 读图拟合。

所有运行的 `trace.npz` 保留三维路标误差、相对误差、ray distance、投影横向误差、距相机中心距离、沿射线误差；近相机中心定义为 <0.1 m。矩阵文件保留整秒与生命周期事件前后完整 P；连续加严比较额外保留每个 200 Hz 时刻的完整 P。`energy_cross_blocks.npy` 用线性求解计算 U，不显式求逆。

- [fixed_REGULAR](figures/fixed_REGULAR.png)（另有独立 SVG）
- [fixed_FAST](figures/fixed_FAST.png)（另有独立 SVG）
- [fixed_PAPER](figures/fixed_PAPER.png)（另有独立 SVG）
- [fixed_STATIC](figures/fixed_STATIC.png)（另有独立 SVG）
- [sampling_rates](figures/sampling_rates.png)（另有独立 SVG）
- [sampling_differences](figures/sampling_differences.png)（另有独立 SVG）
- [gains_full_and_tail](figures/gains_full_and_tail.png)（另有独立 SVG）
- [noise](figures/noise.png)（另有独立 SVG）
- [lifecycle_initialization_REGULAR](figures/lifecycle_initialization_REGULAR.png)（另有独立 SVG）
- [landmark_age_REGULAR](figures/landmark_age_REGULAR.png)（另有独立 SVG）
- [lifecycle_initialization_FAST](figures/lifecycle_initialization_FAST.png)（另有独立 SVG）
- [landmark_age_FAST](figures/landmark_age_FAST.png)（另有独立 SVG）
- [lifetimes_REGULAR](figures/lifetimes_REGULAR.png)（另有独立 SVG）
- [initialization_tail_REGULAR](figures/initialization_tail_REGULAR.png)（另有独立 SVG）
- [lifetimes_FAST](figures/lifetimes_FAST.png)（另有独立 SVG）
- [initialization_tail_FAST](figures/initialization_tail_FAST.png)（另有独立 SVG）
- [extended_full_horizon](figures/extended_full_horizon.png)（另有独立 SVG）

## 7. 结论与下一项修改

2026-10-04 补充：[S_PAPER 正向基准与前4/5/10秒激励复核](paper_baseline_review/report.md)。复用既有结果，新增完整仿真为零；研究优先级先核对论文启发场景。原始表格与身份不变。

具体归因与决策见 [conclusions.md](conclusions.md)。本报告的表格由原始结果自动生成；结论必须结合连续加严、分步比较、完整瞬态与右删失，不能以 valid 或低投影残差代替收敛。

## 8. 预算、未执行项与复现

未执行身份数：0。详见 `evidence/summary.json` 的 missing；失败与中断仍保留在账本，不从预算删除。

```bash
python3 scripts/ltv_standalone_study/run_study.py freeze
python3 scripts/ltv_standalone_study/run_study.py verify
python3 scripts/ltv_standalone_study/run_study.py fixed
python3 scripts/ltv_standalone_study/run_study.py discretization
python3 scripts/ltv_standalone_study/run_study.py diagnostic
python3 scripts/ltv_standalone_study/run_study.py gains
python3 scripts/ltv_standalone_study/run_study.py lifecycle
python3 scripts/ltv_standalone_study/run_study.py initialization
python3 scripts/ltv_standalone_study/run_study.py report
```

默认外部输出路径固定在 protocol.json；也可用 `--out` 指定新目录。仅完整身份（配置、输入生成器、代码与库版本）一致的成功结果复用，绝不复用历史 probe 汇总值。驱动进程排他锁避免重叠调度；固定点阶段串行，后续阶段最多两个模拟子进程。短测并发偏差见 decision_log.md。生产文件不写回，后续 VIO 闭环与 ATE 不属于本轮。
