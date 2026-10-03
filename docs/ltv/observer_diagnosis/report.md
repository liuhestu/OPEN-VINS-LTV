# OpenVINS / VINS-Fusion LTV 差异与 observer 排查

本轮结论：**第一层 observer 在 VINS 上也没有给出比原估计器更准确的速度。最终 ATE 差异首先需要区分融合、门控与实验口径；独立核心试验则已确认特征生命周期足以造成严重收敛问题。**目前没有证据支持用一个时间偏移、轴符号修改或简单增大 LTV 权重解决它。

## 1. VINS 的最终效果与第一层精度是两件事

只读核对 VINS 仓库 `docs/euroc_tuned_results.md` 和 `docs/ltv_handover/evidence/stage7_T2_effective_config.yaml`：

- T2 使用全部 11 条 EuRoC 调参，共 78/80 次启动；平均绝对 ATE 比 B 降低约 2.15%，比 W0 增加约 0.17%。选参目标是逐序列相对变化均值，不能与绝对均值混用。
- 并非完全不退化：T2 的 MH05、V1_02 相对 B 分别退化约 2.30%、1.72%。本轮不重跑 MH04。
- T2 的 G/V quality gate 均开启，**oracle gate 关闭**。G 要求重力模长误差≤0.2、normalized innovation≤0.03、特征≥15；V 要求特征≥25、innovation≤0.03、与原速度差≤0.5 m/s、reset cooldown=10。
- OpenVINS 冻结 C0 的 quality gate 关闭，NIS gate 开启。两者名义 sigma 都为 G=10°、V=1 m/s，但 VINS 使用 Huber delta=2 和滑窗多个状态的非线性因子；OpenVINS 是当前状态视觉与 G/V 一次联合 EKF 更新，保留完整协方差。**相同 sigma 不意味着相同有效权重。**
- 前端、相机模型、噪声、约束时序也不同。VINS G 三维方向差与 OpenVINS 二维切平面残差在小角度附近相关，但有限角度下不相同。共享 IMU/视觉造成的相关性在当前 OpenVINS 独立噪声近似中仍未建模。

进一步读取历史 VINS Passive 日志，在完全相同时间戳、同一 GT 机体系速度参考、双方有效的公共样本上比较（单位 m/s）：

|序列|样本|VINS LTV RMSE|OpenVINS LTV RMSE|VINS prior RMSE|OpenVINS prior RMSE|
|---|---:|---:|---:|---:|---:|
|V1_01|2614|0.4925|0.5137|0.0542|0.0498|
|V2_02|1790|0.8586|0.8615|0.0341|0.0408|
|V2_03|757|1.0093|1.0415|0.0539|0.0599|

双方 raw LTV 同量级且远差于各自 prior。G 的 VINS/OpenVINS RMSE 分别为 2.701/2.782°、2.926/2.616°、3.699/3.451°，也不存在 VINS observer 全面占优。

这不是匹配配置的整机 A/B；VINS prior 是其图像处理日志位置的优化前估计。仍足以否定“VINS ATE 好说明第一层输出更准”。OpenVINS 在这些序列的轨迹 ATE 基线更好，但 V2_02/V2_03 的公共样本原生速度反而是 VINS 更准，不能由 ATE 推断每一个状态量的精度。

## 2. 直接运行实际核心：特征更替的因果对照

新增 harness 直接链接 `ov_msckf/src/ltv/ltv_observer.cpp`，没有复制算法来代替测试。无噪声解析轨迹、非单位旋转、非零相机外参、准确时间、30 个固定世界点、20 Hz 相机、200 Hz IMU；所有 observer 参数保持原值。只改变输入 feature ID 的寿命，同一物理 bearing 不变，各点错开更替。

每次运行 60 秒，以下统一统计 30–60 秒，**不是只统计误差较小片段**：

|运动/ID|V RMSE m/s|G RMSE °|valid 比例|
|---|---:|---:|---:|
|普通运动，稳定 ID|0.5341|1.7647|100%|
|普通运动，0.5 秒 ID|1.5948|5.0266|100%|
|普通运动，1 秒 ID|1.5408|5.0241|100%|
|普通运动，2 秒 ID|1.5323|5.0249|100%|
|普通运动，5 秒 ID|1.5252|5.0057|100%|
|平移幅度 ×3，稳定 ID|**0.0995**|**0.3170**|100%|
|平移幅度 ×3，1 秒 ID|**4.7208**|**15.1702**|100%|
|静止，稳定 ID|5.97e−13|2.20e−7|100%|
|普通运动，稳定 ID，1000 Hz IMU|0.5400|1.7854|100%|
|普通运动，1 秒 ID，1000 Hz IMU|1.5401|5.0234|100%|

全部无 reset。真实观测几何残差≤1.3e−14；生产 IMU propagation 与独立逐向量叉积参考差≤8.9e−16。普通运动即便稳定 ID，在 60 秒内也并非完全收敛。更细 IMU 步长没有消除短寿命问题。

原 VINS 交接核心另行编译，复跑两个快速运动对照：source/target 每时刻速度误差最大差≤1.7e−13 m/s，重力角误差最大差≤5.6e−11°。因此上述退化来自共用核心行为，不能归因于 OpenVINS EKF 或本轮 alias 修复。此处不是替代历史 174 事件 parity 验收。

代码机制：新路标状态从零开始，P 初值为 1，和旧状态的交叉块为零；旧路标完整交叉块保留。`valid` 只依据有限值、特征数量和连续健康帧，**不是收敛或可观性保证**。投影观测 `C(p−pBC)=0` 在 `p=pBC` 时瞬时也成立，不能仅凭小 bearing 残差确认深度/尺度正确。频繁补入新路标可能破坏充分的持续激励与收敛过程；“重力吸收加速度”尚未被独立证明为唯一机制。

### 实际选中路标：两次 Passive 插桩验证

对 V1_01 和 UZH indoor_forward_3 使用旧冻结二进制/库、相同输入与配置，仅通过 `LD_PRELOAD` 包装实际 `updateFeatures()`，调用原实现后读取公开 state/slot。两次退出码均为 0，完整回放完成；**trajectory.csv 和 audit.csv 与旧 Passive 均逐字节相同**。开始跟踪 slot 后均无 core reset；启动前的 `reconfigured` 状态不计为中途重置。

|指标|V1_01|indoor_forward_3|
|---|---:|---:|
|已结束 slot 驻留寿命中位数|2.000 s|0.431 s|
|已结束驻留数|1402|2643|
|结束时仍存活、右删失|30|30|
|GT 位姿支持内，观测对应驻留年龄 <1 s|30.3%|79.3%|
|GT 位姿支持内，负 ray depth 比例|45.4%|59.5%|
|GT 位姿支持内，内部距离中位数|0.056 m|0.093 m|

这里的寿命是 observer **选中 slot 的驻留时间**，并非整个 tracker 的 track age；负深度与距离来自内部路标状态，不是 OpenVINS 的三角化结果，也没有路标 GT 来直接计算深度误差。初始化瞬态包括在统计中。负深度和接近相机中心的内部位置表明，小投影残差不能直接视作物理有效的路标估计。UZH 的 GT 支持片段更集中于运动，全程统计与支持内统计明显不同，因此两者均保存在 JSON，不能混报。

真实输入存在短驻留，且与合成实验的失效条件相容；这提高了“路标更替/收敛”解释的可信度，但还没有完成真实序列中只改变寿命的反事实实验，不能把它称为唯一根因。

## 3. 时间戳、轴和公式假设

对既有 9 条 Passive 做离线检查，不改任何校准参数：

- 任意正交变换之后，速度误差仍至少为 `RMSE(||v_ltv||−||v_gt||)`。EuRoC 三条这个下界是 0.375/0.575/0.609 m/s，远大于 prior 的 0.050/0.042/0.062。因此轴置换、符号、固定旋转无法单独解决幅度误差。
- 在固定公共支持上扫描 ±100 ms，步长 10 ms。EuRoC 最佳网格误差仍为 0.500/0.860/0.949 m/s；最优落在 +100 ms 边界，不能据此识别真实偏移，更不能把它写入配置。这只是小偏移不能解释大误差的证据，不能证明所有时间链路无问题。
- 已有原生 Jacobian、协方差、FEJ 验收通过；本轮实际核心的运动/观测方程对照也通过。降低了简单符号、坐标或 retraction 错误的可能性，但不等于证明连续观测器在变化维度、短轨迹、相关噪声下仍满足收敛条件。

## 4. 如何处理用户提出的可能原因

|假设|本地证据与判断|
|---|---|
|滤波融合没融合好|有实质方法差异：门控、Huber、窗口状态、线性化与相关性。值得控制变量对照，但第一层自身差不能全由 EKF 解释。|
|参数没有调好|可能，尤其可信度/门控与收敛时间；现有证据不支持先扩大权重。VINS 在全部 EuRoC 上调参，OpenVINS 本轮采用冻结开发/验证分割。|
|OpenVINS 太精准|轨迹改善空间可能更小；不能泛化成所有 prior 状态更准。公共速度比较已给反例。|
|时间戳不对|没有发现能解释主要量级的固定小偏移；动态延迟和离散化影响尚不能完全排除。|
|公式错误|基础局部公式有原生验证；最值得查的是改变路标集合后的收敛假设与 valid 判据，不能把局部 Jacobian PASS 当作全局证明。|
|EuRoC 太简单，UZH 会更好|快速激励且长期跟踪时合成实验更好；快速运动配短轨迹反而很差。换更激烈数据集不是自动解法，已有 UZH 冻结研究也未证明稳定收益。|

## 5. 产物与复现

计划见 [plan.md](plan.md)。紧凑机器结果见 [evidence](evidence/)。完整 CSV、编译/运行 stdout、stderr、退出码和命令存放 `/home/he/output/openvins_ltv_observer_diagnosis_20261003`，不提交大日志。

工具位于 `scripts/ltv_observer_diagnosis/`：`core_probe.cpp`、`run_probes.py`、`offline_checks.py`、`lifecycle_trace.cpp`、`run_lifecycle.py`。编译核心探针：

```bash
g++ -std=c++14 -O2 -DNDEBUG -Wall -Wextra -Wpedantic \
  -Iov_msckf/src -Iov_core/src -I/usr/include/eigen3 \
  ov_msckf/src/ltv/ltv_observer.cpp scripts/ltv_observer_diagnosis/core_probe.cpp \
  -o /home/he/output/openvins_ltv_observer_diagnosis_20261003/core_probe
python3 scripts/ltv_observer_diagnosis/run_probes.py
python3 scripts/ltv_observer_diagnosis/offline_checks.py
```

本轮没有改生产核心、融合公式、参数、VINS 源码或原 golden；没有 ATE 搜索。进一步结构性实验应优先验证路标信息积累/初始化和基于收敛的有效性判断，再做匹配门控/鲁棒性的融合对照，避免用退化很小掩盖辅助量本身无增益。
