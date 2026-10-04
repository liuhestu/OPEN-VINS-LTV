# G/V 无收益原因分析：冻结 16f02e3

**本轮结论以 B 为主，并有分支级 C：readiness 错过了部分有精度优势的输出；已发生更新的权重也确实很弱。不能把它简化成“全程 LTV 更准，只要降噪声就会受益”的 A。V2_02 的 G 属于 C，停止在这两条序列上继续调 G 参数；V 的正常就绪段总体没有更准，也不开展全程降噪声试验。**

V2_03 的 G/V 和 V2_02 的 V 在预定义的主误差增大段存在局部优势，尤其 V2_03 的 V，主要问题是该输出没有连续满足原 readiness 合同。先定位供给及确认连续性；本次已经给出首个失效原因、逐帧健康诊断和错过的区间，不改变 readiness 阈值或算法。局部 A 型证据保留为后续研究线索，尚不足以单独启动噪声敏感性实验。

源码冻结为 `16f02e30a4ba8308e112eb2c96d30046672007ec`，只读源码在 `/home/he/output/ltv_gv_no_benefit_20261005/baseline_16f02e3`。本次只增加离线脚本和报告，复用上一轮最终 OFF/G/V/GV；**新增真实回放 0，新增参数组合 0，未运行视觉退化对照，未实施 landmark ON**。原 R4 账本仍为真实开始 66 次，未因离线分析增加。

## 支持、坐标与局限

预先固定的定义见 [protocol.md](protocol.md)。同一 OFF 初始化支持、相机到 IMU 时间和 body 坐标上比较；GT 发布速度线性插值，姿态 SLERP，间隙 ≤10 ms；同时保留既有最近邻 20 ms 支持边界和输出 1 μs 检查。没有姿态拟合、尺度或时间优化。GT 和 IMU 的 `T_BS` 均为单位变换。GT 从未进入在线门控。

共同支持为 V2_02 2252 帧、V2_03 1799 帧；初始化后因 GT 边界分别缺少 14、7 帧。LTV 重力另有 3、4 帧零方向，保留为缺失角度；速度均有当前输出。不同分支的启动段保留，不能从全程统计中删掉。

OFF 主状态取同时间视觉更新后状态，LTV 输出在视觉更新阶段；为避免取样阶段偏差，另比较 G/V/GV 的冻结主先验。后者已经受此前融合影响，不能视为独立 OFF；两种比较均支持这里的主要解释。

“视觉弱”仅指 OFF visual_rows 的下 20%（含等于阈值），阈值为 54 / 26 行；有些视觉行少是特征更新调度而非信息真正退化。它不是新的在线门控，也不等同于专门的视觉退化试验。区间切分还在相邻时间间隔超过中位相机周期 1.5 倍处断开，避免把缺输入的时间画成连续证据。

逐帧误差、残差向量、各状态块修正向量、NIS、gain、ready/实际事件在外部 `V2_*_curves.csv`；[statistics.csv](statistics.csv) 保存所有模式及区间的 RMSE 和分位数统计，[summary.json](summary.json) 保存完整分母、缺失、相关性和事件计数。帧间相关，不把帧数当作独立试验样本数。

## GT 精度：先看可用就绪输出

下表均为 OFF 同时间主后验与本模式 LTV，误差 RMSE；ready_OFF 是原就绪帧的条件统计，同时报告全程和 startup，未缩小原始覆盖分母。

| 序列 | 分支/单位 | 区间 | 配对帧 | 主 RMSE | LTV RMSE | LTV 更准比例 |
|---|---|---|---:|---:|---:|---:|
| V2_02_medium | G/° | all | 2249 | 0.845279 | 1.572145 | 41.7% |
| V2_02_medium | G/° | startup | 77 | 0.446722 | 7.108452 | 5.2% |
| V2_02_medium | G/° | ready_OFF | 1592 | 0.861908 | 0.874137 | 44.3% |
| V2_02_medium | G/° | normal | 1328 | 0.826857 | 0.838291 | 43.8% |
| V2_02_medium | G/° | weak_visual | 264 | 1.020118 | 1.035806 | 47.0% |
| V2_02_medium | G/° | interruption | 580 | 0.839605 | 0.882225 | 39.1% |
| V2_02_medium | V/m/s | all | 2252 | 0.038929 | 0.253584 | 35.3% |
| V2_02_medium | V/m/s | startup | 109 | 0.036875 | 1.132961 | 5.5% |
| V2_02_medium | V/m/s | ready_OFF | 1557 | 0.036296 | 0.044646 | 37.6% |
| V2_02_medium | V/m/s | normal | 1300 | 0.035395 | 0.044362 | 36.8% |
| V2_02_medium | V/m/s | weak_visual | 257 | 0.040550 | 0.046055 | 41.2% |
| V2_02_medium | V/m/s | interruption | 586 | 0.045504 | 0.055408 | 34.8% |
| V2_03_difficult | G/° | all | 1795 | 0.867004 | 1.848752 | 60.3% |
| V2_03_difficult | G/° | startup | 131 | 1.173746 | 6.191313 | 63.4% |
| V2_03_difficult | G/° | ready_OFF | 706 | 0.847577 | 0.832976 | 60.6% |
| V2_03_difficult | G/° | normal | 620 | 0.855949 | 0.844413 | 59.0% |
| V2_03_difficult | G/° | weak_visual | 86 | 0.784578 | 0.745352 | 72.1% |
| V2_03_difficult | G/° | interruption | 958 | 0.831050 | 0.806878 | 59.7% |
| V2_03_difficult | V/m/s | all | 1799 | 0.063456 | 0.329387 | 41.1% |
| V2_03_difficult | V/m/s | startup | 135 | 0.106262 | 1.175939 | 10.4% |
| V2_03_difficult | V/m/s | ready_OFF | 637 | 0.039717 | 0.055159 | 33.3% |
| V2_03_difficult | V/m/s | normal | 562 | 0.038695 | 0.056874 | 33.1% |
| V2_03_difficult | V/m/s | weak_visual | 75 | 0.046667 | 0.040038 | 34.7% |
| V2_03_difficult | V/m/s | interruption | 1027 | 0.067756 | 0.079942 | 50.0% |

V2_02 的 G 就绪 RMSE 为 0.862° 对 0.874°，误差模长相关系数 0.973；G-only/GV 的冻结先验复核也没有优势。V2_03 的 G 就绪 RMSE 有约 1.7% 优势，但误差模长相关系数仍为 0.926。共同误差与依赖主位姿的 seed、共享视觉输入相容，**不是仅凭相关系数证明信息完全重复**。

V 在两条序列的全体就绪帧 RMSE 都更差，正常段也更差。全程 RMSE 更被启动期的未收敛输出放大。不能仅从主误差大的局部好帧反推全程降低噪声的合理性，GT 选出的好帧也不能在线作为选择器。

## 主误差较大且正在上升：存在优势，但部分没有更新

预定义为 OFF 分支误差上四分位、较 1 s 前更大（端点容差 25 ms）。下面 ready 和 interruption 是固定 OFF 标签；OFF 精度优势与 GV 同阶段先验比较并列。实际事件属于 GV、只计 receipt 消费且行存在、joint_applied、一次 EKF。

| 序列 | 分支 | 条件 | 帧 | OFF 主 / LTV RMSE | GV 先验 / LTV RMSE | OFF LTV 更准帧 | GV 实际更新 |
|---|---|---|---:|---:|---:|---:|---:|
| V2_02_medium | G | ready_OFF | 268 | 1.358291 / 1.361672 | 1.358537 / 1.361686 | 141 | 268 |
| V2_02_medium | G | interruption_OFF | 85 | 1.506026 / 1.530608 | 1.506632 / 1.530724 | 32 | 0 |
| V2_02_medium | V | ready_OFF | 292 | 0.062686 / 0.054873 | 0.066225 / 0.054870 | 205 | 292 |
| V2_02_medium | V | interruption_OFF | 160 | 0.074064 / 0.059094 | 0.077226 / 0.059093 | 107 | 0 |
| V2_03_difficult | G | ready_OFF | 97 | 1.127672 / 1.082974 | 1.127991 / 1.082948 | 69 | 97 |
| V2_03_difficult | G | interruption_OFF | 139 | 1.183256 / 1.127823 | 1.182200 / 1.127513 | 96 | 0 |
| V2_03_difficult | V | ready_OFF | 42 | 0.081023 / 0.054633 | 0.084463 / 0.054439 | 35 | 42 |
| V2_03_difficult | V | interruption_OFF | 204 | 0.109609 / 0.082909 | 0.112318 / 0.081844 | 168 | 0 |

V2_03 V 的全部高且上升区间为 268 帧：42 ready、204 interruption、22 startup。204 个中断帧里 168 个 LTV 更准（82.4%），主/LTV RMSE 0.109609/0.082909 m/s；这些帧没有实际 V 更新。这是 B 的直接证据。42 个就绪帧仍有较明显优势，GV 先验/LTV 0.084463/0.054439 m/s，39 个分支修正方向降低了对应先验速度误差，但改变量太小，不能等同于显著轨迹改善。

V2_02 V 有 292 个就绪高且上升帧，205 个相对 OFF 更准；另有 160 个中断帧，107 个更准。局部精度和权重同时受限，不能说全程仅被 readiness 卡住。V2_02 G 在同样区间没有稳定优势；V2_03 G 有局部优势，但中断帧数超过就绪帧数。

## 实际联合更新的强度：分别看单位与状态块

增益由原始完整 P、压缩 H 的状态列映射及 S 重算；分支贡献取该分支 K 列乘残差。未把姿态、速度、位置合成一个范数。下表为共同 GT 支持上的 GV 实际分支贡献 P95；它是一阶联合贡献，不能当作额外独立 EKF 或轨迹因果收益。G/V 单独工具的对应统计也在 CSV 中。

| 序列 | 分支 | 实际帧 | 残差范数 P95 | NIS P95 | 姿态修正 ° P95 | 速度修正 m/s P95 | 位置修正 m P95 |
|---|---|---:|---:|---:|---:|---:|---:|
| V2_02_medium | G | 1592 | 0.00642324 | 0.00135438 | 6.5464e-06 | 2.78893e-07 | 3.29885e-07 |
| V2_02_medium | V | 1557 | 0.090119 | 0.00812119 | 8.95958e-06 | 1.873e-06 | 1.24495e-06 |
| V2_03_difficult | G | 706 | 0.00827235 | 0.00224647 | 1.43573e-05 | 6.01256e-07 | 8.61568e-07 |
| V2_03_difficult | V | 637 | 0.0969223 | 0.00939384 | 2.1813e-05 | 2.97365e-06 | 2.43159e-06 |

G 残差为二维无量纲切平面分量，V 为 m/s；G NIS dof=2、V dof=3。共同支持内门控尝试数等于实际更新数，本轮 NIS 拒绝不是收益不足的主要原因。全输入实际 G/V 帧数为 1606/1571 与 706/637，与原报告一致；支持边界以外的帧只从 GT 指标中排除，不从原事件账本删除。

| 序列 | 分支 | 姿态 gain P95 rad/残差单位 | 速度 gain P95 (m/s)/残差单位 | 位置 gain P95 m/残差单位 | 投影先验方差/噪声方差中位数（全事件） |
|---|---|---:|---:|---:|---:|
| V2_02_medium | G | 3.32792e-05 | 0.000103066 | 9.24297e-05 | 1.07527e-05 |
| V2_02_medium | V | 4.352e-06 | 4.358e-05 | 3.22072e-05 | 1.3828e-05 |
| V2_03_difficult | G | 4.25324e-05 | 0.000108791 | 0.000174111 | 1.24964e-05 |
| V2_03_difficult | V | 8.01501e-06 | 5.78543e-05 | 4.6583e-05 | 1.62914e-05 |

原 G 10°、V 1 m/s 噪声相对于主先验投影方差占绝对主导，投影先验/噪声约 10⁻⁵，实际使用包含视觉行的联合增益，不能从噪声比例直接反推单独融合收益。因此极小 gain 和约微度、微米、微 m/s 的贡献有可复核的数值原因（[noise.json](noise.json)）。姿态完整协方差包含 yaw，不用其三轴平均标准差冒充重力方向不确定性。低 NIS 反映保守噪声及小残差，不是外部相关性已经建模或 GT 一致性已经通过。

## B 的供给及确认原因

依据冻结 LtvReadiness 的原条件做只读诊断：15 个观测和成熟点，至少 20 次修正，prediction angle ≤0.02 rad、G 修正速率 ≤1、V ≤0.5、重力模长 [8,11.5]，20 连续 strict-good 确认；原 soft grace 最多两帧/0.1 s，事件间隙超过 0.2 s 会清零确认。这些是原阈值，不是本轮新增阈值。

下面只统计 OFF 中 interruption 且高误差上升、LTV 更准的帧，原因可重叠；“确认未满20”还要求录到 last_strict_good 等于本帧 observer time，不能仅凭 reason=ready 推断两个分支都 ready。

| 序列 | 分支 | 错过的更准帧 | strict-good 已记录但确认未满20 | 成熟点不足15 | 观测不足15 | 无有效修正诊断 | prediction angle 超限 |
|---|---|---:|---:|---:|---:|---:|---:|
| V2_02_medium | G | 32 | 16 | 3 | 0 | 0 | 13 |
| V2_02_medium | V | 107 | 91 | 11 | 6 | 7 | 5 |
| V2_03_difficult | G | 96 | 73 | 22 | 10 | 11 | 1 |
| V2_03_difficult | V | 168 | 80 | 78 | 59 | 67 | 12 |

这些序列全程 physical_fault_count=0、bootstrap_count=1。这里不是对物理恢复成功率的验收，R4 恢复和合成配对缺口继续保留。供给薄弱时 observer 仍保留有限当前状态，但 readiness 为 false；恢复供给后 strict-good 确认重新累积，期间有更准输出也不触发融合。可复核原因及每个 ready→not-ready 转换的观测数、成熟数、速率、角度和时间间隙见 [supply.json](supply.json)。

以下为最长的已观察连续错过片段（时间相对首个冻结相机包；不跨缺输入间隙）；只作为实例，全部片段均在 JSON/CSV 中：

| 序列 | 分支 | 起止 s | 帧 | 主 / LTV RMSE |
|---|---|---|---:|---:|
| V2_02_medium | G | 54.000–54.700 | 15 | 1.148817 / 1.091280 |
| V2_02_medium | G | 113.350–113.700 | 8 | 1.557303 / 1.518931 |
| V2_02_medium | V | 27.550–28.400 | 18 | 0.117450 / 0.069146 |
| V2_02_medium | V | 75.150–75.950 | 17 | 0.122527 / 0.050913 |
| V2_03_difficult | G | 24.150–24.500 | 8 | 1.149041 / 1.037769 |
| V2_03_difficult | G | 98.150–98.450 | 7 | 1.215995 / 1.141833 |
| V2_03_difficult | V | 97.850–98.900 | 22 | 0.115792 / 0.056684 |
| V2_03_difficult | V | 27.150–27.750 | 13 | 0.141756 / 0.082958 |

后续若专门处理 B，先用原输入核对供给不足首帧、特征成熟恢复首帧、每次确认重置以及恢复 ready 的确切时间。不得凭离线 GT 的好帧放宽门限或跳过确认；是否修改原合同另立供给/恢复验收。当前已经完成只读原因定位，未重构任何在线代码。

## 决策与下一阶段边界

1. **C：V2_02 G。** 正常、弱视觉、总体就绪和主误差上升段没有稳定优势，停止这两条序列的 G 噪声调优。G 的共享误差值得研究，但不能假设加权就会得到新信息。
2. **B：V2_03 V，及 V2_03 G 的局部优势。** 关键区间存在更准输出，主要错过在供给/成熟不足和健康确认等待；先处理该供给/恢复合同。
3. **V2_02 V 具有局部 A 型证据，也有 B。** 292 个高误差就绪帧内更准并被极小权重采用，但全体就绪及正常段反而更差。故本轮不认定“全程 A”，不设计/运行噪声敏感性实验。任何未来 A 试验必须在本报告之后预先固定极小参数集合，并保留从未用于诊断或选参的独立实录验证；已经看过 GT 的这两条及历史全 EuRoC 结果不能充当盲验证。

既有证据已经区分权重、readiness 和精度，暂不需要追加那一组视觉退化 OFF/ON。不会为获得 A 结论扩大实验。当前 ATE 微小变化仍只是有效结果，不宣称显著收益。

原因分析可交给下一阶段的 landmark **残差合同设计**，不进入 ON 实现。沿用 [已有合同](../gv_fusion_evaluation/landmark_contract.md)：seed 的世界点依赖主位姿，不能作为独立地图回灌；当前 max_slam=0，主状态没有对应 SLAM 路标。下一份残差设计必须写清锚定/epoch/坐标/时间、同一视觉信息重复使用和外部交叉相关性，并先证明“在哪些区间 LTV 相对量可能更准且可用”，不能从本轮局部 G/V 优势推断 landmark 会有收益。

## 完整曲线

- [V2_02_medium G GT误差、供给、ready/更新](figures/V2_02_medium_G_errors.png)；[残差、NIS、姿态/速度/位置贡献](figures/V2_02_medium_G_updates.png)。
- [V2_02_medium V GT误差、供给、ready/更新](figures/V2_02_medium_V_errors.png)；[残差、NIS、姿态/速度/位置贡献](figures/V2_02_medium_V_updates.png)。
- [V2_03_difficult G GT误差、供给、ready/更新](figures/V2_03_difficult_G_errors.png)；[残差、NIS、姿态/速度/位置贡献](figures/V2_03_difficult_G_updates.png)。
- [V2_03_difficult V GT误差、供给、ready/更新](figures/V2_03_difficult_V_errors.png)；[残差、NIS、姿态/速度/位置贡献](figures/V2_03_difficult_V_updates.png)。

## 复现与验证

外部目录 `/home/he/output/ltv_gv_no_benefit_20261005` 保存紧凑原始抽取 NPZ/JSON、全部时间 CSV、日志和身份；原矩阵、缓存、轨迹保持在上一轮外部目录，不复制或修改。`read_manifest.json` 的 24 个输入诊断文件已与上一轮提交的哈希清单精确匹配。

从仓库运行（全部设置 OPENBLAS_NUM_THREADS=1、OMP_NUM_THREADS=1；最多两个计算进程，本轮无 ROS 回放）：

```bash
python3 scripts/ltv_gv_diagnosis/analyze.py /home/he/output/ltv_gv_fusion_evaluation_20261005 /home/he/output/ltv_gv_no_benefit_20261005
python3 scripts/ltv_gv_diagnosis/supply.py /home/he/output/ltv_gv_fusion_evaluation_20261005 /home/he/output/ltv_gv_no_benefit_20261005
python3 scripts/ltv_gv_diagnosis/noise.py /home/he/output/ltv_gv_fusion_evaluation_20261005 /home/he/output/ltv_gv_no_benefit_20261005
python3 scripts/ltv_gv_diagnosis/render.py /home/he/output/ltv_gv_no_benefit_20261005 docs/ltv/gv_no_benefit_diagnosis
python3 scripts/ltv_gv_diagnosis/verify.py /home/he/output/ltv_gv_fusion_evaluation_20261005 /home/he/output/ltv_gv_no_benefit_20261005
python3 -m unittest discover -s scripts/ltv_gv_diagnosis -p "test_*.py" -v
```

6 项测试通过，验证涵盖原生 JPL/body 转换、GT 时间插值与缺口、完整 P 的状态列映射和分支增益分解、原生 quaternion 修正、缺失分母和互斥标签。另与既有物理 GT 工具逐项复核，OFF 4072 个主轨迹状态与矩阵完全相同，诊断分区与 receipt 实际数一致。[verification.json](verification.json) 记录结果。离线派生结果重复核对数值确定性；G/V/GV 每帧的 G/V readiness 与 OFF 相同，[determinism.json](determinism.json) 保存核对结果。没有生产 C++/ROS/config 变更，沿用 16f02e3 的既有实际编译和测试证据。

离线脚本开发曾有两次退出 1：区间字典混入分支子字典、OFF 无 prior 的空 Rotation；已修复并加入缺失状态/分区检查。它们不是回放异常或滤波失败，不计为新真实实验。最终离线命令退出 0，命令、身份、预算及派生文件哈希见 [delivery.json](delivery.json)。
