# LC-04–06 离线联合参考证据

本目录给出执行结果，适用范围为独立构造的低维联合参考，不是 OpenVINS 实际误差模型的校准。所有融合保持 OFF。真实 seed/Observer 局部映射、真实主状态交叉块和独立 landmark 真值必须由对应在线/受控片段证据另行闭合。

合同在首次采样前写入 `experiment_contract.json`。随机种子 20261008，每种情景 12000 次独立抽样；没有重采样、调阈值或删除不利幅度。全部高斯投影方差使用 n−1 自由度 χ² 区间，以族显著性 0.01 对预先登记的 995 个投影做 Bonferroni 修正。交叉协方差区间由 sum/difference 投影方差区间重建。原始预测矩阵、样本矩阵、逐项区间及合同/脚本哈希保留在 `results/`。

## 已执行结果

| 项目 | 判定 | 实测证据及范围 |
|---|---|---|
| LC-04 小系统残差消元 | PASS（参考范围） | 7 个情景：物理独立源映射直接生成残差，与 R/N/S、视觉交叉块及广义 Joseph 块公式一致；最大矩阵误差约 1e−15 |
| LC-04 重复信息 | PASS（参考范围） | 完全消去的重复约束有效秩0、增量信息0；额外非零视觉重复反例有效残差秩2，但给定已有视觉后信息秩0 |
| LC-05 置零离线对照 | PASS（实验执行） | 7 个情景，各保存完整及置零模型的谱、K、Kr、P变化、条件信息；全跨消费者块置零仅在离线函数使用 |
| LC-06 高斯联合统计 | PASS（参考范围） | 7 个正确情景的所有预测边缘/交叉项均在同时区间内；零噪声、独立、共享IMU/bias、bearing、seed回灌、重复与近秩亏均覆盖 |
| LC-06 共享来源故意破坏 | PASS（反例被识别） | 保持各消费者边缘分布，分别重抽共享源，15 个协方差项拒绝；预登记归一化相关误差0.35的正态近似功效达到0.99要求 |
| LC-06 控制非线性 stereo-depth | 范围有限 | 固定幅度1e−4、1e−3未检出线性化偏差；幅度1e−2有3项偏差，保留为 `LINEARIZATION_DEVIATION`，没有选择有利幅度覆盖它 |
| 实际 OpenVINS seed/Observer 校准 | NOT_EVALUATED | 本参考不执行实际 C++ seed/Observer。需要冻结真实局部输入、原生误差 Jacobian 和同源受控扰动 |
| 真实 landmark 统计校准 | NOT_EVALUATED | 没有独立 landmark 真值；EuRoC 位姿及共享 bearing 不是独立点真值 |

`duplicate_information` 的完整模型保持主先验 trace=1.10，信息秩0；交叉块置零模型产生虚假 P 缩减。非零的完全重复视觉观测反例避免把结论局限于零残差；条件于视觉之后无新增信息，但删掉相关性会出现假信息。全消费者间跨块置零保留块对角 PSD，因此该对照模型不兼容次数为0；这不证明它是正确物理模型。另有故障注入检查：只删除三个高度相关标量中的一个交叉块使联合矩阵非PSD，求解明确拒绝，未使用 ridge/裁剪修复。

`x` 为三维低维误差 true-minus-estimate，`a/t` 为两时刻点误差 estimate-minus-true，`v` 为二维视觉残差；全部使用归一化单位。固定 A 是二维平面旋转嵌入 SO(3)，H 是已登记线性映射。本参考没有 quaternion/JPL 注入，因此不能替代实际主系统的原生 JPL Jacobian 检查。物理源为15维独立单位高斯，同一列在所有消费者间共享，来源ID在矩阵产物内保存；anchor/current分别表示冻结的 t0/t1。Monte Carlo 的主、anchor、current、visual 消费者使用同一抽样行，没有把 Euler 子步的重复 bearing 当成新独立随机输入。

残差计算两条路径独立实现：源系数直接矩阵乘法及源空间 SVD，与联合边缘/交叉块消元及协方差特征分解。条件于视觉后的信息增量也用源空间正交投影核对。有效秩阈值为 `1e-11 * max(max_abs_eigenvalue,1)`，未修复负谱。所谓近秩亏场景把 `1e-14` 方差按已冻结分辨率视为数值不可分辨；输出保存全部谱。

控制非线性片段使用 stereo seed 深度 `z=b/disparity`，主深度/bearing/disparity为相关物理源。每个消费者都使用同一源扰动。Gaussian χ² 区间对这些非线性输出仅用于检查线性化偏差，不宣称为非线性分布的有效置信区间，也不作为通过真实统计校准的依据。高噪声幅度接近低 disparity，线性化偏差是此合同应报告的边界。

## 复核

```bash
OPENBLAS_NUM_THREADS=1 python3 scripts/ltv_landmark_correlation/reference/run.py \
  --output /tmp/ltv_joint_reference_check
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover \
  -s scripts/ltv_landmark_correlation/reference -p 'test_*.py' -v
```

实测退出码0，6项独立测试全部通过。`math_and_lifecycle_checks.json` 文件只承载此参考的消元/统计检查；文件名遵循总合同产物命名，**不包含 LC-03 在线生命周期事件验证**。本结果不授予 landmark ON 资格；必须等待实际 cross blocks、完整真实 shadow、来源拆分和可识别增量信息。

补充数值审计：`residual_support.csv` 区分联合协方差PSD与具体r的可支持性。原offline对照同一固定r=[.1,-.08,.05]在零噪声的两个模型、完全重复的完整模型、近秩亏的完整模型共4处不在S的有效子空间。原Kr/P仅为形式协方差诊断，这4处不构成可提交候选；没有通过投影r伪造物理可解性。有效子空间/range条件分别沿用原rank阈值1e-11和矩阵绝对误差1e-10，未调整验收容限。重复信息的P假收缩反例仍成立，物理完全重复情况下r应为0且不增加信息。复核：`OPENBLAS_NUM_THREADS=1 python3 scripts/ltv_landmark_correlation/reference/residual_support.py`，退出0。
