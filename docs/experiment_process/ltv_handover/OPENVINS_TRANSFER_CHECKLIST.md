# OpenVINS 跨框架约定与迁移检查表

本任务没有目标OpenVINS工作空间/commit，没有实现目标更新。下列勾选项留给目标Codex。官方文档仅帮助定位惯例，不能替代目标HEAD审计：[rotation/propagation](https://docs.openvins.com/propagation.html)、[JPLQuat](https://docs.openvins.com/classov__type_1_1JPLQuat.html)、[IMU state](https://docs.openvins.com/classov__type_1_1IMU.html)。这些文档描述global→IMU旋转、JPL左乘误差以及`[q,p,v,bg,ba]`；目标改版仍须逐项确认。

## 第一关：源核心数值复现

- [ ] 记录目标pwd/root/branch/HEAD/status、Eigen/compiler、license；比对本包source_core四个SHA，不改变源副本。
- [ ] 原样编译源observer，按parity README运行全部事件；JSON浮点用明示absolute+relative容差，slot/reset/substeps/valid等精确比较；不比较wall time。
- [ ] 验证非零omega、Ba/Bg、R_BC及p_BC，连续多帧correction，N≠M，add/miss/middle-delete/compact，healthy warmup，time tolerance，重复frame、gap、reset和substep上限。
- [ ] 独立复核P交叉项保留、symmetry/finite/PSD；知道源alias探针已复现风险。若修复`.eval()`，先保留源golden并明确这属于新算法变更，不要求与改前逐位相同。
- [ ] 最初关G/V，保持目标惯性预测与视觉主更新；不得把camera Euler子步数变成多次独立伪测量。

## 第二关：输入、时间、状态身份

| 要核对 | 源工作空间已确认 | 目标必须明确 |
|---|---|---|
| body/IMU轴 | 全部raw IMU及bias在B轴 | IMU intrinsic/axis校准、sensor→IMU映射是否已在别处完成；不能叠加校准或重复减bias |
| specific force | `a=raw_acc-Ba`，gravity另由eta表示 | 绝不能输入世界去重力acc；若输入已校正数据，应清楚地只扣一次bias |
| bias顺序 | API是accel_bias再gyro_bias；VINS SpeedBias=`V,Ba,Bg` | 用目标named accessors，不按vector列号猜；官方目标IMU通常`q,p,v,bg,ba` |
| R方向 | 源Hamilton R_WB，`v_B=R_WB^T V_W` | 目标R_BW直接乘V_W；不可直接把JPL四元数coeffs塞Eigen Hamilton |
| gravity | LTV物理向下g，VINS g符号相反 | 从目标预测方程核对具体gravity符号，不凭变量名/9.81猜 |
| camera外参 | `x_B=R_BC x_C+p_BC`，p为camera中心在B | 如果目标配置是`x_C=R_CB x_B+p_CB`，先验证矩阵定义后才用`R_BC=R_CB^T,p_BC=-R_CB^T p_CB` |
| bearing | camera0去畸变 `[x,y,1]`再单位化 | 明确使用哪台相机/多cam策略；给pixel必须先模型反投影，feature ID不能每帧重新编号 |
| offset | `imu_t=camera_header+td` | 目标时间偏移符号和变动时前后积分端点，不能照搬相同变量名 |
| boundary | 源下一条IMU值截短dt，无线性插值 | 用源harness数值parity后再决定目标适配，若沿用目标插值，应记录预期差；严禁改原目标prediction |
| time uniqueness | source核心propagate只收dt、无sample去重 | 独立记区间/last fed timestamp，禁止同区间多喂；禁止用接收时间替header time |
| camera time | frame严格增，imu累计和camera错配≤tolerance | 明确正camera_dt；不要利用源容差允许负dt；单样本、offset变化、丢包、长gap专测 |
| reset | core无epoch；start会清原因 | 新epoch只start一次，记录触发reset原因；清快照/冷却/时间/feature ID复用边界 |
| snapshot | 值快照不包含P/slot等，冻结在主求解前 | 用物理时刻+epoch标识一次消费，不使用源滑窗索引或clone近邻替代时间对齐 |

## 第三关：G/V 辅助更新的重新推导

```text
source Ceres r = prediction - measurement
target EKF innovation = measurement - prediction
target H = derivative of prediction wrt target error state
```

- [ ] 定义`h_g=normalize(R_BW*g_phys,W)`、`z_g=normalize(eta_LTV)`及rank2方向误差；目标误差旋转/FEJ要求需重新推导H，有限差分应调用目标真实`update`，不是Eigen右扰动。
- [ ] 定义`h_v=R_BW*V_W`、`z_v=v_LTV`；验证world/body速度单位及零position/direct-bias导数。相关性可带来间接bias影响，不能因源factor直接bias列零就假设EKF与bias无关。
- [ ] 正确选择目标current state，或明示重建clone时刻全IMU速度；clone pose不能替代clone velocity，source窗口有SpeedBias而目标clone不一定有。
- [ ] 依据目标左乘JPL误差state重新推导H；源3×7是人工Manifold局部列布局，不是真ambient导数，不能直接用作EKF矩阵。
- [ ] 重做测量R/权重与统计gate；源sigma是工程regularization，Huber不是目标统计chi-square门控的等价转换，`innovation/sqrt(M)`不是NIS。
- [ ] 明确G与V共享原IMU/bearing、二者互相相关以及与主MSCKF视觉/IMU数据相关；不能把P_LTV或它的eta/v块直接作为独立sensor R。联合G/V协方差或保守辅助设计需单独给证据。
- [ ] 明确先后更新、主视觉更新前后、bias反馈时机、FEJ与可观性策略；不得反复利用同frame/frozen snapshot增信息。
- [ ] 开关独立关闭、base资格+独立G/V gate、epoch与cooldown；复现前看清源非法G gate会disable、V gate会fail closed的不同策略，不声称已有安全保证。

## 最后验收与限制

- [ ] 先通过全部源parity，再新增目标适配器时序/坐标/Jacobian验证，再做经用户授权的小数据集验证。
- [ ] 不移植第二层pose observer、世界landmark更新、VINS marginalization/Ceres/滑窗/Oracle或旧ROS replay；保持原惯性预测。
- [ ] baseline/binary/effective config/evaluator/匹配支持集分开留证，迁移ATE若无改善也如实记录；源T2仅为EuRoC调参集结果。
- [ ] 不把动态路标的global warmup解释为每点收敛，也不从源小测试推断目标理论GES/AGAS、统计一致性、gate安全或跨数据集改善。
