# OpenVINS + LTV：可开关 G/V 联合辅助更新执行规范

**读者：OpenVINS 工作空间中的 Codex。**  
**本阶段：只迁移 LTV 第一层，并实现 Gravity / Velocity 辅助测量；不接入路标初值或路标观测。**  
**架构：原 IMU 预测不变；MSCKF 视觉约束与本帧 G/V 辅助约束合并，调用一次原生 EKF 更新。**

## 0. 本文是什么，以及不能宣称什么

本文是新实现的规范，不是“代码已通过验证”的报告。必须先读取源工作空间生成的 `LTV_HANDOVER.md`、源码清单与 `source_core/`，再执行。将整份源交接目录放入目标工作空间的 `docs/ltv_handover/` 或提供其实际绝对路径；不能假设另一工作空间的文件自动可见。

需严格区分三层：

1. **模型代数正确性**：坐标、残差、局部扰动、Jacobian、矩阵维度、协方差代数可以通过推导和测试逐项核验。
2. **原生实现正确性**：必须在本地真实 OpenVINS 的 `JPLQuat/IMU/StateHelper/Propagator` 上验证，不能用 Python 检查替代。
3. **统计一致性与效果**：LTV 与 OpenVINS 共享 IMU/bearing/bias，普通伪测量融合采用相关性忽略近似；Jacobian 全通过并不证明该近似统计一致，也不保证 ATE 不退化。

本阶段允许实现的是**显式标记的实验性相关伪测量融合**，不实现交叉估计器协方差传播，不宣称 GES/AGAS 传递到整个 OpenVINS 系统。原论文的第一层收敛条件与真实动态特征、离散化和双向 bias 使用不同。[P1]

对于用户要求的“不能出错”，执行标准是：**每个约定有源码证据，每个关键公式有测试；任一关键测试不通过即停止，禁止用调大 sigma 掩盖错误。**不能承诺未经本地执行验证的代码绝对无误。

### 0.1 源码参照与版本

本文核对的 upstream 为：

```text
repository: rpng/open_vins
commit: 69488123ed9362dd44b6f28e7f4680abbff1442b
```

本地若不同，先输出相关文件差异和实际接口，不自动切换或覆盖用户代码。源 LTV 也必须固定 local HEAD、工作区 diff、文件 SHA；在线 main 只是参考。

原有阶段材料区分了 Baseline、Passive、G 和 V；本阶段沿用这个思想，但所有指标均重新以当前 OpenVINS 基线为准，不混入旧 VINS 结果。

## 1. 范围锁定与五种模式

| 模式 | LTV observer | G 辅助 | V 辅助 | 作用 |
|---|---:|---:|---:|---|
| B | OFF | OFF | OFF | 原 OpenVINS 路径 |
| P | ON | OFF | OFF | 只递推 LTV 和诊断，不写 OpenVINS 状态/P |
| G | ON | ON | OFF | 二维重力方向辅助 |
| V | ON | OFF | ON | 三维机体系速度辅助 |
| GV | ON | ON | ON | 2+3 维辅助，与 MSCKF 视觉联合更新 |

- 所有新开关默认 OFF，旧 YAML 缺少新字段时按 OFF。
- G/V 可以分别拒绝或通过；V 不依赖 `gravity_valid`，G 不依赖 V 开关。
- 关闭 G/V 辅助，不等于关闭 LTV 内部对应状态：LTV 的 v、eta、landmarks 仍共同递推。
- LTV 内部 landmark 仍然必要。本阶段只是**不将 landmark 输出用于 OpenVINS 初始化或测量**。
- 不改原来的惯性预测、原视觉模型、特征追踪、状态维度、FEJ 主逻辑、原边缘化内核。
- 不增加已知地图 pose observer、Ceres、RL、Oracle、在线调参、回环或预测端状态覆盖。
- v1 使用固定相机/IMU标定和固定时间偏移；在线标定开启时，LTV active 模式明确报“本阶段不支持”，不能暗中把原配置改掉。
- 验证配置优先使用 MSCKF-only（无持久 SLAM 点、无 ArUco），但不删除原模块。B/P/G/V/GV 的原生配置必须相同。
- 本阶段只验收同步 camera0 或同步双目包。异步多相机同一时间多次进入状态更新的情形先识别为未支持，不隐式重复辅助更新。

## 2. 架构与函数分类

保留三个主要对象和一个小型合并工具，避免增加泛化中间件或复制第二套 EKF 内核。

```text
OpenVINS IMU/camera input
       │
       ├─ 原始 Propagator / tracker / clone state
       │
       └─ LtvAdapter
             ├─ 独立 IMU 缓存、时间边界、固定标定与 bias 校正
             ├─ feature ID 映射和当前 bearing 复制
             └─ LtvObserver：持久的 s_L、P_L
                         ↓
                   immutable snapshot
                         ↓
                   UpdaterLTV
                    只构造 G/V block
                         ↓
原 MSCKF 压缩后视觉 block + G/V block
                         ↓
                  JointMeasurement 工具
                         ↓
            原 StateHelper::EKFUpdate 一次
```

### 2.1 `LtvObserver`：论文第一层数学核心

从源工程移植，不引入 OpenVINS 类型。主要接口保持源行为：

```cpp
configure(config);
reset(reason);
start(imu_time);
propagateImu(dt, acc, gyro, accel_bias, gyro_bias);
updateFeatures(camera_time, imu_time, observations, R_BC, p_BC);
snapshot(...);
state();
covariance();
```

内部维护 `s_L、P_L、slot、warmup、reset`。不读取主滤波器状态，不自行查询 GT，不调用 EKF。

允许去掉 `../utility/utility.h` 依赖，仅将其所需 skew 函数替换成 Eigen-only 等价函数；必须通过源/目标 parity fixture。其它算法修改独立报告，不能以“重构”为名改变数值顺序。

### 2.2 `LtvAdapter`：输入、时钟与快照生命周期

建议接口：

```cpp
void feedImu(const ImuSample& raw);                  // 只缓存，不读写主滤波状态
void reset(ResetReason reason, uint64_t new_epoch);
LtvSnapshot processCamera(
    const CameraInputView& camera,
    const FrozenImuCalibration& calibration,
    const FrozenBias& bias_for_interval);            // 在单一状态处理线程调用
```

负责 IMU 区间选择/插值、固定标定、一次 bias 校正、ID 转换、持久 observer 调度。主状态读取由管理器在明确时刻复制后传入，避免跨线程访问。

不负责 EKF residual/Jacobian；不修改原 tracker 的 feature database；不重设 observer 的 v/eta 来追随每一帧 MSCKF 后验。

### 2.3 `UpdaterLTV`：测量模型与门控

建议接口分成纯数学与状态适配两层：

```cpp
LtvLinearizationContext makeContext(const State& state, const Snapshot& s) const;
MeasurementBlock buildGravity(const Context&, const Snapshot&) const;
MeasurementBlock buildVelocity(const Context&, const Snapshot&) const;
LtvDecision gate(const Context&, const MeasurementBlock&, const State& state) const;
MeasurementBlock build(const State& state, const Snapshot& s) const;
```

`build` 只输出 immutable `H/res/R/order/diagnostics/token`，不得写名义状态、FEJ、P，也不得推进 LTV。函数可放在同一 `.cpp` 中，不必为每个数学函数新建类。

### 2.4 合并工具：只组织矩阵并调用原接口

```cpp
MeasurementBlock mergeIndependentApproximation(
    const MeasurementBlock& visual,
    const MeasurementBlock& ltv);

ApplyResult applyMeasurementOnce(State& state, const MeasurementBlock& block);
```

实现按类型/协方差列范围对齐，并调用原 `StateHelper::EKFUpdate()`；不另写一套生产 Kalman filter。后者的数值预检和元数据检查可放在 UpdaterLTV 附近的小工具中。

`MeasurementBlock` 至少包含：

```text
vector<shared_ptr<ov_type::Type>> order
MatrixXd H          # rows m, columns sum(order[i]->size())
VectorXd res        # m
MatrixXd R          # m×m, discrete pseudo-measurement covariance
epoch / camera_time_ns / imu_time / state_version
row labels G/V/visual, diagnostics
```

## 3. 最少侵入的代码修改清单

### 3.1 新增文件（示意路径，先核对本地构建方式）

```text
ov_msckf/src/ltv/
  ltv_observer.h/.cpp     # 源核心，保留 source provenance
  ltv_types.h            # 精简核心类型；不携带 Ceres/Oracle/window 类型
  LtvAdapter.h/.cpp
  LtvOptions.h           # options 与校验，可 header-only

ov_msckf/src/update/
  UpdaterLTV.h/.cpp       # 纯测量模型、门控、block 合并小工具

测试目录由本地 CMake/ROS2 结构决定：
  test_ltv_core_parity
  test_ltv_gv_jacobian
  test_ltv_gv_covariance
  test_ltv_joint_layout
  test_ltv_timing
  test_ltv_disabled_path
```

### 3.2 允许最小修改的原有文件

| 原文件 | 允许修改 | 不允许顺便修改 |
|---|---|---|
| `core/VioManager.h/.cpp` | 持有 adapter/updater，给 observer 输入，在同一帧构建 block，传给 MSCKF | 传播公式、原 tracker、ZUPT 算法、重置整个 VIO 的策略 |
| `core/VioManagerOptions.h` 或本地 options 入口 | 可选解析独立 `LtvOptions`，缺省关闭 | 改原生噪声/相机标定默认值 |
| `update/UpdaterMSCKF.h/.cpp` | 可选额外 measurement 参数、联合提交点、三处 early-return 的 LTV-only 分支 | 重写三角化、零空间投影、视觉 chi2、视觉压缩 |
| package 的 CMake | 添加 LTV 源与注册测试 | 升级整个依赖栈、换求解库 |

`StateHelper.cpp`、`Propagator.cpp`、`IMU.h`、`JPLQuat.h`、`UpdaterHelper.cpp` 首版只读。若本地版本确实必须改，先解释不可替代原因，不能直接扩大范围。

### 3.3 联合更新接入点

参照 upstream `UpdaterMSCKF::update()`：原函数在视觉零空间投影、chi2、measurement compression 后构造 `sigma_pix_sq * I`，最后调用 EKFUpdate。[S2]

推荐新增一个默认空参数，保留旧调用：

```cpp
void update(shared_ptr<State> state,
            vector<shared_ptr<Feature>>& features,
            const MeasurementBlock* ltv_block = nullptr);
```

正常路径：原视觉构造/压缩保持不变，在最终 EKF 调用前合并 block。

源函数至少有三处提前返回：`feature_vec.empty()`、`ct_meas < 1`、压缩后行数为零。处理规则：

- `ltv_block == nullptr` 或空：严格走原返回，不增加空 EKF 调用。
- LTV block 非空且当前 state/time/epoch 仍有效：仅在该原返回点提交一次 LTV-only 测量，然后返回。
- 正常联合路径不再调用第二次 LTV 更新。
- 管理器仍保留初始化、传播成功、最少 clone 等原检查；不要为强行更新 LTV 绕过原初始化。

“联合一次”只约束 **MSCKF+LTV 这个阶段**，不要求重写原 SLAM 更新的其它调用。首版基线配置禁用持久 SLAM 点便于隔离验证。

## 4. 坐标、单位、误差状态合同

以下是源定义与目标定义之间必须遵守的合同，测试中用非单位旋转、非零平移验证。

### 4.1 状态

```text
R_BW = state->_imu->Rot()    世界 W → IMU/body B
R_WB = R_BW.transpose()
p_W  = state->_imu->pos()    IMU 原点在世界系的位置
v_W  = state->_imu->vel()    IMU 原点的世界系线速度
```

OpenVINS IMU 误差顺序：

\[
\delta x_I=[\delta\theta_B^T,\delta p_W^T,\delta v_W^T,
\delta b_g^T,\delta b_a^T]^T\in\mathbb R^{15}.
\]

名义 IMU 状态为16维，不可把名义 quaternion 四列当成误差态四列。[S1,S3]

### 4.2 JPL retraction 与残差符号

OpenVINS 的更新定义：

\[
q^+=\operatorname{normalize}([\tfrac12\delta\theta^T,1]^T)\otimes_{JPL}\hat q,
\quad R_{BW}^+\simeq (I-[\delta\theta]_\times)\hat R_{BW}.
\]

**左乘、JPL、矩阵扰动负号三者成套。**不能使用 Eigen Hamilton 四元数乘法代替它。[S1]

EKF：

\[
r=z-h(\hat x),\qquad r\simeq H\delta x+n,
\quad H=\frac{\partial h(\hat x\boxplus\delta x)}{\partial\delta x}\bigg|_0.
\]

`H` 是预测函数的导数，固定测量后 residual 的数值导数是 `-H`。

### 4.3 重力与 IMU 校正

OpenVINS 源码将 `_gravity=(0,0,+g)` 作为被减项；物理重力应是：

\[
g_{phys,W}=-(0,0,g)^T,\quad \gamma_W=g_{phys,W}/g.
\]

LTV 输入的 a 是 specific force，满足静止时：

\[
a_{corr,B}+\eta_B\simeq0,\qquad \eta_B=R_{BW}g_{phys,W}.
\]

**不要先从 accelerometer 减去重力，又在 LTV 中加 eta。**[P1,S4]

源核心目前在 `propagateImu` 内减 bias。目标必须固定一种合同：

- 最简单固定标定为单位时，可传 raw IMU + 冻结 bias；
- 本规范的通用固定标定适配建议：adapter 先完成与 OpenVINS 一致的校正，然后给原核心传 `corrected_acc, corrected_gyro, ba=0, bg=0`。

对应目标源码校正顺序为：

\[
a_c=R_{ACC\to B}D_a(a_m-b_a),
\]
\[
\omega_c=R_{GYRO\to B}D_w(\omega_m-b_g-T_g a_c).
\]

其中矩阵来源必须用本地 OpenVINS 的真实定义和构造函数确认；不能凭变量名猜矩阵方向。取两个区间端点校正后再平均用于原 LTV Euler。该选择属于 adapter 数值合同，必须记录，不要求 LTV 积分器与 OpenVINS RK4/analytic 相同。[S4]

两条输入合同只保留一个默认实现并测试，严禁重复减 bias。source parity fixture 直接调用核心原接口，另外用 identity calibration 测试 adapter 校正合同的等价性。

### 4.4 相机外参

OpenVINS 标定若表示：

\[
p_C=R_{CB}p_B+t_{CB},
\]

LTV 需要：

\[
R_{BC}=R_{CB}^T,\qquad p_{BC}=-R_{CB}^Tt_{CB},
\]

\[
p_C=R_{BC}^T(\ell_B-p_{BC}).
\]

注意 OpenVINS `.pos()` 在这个标定对象中是 body 原点在 camera 中的平移，不可直接当成 camera 原点在 body 中的位置。以实际方程与 round-trip test 为准。

## 5. LTV 核心：原样复用的数学与离散行为

只需在新工作空间文档中简要再确认，不重写整个 observer：

\[
s_L=[\ell_1^B,\ldots,\ell_N^B,v_B,\eta_B]^T,
\quad\dim(s_L)=3N+6.
\]

\[
\dot\ell_i^B=-[\omega]_\times\ell_i^B-v_B,
\quad\dot v_B=-[\omega]_\times v_B+\eta+a,
\quad\dot\eta=-[\omega]_\times\eta.
\]

\[
b_i^B=R_{BC}\frac{[x_n,y_n,1]^T}{\|[x_n,y_n,1]\|},
\quad\Pi_i=I-b_i^Bb_i^{B\top},\quad y_i=\Pi_i p_{BC}.
\]

观测矩阵 `C_L` 仅在对应 landmark 的三列有 `Pi_i`，其它列为零；M 个本帧观测对应 `3M × (3N+6)`，M 不一定等于 N。

\[
\dot{\hat s}_L=A\hat s_L+Ba+P_LC_L^TQ_L(y-C_L\hat s_L),
\]

\[
\dot P_L=AP_L+P_LA^T-P_LC_L^TQ_LC_LP_L+V_L.
\]

以上为论文式(8)、(10)、(14)–(19)。[P1]

源实现采用拆分：IMU 阶段 Euler 推进动力学和 `AP+PA^T+V`；相机阶段冻结 `C_L,y`，按相机间隔对 correction 项做自适应 Euler 子步。移植时保持源实现，包括首帧不 correction、feature lifecycle 与 `sanitizeCovariance` 行为。[S7]

这里的 `P_L` 即使源码叫 covariance，也先称为 **observer Riccati matrix**，不是主滤波器 `P_E`，不是 G/V 观测噪声 `Sigma_L`。

源核心的特征删除/新增与 P 重排必须保留；禁止每帧从 OpenVINS 重新赋值 v/eta，禁止把 observer 的 state 只当临时变量。

## 6. G/V 辅助测量与完整局部 Jacobian

**本节是为 OpenVINS 新增的观测模型，不是论文直接给出的融合公式。** 假设 LTV snapshot 在一次更新内固定，对其输出不反向求导。

### 6.1 速度 V

\[
z_v=\hat v_{B,L},\quad h_v(x)=R_{BW}v_W,
\quad r_v=z_v-\hat R_{BW}\hat v_W.
\]

按第4节扰动：

\[
\begin{aligned}
h_v(x\boxplus\delta x)
&\simeq(I-[\delta\theta]_\times)\hat R_{BW}(\hat v_W+\delta v_W)\\
&=\hat R_{BW}\hat v_W
+[\hat R_{BW}\hat v_W]_\times\delta\theta
+\hat R_{BW}\delta v_W.
\end{aligned}
\]

因此：

\[
\boxed{H_v=[\ [\hat R_{BW}\hat v_W]_\times\ \ 0\ \ \hat R_{BW}\ \ 0\ \ 0\ ]}.
\]

尺寸 `3×15`。实际紧凑状态顺序可以只用 `{state->_imu->q(), state->_imu->v()}`，对应 `3×6`。

位置和 bias 的直接导数为零；它们仍可因完整交叉协方差被更新。这并不表示共享 bias 相关性已被建模。

### 6.2 重力方向 G

\[
u_L=\hat\eta_L/\|\hat\eta_L\|,\quad \gamma_W=g_{phys,W}/g,
\quad d_{cur}=\hat R_{BW}\gamma_W.
\]

若 eta 非有限、过小或不满足已声明的 norm/health 条件，拒绝 G，但不自动拒绝 V。

选择单位向量 `d_lin` 的确定性切平面基 `T∈R^{3×2}`：

1. 从 x/y/z 三个单位轴中，选取与 `d_lin` 绝对点积最小者 a；相同值取固定顺序。
2. `t1 = normalize(a - d_lin * dot(d_lin,a))`。
3. `t2 = d_lin × t1`；`T=[t1,t2]`。

验证 `T^T T=I2`、`T^T d_lin=0`。**T 在此次 residual、Jacobian、门控和数值差分中固定，不在每个扰动点重算。**

\[
r_g=T^T(u_L-d_{cur}).
\]

非 FEJ 局部线性化时 `d_lin=d_cur`：

\[
\boxed{H_g=[\ T^T[d_{cur}]_\times\ \ 0\ \ 0\ \ 0\ \ 0\ ]}.
\]

尺寸 `2×15`，紧凑姿态块是 `2×3`。不新增全局重力状态，不更新 `_gravity` 常量。

**反向退化必须额外检测：** `u_L=-d_cur` 时切平面残差也是0。因此构造前计算：

\[
\alpha=\arccos(\operatorname{clamp}(u_L^Td_{cur},-1,1)).
\]

超过配置角度上限直接拒绝。初版局部更新上限建议30°，这是实验保护参数，不是论文值或理论安全保证。另检查 `u_L` 与 `d_lin` 不接近反向。

### 6.3 行数与紧凑列布局

推荐共用 `{q_current, v_current}` 的6列上下文，合并时丢弃完全无用的尾列也可，但必须规范化和测试。

```text
G only: 2 rows    [Hg_theta, 0]
V only: 3 rows    [Hv_theta, Hv_velocity]
GV:     5 rows   [ G rows ; V rows ]
none:   0 rows   不调用 EKF
```

不要把 current IMU q/v 的列映射到一个历史 clone。clone 仅有 pose，没有独立 velocity。

## 7. FEJ：残差点、线性化点与真实源码一致

### 7.1 两个点必须显式保存

`LtvLinearizationContext` 中保存：

```text
R_current, v_current               # residual 的当前值
R_linearization, v_linearization   # H 的线性化值
T, gamma_world
state_camera_time, physical_imu_time, epoch, state_version
use_fej
```

- `do_fej=false`：linearization=current，用于数学测试与显式诊断模式，不默认关闭上游 FEJ。
- `do_fej=true`：H 用 `_imu->Rot_fej()` 和 `_imu->vel_fej()`，T 用 `d_lin=R_fej gamma` 构造；residual 仍用当前状态。

\[
H_v^{FEJ}=[\ [R_{FEJ}v_{FEJ}]_\times\ \ R_{FEJ}\ ],
\quad H_g^{FEJ}=T^T[R_{FEJ}\gamma]_\times.
\]

不要混成 `[R_current v_current]_×` 与 `R_fej` 两块，也不要把冻结 LTV 测量变成一个 FEJ 主状态。

### 7.2 源码证据与执行时刻

参照 commit 的 `Propagator::predict_and_compute()` 在区间传播结束时同时调用 `_imu->set_value(imu_x)` 和 `_imu->set_fej(imu_x)`。[S4]

因此正常新相机帧的**传播后、视觉更新前**，当前 IMU value 与 FEJ 通常相同；而历史 clones 的 FEJ 不会因视觉后验修正而改变。必须用本地日志和原生测试确认，而不是假设所有版本相同。

冻结 LTV block 后不能先执行普通 MSCKF update 再把旧 block 用在新后验上；那会错配线性化与 prior。联合 block 的上下文必须与该次 visual block 使用的当前 state 版本一致。

### 7.3 数值差分与不可观测性分别测

- 测局部 Jacobian：在给定 linearization point 用原生 IMU/Quat `update()` 扰动，比较预测函数导数，T固定。
- 测 FEJ：用人为设置的不同 current 与 FEJ；res用current，FD在FEJ做，不能要求它等于current导数。
- G 对当前重力轴方向的 `H_g R_fej gamma = 0`。
- 对共同 global yaw，velocity 与 rotation 要一起变，不应要求某一个“yaw列”单独为零。

对 IMU 状态可构造 gauge basis：

\[
N_{trans}=\begin{bmatrix}0\\I_3\\0\\0\\0\end{bmatrix},\qquad
n_{yaw}=\begin{bmatrix}
R_{FEJ}\gamma\\-[p_{FEJ}]_\times\gamma\\-[v_{FEJ}]_\times\gamma\\0\\0
\end{bmatrix}.
\]

应满足：

\[
H_gN=0,\quad H_vN=0.
\]

除了瞬时 HN，还要在原生传播/克隆中构造多时刻测试，核对 `H_k Phi_{k←0} N_0≈0`；与未改动 visual FEJ 使用一致的参考点。不能在测试失败时临时投影 H 或关 FEJ 让结果“通过”。若本地传播分支本身与预设参考不一致，先报告和定位，不能归因于噪声。

FEJ 维护的是线性化可观测性，并不解决共享传感器产生的统计相关性。[S8]

## 8. 协方差：三个矩阵不得混淆

| 符号 | 所属 | 用途 | 本阶段处理 |
|---|---|---|---|
| `P_L` | LTV observer | Riccati 增益设计 | 原样递推，只诊断，不直接作为 EKF R |
| `P_E` | OpenVINS | 完整误差状态协方差，含 clones 的交叉项 | 原生传播与更新，禁止独立覆盖 |
| `Sigma_L` | 新 G/V 测量模型 | 离散辅助观测的工程权重 | 显式配置，可校验的正定矩阵 |

还有 `Q_L,V_L` 属于论文 observer，不是 OpenVINS 的 IMU white-noise/random-walk 参数，也不是同名的速度状态。

### 8.1 初版测量噪声

\[
\Sigma_G=\left(\sigma_{g,deg}\frac{\pi}{180}\right)^2I_2,
\qquad \Sigma_V=\sigma_{v,m/s}^2I_3.
\]

GV 工程原型：

\[
\Sigma_L=\operatorname{blkdiag}(\Sigma_G,\Sigma_V).
\]

- 默认供显式测试使用的起点可设 `sigma_g=10 deg`、`sigma_v=1 m/s`，不是已标定概率参数，不保证不退化。
- 原生开关仍 OFF。
- 此 Sigma 是每次离散伪测量的权重，不自动按 dt 乘除；换更新频率会改变累计信息，必须固定频率并记录。
- 原生 visual block 的 `Sigma_vis=sigma_pix² I` 保持不变。
- 不预先将 r/H 除以 sigma 后又传入相同 Sigma；本阶段统一未白化接口。
- 不复制 Ceres Huber 配置；先验证固定高斯权重和明确拒绝式 gate，不引入 IRLS。

### 8.2 共享输入相关性是显式近似，不是已解决的问题

本阶段不估计：

```text
Cov(OpenVINS prior error, LTV pseudo-measurement noise)
Cov(visual measurement noise, LTV pseudo-measurement noise)
Cov(LTV G error, LTV V error)
跨帧 LTV 输出误差相关性
```

因此需要配置并记录：

```text
correlation_model: independence_approximation
allow_correlated_pseudomeasurements: false  # 仓库默认
```

显式 G/V 测试配置才设 true。这是用户知情的实验范围确认，不是使数据变独立的数学操作。

若模型为 `r=H delta_x+n`，且 `D=Cov(delta_x,n)` 非零，则理论上有：

\[
S=HP_EH^T+\Sigma+HD+D^TH^T,
\qquad K=(P_EH^T+D)S^{-1}.
\]

本阶段调用的标准 EKF 相当于忽略这些相关项。[S6,S9] 保守 sigma、NIS gate、降频或合并一次更新均不等于求出了 D。不能在文档中写“已解决 double counting”或“协方差绝对正确”。

### 8.3 生产更新：复用原 EKF，测试用 Joseph 对照

令合并后的 measurement 为 H、r、Sigma，完整 prior 为 P：

\[
M=PH^T,\quad S=HPH^T+\Sigma,
\quad K=MS^{-1},\quad\delta x=Kr,
\]

\[
P^+=P-KSK^T.
\]

生产代码调用原接口，其实现使用紧凑 H_order 计算完整 M，再更新全部相关状态与 P。[S6]

测试独立计算：

\[
P^+_J=(I-KH)P(I-KH)^T+K\Sigma K^T
\]

与原生结果比较。矩阵参考应在相同误差坐标中；若本地 native API 包含误差重置坐标变换，对两者施加同样变换。**禁止不审查原 API 就额外乘一次 quaternion reset Jacobian。**

注意：

- 完整 P 允许半正定、甚至因精确复制的 current pose clone 而奇异；不可要求 `LLT(P)` 成功，也不要求 `P.inverse()`。
- Sigma 必须正定，S 必须正定，使用 LLT/LDLT solve，不用显式 inverse。
- 检查 P 全矩阵对称性和特征值容差；只有对角线非负不足以证明 PSD。
- 源 LTV `sanitizeCovariance()` 的特征值截断规则只属于 P_L，不能移植成主 P_E 每次更新后的“修复”。
- 若输入/新block失败，拒绝 LTV，原视觉 block 仍可按原路径执行；如果执行原生更新后出现非有限状态/P，整个 run 标记失败并保留现场，不能半路回滚部分状态继续报成功。

## 9. H_order 合并：当前状态与 clone 交叉协方差不能丢

### 9.1 固定规则

输入是原压缩后 visual block 和 LTV block。合并 order：

1. 保留 visual `H_order` 原有顺序；
2. 追加缺少的 current IMU q、v；
3. 以 `Type` 身份和 `[id,id+size)` 验证映射；
4. 同一状态块只占一段列，分配零矩阵后散射每个子block；
5. 行顺序 visual → G → V，Sigma 按相同顺序 block-diagonal。

不允许假定完整协方差中的 q 永远在0、v永远在6；纯数学测试可以用15维顺序，生产定位必须通过 Type API。

**parent/subtype 重叠保护：**若一个 block 已列入整个 `_imu` 或 `_imu->pose()`，另一个又列入其中 q 子块，不能重复登记。v1 常规视觉 block 使用 clones，不会与 current q/v 列重叠，但工具仍需检测：相同范围可合并，部分重叠必须规范化到不重叠布局，或明确拒绝并报告。禁止重复计算同一状态。

### 9.2 原视觉压缩保持不变

只在原视觉压缩后追加LTV。不要对像素单位与方向/速度单位混合的 H/r 再调用原来假设等方差的 `measurement_compress_inplace()`。

未来确需压缩整个 block，必须先用完整 Sigma 白化并正确变换噪声；本阶段不做。

### 9.3 当前 q/v 观测也会影响其它状态

LTV 直接列只有当前 q/v，但 M=PH^T 使用完整交叉块，所以位置、bias和历史clones可能被修正。不得截取一个6维 covariance 独立更新q/v，再把结果覆盖回原P。

## 10. Gate 与失败关闭策略

先实现结构性有效性，经验质量门控只读输入且可单独配置。

### 10.1 必须检查

- observer enabled、started、warmup、snapshot finite、epoch/time一致；
- camera/IMU 时间配对、snapshot 未消费、主状态已初始化且已传播到该帧；
- sigma有限且大于0，H/res/Sigma维度正确；
- G 的 eta有效、norm合理、angle不过大；V 只检查 velocity 自身，不绑 G有效；
- ID/slot、重置和本帧有效观测数符合源生命周期；
- 禁止使用 GT、未来轨迹、Oracle mask；
- 任何拒绝必须有 reason code。

### 10.2 可移植的质量条件

源 handover 中基于 feature数、normalized innovation、eta norm error、velocity disagreement 的 gate 可由 `UpdaterLTV` 实现为纯谓词。保留公式和阈值来源，不能把 VINS 的归一化 innovation 当作已白化 NIS。

若新增 NIS gate，用**同一更新前 P**分别计算：

\[
S_g=H_gP_{q}H_g^T+\Sigma_G,\quad
S_v=H_vP_{qv}H_v^T+\Sigma_V,
\]

\[
d^2=r^TS^{-1}r.
\]

G 为2自由度、V为3自由度。门限使用库计算或明确常数，例如99%是约9.21034和11.34487；此处只是工作模型下的拒绝阈值，不能因通过NIS就宣称统计一致。不能在G更新后再用改变的P门控V，因为本方案是联合更新。

源经验gate与新NIS应分别记录，不混为同一个字段。首版不自动寻找最优阈值。

### 10.3 Gate 关闭的含义

`enable_quality_gate=false` 表示不使用额外经验筛选，**不关闭有限性、时间、方差、反向重力等结构性检查**。

单序列G或V零覆盖属于“没有施加该辅助”，不是软件错误，也不能当作有效性成功。不要为了满足 `0<coverage<1` 强制放行；全通过同样不自动判失败。

## 11. 输入时序和持久状态机

### 11.1 单一状态处理线程

IMU回调只将数据复制到adapter缓存，与原Propagator缓存并列。不要在IMU回调读取主bias后一路把LTV推进到最新IMU时间，再回头处理旧图像。

相机frame过程由原状态owner线程执行。不得让LTV线程直接改主滤波器、访问已清理feature对象或共享可变Eigen矩阵。

### 11.2 明确的每帧顺序

```text
1. 原 tracker 处理同步相机包；从当前 camera0 复制 bearing / ID / timestamp。
2. 确认原初始化完成，IMU 区间覆盖充分；复制 prior bias 和固定 calibration。
3. 原 Propagator 传播到 t_cam，并生成当前 pose clone。
4. adapter 将 LTV 推进到 t_imu=t_cam+dt_cam_imu，使用相同边界。
5. 当前 camera0 bearing 仅校正 LTV 一次；生成 immutable snapshot。
6. 在当前传播后、视觉前 state 上构建 G/V block、门控、冻结 FEJ context。
7. 原 UpdaterMSCKF 完成其视觉 block；与 LTV block 联合调用原 EKF 一次。
8. 使 fast propagation cache 失效。
9. 保留原 SLAM/delayed init/cleanup/marginalization 顺序。
10. 下帧从本帧主后验 bias 开始新的区间；LTV 自己的 s_L/P_L 持续保留。
```

本阶段第1步只复制观测，不写 `p_FinA/p_FinG`，不启用路标初值。

### 11.3 时间精度与插值

- 明确区分相机时钟、IMU时钟、ROS接收时钟。使用传感器header时间。
- `t_imu=t_cam+dt_cam_imu`，v1固定dt。区间端点必须有包围样本，按adapter合同插值，不允许未来外推补数据。[S4]
- 接近 Unix epoch 的 double 秒无法达到任意小误差。可用原始int64 ns作为去重键；与double时间比较时显式设可表示精度容差，例如1微秒，而不是1e-12秒。
- 排序、同时间重复IMU、倒序、区间断裂必须诊断，不将重复数据积分两遍。
- 相邻帧共用端点样本用于积分边界是允许的；**重复积分同一时间区间**不允许。
- source核心每个Euler步的输入取值由handover和adapter合同固定；若使用端点平均，与源VINS调度不同的地方写入对照说明，不冒称整机轨迹字节相同。

### 11.4 初始化与 warmup

- 首次满足原VIO初始化和数据条件时，从当前物理时间启动LTV；其内部零初值与源一致，不能每帧用主R/v覆盖。
- 首个camera校正和warmup按源核心处理；eta初始为0时G必须拒绝。
- warmup期间只运行LTV，不向主滤波器注入。
- 保留原等待足够clone的返回条件；若该条件使adapter本帧未处理，需显式维护时间或采用下述pause/reset策略，不留下未说明的长跳步。

### 11.5 去重、暂停、ZUPT与reset

采用 `(epoch, canonical_camera_packet_time)` 唯一token。本帧产生一次candidate，至多一次attempt；记录G/V分别是否接受。不在同一状态时间重新尝试先前拒绝的snapshot。

同步双目包只产生一次G/V辅助，初版observer仅使用camera0，左右相机不能触发两次相同辅助。

v1建议先在同配置B/P/G/V/GV中禁用ZUPT，但实现仍需正确处理原代码提前返回：

- 原初始化未完成、ZUPT成功或原相机处理跳过时，不注入LTV。
- 对因这些分支未处理的时间，adapter进入PAUSED并使旧snapshot无效；恢复时清理过期缓存、在当前时刻重启LTV并重新warmup，而不是把巨大dt一次积分。
- 若改为暂停期间仍持续维护observer，必须单独实现/测试同一时间约定，不能在本阶段偷偷混用两套政策。
- 主VIO reset/reinitialization：递增epoch、清理adapter与LTV状态、去重表和feature ID映射。
- 仅LTV失败：标记无效并按源规则重启LTV，不能让它自动触发主VIO reset。

### 11.6 feature ID 与数据库安全

OpenVINS feature ID 常为 `size_t`，源LTV当前接口是 `int`。禁止直接窄化转换。v1使用稳定的 `size_t → local int` 映射，同一epoch不将曾使用的local ID别名复用给其它点；溢出显式失败/重启，而不是wrap。

传入本帧真实观测，缺失点由源生命周期处理。必须在原MSCKF标记删除/数据库cleanup前复制所需值，不能保存稍后失效的Feature指针。

## 12. 配置建议与真实生效验证

命名可与本地风格协调，但一经确定保持单一来源。示例是**建议的新schema**，不是原版已有配置：

```yaml
ltv:
  enabled: false
  enable_gravity: false
  enable_velocity: false
  fusion_mode: joint_msckf
  allow_correlated_pseudomeasurements: false
  correlation_model: independence_approximation
  camera_id: 0
  sigma_gravity_deg: 10.0
  sigma_velocity_mps: 1.0
  max_gravity_angle_deg: 30.0
  enable_quality_gate: false
  enable_nis_gate: true
  nis_probability: 0.99
  time_tolerance_s: 0.000001
  log_enabled: false
```

observer核心参数从handover导入单独子节点 `ltv.observer`，不要与G/V噪声混名。已验证source默认max_features、warmup、q/v/P0等以实际handover为准。

任何 `enable_gravity/velocity=true` 但 `enabled=false` 或未显式接受近似的组合，配置加载应拒绝，不可静默改值。

开关是否动态生效首版不实现：配置在启动时冻结。若以后需要运行时切换，切回 OFF 不会恢复过去未加LTV的baseline轨迹，不能宣称即时回滚。

每次run输出：原生有效配置、新LTV有效配置、commit、dirty状态、源码/二进制SHA、source observer provenance。CLI/runner不得悄悄覆盖用户给的G/V参数；必须有“改变sigma使Sigma实际变化”的配置测试。

## 13. 数学与原生代码测试矩阵（阻断门槛）

### T0：源核心 parity

在源/目标核心中读取同一fixture、同一事件顺序、相同配置。比较每个事件后的 v、eta、P、slot、reset、子步数；用明确绝对/相对容差，确定性整数与ID需完全一致。初始目标容差可为 `atol=1e-10, rtol=1e-9`，跨编译器差异需解释，不得随意放宽到掩盖公式差异。

测试去掉 Utility include 的skew、Eigen原地表达式是否保留行为。不得仅比较最终ATE来宣称core移植正确。

### T1：坐标与JPL原生retraction

用真实 `JPLQuat::update()` / `IMU::update()`，不是自写Hamilton替身：

- 单位旋转、90°旋转、随机旋转下 `R_BW v_W` 正确；
- `q` 与 `-q` 表示相同旋转时观测相同；
- `R_new≈(I-skew(dtheta))R_old`；
- 非零相机平移/旋转的extrinsic双向round-trip；
- 静止 `a_corr+eta≈0`，非零bias校正仅一次；
- 四元数normalize、零/NaN输入拒绝。

### T2：Jacobian数值对照

至少500组随机状态，固定随机种子，`eps=1e-5/1e-6/1e-7`，对G/V每个有效状态维度中央差分。用原生retraction扰动状态副本，禁止修改真实state。

比较：

\[
H_{FD}[:,j]=\frac{h(x\boxplus\epsilon e_j)-h(x\boxplus(-\epsilon e_j))}{2\epsilon}.
\]

若测试的是 residual 的导数，则比较 `-H`。T和measurement在扰动中固定；不差分gate布尔函数。

目标 `max_abs_error≤1e-6` 且在非零项上满足合理相对误差（例如1e-5）；用限制到正常速度范围的样本，零项只按绝对容差。测试记录各eps误差，不挑一个碰巧通过的数字。

分别检查 G-only、V-only、GV、位置/bias零列、v=0时速度对姿态的导数为零、重力范数缩放不改变方向残差、反向重力被angle gate拒绝。

FEJ测试明确分开current/FEJ点；FD在FEJ做，不用current导数否定FEJ矩阵。

### T3：完整协方差更新

用随机PSD的完整state covariance，包含非零 current-q/v ↔ p/bias/clones 的交叉块。

- 实际 `StateHelper::EKFUpdate()` 与dense reference的delta/P/Joseph结果一致；
- 当前pose被精确clone导致P奇异时仍应正常，只要求Sigma/S正定；
- 正确保留交叉项，不是仅q/v的6×6小滤波；
- 零innovation时名义状态不动，但接受测量仍可缩减P；
- sigma变大使相同固定工作模型下信息增量减小；度/弧度、平方各一次；
- 非SPD Sigma/失败S分解在修改state前被拒绝；
- 校验P对称性与最小特征值，roundoff相对容差明确；不能靠事后eigen-clipping掩盖失败。

注意原生P访问/设置使用正式StateHelper接口；为测试构造状态所需fixture独立，不打开生产P任意写权限。

### T4：联合block与无视觉分支

- visual Type列顺序打乱、currentq/v不在固定索引，合并仍与dense按全局ID散射结果一致；
- 相同块合并、parent/subtype overlap拒绝或正规化、维度错配拒绝；
- 视觉行保留sigma_pix²，G/V各自Sigma不串位；
- G拒绝V通过、V拒绝G通过、都拒绝、visual空、visual全chi2拒绝、压缩后空均覆盖；
- 每个token在MSCKF/LTV阶段调用EKF次数为0或1；
- 固定线性H、独立噪声下，batch与正确sequential参考等价。sequential第二步创新必须 `r2-H2*dx1`，不是复用未修正r2；不将非线性重线性化情形当等价测试。

### T5：FEJ与不可观测性

验证第7节HN约束、共同yaw+速度变换不变性、共同translation不变性，扩展到真实Propagator和clone增广的多帧参考一致性。

可以在仿真中补充NIS/NEES，但未经相关性建模不应声称概率一致。瞬时HN通过只是必要几何检查，不能代替完整系统证明。

### T6：时序和持久性

合成输入覆盖：非整齐相机/IMU时刻、非零固定offset、边界插值、same-time stereo、重复图像、倒序IMU、缺右端点、短漏检、长gap、主reset、LTV-only reset、ZUPT/pause恢复。

验证每一有效区间被积分一次、未来数据不被提前使用、一个快照一次attempt、内部s_L/P_L跨帧保留、下帧才使用本帧后验bias；同帧不得重新处理bearing来闭环追主状态。

### T7：OFF与Passive原路径回归

对 deterministic 输入，比较同版本：

```text
原版 / 新版所有LTV关闭 / 新版Passive
```

优先逐事件比较主状态、P、FEJ、clone数量/时间、视觉接受feature集合，而非只比ATE。相同环境确定性路径应字节一致；若baseline自身不确定，先建立重复运行容差，再在同一容差比较，不能只放宽LTV版本。

不比较wall-time或新增日志列。被动模式不得改feature迭代顺序、RNG、P、状态初值或传感器消费顺序。

## 14. 诊断字段与完整性

至少输出每个camera packet：

```text
epoch, camera_timestamp, target_imu_timestamp, ltv_cursor_timestamp
mode, snapshot_id, attempt_id, used_once
ltv_v_B[3], ltv_eta_B[3], eta_norm
active_features, observed_features, warmup, core_substeps
P_L_trace, P_L_min_eig (可低频), ltv_reset_reason
R/v current与FEJ差异摘要, state_version
G/V eligibility, quality_gate, NIS, reason_mask
G/V sigma及最终Sigma对角项
G rows, V rows, visual rows, compact_columns
G/V residual_norm, accepted_G, accepted_V
EKF_call_count_in_joint_stage
P_E_symmetry_error/min_eig诊断(测试/低频), update_time
```

失败结果保留日志和配置，不能用旧VINS CSV补齐。Gate覆盖率同时报告相对于全部camera包和base-eligible包的分母，避免混淆。

## 15. 分阶段执行和停止条件

### Phase 0：只做迁移审计和数学原生测试

读取source handover、冻结两端version、审查本地API，生成：

```text
docs/ltv/port_audit.md
docs/ltv/frame_and_jacobian_contract.md
docs/ltv/test_matrix.md
```

复制必要核心到独立新目录，编译source parity与原生JPL/Jacobian小测试。**先不把G/V接入生产状态更新。**完成后报告并停止，供用户检查架构和数学结果。

### Phase 1：Passive 接入

实现adapter、可选配置、持久状态、输入时间和日志；B/P回归通过，证明主状态和P不被改。先一个短合成输入，再V1_01完整序列。

### Phase 2：G-only

完成UpdaterLTV数学block、矩阵合并工具、G-only分支；T1–T7相关测试全部通过后才能跑真实G。不要因为姿态看起来收敛就绕过P检查。

### Phase 3：V-only

加入Vblock，原生Jacobian/FEJ/协方差/时序全部通过；仅开V测试，G状态仍在LTV内部递推但不作为辅助注入。

### Phase 4：GV联合

确认同一个先验同时构造与门控G/V，只一次joint EKF调用。测试全部组合、空视觉分支，然后小规模真实序列。

### Phase 5：效果评估（单独授权，不自动大规模扫参）

先 B/P/G/V/GV 在少量预先指定序列比较ATE、成功率、开销，同时保留辅助姿态/速度诊断。工程通过与效果通过分开报告。

ATE为主要效果指标，但数学/时序/P错误是无条件阻断项，不能以ATE略有改善抵消。

原型目标是正常序列不明显退化且有局部可重复收益，不保证所有序列严格改善。零辅助覆盖或极弱到无作用时记为无有效证据，不记为方法成功。

## 16. 评估口径简约要求

- 对原生OpenVINS与LTV变体使用同输入、固定配置、同一GT、同一eval代码和预先设定的公共时间支持。
- ATE可按每条轨迹各自SE(3)刚体对齐，不使用尺度校正；不同运行初始gauge本来允许不同。
- 不为隐藏失败而截断尾段；报告完整回放、输出覆盖率、失败和共同支持比例。
- Rotation/Velocity辅助评估必须明确各自的gauge/坐标处理。只有已证实所有变体共享相同初始化坐标系，才可使用同一个B对齐旋转；不能机械沿用旧VINS经验把所有实验都强制共享B旋转。
- EuRoC使用官方state GT的velocity列时写明，不称作本地位置差分。没有官方速度的序列单独标记来源，不能混充同口径。
- 不从旧Stage结果复制OpenVINS基线，不宣称迁移必然更准。

## 17. 必须输出的报告

```text
local versions / dirty snapshot / source manifest
changed files + why each original file needed modification
persistent state vs snapshot dataflow
frame / gravity / extrinsic / bias contracts
LTV core parity
G/V residual + current/FEJ Jacobian
measurement noise vs P_L vs P_E
native covariance/Joseph/column mapping tests
FEJ instant and temporal nullspace tests
timing, duplicate and reset tests
OFF/Passive regression
small runtime results and actual auxiliary coverage
engineering approximation limitations
remaining blockers and next authorized phase
```

每项写明 `PASS / FAIL / NOT RUN / UNSUPPORTED`，附命令、退出码与产物。禁止“编译通过所以Jacobian正确”这种推断。

## 18. 独立数学参考脚本与使用限制

随本规范附 `verify_ltv_gv_math.py` 和 `math_verification_report.json`。

运行：

```bash
python3 verify_ltv_gv_math.py --output math_verification_report.json
```

本次参考检查执行了500组随机状态、三个差分步长及50组完整PSD协方差例子：

- G/V局部Jacobian最大绝对差：约 `1.22e-8`（三个eps中的最大值）；
- FEJ参考点差分最大绝对差：约 `1.22e-9`；
- 瞬时gauge `||HN||` 最大值：约 `3.17e-15`；
- Joseph与简式更新最大相对差：约 `9.39e-16`；
- 固定线性模型下顺序/联合更新最大相对差：约 `8.58e-16`。

这是**独立NumPy数学检查**，没有运行OpenVINS原生类型、真实Propagator、多线程、LTV source parity、数据集或统计一致性验证。它只提供参考答案，不能替代第13节原生测试，也不代表本文所述生产接入已经完成。

## 19. 直接给 OpenVINS Codex 的首次执行提示词

```text
请读取 docs/ltv_handover/LTV_HANDOVER.md、source manifest 和这份
02_OPENVINS_LTV_GV_IMPLEMENTATION.md。

本轮只执行 Phase 0：核对本地OpenVINS版本、坐标/JPL/FEJ/重力/bias/时间约定，
完成source LTV最小可移植核心清单、函数分类、最小改动方案与原生数学测试。

最终架构已经固定：LTV内部s/P持续递推；不改原IMU预测；UpdaterLTV只构造
可独立开关的G/V测量；在UpdaterMSCKF最终EKF调用处合并视觉与G/V，
只调用一次原StateHelper::EKFUpdate。暂不做landmark初值/观测、RL、Oracle。

不要把源Ceres的残差符号与ambient Jacobian直接搬过来；不要将P_LTV当作
测量噪声；不要把近似独立的伪测量融合宣称为已解决相关性。

先输出port_audit、数学合同、测试清单和实际测试结果。
任何关键约定或测试不通过即停止，不允许靠改sigma、关FEJ或PSD截断掩盖。
本次不要开启G/V生产状态更新，不要全量回放、自动扫参或修改用户已有工作。
```

## 20. 资料与证据定位

本文标签含义：`[P1]` 为用户提供原论文；`[S*]` 为已核对的上游/源代码事实。测量模型、类划分、测试和执行规则是本次拟定设计，不应误写为原文现成功能。

- **[P1]** Wang & Tayebi, *Pose, Velocity and Landmark Position Estimation Using IMU and Bearing Measurements*, ACC 2025, pp.1192–1197；式(8)、(10)、(14)–(19)；第二层已知地图假设不属于本阶段。用户提供PDF，扩展版本标识 arXiv:2407.18099。
- **[S1]** [JPLQuat.h](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/types/JPLQuat.h)：JPL左乘及矩阵负扰动。
- **[S2]** [UpdaterMSCKF.cpp](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/update/UpdaterMSCKF.cpp)：视觉构造、early returns、压缩、EKF入口。
- **[S3]** [IMU.h](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/types/IMU.h)：15维误差顺序和状态retraction。
- **[S4]** [Propagator.cpp](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.cpp)：时间、IMU校正、重力、set_fej生命周期。
- **[S5]** [VioManager.cpp](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/core/VioManager.cpp)：状态更新与ZUPT/初始化/清理调度。
- **[S6]** [StateHelper.cpp](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/StateHelper.cpp)：紧凑H_order、完整协方差与state update。
- **[S7]** [源ltv_observer.cpp](https://github.com/liuhestu/VINS-FUSION-LTV/blob/main/vins/src/ltv/ltv_observer.cpp)：在线读取的blob SHA为 `951e610425b12c3a68b8b9f7a55552d14a5d41b7`；本地handover必须重新锁定自己的commit/dirty SHA。
- **[S8]** [OpenVINS FEJ说明](https://docs.openvins.com/fej.html)：原可观测性背景。本文HN推导是新辅助模型的局部验证，非完整一致性定理。
- **[S9]** [OpenVINS Measurement Update](https://docs.openvins.com/update.html)：标准测量噪声独立假设和EKF代数背景。
