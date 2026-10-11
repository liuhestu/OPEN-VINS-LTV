# 论文式 → 本地代码 → 离散化 → 验证

源：本地 `docs/Pose_Velocity_and_Landmark_Position_Estimation_Using_IMU_and_Bearing_Measurements.pdf`，ACC 2025 印刷页1193–1195（PDF第2–4页）。本地 PDF SHA 见 SOURCE_MANIFEST。这里采用交接符号 `s_LTV,ell,R_WB,R_BC,p_BC,C_L`，对应论文 `x,{}^Bp_i,R,R_c,p_c,C`。论文式(5)的文字方向应结合其实际变换公式阅读：`p_C=R_c^T(p_B-p_c)`，所以用于 observer 的 R_c **把 C 向量转 B**；不按文字“from B to C”猜转置。

所有代码定位均相对仓库根，`[代码事实]` 指审计HEAD；以下解析式是数学含义，不替换实际浮点运算。测试执行证据在 [TEST_EVIDENCE.md](TEST_EVIDENCE.md)，golden 为源实现输出。

## 逐式映射

| 论文 | 内容与条件 `[论文]` | 本地定位及差异 `[代码事实/工程近似]` | 验证/缺口 |
|---|---|---|---|
| (3a,b) | `Rdot=R[omega]x; pdot=Rv` | 第一层没有 R/p 状态；宿主VINS独立预测，`estimator.cpp:539 + processIMU` | 不把VINS姿态预测打包成第二层 |
| (3c) | `vdot=-[omega]x*v+R^T*g+a` | `ltv_observer.cpp:130..148 + propagateImu`，a=raw acc−Ba，eta=独立状态，g为物理向下 | `LtvObserver.ImuPropagationUsesBodyFrameApparentAcceleration` + 非零omega/bias fixture |
| (4) | `ell_i=R^T(p_i-p)`，固定世界地标 | 内核仅估ell，无世界地标输入/世界pose回推；`ltv_observer.cpp:229 + rebuildState` 零新点 | 动态集合理论未证明 |
| (5) | `p_i^C=R_BC^T(ell_i-p_BC)` | `estimator/parameters.cpp:249 + readParameters` 加载body_T_cam0；`estimator.cpp:587 + processLtvImage` 传ric/tic | 非单位R与非零p fixture；目标外参必须另转换 |
| (6),(7) | 单位bearing；pinhole从像素K逆投影、无深度 | `feature_tracker.cpp:598 + undistortedPts` 用具体camera模型liftProjective→x/z,y/z；`:465 + trackImage` 构造[x,y,1]，observer `:353` 归一化 | 可适配非pinhole已标定bearing；不把畸变像素直接传入 |
| (8) | `y_i=Pi_i*p_BC=Pi_i*ell_i`，`Pi=I-bb^T`，`b=R_BC*z_C` | `ltv_observer.cpp:347..358 + updateFeatures`，3行每个观测、slot列块选择 | `LtvProjection.IsSymmetricIdempotentAndOrthogonal`；fixture C/y 为重建矩阵 |
| (9) | 每点世界bearing满足PE：`integral_t^(t+delta) Pi(R_WB R_BC z_i) dτ > mu I3` | 没有PE检测器；feature数量/年龄/创新gate并不检验该积分不等式 | `[待核实]` 实际数据的每点PE、固定集合GES |
| (10) | `elldot=-[omega]x ell-v; vdot=-[omega]x v+eta+a; etadot=-[omega]x eta; y=Pi ell` | `ltv_observer.cpp:136..149 + propagateImu` 名义部分 + `:391..397` 创新部分 | 已有测试只部分名义；本次fixture为全状态数值基准 |
| (11) | `xLdot=-Aomega*xL-Gamma*v; y=Lambda_z*xL` | N动态且本帧只有M观察；按slot构造非连续的C列 | 维度/compaction测试；非完整固定Lambda_z模型 |
| (12a) | `Aomega=blkdiag([omega]x)`，`Gamma=[I,...,I]^T` | `propagateImu:138` 路标每块−skew和−I | 非零omega fixture记录A/B重建值 |
| (12b) | `Lambda_z=blkdiag(Pi_1,...,Pi_N)` | M=N且slot全有观测时同论文；M<N时堆观测selection blocks | 漏检段fixture，行数3M列数3N+6 |
| (13) | `sdot=A*s+B*a; y=C*s` | IMU与camera分离执行；`ltv_observer.cpp:113/285` | `[工程近似]` 非同时连续积分 |
| (14a,b) | 下文完整A/B/C块矩阵 | `ltv_observer.cpp:136..146/347..358` | 状态尺寸、投影测试；完整fixture矩阵 |
| (15) | `shatdot=A*shat+B*a+K(y-C*shat)` | IMU名义Euler；camera当前测量冻结多Euler子步，`ltv_observer.cpp:148/394` | fixture正常至少两次camera correction；非离散Kalman |
| (16) | `K=P*C^T*Q`，维度 `(3N+6)×3N` | M observations时 `(3N+6)×3M`，Q=q I，`ltv_observer.cpp:392..395` 临时pct与q相乘，无常驻K | 以实际state/P数值parity检验，不仅移植名义方程 |
| (17) | `Pdot=A*P+P*A^T-P*C^T*Q*C*P+V`，P0、Q、V满足SPD及有界条件 | `processNoise:99` 对角V；`propagateImu:149`前三个传播项；`updateFeatures:396`负校正项；`:437`谱修复 | production量级PSD测试 + fixture；q=0的测试配置不满足论文正定条件 |
| (18) | `errdot=(A-KC)err`、GES依赖PE等假设 | 没有true state输入或error dynamics在线实现，动态birth/death加reset破坏原证明条件 | 不声称本次证明GES/AGAS |
| (19a,b,c) | 名义三条动力学**加Ki/Kv/Keta的创新项**；各gain为K相应3行 | 同一个 `pct=P*C^T` 对**全部状态**纠正，路标-v/eta交叉块传播后把bearing创新传给v/eta | 本次golden同时记录v和eta；不能把三条名义式当完整observer |

## 完整块矩阵

令 d=3N+6，W=`-[omega]_x`，路标顺序按slot。`[论文 + 代码事实]`：

\[
A=\begin{bmatrix}
 I_N\otimes W&-({\bf1}_N\otimes I_3)&0_{3N\times3}\\
 0_{3\times3N}&W&I_3\\
 0_{3\times3N}&0_3&W
\end{bmatrix}_{d\times d},\quad
B=\begin{bmatrix}0_{3N\times3}\\I_3\\0_3\end{bmatrix}_{d\times3}.
\]

非零块仅：`A[ell_j,ell_j]=W`、`A[ell_j,v]=-I`、`A[v,v]=W`、`A[v,eta]=I`、`A[eta,eta]=W`、`B[v,:]=I`；没有eta→ell直接块、ell→v/eta名义块、bias状态块。landmark每个slot也是3维，eta不强制归一化或固定9.81。

本帧排序观测slots `j_0,..,j_(M-1)`：

\[
C_L=\begin{bmatrix}\Pi_{j_0}E_{j_0}&0&0\\\vdots&\vdots&\vdots\\\Pi_{j_{M-1}}E_{j_{M-1}}&0&0\end{bmatrix}_{3M\times d},
\quad y=\operatorname{stack}(\Pi_{j_k}p_{BC})\in\mathbb R^{3M}.
\]

E_j 从3N路标列选3个slot列；每3行仅对应slot有3×3 Pi，其余零。M可小于N（保留漏检路标仍传播）、可为0，不能将 rows 绑定state_features。Pi rank2，三个输出分量不是三份独立bearing自由度。Q=qI_(3M)，P∈R^(d×d)，K∈R^(d×3M)。V=`diag(v_landmark*I_(3N),v_velocity*I3,v_gravity*I3)`；P0 base只有v/eta，landmark加入时另初始化自身block。

## 单IMU段与单camera子步的旧值/新值

`[代码事实]` 生产文件 `ltv_observer.cpp:130 + propagateImu`：固定本段omega/a后创建A/B，

```text
s_old,P_old → s_plus = s_old + dt*(A*s_old+B*a)
            → P_raw = P_old + dt*(A*P_old+P_old*A^T+V)
            → imu_timestamp += dt
            → finite/PSD检查和谱修复
```

P传播不依赖刚更新的s。默认dt≤0.05，正值不自动细分（max_imu_dt只是拒绝阈值），没有IMU adaptive substeps。

`[代码事实]` `updateFeatures:327` 先更新生命周期，再构造当前C/y，再计算**校正前**创新norm。首camera不校正也可以计healthy；后续camera用 `camera_dt=imu_t-last_camera_imu_t`（并非固定freq，不使用frame_time差直接作dt）。只校正q相关项：

```text
for each Euler step:
    r_old = y - C*s_old
    pct_old = P_old*C^T          # 明确物化的Eigen::MatrixXd
    s_new = s_old + h*q*pct_old*r_old
    P_raw = P_old - h*q*pct_old*C*P_old
    P_new = sanitize(P_raw)
```

`s_new`不会进入本步P公式；下步用新s/P和同一C/y。状态的乘法与P乘法在当前Eigen下中间表达式求值为临时矩阵，不使用`.noalias()`强行原地覆盖。独立 `validation/eigen_alias_probe.cpp` 分别比较这些原写法与物化old值，差均为0；但 `P=.5*(P+P.transpose())` 未物化则确实存在alias，差9。不能把后者藏在“Euler旧值全部安全”结论中。

`[代码事实]` `rho=max(0,lambda_max(q*C*P*C^T))`，`n=max(1,ceil(dt_camera*rho/max(1e-3,configured_safety)))`；n超 `max(1,configured_cap)`重置CovarianceFailure，n不在子步内重算。h=dt_camera/n。PSD检查在每个IMU后、每个camera子步后及camera结束再执行。最终trace/min/max diagonal只是日志，谱下限阈值看eigenvalue，不看diagonal。

`[代码事实]` camera时间检查未独立强制imu_t严格递增；只强制frame严格递增、与累积IMU时间错配≤tolerance、gap≤reset_gap。因此在极端输入下，frame增加但imu_t在tolerance内轻微倒退，camera_dt可能负，仍进入n=1的负h校正。当前EuRoC td固定、header单调下通常不触发；目标适配器需要明确其约束并新增目标时序测试，不能假设core已自动拒绝此情况。首frame sentinel `last_frame>=0` 与首camera sentinel `last_camera>=0` 也假设使用非负时间。

## 论文和工程模型的差异清单

| `[论文]` | `[代码事实/工程近似]` | 迁移要求 |
|---|---|---|
| 固定N≥3、持续观察和PE | max_features、漏检、零初始化birth、任意N甚至0；min_features只是warmup | 保留逻辑做parity，但不继承证明 |
| 连续omega/a/bearing | 分段IMU右端值，离散camera整段冻结C/y，split Euler | 不当成离散Kalman公式，不重复注入视觉 |
| Q/V连续有界一致正定 | 标量/分组配置且configure未全面验证非负/finite；测试会设q/V=0 | 全局设计尺度非noise covariance；异常配置还需专测 |
| P满足CRE | Euler+谱floor/relative failure+原地对称化alias风险 | 原输出为golden；任何修复另做明确算法变更 |
| 已有可测量omega/a | 最新宿主bias先扣减；共享输入且bias受G/V历史优化影响 | bias接口、反馈延迟和数据相关性都要保留 |
| 第一层估v/eta/ell | LTV不导出世界pose，不使用已知世界landmark | 禁止移植(20)及之后第二层 |

P各块来自混合单位状态（m、m/s、m/s²），没有真实噪声识别或与宿主状态联合协方差递推。设计尺度可以定义形式上的单位，但本仓库并未证明它们等于真实误差方差；Q通常相当于设计精度权重，不能直接把q=1e-4当rad²视觉noise。eta方向归一化的误差还需另作投影，不能用原P_eta直接当三维方向测量R。
