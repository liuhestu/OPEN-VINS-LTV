# Landmark 两时刻残差与相关性方案（设计草案）

状态：**仅合同设计，没有 landmark ON 实现或回放；不预判收益。**
V-only 实验已以 `b1ca2c1` 提交，实验轮次结束，生产确认保持20。
本方案细化[阶段交接合同](../gv_fusion_evaluation/landmark_contract.md)，保持 landmark/history/seed、Active Consistency 和初始化算法冻结；R4未闭合项目仍未通过。

## 推荐对象及现有证据

推荐研究同一静态点在两个物理 IMU 时刻的 **LTV 三维相对路标约束**，更新当前主位姿和仍存活的 anchor clone。保留 `max_slam=0`；不新增独立世界地图，不把三维残差换成同一 bearing 的普通视觉行。
两时刻形式消去世界点均值，但**没有消除 seed、主位姿、共享输入的相关性**，不能据此宣布测量独立。

现有 [LtvSeedEstimator.cpp](../../../ov_msckf/src/ltv/landmark_adapter/LtvSeedEstimator.cpp) 从 history 的主位姿、外参与 bearing 三角化 `point_W`，再转换为执行时刻 `landmark_B`；这不是外部地图。
[LtvSeedUncertainty.cpp](../../../ov_msckf/src/ltv/landmark_adapter/LtvSeedUncertainty.cpp) 已有输入 Jacobian 和局部协方差传播，其结果明确标记 `LOCAL_LINEAR_RISK_NOT_CALIBRATED_CI`。该边缘协方差不提供跨时间或与主状态/视觉的完整相关性。
[当前配置](../../../config/ltv_euroc/estimator_config.yaml) 为11个clone、0个SLAM点。旧缓存可以审计身份、残差和边缘矩阵，不能从两个边缘P唯一恢复交叉块。

| 候选 | 测量及主状态归属 | 必要条件 | 当前选择 |
|---|---|---|---|
| 两时刻相对路标 | 两个body系三维点；当前位姿与活跃clone | 同点跨时刻对应、完整联合相关性、anchor生命周期 | 推荐先设计/审计，前提未满足不得ON |
| 世界点对主位姿 | body点与世界地图的预测差 | 外部地图或联合估计的地图误差、锚定及交叉块 | 当前seed不能充当独立地图 |
| 对主SLAM点 | observer点与主地图点的表示差 | 主状态实际含对应点、表示/重锚Jacobian、联合相关性 | 另立启用SLAM的匹配OFF基线 |

## 预测、Jacobian 和 FEJ

定义主状态世界到body旋转 `R_a,R_t`，世界系IMU位置 `p_a,p_t`；observer快照为 `ell_hat_a,ell_hat_t`，单位米，分别属于时刻a、t的物理IMU坐标。代码seed的 `R_WB` 是body到世界，本文件 `R=R_WB^T`，不可混用。
同一静态世界点满足：

\[
 h(x;\hat\ell_a)=R_t(R_a^T\hat\ell_a+p_a-p_t),\qquad
 r_L=\hat\ell_t-h(x;\hat\ell_a).
\]

均值残差使用当前主状态；anchor测量快照固定，不随主位姿重算以制造零残差。Jacobian与噪声线性化采用同一冻结FEJ状态，FEJ关闭则使用冻结当前状态。无需GT。
遵循 [JPLQuat.h](../../../ov_core/src/types/JPLQuat.h) 的注入 `R'=Exp(-delta_theta)R`，位置加法注入。令 `h_f` 为FEJ预测，`A=R_t^f R_a^{fT}`，观测方程 `r=H delta_x+nu` 的非零直接块为：

\[
 H_{\theta_t}=[h_f]_\times,\quad
 H_{\theta_a}=-A[\hat\ell_a]_\times,\quad
 H_{p_t}=-R_t^f,\quad H_{p_a}=R_t^f.
\]

速度、bias等直接块为零，但主滤波完整P仍可产生间接修正。FEJ anchor必须是主clone对应的首次线性化值，不能从observer均值重新生成。该几何残差对共同世界刚体变换不变；不会给出绝对地图锚定。系统整体的惯性可观测性另需验证。

## 身份、时间和 anchor 生命周期

- 关联键至少包含 tracker ID、observer epoch、birth generation、core身份、slot版本、相机标定身份。slot相同不等于点相同；retire/reset/ID污染终止关联和待消费receipt。
- 时刻使用相机时间加既有相机到IMU偏移，核对cursor、snapshot、当前状态、anchor clone物理时间。禁止插值一个不存在的clone或使用未来history。
- 候选anchor确定性地选择仍在主窗口、保存有同代次有效observer点快照的最早时刻。没有合格anchor则不生成约束；成熟/有效性的landmark资格须沿用现有合同，不能套用V10实验确认。
- anchor点的随机误差及其与当前点、主状态的交叉块须一起保留。clone边缘化时取消旧约束；选择新anchor创建新版本，不延长窗口或复活已边缘化主位姿。
- 同一冻结先验形成主视觉和landmark行，一次EKF调用。receipt绑定先验版本、anchor版本、观测来源集合和行集；重复、stale prior/anchor、时间不符或映射不符明确拒绝。零视觉行也适用相同合同。
- 关联几何隐含静态物理点假设。不能仅凭重复ID证明静态；无法通过既有质量/身份合同的点不获得测量资格，不新增GT门控。

## 噪声与主状态/视觉相关性

定义 observer 误差 `e_a=ell_hat_a-ell_true_a`、`e_t=ell_hat_t-ell_true_t`；主误差为上述注入坐标的true-minus-estimate。线性化测量噪声是：

\[
 \nu_L=e_t-Ae_a,\quad
 R_L=\Sigma_{tt}+A\Sigma_{aa}A^T-A\Sigma_{at}-\Sigma_{ta}A^T,
 \quad N_L=\operatorname{Cov}(\delta x,\nu_L)=C_{xt}-C_{xa}A^T.
\]

`Sigma_at=Cov(e_a,e_t)`，`C_xa=Cov(delta_x,e_a)`，均不能假设为零。
多点需要保留所有跨点/跨时间块。共享IMU、主bias供给、seed和LTV correction都可能造成相关性，甚至相关性抵消大部分表面测量信息。
与主视觉堆叠时，`R`还需 `Cov(nu_L,nu_visual)`，`N`需所有观测噪声与主状态的块。主状态完整P参与增益不等于这些外部块已经存在。

对联合残差采用有相关观测噪声的线性条件更新，令 `N=Cov(delta_x,nu)`：

\[
 S=HPH^T+R+HN+N^TH^T,\quad B=PH^T+N,\quad K=BS^{-1},
 \quad P^+=P-BS^{-1}B^T.
\]

等价的广义Joseph形式（`J=I-KH`）为：

\[
 P^+=JPJ^T+KRK^T-JNK^T-KN^TJ^T.
\]

未知交叉块不是零交叉块。要求联合误差协方差PSD、S的可用秩及白化分解可验证；若共享信息使有效秩为零，不更新。秩亏时只在可证的有效子空间处理，并检查残差的零空间一致性，不用任意对角ridge掩盖退化。上述公式是设计合同，现有G/V块对角更新不因此具备该能力。

## seed 依赖和可实施的相关性路径

设 seed 为 `ell_hat=f(x_hat_history,z)`，局部误差近似：

\[
 e_{seed}\simeq -J_x\delta x_{history}+J_z\epsilon_z,
 \quad C_{x,seed}=-P_{x,history}J_x^T+operatorname{Cov}(\delta x,\epsilon_z)J_z^T.
\]

必须将已有seed Jacobian的旋转扰动约定转换到主JPL误差坐标后再使用。仅采用seed边缘方差而删除上式主位姿依赖，会把主状态自己的信息回灌。

推荐下一步先做**只读联合误差传播方案与来源审计**，不改变现有估计均值/seed算法。它至少覆盖主完整状态、LTV完整状态（含G/V/各点）、受保护anchor误差及仍被重复使用的观测噪声：

1. seed增广：完整输入联合协方差和主交叉块，含history姿态/位置、外参、时间标定和bearing噪声；保存输入来源。
2. IMU/observer传播：主与observer耦合的状态转移、共享IMU噪声、主bias供给，传播cross块；不能各自传播P后认为独立。
3. 主视觉/LTV correction：同一像素/bearing的噪声及已有消费记录进入联合误差映射，包含主状态更新后observer误差的变化与JPL reset Jacobian。
4. slot置换、birth/retire、clone增广/边缘化、anchor替换：对联合误差执行同一身份映射，不丢失仍活跃的cross块。
5. 主视觉与候选landmark同批：从共享源形成R交叉块及N，避免在相同先验中重复当作独立证据。

需验证联合 `[P_main,C;C^T,P_LTV]` 与所有保留误差块兼容。既有P_LTV可能只有其原假设下的边缘意义；shadow传播若与该边缘P不兼容，必须记录方法缺口，不能用置零cross或裁负特征值伪造可融合的联合分布。精确传播成本可能超过当前预算，合同阶段不承诺一个有限cross列表一定足够。

建立观测来源账本，键为相机/物理时间/ID/epoch/代次，记录 history、seed、LTV correction、MSCKF/SLAM及最终receipt的使用。被MSCKF使用过的历史bearing仍可能留在LTV中；只从本帧视觉行删除同ID，并不能消除历史重复信息。
Covariance Intersection可另行研究，但三维相对约束并非独立的完整位姿估计，不能直接对6D位姿套CI。任何保守替代须先证明适用范围、相关性界和秩/规范自由度处理；任意调大R或添加“允许相关”开关不构成方案。

## 下一阶段独立 OFF/ON 合同与停止条件

本文件**没有执行**以下实验，也不授权直接实施ON。确认残差、相关性路径、资格和预算后才进入实现。

- 优先保持 `max_slam=0`，独立基线使用冻结C02、G/V均OFF、生产确认20、相同landmark/history/seed算法。不得把V10与landmark ON同时引入。OFF/shadow/ON使用相同诊断，先证明OFF精确复现；只改变landmark融合开关。
- 若转向SLAM点路径，启用SLAM的状态构成、点表示、初始化、重锚、ID映射单独冻结，建立匹配SLAM OFF；不与当前0点OFF混作landmark收益。
- 先证明身份/时间/receipt、FEJ/Jacobian、共同世界规范不变量、相关噪声更新、PSD/秩、seed反馈和视觉重复信息处理。没有cross方案或可证明的保守替代即停在shadow，不运行ON。
- 预算、噪声来源、门控及评价支持须事前冻结，不按GT拟合参数。GT仅离线评价主轨迹；EuRoC位姿GT不等于独立三维点真值，用相同seed bearing加GT位姿重新三角化也不构成独立landmark准确性证据。
- 有效信息/收益先考察预先留出的未消费bearing预测或独立地图参考（两者都需要新的明确数据合同），再分别评价ATE/RPE、覆盖、失败、修正及资源开销；预测指标不能代替联合统计一致性证据。
- 实际更新以对应行、一次性receipt消费和joint_applied判定；首次主/P差异应对应真实更新。若无新信息、供给不可靠、数值合同不满足或质量恶化，停止，不持续调权重。

## 本次设计核对

[check_math.py](check_math.py) 是离线合同验证，不连接估计器或GT：有限差分核对原生JPL旋转约定下四个直接Jacobian块、世界刚体变换不变量、联合噪声块的消元、相关噪声条件协方差/Joseph等价，以及重复主信息导致零更新信息的反例。
结果记录在 [math_checks.json](math_checks.json)。这些检查只验证公式，未实现cross传播，也未证明真实landmark输出更准或统计一致性通过。
