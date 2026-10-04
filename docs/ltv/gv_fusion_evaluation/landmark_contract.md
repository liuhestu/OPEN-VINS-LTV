# Landmark 融合阶段交接合同（待确认，不实施或运行）

本文件是下一阶段设计合同。G/V 的工程/收益结果不授权直接实现本合同。
现有 landmark/history/seed/Active Consistency 算法保持冻结；R4 缺口仍开放。

## 融合对象和状态归属

推荐下一阶段研究“LTV 三维 IMU 相对路标对主位姿形成约束”，而非再次把
同一 bearing 当作新的独立视觉测量。LTV 的路标是 observer 自身状态中的
l_B；不是主状态 SLAM 点，不能直接把它拼入主 P 或当作独立世界地图。

理想冻结地图点 p_F^W 下，预测为 R_WtoB(p_F^W-p_B^W)，残差为
l_B,LTV-R_WtoB(p_F^W-p_B^W)。地图必须具有明确锚定、版本和不确定性；
当前 seed.point_W 由主位姿/history 三角化，不能当作精确、独立地图。
在当前结构中直接采用该残差会把 seed 对主状态的依赖回灌，设计尚未满足
相关性合同，因此下一阶段先做只读 shadow/相关性审计再确认 ON 实现。

若选择两个时刻间的相对位置/相对路标变化约束，必须另写对应两时刻预测、
误差传播、clone 归属和 covariance 合同。不能把当前 l_B 的差当作世界
位移，或把它随意换成普通像素/切平面视觉残差而仍称“路标融合”。

## 身份、坐标和时间

每条测量绑定 feature_id、observer epoch、birth generation、core_id、slot、
地图点 anchor/version、主 clone 时间/版本及 calibration identity。
相同 tracker ID 不足以证明是相同物理点；retire 后不得跨 epoch/代次关联。
重置、ID 污染、slot 复用或地图重锚都终止旧测量资格和 receipt。
映射必须一一对应，并有未对应、重复、已退役、版本不符的显式拒绝原因。

B 为标定的 IMU body，W 为主状态冻结的局部世界；相机 C 使用固定外参。
保存 l_B 的物理 IMU 时间，验证 camera+offset、cursor、snapshot 与 clone
时间。禁止使用未来帧/history、GT 或评估标签构造在线测量。
同一冻结主先验构建视觉和路标行，receipt 一次性消费，过期先验拒绝。

## 噪声、地图锚定和重复信息

需要传播完整 LTV 路标协方差及路标之间、路标与 G/V 的交叉块；
位姿约束还需要地图点/anchor 和当前主状态之间的相关性。
若共同使用 G/V，不能默认新的 R 仍块对角。主完整 P 不代替
Cov(LTV,main) 或 Cov(LTV,visual)，也不代替地图 anchor 的不确定性。

为同一观测创建因果使用账本：相机、物理时间、ID/代次、history/seed 使用、
LTV correction 使用、MSCKF/SLAM 使用、最终约束 receipt。
seed→LTV→main 和同时 MSCKF 的重复信息必须显式处理；保留现有算法意味着
不能通过修改 seed/history 假装生成了独立观测。
可选工程路径是联合增广/显式交叉相关性传播；这将明显扩展状态和边界。
也可研究有证明和明确假设的保守融合，但不能把任意放大噪声或“允许相关”
标志当作一致性证明。预先固定方法与参数，禁止按 GT 结果回调。
本合同不预选未经验证的新阈值；进入实现前必须确认噪声来源和相关性方案。

## 两种主状态路径的必要前提

| 路径 | 主状态要求 | 地图/相关性前提 | 基线要求 |
|---|---|---|---|
| 对主位姿形成约束（推荐先研究） | 当前 IMU 位姿与所需 clone；保持 max_slam=0 | 明确冻结/增广的地图 anchor、l_B 对应、seed 主位姿依赖和重复观测账本 | 原 C02 + 已固定 G/V 模式，landmark OFF/ON 成对 |
| 对主 SLAM 路标形成约束 | 主滤波器启用 SLAM，真实存在同一物理点的状态 Type，明确 GLOBAL_3D 或 anchored 表示 | observer 与主 SLAM 的 ID/epoch/anchor 对应、表示 Jacobian、地图重锚/边缘化及完整交叉相关性 | 另立启用 SLAM 的匹配 OFF 基线，不与当前 max_slam=0 的 OFF 混比 |

当前 max_slam=0，所以第二条路径没有可更新的主 SLAM 路标。
推荐先保持此设置，评估位姿路径所需的因果账本与相关性建模成本；
只有其前提达成才实现实际约束。若无法达成，则保持 shadow 结果并说明
阻塞，不提交一个有经验 ATE 收益却未定义地图/相关性的生产更新。

## 独立 OFF/ON 实验合同

确认前必须冻结：具体融合对象/残差/Jacobian/FEJ、状态归属、anchor 生命周期、
对应规则、所有时间/token/一次性 receipt、噪声来源、交叉相关性方案、
信息使用账本及可判定的接受/拒绝条件。合并策略须保证一冻结先验的一次
EKF；零视觉行、重置/退役、ID 重用、stale anchor 和重复 receipt 都有测试。

OFF/ON 均开启相同诊断，固定主配置及 G/V 模式；仅改 landmark 开关。
冻结完整输入、初始化、支持和评价口径，先验证 OFF 精确复现对应基线，
再运行 ON 和确定性重复。只有实际 receipt 消费且对应行参与 EKF 才计更新。
报告 ATE/RPE/覆盖/失败、首次状态/P/observer/事件分歧、资源开销、
信息重复使用次数和一致性证据；收益与一致性分开。
若启用 SLAM，先建立/验证新的 SLAM OFF；该结果属于另一个实验，不能
把 SLAM 状态扩展的收益归于 landmark 融合。

下一步仅为合同确认；本轮不新增 landmark 更新行、状态或真实 ON 回放。
