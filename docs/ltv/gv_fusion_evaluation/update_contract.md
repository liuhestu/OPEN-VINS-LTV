# G/V 更新及评测合同

冻结提交：83c71cbc585123381f033c8dbcd625c14e8e7fac。
分支：feature/ltv-gv-fusion-evaluation。独立基线：/tmp/ltv_gv_baseline_83c71cb，
源码文件去掉写权限，不用于构建新实现。外部输出：/tmp/ltv_gv_evaluation。
冻结身份见外部 freeze.json，交付 manifest 绑定其 SHA256。

## 数学和因果合同

R 是世界到 IMU 的旋转，主速度 v_W 位于世界系；LTV 输出 v_B、eta_B。
gamma=[0,0,-1]，T 是 FEJ 预测重力方向的正交二维切平面基。
重力残差为 T^T(eta_B/||eta_B||-R gamma)，Jacobian 的姿态块为
T^T skew(R_FEJ gamma)，速度块为零；不融合重力模长。
速度残差为 v_B-R v_W，Jacobian 为
[skew(R_FEJ v_W,FEJ), R_FEJ]。关闭 FEJ 时以当前状态线性化。
残差始终使用当前主状态；JPL G→I 四元数系数在离线 Hamilton 工具中表示 I→G。

G、V 分别要求 ready_G、ready_V；未 ready 的有效事务记录 not_ready_G、
not_ready_V。公共 disabled/时间/token/快照无效检查优先，原有效性、角度、
quality、NIS 和先验检查保留。未引入 readiness 门限、warmup 或新状态机。

相机时间、相机到 IMU 偏移、cursor、snapshot 两个时间、非零 epoch/sequence、
available 检查保留。adapter claim 防重复，receipt 消费一次；主状态、FEJ、
完整 P、clone 上下文必须与冻结先验完全一致。G/V 与 MSCKF 行合并，
一次原生 EKFUpdate；视觉行为空时允许辅助行单独更新。
实际融合定义为 receipt consumed、对应行数>0、submit_reason=joint_applied、
ekf_calls=1；零创新也可能实际减少 P，不能以非零均值变化判断。

G sigma=10°（换成弧度后平方），V sigma=1 m/s，G/V NIS 门限
9.21034037197618 / 11.3448667301444。G 2 DOF、V 3 DOF。
GV NIS 和含视觉的 joint NIS 仅用于诊断，不新增联合门限。
C02 landmark/history/seed/Active Consistency、初始化、Q/V/P0、噪声和
原有其他开关保持冻结。生产配置中的 G/V 默认关闭。

## 相关性边界

融合 R 是分支块对角矩阵，G/V 之间、LTV 与主状态之间、LTV 与视觉残差之间
的交叉相关性没有传播或建模。seed 使用主位姿和视觉历史，LTV 与 MSCKF 也
共享视觉与 IMU，这些相关性真实存在。allow_correlated_pseudomeasurements
沿用冻结配置的显式 acknowledgement；它不意味着协方差已正确建模。

主滤波器完整 P（包括所有主状态与 clone 的交叉块）用于 K 和后验更新。
该主状态内部完整协方差与上述缺失的外部交叉相关性是两个不同问题。
ATE 改善只能说明这些输入上的经验结果，不能证明统计一致性。

## 仪器和执行

run_ltv_feature_passive 保留原配置拒绝和运行时零辅助注入断言。
run_ltv_gv_evaluation 复用其同一源码、同步、缓存、审计、单线程控制；
输入 YAML 必须是冻结 OFF。运行参数 OFF/G/V/GV 在解析后仅修改两个算法
开关，另将 passive_assert_no_injection=false、gv_evaluation_diagnostics=true。
这两个字段仅供 C++ 仪器控制，不增加 YAML 生产入口；独立 instrumentation.json
公开记录差异；配置校验、adapter 和 manager 中全部仅适用于 Passive 的
注入断言受同一开关控制，其他有效性检查保留。所有融合模式的诊断覆盖相同；原 Passive 文件格式维持兼容。

逐帧 fusion.jsonl 保存请求、receipt、readiness/grace、原因、残差、原始
噪声（本轮 Huber OFF）、门控 NIS/DOF、更新贡献、状态协方差检查及数值摘要。
matrices.jsonl 保存冻结 prior、joint H/R/res/S、列对应 Type id/size、
即时 joint 后验，以及完整相机事务后的主状态/P。没有实际联合调用时即时
后验为空；相机边界后验仍保存。完整 LTV x/P、输入、IDs/slots 和生命周期
见同一 cache.bin；features.jsonl 保存完整 readiness 和 Consistency。
若抛异常，保存当前矩阵、输入 cursor、时间和调用路径；上一行保留异常前边界。
完整矩阵检查沿用 1e-10×max(1,spectral_scale) 谱/对称性边界。

顺序为每序列 PASSIVE、融合 OFF、G、V、GV、GV repeat，共十二次完整输入。
原 Passive 首先逐字段/逐字节精确复现冻结证据，再验证新工具 OFF 同值。
仅排除 features.jsonl 顶层 compute_time_ms。GV 重复要求数值、事件、缓存、
完整矩阵精确相同。单线程；最多两个计算任务。新增真实回放都追加到原
R4 attempts.jsonl，同时在本轮外部目录留独立命令/身份/退出码记录。
失败与重跑都消耗次数；不重做历史实验。

普通 NIS 拒绝是有效结果。数值异常暂停对应 GV；只修复明确阻断问题，
重跑 OFF 验证后再继续。ATE 变差、融合未触发或全部拒绝不调参、不隐藏。
R4 NOT_ACHIEVED 保留，恢复及成对合成验收不属于本轮必须补做范围。

## 离线评价

支持固定为融合 OFF 首次初始化后的完整配对相机时间，不针对 ON 裁剪。
GT 最近邻最大20ms，输出匹配1μs，无尺度 SE(3) 对齐；
1s/5s RPE 的端点匹配沿用25ms。GT 只在 Python 离线评价中读取。
覆盖门限沿用99%，初始化差异不超过1μs；定位失败沿用
ATE>max(5m,GT bounding-box diagonal)。缺失、崩溃、覆盖失败仍保留
可计算的原始指标，并明确标记可用样本条件及失败，固定支持的分母不变。
