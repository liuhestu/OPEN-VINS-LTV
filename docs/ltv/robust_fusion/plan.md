# 鲁棒融合与全 EuRoC 调参对照

授权：用户要求验证质量门控、Huber、滑窗非线性因子及全 EuRoC 调参是否改善 OpenVINS LTV，并追问方法是否适合。

结论边界：当前只能说冻结的接入方式没有证明稳定增益，不能判定 LTV 与 OpenVINS 不兼容。全 EuRoC 调参后的 EuRoC 结果属于样本内结果，不称独立泛化验证。既有报告、数据、golden 和默认关闭行为保留。

## 分阶段执行

1. 已有 quality gate 直接复用。增加默认关闭的 G/V 分支 Huber：在当前 prior 上使用原始 R 白化残差的块范数，w=min(1,delta/||R^(-1/2)r||)，有效 R=R/w。原始 NIS 检验在鲁棒降权前执行，避免降权掩盖原本拒绝的异常值。仅做一次冻结权重的联合 EKF，不称 Ceres 非线性优化。记录有效方差和权重。
2. 原生测试：关闭时完全等价；小残差权重1；大残差信息下降；G/V独立；原始NIS仍拒绝异常；完整状态/协方差与dense参考一致；一次提交和FEJ原测试继续通过。隔离构建，不覆盖历史冻结库。
3. 冻结有限候选后，全 EuRoC 可运行序列（10条，沿用用户允许跳过MH04）进行统一对照。计划候选为 B、原GV、quality-only GV、Huber-only GV、quality+Huber GV，以及组合权重强/弱各一档（sigma成对乘0.5/2）。质量阈值、Huber delta=2 在运行前写入manifest；不滚动搜索observer参数。最多70次候选完整启动+10次选中配置重复，失败同样计数。输出逐序列ATE/RPE、均值/中位数/最坏退化、门控通过率、鲁棒权重、G/V更新量、完成率及耗时。选参目标预先固定为10条相对B的ATE变化均值；工程失败候选无资格，仍报告全部失败。
4. 最终报告明确：选参和报告用同一10条，因此不构成泛化证据。UZH旧冻结结果不改；新参数跨数据集验证另列阶段。若最佳组合几乎不接受LTV或更新趋零，只能说抑制了伤害，不能称第一层有用。
5. 真正滑窗非线性因子另列架构实验：必须明确窗口内速度/姿态变量、IMU和视觉因子、边缘化先验、FEJ/规范自由度、与EKF重复信息及输出策略。不能把clone上反复提交同一LTV观测当作实现。当前正在询问用户希望本轮是否包括这一范围；门控/Huber准备不依赖其答案。

每阶段及时commit并记录命令、退出码、二进制/配置/数据SHA。核心observer、主IMU预测、原golden不改；没有发现稳定收益也如实报告。

## 实现检查点

2026-10-03：新增 `ltv_enable_huber=false`、`ltv_huber_delta=2`；原始NIS/quality通过后按G/V独立块降权，有效方差写入已有G_variance/V_variance列，权重=名义方差/有效方差。10个原生测试全部退出0，含新Huber/dense Joseph/原始NIS防绕过及既有Jacobian、FEJ、联合更新、时序、shadow测试。ROS2独立ov_msckf构建完成（只复用旧ov_core/ov_init依赖，未改写历史安装）。

本次质量阈值固定为eta误差0.2、normalized innovation0.03、G/V均至少25特征、V disagreement0.5m/s；复用现有接口，不声称逐条复刻VINS T2的G最低15特征与独立cooldown。默认observer warmup保持20。Huber是白化整块范数，不逐坐标裁剪；有限角度G表示仍保留OpenVINS原切平面定义。

实验脚本 `scripts/ltv_robust/study.py`：独立输出根 `/home/he/output/openvins_ltv_robust_fusion_20261003`，一次最多4个单线程replay，统一硬件并发，选中配置10条重复要求轨迹与完整审计逐字节一致。首次运行前冻结候选、配置、输入与二进制哈希；每次保存命令、退出码和有效参数。分批遇工程失败停止后续批次，已有结果保留。

门控口径补充：复核VINS实际gate函数后确认，其normalized innovation除以sqrt(N)，OpenVINS既有quality gate除以sqrt(3N)。本轮冻结的0.03沿用OpenVINS现有定义，因此其同名数值不等价于VINS的0.03（对应VINS口径约0.052）。结果必须称“现有OpenVINS门控+Huber对照”，不能声称已完成T2门控逐项parity；不中途修改已经冻结的候选。若做精确T2对照，应另外冻结阈值0.03/sqrt(3)、G/V不同最低特征数和cooldown。

补充低成本回归：V1_01前800个相机包的B/P运行（Huber开关开启但G/V均关闭），trajectory.csv与audit.csv逐字节相同。两次短段单独记录，不计完整80次预算；不用于选参。短段验证曾与完整矩阵并行，运行耗时不是严格隔离的性能基准。

## 本轮完成与停止点

80次完整回放全部完成（70候选+10选中配置重复），两个短段回归通过，冻结二进制/库/输入SHA未变。按预定ATE目标选择raw：平均逐序列相对ATE改善0.861%；仅Huber改善0.796%；门控组合改善0.037%。三档组合未超过raw；QH强档1秒RPE改善但ATE目标更差，未据此改换目标。重复轨迹及完整审计全部逐字节相同。详情见[报告](report.md)。

本轮停止于已授权的鲁棒EKF对照：Huber默认关闭，质量门控默认不变，未把选参结果写成默认配置；没有实现非线性滑窗后端。滑窗设计及信息重复使用约束见[window_design.md](window_design.md)，不得将此次结果用于否定尚未实现的架构。
