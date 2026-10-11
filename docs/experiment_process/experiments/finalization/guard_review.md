# LTV保护逻辑审查：哪些可以删，哪些需要换结构

本审查针对定稿架构的实际调用链。目标是减少重复计算、重复实现与历史实验限制，同时保留当前G/V/L算法行为。已验证的序列证明对应运行路径可复现，不证明所有异常分支都无作用。

## 目录与两种“更新”

`observer/LtvObserver.cpp` 更新LTV自己的 `state_` 与Riccati矩阵 `covariance_`；`fusion/UpdaterLTV.cpp::apply_joint` 调用 `StateHelper::EKFUpdate`，更新OpenVINS主State/P。因此fusion保持独立，不移入observer。原integration改名runtime：它包含输入转换、IMU缓冲、相机事务、epoch与可用性策略，范围超过“输入处理”。

## 已实施的精简

| 位置 | 原冗余 | 当前处理 | 验证依据 |
|---|---|---|---|
| LtvOptions::validate与LtvRuntime构造器 | 多份managed/注入/可用性依赖判断 | 统一到validate_runtime；两个公开入口共享定义 | 保留非法配置测试；构造器直接调用也有边界检查 |
| LtvOptions::validate | 通用API硬要求Q/V/P0与30/15容量必须等于某次实验C0 | 去除精确等式；正值、计数、模型范围约束仍保留 | 新增合法非C0配置测试；冻结回放driver仍核对C0 |
| UpdaterLTV::validate中的S | innovation_matrix已经明确返回selfadjointView的对称矩阵，却再次计算S−Sᵀ | 删除这条无新增信息的判断 | S有限性与LLT成功检查保留；已有消减误差fixture保留 |
| UpdaterLTV::build中的gate | validate和nis分别重建同一个S并做LLT | validate可返回复用该分解所得的NIS | 增加解析例：P=H=R=I、r=[1,2,3]，NIS=7；门控阈值不变 |
| LtvRuntime::processManaged | warm路径先复制frontend_before，再复制evaluated，再复制staged | warm仅一次必要深拷贝，后续移动；cold重启时仍保留必要的原始状态副本 | 原子提交未删除；运行时拒绝/回滚测试及完整回放对照 |
| Observer类型与历史诊断 | 不再读取的旧factor/oracle配置与输出字段；研究cache/value/joint诊断钩子 | 去除死字段、主代码钩子和兼容别名；研究实现归档 | 活动源码、测试和fixture引用检查；同输入回归 |

这些改变不调整现有模式权重、ready条件、质量门限、Riccati离散化或原生EKF公式。

## 可继续删减，但需要同步调整接口或日志

| 候选 | 代码证据与作用 | 推荐做法 | 目前为何未直接删除 |
|---|---|---|---|
| 分支修正量诊断 | apply_joint在原生EKF前额外展开完整H、计算K和分支K*r，仅用于G/V/L及bias修正量日志 | 删除诊断专用K计算，或在回归工具中计算；日志明确省略未测字段 | 不影响核心数学，但影响现有收据/CSV的字段和含义，需同步消费者 |
| 完整矩阵capture字段 | LtvDiagnostics与gv_evaluation_diagnostics保留joint_H/R/S/K/P等可选研究字段 | 从主接口移至专用回归记录器 | 部分数值fixture和错误复核仍用这些字段；不能留下全零假记录 |
| 无辅助量测时的快照 | build在判断是否可用之前就复制主P、IMU、FEJ和所有clone均值 | 延迟到确有辅助候选后捕获；无辅助量测路径快返 | 需统一apply_joint的无量测/拒绝收据语义，避免误标stale_prior |
| processUnmanaged | 旧无可用性策略路径仍保留独立时序、插值和生命周期处理 | 明确只支持最终managed模式后移除；旧探针迁移到当前接口 | test_ltv_timing、旧配置仍覆盖它；当前没有把兼容模式悄悄停用 |
| 数学core旧生命周期路径 | updateFeatures与受控updateFeaturesControlled并存 | 保留单一受控生产入口，旧数学fixture改用明确birth/retain输入后再归档旧入口 | 现有公式与生命周期探针直接使用旧接口，不是已确认死函数 |
| 两份IMU区间采样逻辑 | managed/unmanaged均进行括号查找、边界插值与缓存保留 | 先抽一个采样函数；若删除旧模式，则直接只保留一条路径 | 两条路径的暂停/恢复行为需要维持，不能只按文本相似拼接 |
| 上下文重复校验 | Runtime、Frontend、ActiveConsistency、SeedGeometry均检查部分time/rotation/shape | 让Runtime/Frontend统一入口检查，内部调用使用已验证上下文 | 独立几何工具与测试也直接调用这些公开方法；删内层检查需先改变公开边界 |

## 需要结构替代后才可删的强校验

| 位置 | 当前保证 | 删除前提 |
|---|---|---|
| apply_joint的prior_P/IMU/FEJ/clone对照 | 辅助残差与视觉块确实基于同一个未变化先验 | 把构造与提交放入同一原子作用域，或建立覆盖所有主状态写入的版本计数；当前Runtime版本并不覆盖所有StateHelper写入 |
| Runtime暂存Observer、frontend、ID映射 | seed、ID、x/P要么一起提交，要么都不提交 | 使用可验证的回滚/提交机制替代；已优化副本次数，不删除事务本身 |
| LtvIdentityGuard与受控birth历史 | 防止同一epoch中退役/遗忘后重新初始化同一原始ID | 将跟踪器ID单调/不复用及失踪后不复活作为明确接口合同，并有长期/中断/竞争测试；Observer局部ID单调不能独自证明原始ID不被重新映射 |
| 全主P的谱检查 | 检查整个主协方差，而不只是被测子空间的S | 若原生StateHelper提供可依赖的统一完整P合同，可在唯一边界做；不能以S的LLT成功代替全P有效性 |

## 应保留的算法和数值条件

| 条件 | 原因 |
|---|---|
| 时间单调、IMU左右括号、缺口与dt范围 | 决定是否确实传播到相机时刻；错误会改变A(t)、积分和快照时序 |
| 输入有限、非零bearing、合法旋转和矩阵维数 | 防止归一化、索引、线性求解使用无定义输入；公开几何入口仍需处理失败 |
| Riccati数值检查、相机子步与协方差处理 | 部分检查会实际改变P或决定reset，是离散算法的一部分，不是单纯日志 |
| seed几何条件、相对风险、历史/留出一致性 | 是当前特征选择与退役算法；删掉会改变进入Observer的观测集合 |
| ready/warmup/grace/恢复策略 | 控制何时使用Observer输出，直接改变G/V/L更新次数 |
| R与S的SPD检查、NIS门控 | R、S是不同矩阵；LLT是实际求解条件，NIS决定接受哪些辅助量测 |
| 类型所有权与列不重叠 | 保证H的列对应真实State/P分块；错列会更新错误变量 |
| claim与receipt.consumed | 分别保护相机快照的处理资格和量测的实际提交，作用阶段不同 |
| L信息上限、点数、锚点生命周期 | 是冻结L近似的约束和模型边界，不是可忽略的工程装饰 |
| 固定标定/相机范围检查 | 当前模型没有实现全部在线标定或异步多相机项，取消检查不等于实现这些项 |

优先顺序：已证明重复的实现/计算 → 研究日志与旧模式 → 原子作用域和验证边界收敛。暂不因代码已经跑通而删门控、时间契约或数值算法。重构后的实际验证结果另见同目录的结构化记录；旧实验开销FAIL和统计一致性缺口不会因代码变短而变成PASS。
