# 完整目标完成审计

依据：用户附件40项及接入规范。冻结代码c7fc29b；最终科学结果允许失败/退化，不以ATE改善定义完成。
以下逐项检查实际源码、命令退出码、运行manifest、原始轨迹/审计、统一metrics和复测；不把覆盖有效等同定位成功。
证据根：`/home/he/output/openvins_ltv_autonomous_20261002`（OUT）。[最终报告](../euroc_evaluation/final_report.md)保留全部11条，包括MH_04失败。

| 要求 | 判定 | 实际证据 |
|---|---|---|
| 1 数学/坐标/Jacobian/FEJ/状态/更新位置硬约束 | 完成 | 原生T1/T2/T3/T5，实际UpdaterLTV H；原生受保护文件与起点字节一致 |
| 2 统一有限参数 | 完成 | 初始配置未改、额外候选0；冻结身份及55份实际effective_options |
| 3 数学→数值→NIS→覆盖→ATE | 完成 | D005消减诊断与D006–D008在真实ATE前完成；没有ATE驱动参数修改 |
| 4 更新过强先查实现 | 完成 | H/单位/时间/列/P测试；MH_04原版完整重放字节一致确认原生发散，未调sigma遮掩 |
| 5 弱分支先查作用 | 完成 | 实际G/V行、gain/update、coverage、residual/NIS全保留；有效10序列修正非零，保留初值 |
| 6 最多6候选、仅三开发集 | 完成 | 额外候选0，开发8次full仅V1_01/V2_02/V2_03（含1失败）；正式前不跑其他序列 |
| 7 近似配置选保守 | 完成 | 未搜索候选，D008保留规范保守初值 |
| 8 自主决策日志 | 完成 | decision_log.md D001–D010，含证据/选择/旧新参数/否决方案 |
| 9 关键阻断不绕过 | 完成 | 已修实际S三角构造问题；原生/联合/时序测试通过，运行数值检查无异常退出；物理定位失败单列，非数值corruption |
| 10 原IMU prediction不改 | 完成 | Propagator与a898929字节一致 |
| 11 不增广主状态 | 完成 | State/IMU定义未改；observer独立对象 |
| 12 LTV s/P持续递推 | 完成 | 生命周期/compaction/时序/跨区间bias测试；无主R/v逐帧覆盖 |
| 13 每快照最多一次 | 完成 | claim epoch/version/time/sequence/canonical key与receipt；重复调用P不变；全部运行unique_attempts无重复 |
| 14 同先验一次joint | 完成 | 冻结current/FEJ/P/clones；独立gate、视觉压缩后merge；50组joint/native/Joseph与实际日志<=1次 |
| 15 相关性限制 | 完成 | 显式确认independence_approximation；报告不声称GES/AGAS/统计一致性 |
| 16 Passive | 完成 | 单元测试+完整V1_01 B/P；最终11条B/P轨迹和完整状态审计字节一致 |
| 17 G-only | 完成 | 模型/门控/原生数学/时序及完整G，阶段2757实际G更新 |
| 18 V-only | 完成 | V不绑gravity_valid，实际模型/配置/数学/时序与阶段2781更新 |
| 19 GV | 完成 | 三开发集GV；visual/G/V/GV、真chi2全拒绝、原生压缩空输出组合、统一列/P/一次提交 |
| 20 唯一冻结后全量 | 完成 | c7fc29b与初始配置冻结，55正式run全部引用相同身份 |
| 21 无固定改善阈值 | 完成 | G近乎无效果、若干退化、MH_04失败均保留，未延长调参 |
| 22 全完成条件 | 完成（含可审计定位失败） | Phase1–4 PASS，55正式+16复测+原版诊断，唯一配置、统一评价、完整报告；MH_04不能定位但输入执行完整 |
| 23 冻结后不回调 | 完成 | 全部正式和复测代码/库/参数/evaluator身份一致；冻结后仅只读诊断和外部测试 |
| 24 全量不选参/不删不利序列 | 完成 | 全11原始结果含巨大失败ATE保留；共同有效均值明确10/11，另列含FAIL原始11均值 |
| 25 全11序列 | 完成 | final_paths.json与55份run manifest；指定11条全部存在 |
| 26 五模式 | 完成 | 每条B/P/G/V/GV唯一正式运行，开关矩阵与实际effective_options匹配 |
| 27 同输入/版本/配置/GT/完整性 | 完成 | 输入冻结前后全SHA一致；55个完整同步包/IMU消费，未匹配图像逐条记录；运行退出0、无非有限值/运行期reset |
| 28 ATE无尺度SE3 | 完成 | 冻结evaluator、自测和55份metrics scale=1；当前代码SHA相同 |
| 29 主表及三组均值 | 完成 | final_report/euroc_gv_results；有效组分母7/7、3/4、10/11；含失败原始7/4/11均值另列 |
| 30 Passive附表 | 完成 | 11条P ATE附表+trajectory/audit精确比较，不只比较ATE |
| 31 辅助诊断 | 完成 | euroc_diagnostics.csv 55行：rotation/velocity、residual/NIS/gain/update、三种coverage分母、counts/runtime/P/PL；run日志reset与finite |
| 32 三关键序列复测 | 完成 | V1_01、原始最大变化MH_04、最大有效退化V2_03；追加最大有效收益MH_02；各B/G/V/GV |
| 33 去重复测 | 完成 | 四条唯一序列，不用V1_01占两个角色 |
| 34 复测不调参 | 完成 | 16份repeat manifest与冻结身份一致 |
| 35 重复波动解释 | 完成 | 16/16轨迹+audit字节一致，ATE范围0；报告仅同机确定性复现，不宣称跨场景统计显著 |
| 36 final_report/decision_log | 完成 | 两个文件已写入，含明确效果与失败结论 |
| 37 报告全部章节 | 完成 | 版本/配置/测试/阶段/55完整性/ATE/组均值/复测/G/V/GV/限制/场景/后续研究判断齐全 |
| 38 足够执行证据 | 完成 | command/config/commit/hash/evaluator/日志/metrics/失败/复测均保留；final_artifact_index可逐run追溯 |
| 39 禁止项 | 完成 | GT/golden/容差/evaluator未改；未删失败；参数统一；无旧VINS结果替代、未运行测试虚报或阈值放宽 |
| 40 无正收益也可结束 | 完成 | 如实给出G基本持平、V/GV小幅且场景依赖、MH_04全模式失败，研究路线建议不夸大 |

## 验收边界

- 正常MSCKF正行且含clone正列时，压缩行数=min(rows,cols)>0；T4空压缩使用真实compression的2×0边界和实际apply_joint，未虚报自然视觉数据触发不可达分支。
- 源参数、坐标和adapter表达式已验证；完成审计补测使用实际冻结库验证下一积分区间的新bias、未来缓存、Unix epoch key。没有修改正式代码或配置。
- MH_04所有模式定位FAIL，原版完整重放与冻结B字节一致；仅证明不是接入引入的关闭路径变化，未声称锁定唯一原生初始化/视觉失败根因。
- `metrics_*.json`里的VALID只指有限、完整、覆盖可比较。报告的定位FAIL是额外明确审计；原数值和全11原始均值未删改。
- ROS1、真实在线异步多相机、统计相关性建模、LTV landmark辅助、第二层pose observer不在已验收范围；不由本次结果推断。
