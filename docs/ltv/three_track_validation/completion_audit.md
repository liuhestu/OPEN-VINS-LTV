# 完成交付审计（未完成）

本表对照 execution_contract.md 与用户目标；证据缺失、待执行或范围较窄均不记完成。科学失败可以形成终止结论，未经复核的中间结果不能代替最终交付。

| 要求 | 当前状态 | 收口所需证据 |
|---|---|---|
| 冻结基线与保留本地修改 | 已登记 | session.json、execution_contract.md、初始 dirty 与最终 git status 对照；最后再次核验 |
| 三独立分支/worktree/源码/产物与资源锁 | 已执行，持续核验 | process_isolation.md、统一 registry、source snapshot 和 binary/library 身份；最终遗留进程审计 |
| 冻结验收、数据拆分与评价公式 | 已冻结 | contracts/*；原数值与历史 FAIL 保留 |
| 预测收益与工程准入独立 | 已实施 | engineering_on_admission_amendment.yaml、B3/B4 FAIL 及 B 实际 ON 准入记录 |
| A 两 DEV OFF 原样精确回归 | 已有完整证据 | task_a evidence 中 main state/P、Observer、生命周期、tracker/pure visual 检查 |
| A 四模式不退化与性能 | 最终 STOP，局部 FAIL 完整重复 | V2_03 OFF/G/V/GV 同源完整重复已复现；见 gv_report.md，性能首轮 PASS 不抵销局部 FAIL |
| A 留出与全可用序列扩展 | NOT_RUN | 若候选先在 DEV 失败并 STOP，解释不扩展理由，不称全数据集通过 |
| B 因果门控、尾部、覆盖及预测质量 | B3/B4 已有证据 | 保留历史复用标识和冻结分母；最终逐序列与模型对照报告 |
| B native 残差/Jacobian 与安全短 ON | 修复版待实测 | 新 hybrid Type 布局的联合 merge/apply、receipt/rows、一次 EKF、K_L*r_L 分块重建、OFF 回归 |
| B 完整匹配 OFF/ON、不退化与实际收益 | 待执行 | 修复后的完整 DEV 轨迹、冻结局部/全局指标与生产标量性能；合格再留出/扩展 |
| C 实际离散完整敏感度 | 固定标定/时钟分支有检查 | gain/P_R/归一化/seed/切换来源映射，运行边界计数；不覆盖未知参数依赖 |
| C seed/IMU/视觉/主状态/anchor 联合传播 | 有限单槽已知源范围 | native FD、已知源 MC、实际只读 shadow OFF 对照与源/压缩审计；完整槽与初始化源缺口列明 |
| C 真实误差与程序扰动区分 | 已修正并测试局部图表 | 原生非零注入双输入导数、离线 GT 图表映射；固定 nominal H/K 范围声明 |
| C 相关 residual/updater 与模型版本交付 | 有限 kernel 检查，不具备真实接入资格 | R_L/N_L/visual cross、稳定求解、FD、可消费版本说明；真实源/均值缺口若阻断则具体 STOP |
| 独立候选后组合集成 | 尚不具备准入 | integration_report.md；不得从接口单测或单项性能继承资格 |
| 总报告与各线命名交付文件 | 部分已落盘，最终版本待收口 | gv_report.md、landmark_gate_report.md、landmark_approx_report.md、joint_error_model.md、joint_model_validation.md、integration_report.md、final_usability_and_integration.md |
| 全 run manifest/失败/重跑/配置/原始哈希/命令 | 收集进行中 | 根 execution 清单 + 各线原始/指标/确定性汇总；最终无遗漏核对 |
| 提交、推送所有实验分支并核对远端 OID | 根分支曾核验，全部最终 OID 未完成 | branch delivery 表；A/B/C 和根最终提交 ls-remote 对照；不合并主分支 |
| 最终生产 OFF/未提交用户文件保留 | 持续要求 | 最终 config/default diff、git status 与源码证明 |

当前目标必须保持 active。完整实验资格、真实统计一致性或可用组合尚未证明。
