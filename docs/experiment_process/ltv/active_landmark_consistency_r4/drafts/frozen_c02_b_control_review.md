# 冻结 C02 与 B 控制分组独立复核

本次仅静态阅读及小文件 SHA 核对；没有启动测试、重放或额外重计算进程，没有修改冻结工具或协议。

## 已核对的实际冻结身份

`frozen_config.json` SHA-256 为 `5f9e0763fc2e1f7ba4025f80123be41ae2f0c65901d3e1baee20b897841ba1e0`，与主线程公布值一致。353 个冻结源码条目逐项核对无变化；goal、protocol、acceptance、runtime manifest 和基配置树文件 SHA 均与清单相符。合成 wrapper 路径和完整 SHA 已列入清单，本次没有重新读取大型二进制验证 wrapper 或 runtime 全部依赖，沿用既有 artifact 验证并由执行入口再次检查 runtime。

清单分组为 11 ON、9 新 OFF、7 新 B、2 ON 重复，总共 29 次。协议与 scheduler 的真实上限同为 100、正式保留额同为 29；原有尝试继续计数。此项替代前一审查中 22 次计划的数量描述，不改变科学验收。

## B、OFF 与 ON 路径

| 方法身份 | native 模式 / 输出子目录 | 配置行为 | 工程检查 |
|---|---|---|---|
| B | B / B | active consistency 关闭，并关闭 LTV、feature readiness、passive hardening | 实际零注入、完整输入与主轨迹/audit；要求无 active schema |
| P_PREV（OFF） | P_NEW / P_NEW | 仅关闭 active consistency，保留 R3B 原配置 | 原硬化日志检查、零注入与完整输入；要求无 active schema |
| P_NEW（ON） | P_NEW / P_NEW | 启用冻结 C02 | 原硬化与 active 日志检查、零注入与完整输入 |

`baseline and enabled` 在任何输出创建前拒绝。正式 B 同时要求冻结 `allow_new_B_controls`，并沿用 runtime/config/source/input/合同身份检查；本冻结还明确允许 OFF 控制。B 不执行硬化日志 schema 检查合理：关闭的观察器没有该状态机输出；这不豁免实际注入与主估计器一致性验收。原 native runner 验证模式与配置匹配、禁止 G/V 注入并启用统一 passive audit。`passive_outputs.inspect` 校验实际 receipts/submissions 为零、相机与 IMU 全量消耗及主轨迹有限。

B 配置仍写入固定的六个 active 参数，但 enabled 为 false，因此不能将这些无效参数看成开启算法。OFF 的目录及 native 名称仍为 P_NEW；最终组合必须使用 `identity.mode`，不能按目录名把 OFF 当 ON。

## 调度与剩余义务

`confirm_all.py` 从冻结的三组列表构造任务，CONTROLS 精确为 9 OFF 加 7 B，ON 为 11，REPEATS 为 2。线程池上限 2，每个 native 子任务通过预算登记；失败保留，无自动重跑。通用 `real_run.py` 只检查序列属于全部 11 序列及相应模式许可，没有把单次 B/OFF 限定到各自子列表，也没有独立约束重复次数；因此本轮矩阵完整性须由冻结 dispatcher 输出和最终账本逐项核对，不能仅凭单次 preflight 宣称矩阵唯一性。

**结论：未发现阻断当前冻结分组执行的模式错误。**7 个新 B 是对不可严格复用基线的补齐，不能省略其他 4 个历史 B 的身份验证。至少 8 个有效 B、三方法主状态/P/FEJ/轨迹一致、全部科学指标、重复性及合成/恢复合同仍需按实际结果验收。本页不宣称正在执行的任务已通过，也不覆盖已保留的开发负结果。
