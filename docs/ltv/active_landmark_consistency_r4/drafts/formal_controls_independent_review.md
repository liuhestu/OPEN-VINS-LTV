# 正式 CONTROLS 只读独立审核

结论：**9 个 OFF 与 7 个 B 的分组、身份、终止记录和已保存工程检查通过**。结构化逐运行证据见 `formal_controls_independent_audit.json`。本结论不等于科学验收，也不代替三方法之间的完整状态与缓存数值比较。

## 本次核对范围

- Supervisor 的 complete=true、冻结 SHA 为 `5f9e0763fc2e1f7ba4025f80123be41ae2f0c65901d3e1baee20b897841ba1e0`，16 项无缺项、重复或额外分组，精确覆盖冻结的 9 OFF/7 B。
- 全部 attempt 59–74 在共享账本中各有一次真实 confirmation 预留及一次退出码 0 的 terminal；与各目录 attempt 和 supervisor 退出码一致。
- 根目录与实际输出目录 identity 相同；sequence、mode、phase、freeze SHA、runtime/manifest、完整 source 清单、协议、原基配置树和输入 metadata 均匹配冻结身份。没有将 OFF 的 `P_NEW` 输出目录名误判为 ON。
- 每个保存配置文件的实际 SHA 与 identity 相同；从冻结基 YAML 仅应用记录的 active 配置和 B 开关，可以逐字重建运行 YAML。实际 effective_options 的 B/feature 开关符合分组，G/V 注入配置关闭。
- 16 份 replay 都记录完整相机和 IMU 输入消耗、实际 G/V 提交为零；16 份 passive 工程记录均 PASS、complete、zero_injection。9 个 OFF 的硬化日志工程检查均 PASS_ENGINEERING_ONLY，receipt 数等于实际 camera_packets；全部记录 active schema 不存在。

本次只读取小型 JSON、配置、账本及已保存工程摘要，未重跑大日志 schema checker、未重新散列大型图像/缓存、未读取 GT、未启动计算任务。输入文件与实际日志逐行检查沿用运行时保存的检查证据，不能把本页称为这些检查的第二次完整执行。

## 尚未通过本页验收的事项

11 个 ON 仍由主线程执行；2 个重复、正式合成矩阵及汇总评价尚须独立完成。新 B 初始化并输出有限轨迹不等于满足“至少 8 个有效主基线”的科学定义。其余 4 个复用 B 与 2 个复用 OFF 的完整身份、三方法主状态/P/FEJ/轨迹一致、缓存重放与所有精度/覆盖/持续时间/恢复/资源门槛，均须使用后续对应证据判定。开发 recovery 的已知负结果继续保留。

本页不修改冻结源码、协议或阈值。
