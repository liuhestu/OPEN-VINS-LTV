# 正式 ON 组独立只读审核

**通过：冻结的 11 个 ON 序列全部覆盖，身份、账本 terminal 与已保存工程摘要一致。**逐序列事件数、attempt 与 active 汇总见 `formal_on_independent_audit.json`。

Supervisor complete=true，11 项序列集合精确等于冻结集合且全部 kind=ON、退出码 0。各运行在共享账本中均有唯一 real/confirmation 预留与 exit0 terminal；各目录 attempt 与 supervisor 一致。根目录和 P_NEW 输出目录 identity 相同，绑定冻结 SHA `5f9e0763fc2e1f7ba4025f80123be41ae2f0c65901d3e1baee20b897841ba1e0`、runtime/manifest、完整源码清单、协议、原基配置树和逐序列输入 metadata。保存配置文件实际 SHA 与 identity 一致；六项 active 参数精确为 C02 固定值，没有逐序列调参。

11 份 replay 均完整消耗相机和 IMU，实际 G/V 提交为零。实际配置中 LTV/feature 开启、G/V 注入关闭。所有 passive 摘要 PASS/complete/zero_injection；所有硬化日志摘要 PASS_ENGINEERING_ONLY，receipt 数等于相机包数。所有 active 摘要 PASS_LIVE_COUNTERS_AND_LIFECYCLE，且逐序列 evaluable=pass+skip+retire。

本次仅读取 JSON、配置及账本，不重新扫描 features/cache 大文件，不读取 GT，不执行新计算任务。live checker 的逐行正确性沿用运行时证据，本次复核的是其完整成功记录与计数守恒。汇总 sent 数不能证明被拒观测实际未进 core；该项依赖原 native/cache/core 路径证据。

仍待完成/另证：两个代表重复、完整 native logging OFF 对照、科学评价与至少 8 个有效 B、三方法主状态/P/FEJ/轨迹一致、历史 B/OFF 复用身份、正式合成及恢复/资源合同。本页不将 11 次工程成功称为全部科学目标达成，也不覆盖开发 recovery 负结果。

未修改冻结文件或受测源码。
