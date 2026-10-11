# G/V/GV 最终裁决：STOP

冻结参数下三个模式在 V2_03 的固定末窗均失败；同源、同参数完整四模式重复精确复现。该窗口 OFF RMSE 0.188966692 m，G/V/GV 恶化 5.826/5.790/5.809 mm，超过冻结允许的 3.779 mm。支持仅三个时刻的窗口仍保留，不以全程微改善或首轮性能通过抵销失败。

两 DEV 的原样 OFF 与诊断/标量回归通过；首轮八次生产标量 p95 与峰值 RSS 子项通过。G/V 均有真实联合更新作用，但实际收益未证明，完整跨序列工程不退化失败，统计一致性未成立。留出与 reserve 未消耗，不扫描权重，也不进入 GV 组合资格验证。

完整报告、1008 指标、全部失败/重跑、输入/产物身份和独立 Horn 公式复核见 [A 最终报告](task_a/report.md)、[指标](task_a/evidence/metrics_first_round.csv)、[重复](task_a/evidence/repeat_determinism_v203.json) 和 [A manifest](task_a/run_manifest.csv)。候选配置在 `config/ltv_three_track/task_a/fixed_c0/`，仅供复现已失败实验，生产默认 OFF。

实际二进制源码 OID：`939c689e11aa1ef30adbf60846ef7e5fdbc55290`。最终分支交付 OID：`20c9ebfd52ed85bc95499b2f9447a14a40dfc85c`，分支 `experiment/ltv-three-track-a-20261008`。根分支按该不可变 OID 发布文档和配置，没有集成或启用其算法。
