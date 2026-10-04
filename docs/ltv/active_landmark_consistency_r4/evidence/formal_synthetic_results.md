# 正式8项合成结果与评价边界

固定C02、种子201/202的8次60 s完整运行均 exit 0；每次1201相机事件、12001保存时刻且 full_input_consumed=true，估计器GT访问标记均false。本表只汇总既有指标，不重新求参考或修改科学阈值。`EVALUATED` / 完整运行不等于整体科学验收通过。

| 场景 | raw覆盖 % | ready G / V % | joint最长 s | ready V RMSE / P95 m/s | ready G角 RMSE / P95 ° | 严重ready V/G比例 |
|---|---:|---:|---:|---:|---:|---:|
|STEREO_REGULAR_1_201|100.00|90.59 / 88.93|26.85|0.0282 / 0.0532|0.0936 / 0.1450|0.0000 / 0.0000|
|STEREO_FAST_05_202|100.00|38.63 / 36.80|3.55|0.0616 / 0.1160|0.1396 / 0.2614|0.0000 / 0.0000|
|STEREO_FAST_1_201|100.00|22.65 / 21.82|2.25|0.0640 / 0.1057|0.2547 / 0.4473|0.0000 / 0.0000|
|TEMPORAL_FAST_1_202|100.00|20.57 / 19.82|2.20|0.0465 / 0.0883|0.2316 / 0.2908|0.0000 / 0.0000|
|WRONG_MATCH_REGULAR_1_201|100.00|79.93 / 78.10|17.05|0.0333 / 0.0666|0.1133 / 0.1787|0.0000 / 0.0000|
|STEREO_DROPOUT_REGULAR_1_202|100.00|81.27 / 79.02|26.45|0.0311 / 0.0543|0.1016 / 0.1886|0.0000 / 0.0000|
|TIME_ERROR_REGULAR_1_201|100.00|86.93 / 85.26|25.85|0.0279 / 0.0496|0.0973 / 0.1517|0.0000 / 0.0000|
|PRESSURE_REGULAR_05_202|3.41|0.00 / 0.00|0.00|null / null|null / null|null / null|

## 预登记正常来源验收

固定门槛为每来源至少100次接受、reliable10≥90%、接受/合法机会≥20%；reliable5另报。三个STEREO正常场景检验STEREO来源，TEMPORAL场景检验TEMPORAL_POSE来源。HYBRID的少量回退不是独立纯Temporal机会实验：不能对每个 `normal_source_pass` 字段简单取AND宣称整项失败。负条件与压力不适用正常来源成功率硬要求，所有原字段仍保留在JSON。

| 正常场景/指定来源 | 写入数 / 机会数 | 接受率 % | reliable10 % | reliable5 % | 来源合同 |
|---|---:|---:|---:|---:|---|
|STEREO_REGULAR_1_201 / STEREO|1808 / 1830|98.80|99.28|84.51|PASS|
|STEREO_FAST_05_202 / STEREO|3334 / 3630|91.85|99.19|87.46|PASS|
|STEREO_FAST_1_201 / STEREO|1688 / 1830|92.24|99.05|84.60|PASS|
|TEMPORAL_FAST_1_202 / TEMPORAL_POSE|1822 / 1824|99.89|99.51|93.74|PASS|

HYBRID四项非正常/负条件的回退可靠率不隐藏：WRONG_MATCH Temporal 186次，reliable10=94.62%、机会接受10.20%；DROPOUT Temporal443次，97.29%、24.29%；TIME_ERROR Temporal195次，97.95%、10.69%；PRESSURE Temporal26次，100%、0.72%，不足100样本。正常STEREO三项回退分别16/284/136次，不能据其低全Temporal机会接受率否定已通过的指定STEREO来源合同。

## 未闭合项与适用范围

1. 三项FAST的最长joint分别3.55、2.25、2.20 s，均未形成5 s段；TEMPORAL_FAST的V覆盖19.8168%，低于20%参照线。这些负数值完整保留。原goal§10.2覆盖/连续5 s针对有效真实EuRoC序列，合成正常案例中的同名absolute_contract为额外诊断，不能无说明扩张为原硬合同；合成的严重误ready要求仍单独适用。
2. DROPOUT独立供给恢复整体 `pass=false`：第一段30.00–31.50 s，deadline40.00，SUPPLY_INTERRUPTED_BEFORE_DEADLINE，false；第二段31.70–60.00 s，deadline41.70，PASS。第二段首宽档joint区间33.55–60.00，5 s要求在38.55 s已完成。不能删掉第一段将整体改成PASS，也不能称中断前已获得持续10 s充分供给。
3. PRESSURE有41个raw样本（3.4138%）、G/V ready均0，原severe_ready_pass=false对应严重比例null，没有实际坏ready帧，也没有可用性成功证据。raw速度峰19.0216 m/s在2.00 s，不通过ready输出；正常来源门槛未建立。此实验采用声明的midpoint时间戳与两端常值guard，不能与endpoint输入或旧seed压力数值直接回归。
4. 其余7项具有非空ready支持，G/V严重比例均0；未据此宣称全时域raw正确，完整raw峰值/缺失及错误保留在summary。
5. 全部8项缺少相同输入SHA的OFF对照，raw回归与新机制因果增益均为UNVERIFIED_NO_IDENTICAL_OFF。不能用旧seed42/101代替201/202；不追加运行补证。
6. 这8项不关闭真实MH_04失败、独立资源/重复性/性能等其他合同。文档只总结本组证据，最终状态由全任务合并判定。

## 身份与可追溯性

冻结SHA `5f9e0763fc2e1f7ba4025f80123be41ae2f0c65901d3e1baee20b897841ba1e0`；统一wrapper库SHA `88684ca34b42c5c349e02e94f5b36a9eabc3a7bec50d4d92ab3b226ec4a520de`，mode P_NEW_ACTIVE / ABI5。`formal_synthetic_summary.json`保存逐项input/labels/calibration/runner/library SHA、完整run身份、三阶段attempt、原评价布尔值及恢复区间。源账本 `/home/he/output/ltv_active_landmark_consistency_r4/formal_synthetic_jobs.json` SHA `678087054440f1ca3392b6c89231cdf0460fba6a59f9a05f1c63fcc25bfa6b11`。本次仅读取小metrics/run文件，没有扫描states/matrices或启动算法。
