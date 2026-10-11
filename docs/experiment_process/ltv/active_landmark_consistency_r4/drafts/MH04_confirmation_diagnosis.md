# MH_04 正式确认失败：固定结果及生命周期初步诊断

本页只读冻结 C02 与同身份 R3B OFF 结果，不修改参数或评价支持。正式 all11 汇总为 10/11 序列通过，MH_04 为 NOT_MET；不能以其余序列成功覆盖这个负结果。只读分析 attempt 101 exit 0；未运行新的 observer、仿真或参考重算。

## 证据身份与工程边界

外部根目录 `/home/he/output/ltv_active_landmark_consistency_r4`。评价为 `real_confirmation/CONF_ON_C02_01_ON_MH_04_difficult/evaluation_v1/metrics.json`，SHA256 `894c239635e6698ee4c43a8df50f71adc519c5ffc21b1c64421a21ce0502479f`。前驱为 `CONF_CONTROLS_C02_01_OFF_MH_04_difficult/P_NEW`，原始 B 为 `CONF_CONTROLS_C02_01_B_MH_04_difficult/B`；方法名来自评价显式 provenance，不把目录 P_NEW 名误解为开启 R4。

三条运行均完整消费 2032 个同步相机包、20320 个 IMU；初始化后 1919 包，主状态 audit 和主 trajectory 精确一致，无任何实际 G/V 注入。左右原输入 2033/2032 帧的一帧不匹配在三者相同。工程 PASS 只说明隔离、输入完整和一致性，**不证明主 VIO 数值准确**。

现有评价的同支持 OpenVINS 速度 RMSE 为 410.0703 m/s、重力角 RMSE 为124.9189°，是必须独立复核的异常。这里未读取 GT 原始数据，也未断言异常来自参考、初始化或主估计器中的哪一项；不能忽略它，也不能据此事后删除 MH_04。

## 固定评价数字

| 指标 | R3B OFF | C02 ON |
|---|---:|---:|
| raw 当前帧 |184/1919 (9.5883%)|180/1919 (9.3799%)|
| raw 缺失，均标 DORMANT |1735|1739|
| G / V ready 包数 |0 / 0|0 / 0|
| 最长 joint ready |0 s|0 s|
| 自身 raw 支持速度 RMSE |9.02169 m/s|13.77877 m/s|
| 共同180帧速度 RMSE |9.11632 m/s|13.77877 m/s|
| 共同180帧 eta RMSE |5.39933 m/s²|6.63404 m/s²|
| 共同180帧重力角 RMSE |68.24348°|70.43790°|

共同支持速度增加4.66245，超过原允许增加0.45582；eta 增加1.23471，超过0.26997。因此 raw regression 确实失败，并非仅仅支持集不同。角误差增加2.19442°，未超过该项原允许3.41217°；该单项通过不意味着角度绝对准确。NEW 自身 raw 速度 P95 为18.5100 m/s，PREV 为7.44051 m/s。

两者 ready 集均为空，其绝对误差及严重误差比例均为 null。评价的 severe_G/V=false 是没有足够 ready 证据通过合同，**不是观测到严重误 ready 帧**，也不能将空集合记为零错误通过。raw 的严重比例属于另一个支持集：NEW V/G 为98.8889%/93.8889%，PREV为75.5435%/86.4130%。

## 实际时间轴与差异

时间原点固定为三者首个 initialized target IMU time `1403638132.945097`，最后一包相对95.9000001 s。B 关闭 LTV，not_started 共1919包；不能将 B 的 LTV 空输出当运行故障。

| 相对时间 | 两者共同或不同的实际行为 |
|---|---|
|0–1.00 s|有界零初值预热，0个点、0次健康确认校正；1.00 s因 prolonged_insufficient_geometry 进入 DORMANT，保留触发包实际状态。|
|1.05 s|整体 core 停止，epoch变2，raw不可用；physical_fault_count仍0。|
|33.85 s|两者同一时刻以16个 eligible seeds 执行 ZERO_DELAYED_START。|
|33.90 s|NEW 已 observed=4、mature=0，进入 thin_observations_state_preserved_not_ready。|
|35.50 s|PREV 此时才转 thin 状态，observed=12、mature=11；其 actual_corrections计数32。此计数是 readiness 累计确认诊断，不等于所有真实点校正次数。|
|39.75 s|PREV短暂回 pool_not_mature，observed15/mature14，累计计数33；下一包又thin。|
|41.50 / 41.70 s|NEW / PREV各以0观测 prolonged_insufficient_geometry 进入DORMANT；下一包停core。NEW该段健康校正计数始终未累计。|
|71.60 s|两者再次同刻以17个eligible seeds重建。|
|71.80 s|两者同样 observer_norm_guard：v范数102.54326 m/s，eta范数5.91075；速度/重力修正率1026.98262 /74.65888；下一包停止。|

NEW 丢失而 PREV 当前的四包是绝对时间1403638174.4950972–1403638174.645097（相对41.55–41.70 s）。都有明确DORMANT原因，故 raw_missing_explained=true；但属于启动后支持损失，尚未经可接受性审查，raw_support_no_unreviewed_loss=false。不能把“解释了缺失”替换成“没有退化”。两者均3次bootstrap、无物理fault；cooldown原因均86包，未见重建风暴证据。

## 种子与一致性处理

两者日志均91次 mean_written，来源均71 STEREO、20 TEMPORAL_POSE；该总数不证明逐ID、逐时刻和后续存活完全相同。NEW 301条当前点检查中177可评价：PASS76，HISTORY_INCONSISTENT99，HOLDOUT_INCONSISTENT2；不可评价 FIT_FAILED72、INSUFFICIENT_HISTORY52。行为为49次 history SKIP、50次 history RETIRE、2次 holdout SKIP；不可评价按冻结合同 UPDATE。没有将FitFailed私自当一致性失败。

这些证据表明新增一致性处理在第二个短暂活动段显著削弱当前有效观测供给，并伴随 raw 恶化；但尚不足以将每一次恶化唯一归于删除哪个点。原 R3B 已长期缺供给且从未ready，第三次异常峰两者完全相同，不能称本序列全部失败由 R4 引入。也没有证据支持放宽门槛或重选候选。

可复现只读脚本：`scripts/ltv_active_consistency_r4/diagnose_mh04.py`；输出 `diagnostics/MH04_lifecycle_01.json` 保存三条 features SHA、每次状态/启动/停止切换、完整计数、每次均值写入及已有 error_curves SHA/schema。缺失字段用null；未将B不具备的raw字段假设为有效观测。后续若审查主VIO/参考异常，应独立保留此正式负结果，不能重写它。
