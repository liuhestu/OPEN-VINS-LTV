# C01 完整 V2_03：两目标的真实生命周期后续证据

本页只读完整 native attempt32 的 `real_development/C01_V2_03_01/P_NEW/features.jsonl`，不覆盖两份局部干预报告。时间原点仍为1413394887.3557606；两个目标整数receipt为1413394964105760512与1413394966255760384。未改代码、运行新仿真或再拟合几何。

**连续启用路径确认：27537的目标坏观测确实跳过且slot保留；29636在原目标前0.30 s已因连续一致性失败退休，原目标时由身份保护阻止重新准入。** 全序列验收仍未完成：主线程已保存evaluation_v1显示最长joint4.55 s，小于固定5 s。

## ID27537

| 相对时刻 | 实际动作及原因 | fail count | 当前观测/slot |
|---|---|---:|---|
|73.45 s|一次STEREO准入，admitted=1、mean_written=1|0|正常入槽|
|74.25 s|HOLDOUT_INCONSISTENT；history max0.021793、holdout0.042415 rad|1|SKIP，不在corrected_ids；仍retained|
|74.30 s|PASS；history0.015738、holdout0.025729|0|UPDATE，成功清连续失败计数|
|74.35 s|PASS；history0.006002、holdout0.007438|0|UPDATE|
|76.75 s（目标）|HOLDOUT_INCONSISTENT；history0.002366、holdout0.350263|1|SKIP，不在corrected_ids；仍retained|
|76.90 s|ACTIVE_RETIRED，lost_or_identity_invalid，missed_frames=3|—|退出；不是一致性两连败退休|

目标检查使用11条可解析历史、0.5 s跨度，另9条历史缺当前clone支持。该ID全程只有一次实际seed均值写入，65个真实corrected_ids出现，66次active-consistency记录（birth本包尚不是已活动检查对象）；两次拒绝均没有伪造tracker missing。76.75 s最后可见，随后实际三包失踪到76.90 s退休；没有把消失宽限当成额外可见时间。

该目标完整路径整帧observed/mature/state为22/22/30，prediction P95=0.0050233 rad、v correction rate=0.293577；ready_G=true、ready_V=false。这里所有其他点和共享状态已经受更早启用影响，不能把该完整路径数值直接当成局部单点快照干预的重复结果。

## ID29636

| 相对时刻 | 实际动作及原因 | fail count | 当前观测/slot |
|---|---|---:|---|
|78.45 s|一次TEMPORAL_POSE准入，admitted=1、mean_written=1|0|正常入槽|
|78.50 s|PASS；history max0.001027、holdout0.001554|0|UPDATE|
|78.55 s|HOLDOUT_INCONSISTENT；history0.001101、holdout0.083005|1|SKIP，仍retained，不校正|
|78.60 s|HISTORY_INCONSISTENT；history max0.070650|2|RETIRE，不校正，从retained移除|
|78.65–79.00 s|共8包原ID仍有观测，identity_guard_rejected_ids记录拒绝|—|均未重新入槽/均值写入|
|78.90 s（原目标）|已经退休，当前cam0仍提供该ID|—|guard拒绝；不在retained和corrected_ids|

78.55 s使用7条历史、0.30000019 s跨度；78.60 s使用8条、0.35000014 s跨度，无缺clone。退休事件的record.reason=`active_consistency`，missed_frames=0，因而不是把视觉拒绝当tracker失踪。全程仅一次实际seed均值写入、两个真实校正事件（birth及78.50）、三次active检查。退休后的日志仍可能含该ID的候选几何诊断，但所有后续admitted/mean_written均为0；计算候选诊断不能误报为重复seed。

原78.90 s整帧observed/mature/state为28/18/30，P95=0.00431165 rad，v correction rate=0.155564，ready_G/ready_V均true。此时不应期待再次出现29636的active `HistoryInconsistent` 项：身份早已退休，该目标包正确由guard路径处理。

## 对照边界

旧R3B目标事件中27537参与校正；29636在78.90 s仍是活动旧点。局部short24/25从旧完整after-lifecycle快照仅过滤指定目标，证明同起点因果干预收益。这里是从头连续启用C01之后的真实生命周期，尤其29636的提前退休补齐了局部first-skip实验不能证明的顺序行为。两类证据互补，不能混合当同状态比较，也不能用两目标成功替代全域coverage、误差、恢复或连续5 s合同。
