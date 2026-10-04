# 选中C02：完整轨迹两目标生命周期及全域修正率

本页对应 **C02双角阈值0.02 rad**、`real_development/C02_V2_03_01/P_NEW/features.jsonl`，不是C01轨迹。只读analysis **56** exit0；脚本 `scripts/ltv_active_consistency_r4/review_selected_lifecycles.py`，结构化证据 `/home/he/output/ltv_active_landmark_consistency_r4/C02_target_lifecycles_and_rates.json` 保存输入SHA、所有目标active检查、出生/退出/guard及1921个原整数receipt的old/new诊断时间轴。没有新增真实/合成运行或修改策略。

**C02在原目标时刻均不再使用这两个ID，但原因不同：27537已因连续一致性失败退休；29636在该完整路径中根本没有入槽。** 不能套用C01中29636的active退休结论。该序列既有evaluation_v1为PASS，整体多序列/恢复/资源验收另由主线程汇总。

## ID27537：先历史失败，再留出失败而退休

时间相对first initialized IMU时间1413394887.3557606，epoch=1。

| 时刻 | 原生记录动作/原因 | 连续失败数 | 校正/保留 |
|---|---|---:|---|
|73.45 s|STEREO准入；唯一admitted=1、mean_written=1|0|正常入槽|
|73.50 s起至74.20 s|可评价PASS；初次history max0.002962、holdout0.001635 rad|0|正常UPDATE|
|74.25 s|HISTORY_INCONSISTENT，history max0.021793 rad|1|SKIP，retained=true、corrected=false|
|74.30 s|HOLDOUT_INCONSISTENT，history0.015738、holdout0.025729 rad|2|RETIRE，retained=false、corrected=false|
|74.35–76.75 s|49包identity_guard拒绝该仍可见ID|—|无重新准入/均值写入|
|76.75 s原目标receipt|current_cam0=true、guard_rejected=true|—|不在retained或corrected_ids|

总计16个真实corrected_ids出现，一次实际seed写入。退休记录为ACTIVE_RETIRED / active_consistency，missed_frames=0，说明不是tracker失踪退休。所有已发生的可评价检查都保存在JSON，没有仅保留失败包；该点首次FAIL后没有PASS归零，下一包就是第二个FAIL。**C01的74.30 s PASS在C02变为FAIL**，是固定0.02与0.04门槛的真实差异，不是计数错误。

原76.75 s整帧observed/mature=23/23，ready_G和ready_V均true；此时目标早在2.45 s前退出，不能再要求本包出现目标的holdout检查，也不能把同包结果当作旧轨迹单点干预的重复。

## ID29636：从未入槽，最后为候选TTL

全域admitted=0、mean_written=0、corrected_ids出现0、active-consistency记录0，没有一次活动点的一致性计数可以追踪。

| 时刻 | 原生记录 |
|---|---|
|78.45 s|TEMPORAL_POSE几何有效，risk0.045987、seed residual0.001098；SEED_READY reason=capacity，状态池30，admitted=0|
|78.50 s|仍SEED_READY / capacity，risk0.038101、residual0.001191；状态池30，未准入|
|78.55 s起|原seed质量规则angular_residual拒绝（该帧0.070618 rad）；不属于active consistency失败|
|78.90 s原目标|current_cam0=true，但retained=false、corrected=false；guard_rejected=false，因尚未成为受保护活动ID|
|81.05 s|CANDIDATE_TTL / candidate_ttl；entered=seeded=-1、seed_written=0|

容量原因由当帧track `SEED_READY/capacity` 与无birth证据给出；没有把仅用于其它拒绝类型的capacity_rejected_ids空列表强行解释成“无容量竞争”。原78.90 s整帧observed/mature=27/17，G/V均ready。该路径说明持续启用改变了池竞争；不能宣称C02通过active检测退休29636。R4局部固定旧快照实验仍独立证明该ID若已活动，会被HistoryInconsistent识别；两者不同起点，不能混报。

## 固定全时间轴的修正率比较

比较旧R3B与C02一致的1921整数receipt及精确physical target。只使用原logger的 `correction_diagnostics_valid` 和相同字段；未读取GT挑点。统计“超限”沿用原健康界v rate>0.5、eta rate>1，不另选有利大值门槛；角阈值沿原P95健康0.02。缺诊断不补零。

| 指标 | 旧R3B自身有效1472包 | C02自身有效1436包 | 旧R3B在共同1436包 | C02共同1436包 |
|---|---:|---:|---:|---:|
|v修正率>0.5次数|187|110|180|110|
|eta修正率>1次数|69|59|68|59|
|prediction P95>0.02次数|181|125|173|125|

自身有效支持外分别449/485包（包含初始化和其它诊断不可用），完整时间轴均保留；不能仅凭有效分母减少宣称收益。共同支持上v超限180→110、eta68→59，提供同时间位置的描述性对照。

**极端值没有全面改善。** 两路径最大v修正率均9.170326 m/s²，最大eta修正率均5.111550 m/s³，均发生于初始化后1.10 s。最大prediction P95发生于51.80 s，旧2.846053→新2.846999 rad，略增。JSON保存每项top10及全部事件，并未删掉启动峰或坏时段。本页统计的是均值校正差除真实相机间隔，不是物理加速度；大修正率不等于真实误差。

这些整段轨迹已经具有不同的状态、P和准入集合。上述同时间轴对照支持“共享修正率超原界的事件减少”，不能把全部变化归因于任一个目标点，也不能替代既有metrics中全域raw/ready误差和coverage的共同支持评价。局部干预、连续生命周期、完整科学验收三类证据分别保留。
