# R3C shadow 固定合同与全域科学口径独立复核

独立 analysis293 exit0，脚本 `tests/audit_r3c_shadow_result.py`。结构化全表与全部 episode 保存于 `/home/he/output/ltv_passive_hardening_v2/diagnostics/R3C_shadow_01/independent_scientific_audit.json`。只读已保存 policy、原误差数组和 features；没有重跑 observer、生成新候选 mask、改变阈值或实现 R3C。

**结论：两处用户指定事件确实没有被 shadow 放行；但全域 shadow 仍未满足连续 joint 5 s，最长只有3.05 s。不能将这两处反例未放行或新增80个 V-ready 样本误称候选成功。**

## 固定策略与目标事件

静态复核 `diagnostics/r3c_shadow_audit.py`：合同 SHA 必须与已登记286一致；全域 policy.jsonl 及 provenance 在读取 error_curves 之前写出。历史项先移除过期、按 Φ 传输、再追加真实 Δ；角区间与校正原子分开。ordinary net/activity/mean/impulse失败只清确认，不丢历史；共享 hard failure 清历史；窗口完整之后才开始20事件确认。对1921个保存事件，293独立重算其已记录预算布尔值、G/V确认递推及输出，均一致。1806个 initialized 事件按整数camera_ns精确关联原物理时间，不改变分母。

|目标显示时刻|最近共享hard事件|该事件P95 rad|下一hard-valid anchor|目标有效窗龄|G/V-ready|
|---|---|---:|---|---:|---|
|76.75|76.199999809|0.042780189|76.25|0.500000000 s|false/false|
|78.90|78.549999952|0.062119948|78.60|0.299999952 s|false/false|

两个先前角度都超过冻结0.04外界，按合同清窗合理。anchor从下一个合法事件起，不回填前面的无效区间。目标时刻不足1s，所以完整窗和20事件确认尚未成立；没有发现靠错误清计数/清窗制造这两个 false 的证据。

但两个目标的**已存在部分窗预算均通过**：76.75 的 v net/activity/original impulse 为0.154298/0.492606/0.172563；78.90为0.101736/0.230944/0.111615。它们不是被累计校正数值本身识别为“不准确”，而是被此前hard故障后的等待约束排除。不得据此宣称净校正阈值已学会辨别真值误差。主analysis292同时间pre/post比较还显示76.75校正确实增大误差；这一事实与本次未ready可以同时成立。

## 全域固定验收口径

分母保留全部1806个 initialized 包，权重和1.5×median gap的原 episode 口径不变。G、V、joint 在这份 shadow 中恰好相同，均385包；这是一条数据的结果，不是把G/V合并成同一实现分支。

|固定指标|shadow结果|该项判断|
|---|---:|---|
|G-ready / V-ready比例|各21.317829%|达到20%|
|G-ready / V-ready参考时长|各19.250012 s|达到10 s|
|最长joint episode|3.049999952 s|**未达到5 s**|
|ready V RMSE / P95|0.052653 / 0.089186 m/s|达到0.2 / 0.5|
|ready G角RMSE / P95|0.831494 / 1.210061°|达到1 / 2|
|ready V / G严重误差比例|0 / 0|达到≤1%|
|全部ready参考缺失|0包|完整支持|

全部ready最大v误差0.175416 m/s，最大G角误差1.515780°；不是只查新增ready子集。最长joint位于init+7.75至10.80 s，62个包。以同385事件支持比较，P_PREV与R3B的误差完全一致；OpenVINS v RMSE/P95=0.037248/0.067427 m/s，G角RMSE/P95=0.844050/1.185391°。这不支持“LTV整体精度优于主VIO”的说法。

## 工具和证据边界

293验证保存的预算到确认/ready的递推，没有独立重建每个Φ或全部signed Δ。数学传输仍依赖276/284的既有公式/数值审查，不能把当前shadow称为生产实现验收。调用端对当前数据做了诊断时间/epoch/前cursor与dt检查，但共享 transition 行本身没有完整 previous-cursor 身份字段；不能泛称支持任意换源或缺失记录。已知原始输入关联证明仍需随证据保留。

shadow的gap代码为 `>0.20+1e-9`，而文本描述为既有0.20 s物理界；在当前20 Hz/偶有0.10 s数据上没有近边界影响。正式实现和数学反例必须明确使用既有生产合同，不能从这份shadow暗自继承新容差。Window类本身也不是完整外部输入校验API；不能用六个shadow fixture代替非有限时间、非零offset、大epoch、缺失transition、容量和默认OFF全状态回归。

此结果只回答冻结R3B数值轨迹上的已登记离线策略问题，不验证实际在线开销、缓存字段、全11序列、恢复、seed质量、资源或全状态不变性。两处反例未放行不撤销76.75真实误差增大的证据；全域仍NOT_MET，应保留失败而非改窗口、确认或连续性门槛追求5s。
