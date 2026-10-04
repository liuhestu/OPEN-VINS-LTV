# R3B供给与持续性诊断（开发证据）

**结论：目前没有证据证明，单靠维护成熟点或改变合法供给管理即可解决V2_03的连续5秒joint-ready不足。** 本诊断不调整grace、ready或误差门槛，不新增候选，不宣称其他路径不可能。

数据为 `OUT=/home/he/output/ltv_passive_hardening_v2` 下 `real_development/R3B_grace_V2_03_01/P_NEW/features.jsonl`，误差直接读取同目录 `evaluation_v1/error_curves.npz`，按原整数camera_ns映射。时刻均相对初始化首包；窗口是离线诊断支持，不更换原验收支持。离线episode使用既有0.075秒缺包划分，仅用于统计，未用于online迟滞。

| 连续pool有效窗口 | 时长 | raw速度RMSE | raw方向RMSE | 主要中断证据 |
|---|---:|---:|---:|---|
| 0.30–5.50 s | 5.20 s | 1.3013 m/s | 6.3860° | 真实启动大误差，不宜把确认等待删掉 |
| 5.80–13.95 s | 8.15 s | 0.08778 m/s | 0.90908° | 28包strict失败：预测残差超限22包、V修正率超限13包、G修正率超限1包（类别重叠） |
| 72.15–81.70 s | 9.55 s | 0.10502 m/s | 0.68996° | 41包strict失败：预测18、V修正35、G修正5（类别重叠） |

全时域current最长15.60秒；observed≥15最长12.35秒；pool最长9.55秒；瞬时strict-good最长3.80秒；实际joint最长2.85秒。因此不是输入或pool从根本上不提供5秒窗口，但也不是仅取消重新确认延迟即可完成。

current状态中414包pool不足：278包当前观测不足15，136包仅成熟点不足；其中只有1包前端cam0总数不足15，37包状态池占满30，26包存在**当前可见、当前last_seen且reason=capacity**的候选。26包提示局部容量竞争可以研究，不证明修改容量/准入能消除两个长窗口里的健康中断。2716个active退出均记录 `lost_or_identity_invalid`；没有退出时仍出现在当前cam0集合的ID。未发现仍可见成熟点被主动淘汰的直接证据。

“仅看成熟点预测残差”不足的具体反例：9.85、9.90、9.95、10.00秒均为30个observed=30个mature且零birth，预测P95分别0.02105/0.02338/0.02568/0.02802 rad；9.95秒第三个soft包撤销joint时最年轻驻留也已有0.25秒。10.05–10.10秒仍27=27 mature、零birth且预测超限。后两个长窗口分别7/22与2/18个预测坏包全部观测均成熟。76.75秒为23=23 mature、预测0.01155正常，但V修正率3.451、G修正率1.447；不能只筛预测指标而隐藏共享状态校正影响。

现有features日志没有逐点observer预测残差，不能将种子三角化residual冒充该量，也不能从相关性推断具体点的因果贡献。

## 复现与更正记录

- analysis270运行 `/tmp/ltv_r3b_supply_audit.py`，exit0。原脚本逐字节存为 `scripts/ltv_passive_hardening/tests/r3b_supply_audit_v1.py`，SHA256 `b8eb45e28f47c125f3bc35def9e4e935ee13d0972af1f49a45af6d9d0bf486f4`。
- v1把所有track中残留的 `reason=capacity` 都计作当前候选，错误计出302包；包含已不当前可见的陈旧记录。保留原脚本、账本与 `supply_break_audit.json`，该302不可作为结论使用。
- analysis271以当前cam0与last_seen==当前物理时刻约束修正为26包，并补窗口误差与全成熟反例，exit0。原执行脚本逐字节存为 `scripts/ltv_passive_hardening/tests/r3b_supply_audit_v2.py`，SHA256 `1874cd772dd8d07827a2f739f2b2a58850c61158315fa5e0c5abda57b099970d`。
- 正式诊断产物为运行目录的 `supply_break_audit_v2.json` 与 `supply_and_error_detail.json`。两个脚本固定该输入/输出路径，是复现已有分析的脚本，不是通用验收器；它们执行会重写各自分析JSON。需要重算时先保留原产物，并经共享budget以analysis运行，不能直接占用未登记计算槽。

本次归档没有重新运行分析、完整仿真或真实回放；策略和冻结合同均未修改。
