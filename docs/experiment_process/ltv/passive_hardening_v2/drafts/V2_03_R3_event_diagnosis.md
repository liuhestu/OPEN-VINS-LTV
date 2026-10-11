# V2_03 R3开发事件归因（离线）

结论保持NOT_MET；没有调整算法、门槛、候选或评价窗口。使用186原始输出及188保存数组，192离线归因、201只读decoder编译、202原cache目标事件解码。没有额外估计器运行或GT在线访问。

## raw首次分歧

以首次初始化时刻1413394887.3557606为0：前1.70秒raw V精确相同；1.75秒（camera_ns=1413394889105760512）出现4.87e-16差异及首次管理分歧。NEW出生STEREO4446，PREV出生TEMPORAL4371。下一事件1.80秒速度向量差0.00148252m/s，1.90秒0.02472863m/s，17.35秒首次超过0.1m/s。全程只有初始一次bootstrap，无Dormant/Recovering及中间raw缺失，所以不是再次零初始化。

两候选相同帧几何与风险一致：4371风险0.02315482094，4446风险0.04200849426。4371两边都有足够历史，NEW理由为SEED_READY/capacity，即30个active槽准入竞争，不是历史缺失或guard拒绝。4446在NEW从1.60秒首次记录，至1.75秒积累4帧/0.15秒；PREV同区间有几何seed诊断却没有该ID的manager track。不能把seed几何OK等同完整历史质量已接受。

独立`decode_cache_event.cpp`直接解析两份原cache的实际expected evidence及context，没有调用Adapter/replay或依赖overlay出生结论。202确认原cache实际births就是NEW4446、PREV4371；准入前四象限数均[3,7,8,10]，4446处sector0、4371处sector1。当前bearing输入相同，NEW选择4446符合先稀疏象限、再风险的固定规则，不是随机排序证据。

有界manager会在退休后立即forget对应history，旧模式保留至历史TTL；当前包确有history capacity拒绝其他候选。PREV没有4446 track而NEW有，支持“历史容量释放改变候选可用集合”的解释。cache未保存完整history map，无法直接重建每帧256容量构成，因此具体容量占用链仍是源码支持的机制推断；不声称已通过直接history快照证明。不能推断该最早差异独自解释全程RMSE增加，也不把全部差异归因guard误拒。当前源同输入P_PREV若另行计预算，可进一步隔离历史版本与有界机制。

## joint最长2.80秒：先区分输入缺帧与健康撤回

最大严格episode为1413394933.3057606至1413394936.1057606，57包、2.80秒。下一可见相机事件4936.2057607间隔0.1000001秒，超过现有离线episode规则1.5倍中位0.05秒=0.075秒，直接断开连续段；这一帧及之后两帧G/V仍均ready。到4936.4057605才撤回：17个observed/13个mature，pool_not_mature，同时预测角0.0202361rad超过0.02rad。因此最长2.8秒不能只解释为健康门槛失败；需要明确输入事件间断这一固定评价规则。此次未改该规则。

整条序列共15次joint true→false：预测角违规6次、pool不足5次、V校正率违规6次、G校正率违规1次、诊断无效3次，类别可重叠。5次G继续ready、仅V撤回；其余10次两个分支一起撤回。raw始终保留，ready撤回不代表状态重置。完整时间/状态/阈值数值保存于event_diagnosis.json。

证据目录：`/home/he/output/ltv_passive_hardening_v2/real_development/R3_V2_03_01/evaluation_v1`，含`cached_event_new.txt`、`cached_event_prev.txt`、`cached_event_comparison.json`及`event_diagnosis.json`。历史overlay仅辅助track生命周期分析；raw数值来自独立核验过的原始结果，关键births已由原cache直接证明。

## 历史容量兼容修复后的独立完整验证

新runtime `1d5c04fef6f0a5f56570480484eec9e46c04437e946db5527340b3c10a70e236`、native227完整exit0，未调整gain/ready或评价阈值。离线231严格历史输入/配置/main audit/trajectory全部通过。原始整数camera_ns由main audit精确一致证明。

独立只读cache232通过1921个相机事件及23370个IMU记录：原始输入字节、完整x/P、active外部ID/coreID/slot、retained/retired/correction inputs/births逐事件完全一致。1921个事件的health/log metadata不同、1788个事件非活动旧mapping不同被单独报告，未混入数值比较。此证据比仅raw RMSE相同更强，证明本条完整正常序列的资源兼容性修复确实消除了数值偏差；不外推所有序列。

raw V/eta/angle RMSE分别精确等于历史0.3340292818、1.1751572879、8.686851247。序列状态仍 **NOT_MET**：只剩joint最长2.85000014s不足5s。G/V coverage28.6822%/21.2071%，有效参考25.9000/19.1500s；绝对G/V、严重错误与raw回归均通过。不得抹去修复前186的负结果，也不得因修复后raw一致宣布完整合同通过。

日志234通过：1921receipts、2727出生(STEREO1532/TEMPORAL1195)、2716active退役、4110TTL、3237guard拒绝逐帧守恒。严格唯一JSON键检查通过，1794个非空实际校正输入事件、127个无当前校正事件；actual_corrections计1528（仅足够特征时增长）。所有正式raw包1806/1806，无缺失。

新证据目录 `/home/he/output/ltv_passive_hardening_v2/real_development/R3_history_compat_V2_03_01/evaluation_v1`；232比较报告位于统一账本attempts/0232/stdout.log。
