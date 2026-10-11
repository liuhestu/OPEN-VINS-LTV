# R3资源语义修复设计：历史缓存占用与再次准入分离

本文件是待独立审查的实现方案，不是已通过的修复。当前只新增方案和反例测试，尚未改manager/history实现或运行测试。

已证事实：V2_03首次准入分歧在1.75秒；新版4446在1.60秒进入历史、累计四帧后选入sector0，而旧版4446没有历史，旧版选4371 sector1。输入bearing与sector相同。提前forget退休历史和在history更新前滤掉身份保护命中，改变了256缓存的竞争；这违反R3资源修改应保持正常输入数值的合同。

## 最小修改

1. 原始观测包首先完整验证（512观测、合法camera、唯一ID/camera、有限bearing等），候选包上限也在构造map前验证。保持原子拒绝非法输入。
2. 只读记录旧manager候选的TTL到期集合，依据history更新前的last_time。此集合只为本次生命周期退出，不修改history、guard或时间。
3. `history.update(epoch,time,version,original_observations,retained_before_event)`使用完整原始观测。保留原排序、原TTL、原protected集合；即使ID已退休或被Bloom拒绝，其原始观测仍按旧语义参与历史缓存容量竞争与时间刷新。history仍固定最多256记录、每点21样本、每包512观测。
4. 只有history成功接受后，提交manager生命周期。已准入退役输出ACTIVE_RETIRED事件并删精确manager记录；已过期从未准入候选输出CANDIDATE_TTL、插guard、删精确记录。**不调用history.forget**。历史记录最终按history原有TTL自然清理，原始观测持续出现则按原规则刷新。
5. manager用于可见性/准入的视图仍排除guard命中和第2步过期ID。已有精确ACTIVE/COASTING优先，不因自己admit时插入的guard被误杀。guard拒绝是“禁止再准入”，不是“禁止保留输入历史”。
6. TTL同事件返回：第2步判定永久退出，即使第3步原history删旧条又重建同ID，也必须typedTTL并禁止准入。这保留已修复的生命周期边界；同时重建的history条依旧按原输入占用容量，不冒充精确manager活体。以后再见仍guard拒绝，无重复事件/seed。
7. 容量拒绝后不insert guard，也不forget历史；online统计为原有标量与typed流。溢出不触发epoch/reset，冷却、ready阈值、增益不变。

## 有界性与兼容范围

manager精确活体≤history容量，退休即删；history自身≤256且21样本/点；guard固定1MiB。两者内容不要求一一对应：history允许有限的退休ID输入缓存，不能把其存在解释为可再seed。默认OFF保持原路径。

正常输入等价需要历史管理选中集合、准入顺序、seed、x/P逐帧检验。显式越界输入、Bloom保守碰撞和修复后的TTL同事件返回，本就有新拒绝语义，单独报告；不能以此免责普通数据1.75秒分歧。高于20Hz的21样本截断与protected缺历史等异常边界继续遵循冻结容量规则，不放宽为复制旧无界行为。

## 新小反例

`test_ltv_history_competition.cpp`使用相同15容量模拟真实256竞争：先15点三帧达到准入，然后三帧缺失退休。0.30秒提供另外15点。旧history仍被退休点占满，新点没有history；当前有界实现提前forget，错误提前缓存新点。测试要求两路径history key集合/样本数/时间/出生与retained相等，同时新版精确manager已删除退休点。

退休ID再次出现时，两路径都保留其有限history输入占用，但无任何再次birth。TTL自然到期后新ID可进入并经同样三帧确认；这同时验证不能靠永久占用history阻止后续工作。现有长间隔candidate TTL反例继续必须通过，不过其“无history”断言需改为“可有输入history、无精确manager、无birth且guard只插一次”，因为禁止复活不等于删除原始历史。

审查通过后再修改实现与既有断言，进行小测试，然后由主线程安排同一native输入的首次分歧及完整状态检查。本方案不增加机制轮次或调参。

## Implementation and short verification

Independent design review approved the repair. Build204/test205 reproduced the
old failure. After the minimal manager patch, build207/test208 passes the new
competition counterexample. Test210 exposed an older assertion that required
the very eager-release behavior being repaired; the revised fixture checks
continued old occupancy, no resurrection and normal admission after TTL.
Build211/test212 passes bounded manager, including same-event TTL return,
nonfinite guarded input and bounded600s candidate-only churn. Old manager213/214
and bounded history215/216 pass. These are small component checks; integrated
native build218 and full V2_03 counterfactual validation remain pending.
