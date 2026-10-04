# C02 冻结前独立审查

仅只读源码、31/38候选注册、四份开发metrics和既有独立测试/事件证据，没有启动真实进程、新增候选或改门槛。审查时43完整enabled缓存仍在运行，不能提前记PASS。

**C02可作为当前待冻结选择，但尚不具备无条件进入正式全矩阵的证据/执行计划：必须补冻结依赖覆盖，解决同配置P_PREV缺口与剩余预算的冲突。两序列开发成功不是PASSIVE_READY_EUROC。**

## 选择规则与源码范围

C01在31事前注册0.04，V2_03仅joint4.55s不足，正常V2_02通过。C02在38事前注册，只改允许的history/holdout角上限为0.02；min5/span0.20/window1/retire2/SeedEstimator及原ready配置不变。39/40和41/42两序列全部单序列决策通过，joint分别6.30/12.80s。配置等价证明只允许六个R4键；没有将R3B grace或R3C时间窗混入选择。未执行0.08、未出现支持备用backend的正常点拟合不稳定证据，所以停止候选搜索合理。

静态与独立18已经验证当前ray/future不进past fit、单一current pose上下文、rank失败中性与valid高history residual可评价FAIL。pipeline在history更新前评估；history仍保留原raw输入；manager只过滤送core的observation，保slot/交叉P规则及两失败退役；没有改LTV方程、gain或主VIO数学。git相对258b0b6的已跟踪core/math变更仅在授权adapter/pipeline/manager/options/diagnostics/cache，不含ltv_observer及主传播/更新；新增untracked模块也被source_identity的rglob扫描，不能遗漏出冻结。

两目标局部真实core对照证明same-snapshot skip有益，但单帧人为first failure不是完整因果history。正式结论需从C02连续日志报告27537/29636是否何时准入/skip/retire，若更早退出或根本没准入如实记录，不能复制旧锚定结果冒充该轨迹事实。

## 冻结依赖阻断

当前R4 `budget.source_identity()`调用旧feature_passive scanner再加R4目录；它不扫描 `scripts/ltv_passive_hardening/`。实际执行却动态导入该目录的 `check_hardened_log.py` 与 `real_evaluate.py`，后者还读取旧hardening `protocol.json`（含all11等定义）。若冻结只保存当前自动source map，这些文件改变不触发real_run的source循环，冻结不完整。

冻结需显式绑定这些实际依赖和旧protocol，连同R4 wrapper/physical evaluator/原acceptance、native/runtime manifest、effective config/base tree、输入metadata、全部目标序列与输出身份。R4 evaluator目前复用base.evaluate的`__file__`，若provenance仍仅记旧evaluator_sha，应增加wrapper SHA及其配置白名单身份。冻结后新analysis如要新增工具，需明确在冻结外，不让它改变已冻结决策。

real_run当前已在mkdir前检查freeze内容且不允许confirmation enabled=false；因此它本身不能生成正式OFF/P_PREV对照。若计划新增同配置OFF，需明示合法对照入口及身份，不使用隐藏phase开发标签绕开final reserve或把新OFF结果改标旧历史。

## all11对照和预算的硬账

目前确认同R3B配置P_PREV仅V2_03历史完整原输出及新V2_02 OFF33。其他历史f76/旧P_NEW即使输入名相同，也可能有不同hardening/readiness/状态生命周期。不能把旧raw标为R3B同身份；相同C0和主轨迹相同不足以证明LTV输出相同。纯配置差异“仅ready”也需逐条证明未走状态重建/暂停/池差异，不能根据一条序列的full cache等价泛化。

审查时已使用8 real（包括完整cache），剩16：

- 若允许把两个C02开发运行按完整同身份复用为“已执行all11中的两条”（明确已见开发，非新确认），仍需9条ON+9条缺失的精确OFF=18次，尚未计两代表性full repeats。
- 再做两次完整repeat至少20次；若所有11条ON均冻结后重跑，则22次。
- 任何新增B回放会进一步增加数量。因此“剩16直接跑11ON再补几项”不能完成当前对照合同。

预算内可行性必须在freeze前解决：

1. 逐序列建立可审核的既有B/P_PREV复用清单：真实路径、config/input/runtime SHA、完整消费、主state/P/FEJ/trajectory证据及LTV等价依据。仅真实证明充分者减少新回放数，不通过放宽config_equivalence强行纳入旧输出。
2. 可考虑一遍原始输入透明运行ON/OFF两个只读observer的driver方案，完整记录两个observer/工作量，先做OFF和主旁路等价及日志/cache校验。先确认其计费符合本轮“输入遍历/完整replay”规则并写协议，不能在运行后为省预算重解释。两个observer的数值必须独立递推，不能拿ON轨迹重贴OFF标签。
3. 若上述证据/入口不可得，明确预算或对照覆盖阻断；不能缩到两条成功序列称EuRoC级完成。剩两候选额度无助于补身份或计算缺口，也不应继续搜索。

同输入双observer可能改变测得CPU开销，性能验收仍须报告选中ON单独分支成本，不能用两者总耗时或关闭关键诊断后的不同算法混淆。若需重新构建driver， observer源码/库保持相同才可复用开发C02科学身份，新的runner完整性和主旁路需要另证。

## 尚未闭合的完整合同

- all11每条实际NEW、至少8个B有效；B失效原因不能用NEW表现排除序列。
- 全initialized raw支持与同支持精度/回归、G/V覆盖、5s连续joint、严重率、V1_01原参考例外，零注入、主P/FEJ/clone/visual/trajectory不变。
- 选中版本完整cache43、两代表性完整确定性复测、logging开关数值一致、OFF数值/字节；已有30是OFF core/adapter缓存证据，不自动证明每条native输出。
- 新active拟合引入每帧计算，需要实际camera成本与bounded运行证据；不能直接继承R3未启此模块的CPU性能结论。
- 继承acceptance中的恢复/故障撤回、bad seed/几何负对照和seed来源可靠性义务：可严格同身份复用未改变部分，但active skip/retire改变未来集合及恢复，不能一概视为未受影响。12 synthetic预算的具名计划必须覆盖真正所需项。
- evaluable/skip/retire与false-reject proxy全分母、typed退休/guard、无额外seed写入，以及两个目标连续运行真实生命周期。

当前日志checker已新增reject不在corrected_ids与typed退休检查，关闭了旧27对应静态缺口；最终仍须以实际live通过及新源SHA为证。已有开发表现值得保留，但本审查不批准降低任何验收项来适应剩余额度。
