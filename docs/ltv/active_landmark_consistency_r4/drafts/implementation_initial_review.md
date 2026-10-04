# R4 第一版共享集成静态审查

本次只读审查新LtvActiveConsistency、pipeline/manager/Adapter/options修改及纯模块测试；没有编译、执行或改受测源码。独立fixture已写 `scripts/ltv_active_consistency_r4/tests/test_active_consistency_independent.cpp`，尚未运行。

## 必须修复：OFF新增旧配置限制

Adapter无条件将已有 `feature_manager.history.history_window` 复制到active配置；Pipeline无条件构造纯模块；纯模块constructor无论enabled都校验window>=minspan默认0.20。旧合法history_window=0.10因此在新开关关闭时也抛异常。不能修改旧history允许范围解决；应在disabled时不执行新增算法参数校验，或避免构造开启语义的检查。新增fixture将此设为首个always-on断言前构造。

这不意味着当前默认1s配置必然数值不同，而是当前代码尚不满足一般default-OFF兼容合同。实际全x/P/slots/ready/cache/trajectory parity仍须运行。

## 当前静态符合的部分

- pipeline评估发生在manager.step/history.update之前，遍历旧retained且当前cam0 match-valid。新birth不受同帧active计数。没有调用原会加入current的seed构造lambda。
- 纯模块history严格 `[t-window,t)`，仅cam0、同ID有效条目；当前pose用于输出坐标，当前ray不加入SeedInput。过去clone来自同一当前context，排除execution pose及非past匹配；重复/模糊匹配拒绝。caller原epoch验证在评估之前。
- `SeedEstimate.valid=false`包括负ray走FitFailed/non-evaluable/pass；valid高history max走HistoryInconsistent，不先调用seed quality门限。HistoryInconsistent优先返回，holdout保持NaN；logger应输出null而非0。
- manager保留原输入给history；skip不伪装match_valid=false。tracker实际可见仍Active、missed_frames清零，consistency单独计数。PASS清fail_count，non-evaluable保留；第二次失败走原typed退役、retained删除与guard防复活。
- consistency_skipped仅过滤最终out.observations；mature_visible亦只计未skip的实际允许观测。未见通过多算成熟数维持ready的路径。
- Hardened Adapter现有评估/提交副本承载manager新增字段，core失败仍按原camera事务回滚；没有直接v/η/P补丁。冷处理复制副本的最终路径需用测试证明只计一次。
- 新enabled约束在Adapter禁止G/V注入；未改core动力学/Riccati。旧Options配置若未提供新键维持默认关闭。

## 还需测试/日志完成

独立fixture覆盖旧短window OFF构造、负rayFitFailed与高残差HistoryInconsistent分流、当前/未来ray无泄漏；暂未执行，不能报告PASS。主测试还需真正manager/core skip-retire事务、失败回滚、reset/duplicate、min-span及大epoch边界、完整default-OFF和logging parity。

pipeline若retained的history意外缺失，目前不产生该ID的consistency记录，等价保持旧行为。正常protected history应保证存在，但live验收必须证明此不变量；若需容错，显式输出MissingPoseSupport/InsufficientHistory，不能无声从可评价分母消失。帧级active_count是处理前retained数量，包含不在当前相机的coasting；必须另报当前可见/有诊断数量，不能宣称active=evaluable+pass+skip（pass与evaluable重叠）。正确可评价守恒是evaluable=pass+skip+retire；其余单列不可评价/不可见。

管理API目前信任pipeline结果语义，preflight检查ID/retained/必要finite，但没有完整reason与所有数值范围对应检查。此为内部接口可接受的封装前提，不是任意外部consistency map都受完整验证；不要让未来缓存输入直接伪造可评价Pass。事务测试应覆盖错误ID、不可评价却pass=false、NaN可评价结果等已声明拒绝路径。

logger/cache尚由主线程整合，不纳入本次完成声明。需要enabled条件后缀、每包原整数receipt/time/epoch/version、history/holdout缺失明确、每点counter/action、typed retire、actualcorrected IDs和所有raw denominators。OFF新增诊断字段不得改变原缓存字节；旧decoder不可静默把enabled后缀当旧格式完整验证。

## 评价合同

本轮明确继承 `docs/ltv/passive_hardening_v2/acceptance.json` 数值规则，机器合同的严重率≤1%优先于goal摘要中的“<1%”缩写，报告精确比例。不得趁实现更改ready阈值、科学连续性或分母。两事件局部通过之后仍须真实完整序列和全局合同，原三轮失败不重写。

## 修复后独立验证与logger/cache补审

manager已把constructor的新校验限制为enabled，纯模块原test10通过。独立build13成功；test14失败在本审查新写的高残差fixture。诊断build15/test16证实该fixture的强交替水平扰动使原SeedEstimator得到 `NONPOSITIVE_RAY`，模块按合同正确返回FitFailed/non-evaluable，失败不是算法分流缺陷。保留14/16，不改受测逻辑；仅把正ray高残差反例改成明确的交替横向扰动，负ray仍有独立专测。最终build17/test18 exit0：旧0.10s history关闭新开关的Pipeline构造成功，负ray中性、正ray高残差HistoryInconsistent（solver=OK、residual≈0.20748）、current/future不参与past fit均通过。OFF构造阻断已关闭，但完整数值OFF尚待native证明。

新 `writeActiveConsistency` 仅enabled输出，非有限残差写null；在bound管理writer被live runner实际调用，非hardening可用帧亦有显式路径。per-ID记录保留history_count/span、missing、reason/fit_reason、fail/reject计数与action；外层已有integer receipt、epoch和物理时间，未创建第二套日志历史。

cache evidence在原后缀之后仅enabled追加 `ACTIVE_CONSISTENCY_V1`，保存帧计数、逐点result/point_W/condition/reason及fail/reject/action；feature_id显式UINT64_MAX。旧OFF执行不到该写入分支，静态未见额外字节，但仍需实际旧新缓存对比。exact replay通过重新生成完整evidence字节比较，能检查新后缀；它不是只读旧decoder忽略尾部的伪验收。cache不单独嵌入完整新配置，运行必须用外部冻结config identity绑定，不得脱离provenance称自包含checkpoint。

startup/pause没有当前management时新per-point后缀可缺，但必须外层明确OBSERVER_UNAVAILABLE；已有managed cold帧不得缺schema。下一步是live真实输出的严格schema/守恒和同runtime cache重放，而非从源码审查直接宣布日志或OFF数值验收完成。
