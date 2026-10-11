# R3软失效迟滞候选独立设计审查

只读审阅`r3_ready_soft_grace_contract.md`，未实现、未运行、未接触确认数据。设计可进入登记和小反例验证；这不表示认可为达标方案。

授权边界：goal§3.2允许迟滞，§6.4允许开发数据校准ready策略；protocol每轮最多4候选，R3尚未最终冻结。可明确记为R3第2availability候选，不能另起第4机制搜索，也不能冒称修复日志或资源兼容bug。它真实改变了ready退出策略：held包允许超出原在线好值；科学误差、严重错误、持续性与覆盖验收线仍不变。旧ready实测结果不能自动复用于新策略。

状态合同合理：首次及撤回后20实际好帧不变；只有已ready分支可held；last_good仅真实good更新；计数≤2且物理elapsed≤.10双限制；第三/超幅/任一硬无效立即撤回；G/V独立；epoch/bootstrap清状态；原raw x/P及几何/增益不变；原始诊断值不被改写，held标记及错误完整评价。

实现前需固定两项歧义：

- “时间不连续”沿现有在线物理故障/confirmation_max_gap规则，不引入离线episode的0.075秒判定线。二者作用不同。
- EuRoC约1e9秒时间戳下，double减法在.10秒边界存在ULP影响。必须测试实际epoch的+.05/+.10/边界后一可表示值，明确固定比较表达式及已有容差；不能看到结果后调容差。若表示精度使边界更保守，应明确记录，而不以秒级实数表达式假称所有两个20Hz包都必然held。

必须保留的风险：good/soft交替可以长时间持续ready，单个连续soft段≤2并不限制全局held比例。必须报告全部held样本的误差、严重比例、持续模式及默认OFF对照，不据局部短测试声称安全。235/236瞬时上限3.8秒说明单纯去确认也不能解决5秒；新迟滞是否有效、是否增加错误ready仍未知。若失败不追加扫描倍率/事件数/时长。

## 已预登记的浮点时间合同

候选237在实验前明确：`deadline = nextafter(last_strict_good + 0.10, +inf)`，事件时间必须`<=deadline`，`nextafter(deadline,+inf)`拒绝，最多2个失败事件仍独立限制。接受此表示精度合同；它是相对于double加法结果的一向上ULP包络，并非无限精度实数严格0.10秒保证。任何后续结果不允许继续增加ULP。新增always-on独立反例覆盖零起点和EuRoC大epoch的含端点/下一浮点与第三事件拒绝；尚待稳定源码后执行。

## 策略源码独立验证

manager声明源码稳定后，独立build240/test241均exit0，`test_soft_grace_independent.cpp`使用always-on抛错检查（并显式-UNDEBUG），直接编译稳定LtvReadiness.cpp，无full运行。覆盖：零起点及EuRoC大epoch的20good首次进入、含单ULP端点、下一浮点第一包即拒、两包后第三包撤回、三个软包都在.10s内仍撤回、撤回后必须再20good、G/V交错（V超幅不能误撤G，Gheld不能复活V）、grace期间14observed/14mature/physicalfault/零η/NaN rate立即撤双方、OFF恢复旧即时拒绝。

源码审阅确认grace不更新last_good，实际strict good才更新时间并清soft计数；每支confirm/soft/last_good分开；硬前置未通过时不满足envelope。此次通过仅是状态机反例验证，不证明held输出的GT准确性、完整Adapter数值等价、日志schema或正式科学验收。

## 无效时间/pause路径及集成日志审查

只读调用链表明：pauseHardened在无效时间policy早返后仍reset core并标记新epoch；下一合法process以cold core输入policy，不能直接发布旧ready。独立纯policy模拟此真实cold输入序列的245/246通过duplicate及backward两条路径：拒绝包不发布ready/grace，后续新epoch不能继承旧latch，合法bootstrap之后仍需要20good。此为调用链静态复核+policy反例，未冒称新的完整Adapter实验。

243编译成功、244测试失败属于fixture：29.9秒已开始累计供给，30.05已发一次bootstrap请求，fixture却等30.10要求再发一次；pending合同本来禁止重复请求。仅改fixture为30.0起三帧，245/246通过；受测策略无修改。失败记录保留。

另外识别一个拒绝输出诊断问题：processHardened的duplicate/backward分支从frame复制旧health，只清外层raw/ready；上一帧held时该返回副本可能grace=true而外层ready=false。正常native严格整数receipt会拒绝重复，不能据此推断已有正常实测日志污染，但Adapter直接API/cache边界仍应自洽。建议只清返回f.health的ready_G/V/joint/grace/observer_valid/accepted_time并设置拒绝reason，不修改内部readiness/core/确认计数；交主线程实施，不是策略调整。

新native字段直接记录启用标志、两分支grace/count/last-good，未覆盖原诊断值；cache在启用时追加明确READINESS_SOFT_GRACE_V1后缀，OFF不追加，保持旧证据字节的设计合理。尚须集成native/cache实际测试，不把静态布局审查算执行通过。

## 拒绝返回副本补丁复核

主线程已将duplicate/backward返回副本的accepted_time、observer_valid、两ready、joint及两grace清false，并替换拒绝reason；位置在last_camera/last_version更新之前。只读复核确认没有写内部health_、readiness_、core或确认计数。历史last-good/计数仍是内部既往状态的快照，未伪装当前接受事件。

集成测试新增这些返回诊断的always-on断言，并继续检查完整x/P不变。其accepted_time断言在上一有效事件后能捕获旧实现，即使上一包尚未进入grace。此处仅复核补丁与测试判别力，未另编译；最新主构建/执行结果以主账本为准，不继承此前242为新补丁的执行证据。
