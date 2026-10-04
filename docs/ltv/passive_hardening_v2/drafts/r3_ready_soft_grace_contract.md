# R3轮内availability候选：已ready分支的有界软失效迟滞（待注册）

本稿只定义候选与反例，不修改实现，不执行实验，不宣称通过。依据是audit235：可用输入最长15.6秒、瞬时健康最长3.8秒、joint仅2.85秒，38段1–2包软健康波动触发完整重新确认。该证据来自开发数据，不能算新确认。

## 授权及轮次判断

`goal.md` §3.2明确允许readiness的“迟滞、冷却、恢复后确认”，要求不能仅基于主VIO分歧或GT；§10评价成功门槛不能移动。本候选不改变core、x/P、增益、几何准入、最低15点、初始化/恢复机制或评价器；只细化R3已有availability策略的退出迟滞。因此可作为R3轮内第二个有明确证据的候选，单独登记并计候选预算，不另起第4机制开发轮。

必须坦诚：保留ready时允许预测/修正指标暂时超过原好值，这是新的**ready退出策略**，不是“ready行为完全不变”或纯实现修复。2倍仅是短时退出包络，不能当成新的进入好值门槛；最终误差/严重ready/覆盖/持续性验收线全不变。它可能连续拼接出错误ready，因此只有后续独立证据能证明可用性，不预先保证通过。若失败，不再扫描倍率、帧数或时长。

## 固定配置与每分支状态

默认 `ready_soft_grace_enabled=false`，OFF精确原行为。候选固定：最多2个连续失败相机事件、距最后**实际good**物理时刻≤0.10秒、各相关软指标≤原限2倍。冻结浮点边界为 `deadline=nextafter(last_strict_good + 0.10, +infinity)`，`time<=deadline`允许，`nextafter(deadline,+infinity)`拒绝。这是预声明一个向上ULP的表示包络，不宣称严格实数≤.10秒，也不引入事后容差。原online时间gap仍为confirmation_max_gap=.20与原fault语义，不采用离线episode的.075秒界。

G/V各自维护：`latched_ready`、`good_confirmation_count`、`last_good_time`、`soft_failure_count`。全量reset/bootstrap/物理fault清两分支状态。grace不是新的good，不更新last_good_time，也不累加确认次数。

| 类型 | 每包固定条件 |
|---|---|
| 硬前置 | 当前合法时间/IMU，无物理fault，observer current/finite且通过R3 norm guard，geometry_valid，当前与成熟点均≥15，pool_ready，actual_corrections≥20，correction_diagnostics_valid，各诊断有限非负，η模长[8,11.5] |
| G实际good | 硬前置＋prediction angle≤.02rad＋η correction rate≤1 |
| V实际good | 硬前置＋prediction angle≤.02rad＋v correction rate≤.5＋η correction rate≤1 |
| G软包络 | 硬前置＋prediction angle≤.04rad＋η correction rate≤2 |
| V软包络 | 硬前置＋prediction angle≤.04rad＋v correction rate≤1＋η correction rate≤2 |

G不因**有限非负**的v correction rate超限而退出；V同时检查两rate。由于原诊断合同包含两rate的有限非负性，NaN/负值即硬无效，不能藉G独立性掩盖非法诊断。`prediction_angle_p95`仍是同一个原始当前事件预测残差，不重算/平滑。η norm guard和G物理模长检查仍区分：100/50原始状态界不替代[8,11.5] ready物理范围。

## 确切转移顺序

1. 包无效、时间故障/不连续、物理fault、非current/nonfinite、norm越界、少点/不成熟、geometry失效、诊断无效或G物理模长不合格：相应分支立即ready=false、grace清零、确认计数归零。原停机/恢复语义不改。任何一项硬前置都不能用2包grace绕过。
2. 分支未ready：只按原实际good累计连续20包；非good立即清零。第20包实际good才进入ready并设last_good_time。启动阶段没有grace。
3. 分支已ready且实际good：保持ready，last_good_time=current_time，soft_failure_count=0。
4. 分支已ready但不是实际good：仅当该分支软包络成立、将要递增后的soft_failure_count≤2且current_time-last_good_time≤.10秒时保持ready，并输出显式grace状态；计数递增，last_good_time不变。
5. 否则立即退出ready并清确认计数；随后即便下包good也必须重新累计20包。第三个连续失败始终退出，即使三个包总时间不足.10秒。
6. G/V分别执行上述状态机，joint_ready始终是当前两个输出的AND。不能因为一支恢复而继承另一支确认或延长其grace。

## 时间、日志与验收

使用物理IMU时间计算last-good age，按实际已接受camera事件计包，绝不拿同包update→ack算两个事件。duplicate/乱序不能延长grace或发布当前ready；非法时间接口仍执行原拒绝语义，adapter发布ready=false。

建议新增每分支输出：`ready_G/V_soft_grace`、`ready_G/V_soft_failure_count`、`ready_G/V_last_good_time`，以及显式reason（两支都held时combined）；快照必须保留实际原始prediction/rates，不能改写成限内值。记录held样本的GT误差与严重ready只是离线评价，不回流在线决策。全量时间轴保留，grace样本不得从误差/覆盖统计删除。

## 独立反例合同

- 默认OFF与原状态/ready事件严格一致；开启后首次19good不ready，第20good进入。
- 首次进入前插入单包轻微超限，确认归零，不能借grace提前ready。
- 已ready后在+0.05、+0.10秒两包软失败保留；第三包无论在+.11或+.09都撤回。下一good不能立即恢复，第20个连续good才恢复。
- 第一个软失败距lastgood>.10秒即撤回；重复事件不能刷新lastgood或消耗新确认。
- 任一软指标恰等2倍允许包络，超过2倍任意可表示正量立即撤回；原好值边界与2倍退出包络分别测试。
- good→soft→good必须清软计数；一串交替good/soft可能长期维持ready，必须保留全部held标记与误差评价，不以短单段测试保证安全。
- 14点、14成熟点、diagnosticsfalse、零/NaNη、η模长越[8,11.5]、负rate、非finite residual、staleobserver、physicalfault：即便是第一包也立刻撤回。
- Ggood且仅v rate处于(.5,1]：G正常ready不进入grace，V进入grace；v>1仅V立即撤回（若其余诊断有限）；η超good但≤2同时影响G/V，两个计数不串扰。
- bootstrap/epoch完整替换清latch/lastgood/grace；warmup、zeroobs停机、30秒质量冷却仍通过原独立测试。

注册及独立审查完成前不实现。本方案不能补偿输入始终少点或长期健康不满足；若无法满足有效EuRoC的5秒joint合同，则保留失败，不将诊断包络解释成新的误差门槛。

边界测试必须覆盖last-good=0量级和1.4e9量级：+.05、+.10、deadline以及deadline的下一可表示数，并独立限制至多两个失败事件。
