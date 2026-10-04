# Readiness 与在线 receipt 独立初审

范围：LtvReadiness.{h,cpp}、test_ltv_readiness.cpp、新 LtvEventReceipt.h、runner logger diff。只读审查，无新执行；以下不是整个工程或科学合同通过声明。

## Readiness：待修复后再集成

**必须修复：bootstrap acknowledgement 没有请求绑定。** 当前 `bootstrap_committed` 验证 source/finite/current/cooldown，但没有要求 `awaiting_bootstrap_`。因此可首次没有供给确认而直接commit，也可Tracking超过30秒后无pending请求commit。正确接口必须将ack绑定到仍有效的pending请求及physical/quality类别；取消供给后的旧ack应拒绝。拒绝不得消费事件时间、计数或状态。独立反例：无请求首次commit、Tracking中commit、pending取消后commit、physical标志不一致。

**需要明确的集成时序：** `update()` 在请求时已消费输入time，同一time再次带bootstrap_committed会被duplicate拒绝。若采用下一事件ack，必须保存请求身份，observer原子初始化时刻与输入时刻一致且期间不能发布旧状态为当前raw；若要求同事件事务，应提供明确commit API而不是绕过重复保护。绝不能为方便集成两次调用导致重复累计confirmation。

**需要明确的连续性：** supply/ready确认按update次数累计，本模块不限制相邻time差。跨长缺包仍可能累计确认。需由policy或adapter显式标记物理时间缺口；测试缺口前19个好帧、缺口后1好帧不能未经恢复立即重新ready。非法非有限/倒序time当前早退、返回not-ready但保留内部状态；若接口有意不消费无效事件，需区分重复幂等拒绝和真正物理时间故障，后者进入明确失效路径。

其他静态结果：默认关闭无请求/ready；无GT、未来阶段标签、序列名或VIO误差输入；空观测首包即撤ready；gravity norm和数值健康检查保留；30秒质量冷却与物理故障恢复分开；没有ack时不把有限旧state认作ready。单元测试覆盖上述基础路径，但尚不能证明observer重建原子性或实际10秒恢复精度。持续有限但错误预测/模长不健康仅停留BOOTSTRAPPING，不主动DORMANT；应在实际机制验证中观察是否需健康超时，不能由静态代码宣称恢复保证。

## Logger：小修复静态通过，完整验收仍待真实运行

本轮最终receipt实现已经补齐：原始整数时间严格递增、重复/倒序拒绝不增加receipt，相邻原始ns即便映射到同double仍有不同事件ID；available event需物理double和epoch/sequence一致；输出先validate，写入检查成功后complete。runner没有改变IMU/相机传播端点。非representable整数ns的旧guard已去掉，避免再次静默遗漏合法管理字段。

header单测执行结果由所有者提供build6/test7 PASS，本审查未独立执行。还须实际native build及短live/cache一一对应、日志开关数值exact、raw不存在时null/invalid、schema和最终flush/close错误检查。永久累计TTL/tombstone来源仍是旧manager内存，logger没有增加独立历史，但不等于已满足10分钟有界内存合同。

## Revision 2 独立复核与可复现剩余问题

所有者修复了pending绑定、physical类别、实际向量finite验证，并新增不消耗第二个事件的 `acknowledgeBootstrap()`；时间缺口清空确认计数，避免稀疏帧累计。独立新测试 `scripts/ltv_passive_hardening/tests/test_readiness_independent.cpp` 验证同事件ack、无限η拒绝、重复ack拒绝、20个post-ack确认、空观测首包撤ready、恢复重新确认，以上先行检查通过。

但保留的 `update(bootstrap_committed=true)` 延迟提交入口仍在计算本包供给前commit：前包pending，本包几何/seed供给消失，可错误地接受bootstrap。独立 **build23 PASS / test24 FAIL** 已重现，错误信息为 `delayed commit accepted after current seed supply disappeared`。已通知所有者修复。建议最终adapter使用 update→原子事务→同事件ack路径；不能以调用者暂不使用为由把未保护公共入口认作完整验收通过。失败记录保留；没有完整仿真或真实运行。

## Revision 3 独立复测：policy 缺陷已修复

延迟 `update(bootstrap_committed=true)` 入口已无条件拒绝，唯一提交接口为同事件 `acknowledgeBootstrap()`。复用原独立反例、不改变断言，**build28 PASS / test29 PASS**。初审列出的pending、physical类别、实际向量finite、同事件时序及连续确认缺口在policy层已解决，可进入adapter集成验证。该通过不等于已证明共享状态原子重建或科学恢复门槛。

## 新 Hardened adapter 首次静态检查

slot位置读取 `3*slot` 与core布局一致；预测角度使用旧slot当前预测射线，校正率来自同一相机校正前后的v/η差除以物理dt，不混入IMU动力学加速度。cold集合预热不传播未启动observer；birth在首次校正前由受控接口写入；pipeline/core/IDs采用暂存后提交。

交给主代理的集成待办：

1. COLLECTING分支处理了当前pipeline并增加sequence，但frame.available默认false。旧logger若继续用available打开管理日志，会遗漏所有冷收集候选/opportunity/TTL。管理事件存在和raw状态有效必须用不同标志。
2. 暂存覆盖准入事务，但last_camera、IMU队列裁剪、实际预测/cursor和诊断计数/health不是全事件暂存。后续拒绝不得声称整个事件无变更；应明确“合法预测已完成、准入不提交”边界并独立验证。如合同要求整事件回滚则需扩大staging。异常不能造成半次seed或manager/core不一致。
3. `c.R_BC`只预检finite，core稍后才检查旋转合法性；seed context相机外参与c也需保证一致。错误标定/不同执行pose的反例要在有效状态发布前拒绝，不能让seed和bearing校正使用不同相机几何。

本次adapter审查只读，未执行adapter或真实数据；主代理继续拥有共享adapter整合。

## Integrated adapter 拒绝事务独立测试

独立新增 `scripts/ltv_passive_hardening/tests/test_adapter_rejection_independent.cpp`，复用已有Harness构造但使用独立拒绝断言。**build49 / test50 PASS**。非法context版本/时刻、非正交R_BC、context杆臂与core calibration不一致、NaN offset均被预测前拒绝，full x/P/IDs/history保留。对预检查之后才由pipeline检测出的非法joint covariance，允许合法IMU预测先提交：实际完整x/P与独立纯IMU预测副本逐值一致，camera校正计数、slots、ID映射、manager history及admission/seed统计都未消费。没有通过异常后继续运行掩盖拒绝；这个测试只认可主代理明确的“prediction先提交，camera准入原子”边界。

最新 `check_hardened_log.py` 已强制每行hardening schema、整数receipt/双目时间一致、raw/null/finite/游标时间、ready时pool/observer/重力物理健康和全部management_present行（包括cold）计数。因旧skip导致的短测漏验已有独立validation_correction记录，旧通过不能替代新版严格检查。工程检查仍不证明误差精度与恢复科学目标，也不等于未执行的11序列验收。

## Synthetic Adapter输入包装静态边界

新runner不读取GT/labels；旧run模块仅提供prior joint covariance。wrapper按actual_ns逐观测同步筛选并记录异步拒绝，端点IMU交给同一Adapter；200Hz保存实际快照/游标，不额外传播core。因此离相机事件的held snapshot不能被评价为新的current输出。已建议所有者明确合成时间零起点或去除llround反向等值校验、验证硬编码10 tick相机节拍与输入数组一致、绑定共享wrapper实际依赖的冻结数值库身份。本审查未新增运行合成。

## R2 bounded initial warmup 初步只读审查

新policy仅在尚未bootstrap、无physical recovery且已有当前有限core时允许显式初始warmup请求，ack source必须为 `BOUNDED_INITIAL_ZERO_WARMUP`。ack当时缺足够几何则从当前时刻开始shortage计时，不能无限裸积分；同事件提交仍不额外累计confirmation，后续进入既定30秒quality cooldown。adapter仅首次尝试创建零状态，不复制GT/VIO均值，source有区分。这是授权候选机制的正确性边界初审，不是科学结果或回归成功声明；仍须新单测、资源与完整开发证据。

## R2 初始warmup独立反例：build97/test98 PASS

新增 `tests/test_initial_warmup_independent.cpp`：错误source不得确认、拒绝不破坏pending；下一事件ack拒绝而同事件ack成功；重复ack拒绝；空池warmup全程not-ready，1.0秒发出DORMANT请求；1.05–29.95秒即使有限current core和足够seed仍不能绕过质量冷却，30秒允许正常恢复请求。首次实际物理故障后，无供给的initial warmup不得重新启用，必须经3包供给确认并明确physical恢复。上述独立构建97及测试98通过，无full运行，槽已释放。

对于集成测试90失败后94的断言修订，合同区分“触发停止的本帧已计算raw”与“下一帧不再积分”。保留触发帧记录不等于继续声明ready；不应回溯抹掉已计算状态。policy独立测试确认1.0秒DORMANT/request；停止后不再推进的完整adapter事实由集成94提供，本次不把policy测试冒充adapter执行。
