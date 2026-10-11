# R4 real runner 与 live checker 独立审查

只读检查 `real_run.py`、`check_live_log.py` 及其调用的budget/artifacts/旧hardened检查。新增专属fabricated反例 `tests/audit_checker_gaps.py`，统一账本test27 exit0，结果是**确认检查器缺口**，不是实际工程验收通过。没有真实回放或修改共享源码。

## 正式执行前必须补齐

1. **confirmation未绑定冻结内容。** real_run只校验调用者runtime本身完整、sensor metadata SHA；budget只要求frozen_config.json文件存在。没有读取并验证冻结runtime/manifest、base config、新active参数、协议/评价SHA、序列名单、mode与repeat身份。当前任意受支持angle甚至enabled=false仍能用confirmation标签启动。必须在创建输出目录和reservation之前做完整freeze identity对照，不能把存在一个JSON文件当冻结许可。没有冻结时当前最终会拒绝run，但已创建目录/配置/identity，仍不能称零副作用preflight。
2. **实际runner没有调用R4 checker。** 成功路径只调用旧passive.inspect与旧check_hardened_log。旧checker不要求active_consistency schema；因此新模块日志可以缺失而real_run工程成功。enabled运行应明确调用并保存新checker结果；disabled运行应明确要求无新后缀，保留旧路径/parity，不能强行用ON checker验OFF。
3. **新checker不能独立证明日志完整。** 空文件或所有帧management_present=false会返回PASS，无replay packet count/receipt/有管理事件要求；当前也不拒重复JSON key。必须与旧严格全receipt核验组合，明确至少有初始化/managed数据，不能仅看返回的PASS字符串。

## test27 已复现的具体漏洞

- 空文件被标 `PASS_LIVE_COUNTERS_AND_LIFECYCLE`。
- FAIL/SKIP点17仍在corrected_ids且sent=1，checker仍PASS；其limitations虽明确依赖native cache，但live已经有corrected_ids，应直接检查reject集合不交实际校正集合。
- consistency_reject_count=999未被核验，仍PASS。
- 第二个FAIL标RETIRE、从retained删除，但无typed retirement event，仍PASS。

这些反例没有推翻生产manager计数实现：源中可评价FAIL恰好同时增加fail/reject；SKIP和RETIRE分开计，retire帧也属于一次reject。反例说明评价器的证据范围不足，不应改变算法去迎合检查。

建议逐ID维护 `(epoch,id)` 的fail与累计reject：PASS只清fail，non-evaluable维持两计数，FAIL使两者各+1；RETIRE包括这次FAIL，必须达到固定阈值且本帧有同ID ActiveRetired/reason=active_consistency事件，之后同epoch不得出现该ID active更新。每个PASS/非可评价必须UPDATE并retained，每个SKIP必须retained且不在corrected_ids，每个RETIRE必须不retained且不corrected。阈值从本次固定active_config绑定，当前2不能默默作为对未来任意配置的通用检查。

## 当前正确部分与范围

checker将evaluable拆为pass+skip+retire，RETIRE没有漏算拒绝分母；non-evaluable要求pass=true/action UPDATE且fail不变，与初版中性语义一致。tracker不可见事件没有诊断时last计数保留，复见仍按原counter核对；epoch作为key隔离重置。短史/缺pose不应被伪造成FAIL，必须记录具体reason。

不过现在没有检查reason与evaluable/pass一致、PASS动作、reject_count以及残差范围/有限性（非JSON数值常量被拒不等于所有字段契约被验证）。绝不能把“fit failed”记录为evaluable历史坏或在缺history时隐藏记录。当前evaluable_fraction分母是diagnostics，仅表示当前可见有效且有诊断的active集合；不是所有retained/current前端点的可评价比例。应同时给active含coasting与当前visible分母、缺历史/缺pose计数，保留已有说明。

real_run有sensor元数据文件SHA验证、immutable runtime验证、明确复制配置并记录tree/source/manifest身份、不把GT路径传给observer，这些方向正确。但development也应事前验证所拷配置确为已授权baseline及主Passive约束，不等完整运行结束才由旧hardened checker发现不是hardening路径。新开关关闭时仍写入默认active参数没有数学影响（constructor OFF修复后），但必须把“配置文本不同、effective OFF行为相同”和严格基线配置身份区分。

所有判断仍沿用既有acceptance.json；本次没有更改门槛。当前局部事件或checker改进不能替代正式冻结和全EuRoC比较。

## 修订后独立test28

主线程已将confirmation冻结内容检查移到mkdir前；enabled路径实际调用新checker，disabled拒绝ON schema；checker新增empty/no-management/重复key拒绝及累计reject_count。独立 `tests/test_runner_review.py` 在临时目录mock环境运行12项小测试，test28 exit0：缺冻结、错误angle/disabled/sequence、manifest/config/source/acceptance变更都在runtime verify及创建output前拒绝；六R4键配置白名单通过，主配置/gain更改与旧方法已开启一致性拒绝；空日志/无管理/重复key拒绝。未启动native。

尚未在这次源版本补齐的是SKIP/RETIRE与corrected_ids排斥、RETIRE对应typed事件；test27的这两项漏洞应由新版fixture继续检查，而不是把27整体作为修复后仍能通过的测试。原27保留为旧源码反例，不重复运行要求旧漏洞继续存在。

新R4 real_evaluate仅替换六个R4 YAML键的等价白名单，其余主配置与LTV/gain/ready配置仍要求完全相同。继续调用原参考变换/误差/episodes/阈值，方向正确。正式证据还应增加wrapper本身SHA：base.evaluate内部的`__file__`指向旧评价器，当前默认evaluator_sha没有涵盖新增config_equivalence。冻结合同时还应绑定输入metadata manifest版本，不能仅依赖运行时任意metadata自洽。

两份事件报告只宣称同一after-lifecycle快照的真实core单帧干预、目标first failure且保slot，明确全P重推/子步可变、不做旧贡献直接相减；29636连续提前退休仍未证明。这些限制表述符合当前证据，不应改成完整启用后收益。
