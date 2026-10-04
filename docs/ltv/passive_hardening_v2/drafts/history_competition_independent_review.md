# 历史容量语义修复独立设计审查

审查对象：`bounded_history_competition_repair.md`与`test_ltv_history_competition.cpp`。仅只读审查，未修改或运行受测实现；本结论不是测试通过。

方案可实施。保留全部已验证输入在有界history中的原TTL/容量竞争，同时立即删除退休manager精确记录，能保持容量上限并避免过早释放history改变普通候选选择。身份guard禁止再准入，不应被解释为禁止原始输入占用历史缓存。默认OFF保持原分支。

15容量反例覆盖：相同输入先准入/退休、0.30秒新ID不得提前缓存、0.35秒已退休ID输入可刷新history但不能再seed、自然TTL后新ID同序正常出生。逐帧history key/first/last/sample count与出生顺序、retained对照充分针对本次已发现缺陷；真实序列与完整x/P仍须后续验证。

实现必须满足以下条件：

- 更新前只读保存expired_candidates，更新成功后清理条件显式包含该集合，不能只检查history.find，因为同事件返回可能已重建history。
- 到期候选一次typed TTL、一次guard插入并删除精确manager；当前及后续准入视图排除该ID，即使历史继续刷新。
- 非法包或history.update失败之前，不提交time、guard、typed counters或manager变动。guarded ID含非法bearing也必须先被完整原包验证拒绝。
- 已经admit的ACTIVE/COASTING不得因自己的guard位被过滤；退休后任何输入不得再产生birth。
- 增强现有长gapTTL反例：同事件历史重建及后续再次观测均有history、无manager/无birth；TTL事件与插入计数仍只一次。不能把旧“无history”断言机械删除而不验证身份合同。

本次审查未要求放宽256/21/512等上限，未调整ready/gain/噪声或科学阈值。最终必须区分边界异常的显式拒绝与普通输入兼容性，不能用有界性为普通场景非预期选择差异免责。

## 最终补丁只读复核与数值比较工具

最终manager补丁已经逐项落实：history接收完整原包；expired_candidates显式参与提交后清理；删除提前forget；guard只限制准入；失败history更新不提交guard计数。独立只读复核通过。manager报告修前205预期失败、修后208竞争通过、最终212/214/216通过；209/210保留旧错误容量释放断言的失败记录，不掩盖或重标。

新增只读`tests/compare_cache_numerical.cpp`。build220/test221 exit0；短cache200相机事件自比通过；复制出的单事件fixture修改一个x double后被`full x/P`检查拒绝。219是添加fixture前的编译，220重编最终源码。工具不执行Adapter、观测器或GT，仅解码原cache的实际expected evidence。

运行：`/home/he/output/ltv_passive_hardening_v2/compare_cache_numerical LEFT/cache.bin RIGHT/cache.bin`，应经统一budget analysis。逐记录输入bytes/事件类型、camera/imu/cursor、完整x/P字节、active外部ID/coreID/slot、retained/retired/correction observations/births均精确比较。非活动历史mapping及health/log metadata允许不同并报告计数；它们不参与数值通过。cache保存double物理时间，**原整数camera_ns及主state输入一致性必须另外由两run audit.csv精确比较证明**，不能用double往返伪装整数时间证明。完整新V2_03结果尚未运行/比较，本次仅证明工具正负fixture。
