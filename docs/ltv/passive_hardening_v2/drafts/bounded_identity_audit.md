# 有界身份账本设计独立审查（实现前）

只读设计检查，无构建/运行；R2真实与合成当前占两槽。本文件不授权改变旧模式或清除历史证据。

## Core高水位成立所需前提

高水位仅用于adapter自分配、同epoch单调递增的内部ID。应由独立显式新模式启用，默认OFF及P_PREV继续使用原已准入集合。不能直接把前端外部ID当作已验证单调序列。

批次准入先按事务开始时的旧H检查全部birth：ID非负、未占用、无重复、全部大于H；全部验证和更新成功才提交新H=max(batch)。birth枚举顺序不应改变是否接受。小于等于H的非活动ID均不得再准入，包括曾跳过的号码；这一保守语义须记录。当前仍活动且ID≤H的状态保留合法，不能误判旧点全应删除。INT_MAX边界拒绝且不回绕；只有明确reset/new epoch允许重建编号。

如果失败批次推进了H，重试可能错误拒绝未曾实际准入点；如果逐个birth边遍历边比较新H，逆序batch可能出现半提交。core和adapter的临时next-ID/highwater要与full x/P/slots及manager的camera事务一起提交。

## Adapter映射清理边界

只清理本次retained之外的内部映射，临时coasting但retained的点必须保留。已有slot对应的映射必须稳定；新birth分配新内部ID，next-ID不可被清理映射大小重置。清理不能发生在core成功之前并留下回滚不一致。

核心风险：外部旧ID退役后mapping被清掉；若manager允许它重新准入，adapter会给它新的更大内部ID，core高水位并不能识别它曾写过seed。因此manager必须独立保证同epoch外部ID不复活。有界core/mapping不自动证明history/候选/tombstone/manager全局有界，需单独容器容量、10分钟RSS及旧ID重现证据。

## 必需反例

1. 旧H=7，同批[9,8]合法且事务后H=9；与[8,9]准入集合一致，数值slot按显式保留顺序。
2. 后续birth=8/9重复、退役后birth=8、从未准入但≤H的gap ID均拒绝；活跃旧ID继续观测和coast不受影响。
3. 批内重复ID、NaN seed、错误epoch/time、非法retained集合拒绝，不消费H/next-ID/seed，full x/P/slots/camera计数保持拒绝合同。
4. 大量birth/retire后mapping大小≤active上限且core记录为常数空间；coasting/reappearance保留旧均值和cross P。
5. 外部旧ID退役并mapping已删除后重现不能再seed；正常从未见过的新外部ID不误关联旧slot。
6. INT_MAX前后一致拒绝，不发生负ID或循环重用。
7. reset/new epoch允许合法重建；旧epoch事件拒绝且不清当前高水位。
8. 新模式关闭，原controlled/default路径完整x/P/slot/事件与既有证据逐值一致；新模式相同合法单调ID序列也应同原集合实现数值一致。

## R2 V1_01待执行离线命令

待主线程确认完整运行完成并释放槽后，通过新budget的analysis类别执行：

```bash
python3 scripts/ltv_passive_hardening/real_evaluate.py \
  --sequence V1_01_easy \
  --new /home/he/output/ltv_passive_hardening_v2/real_development/R2_V1_01_01/P_NEW \
  --out /home/he/output/ltv_passive_hardening_v2/real_development/R2_V1_01_01/evaluation_v1
```

评价器已有initial_warmup白名单，仍需完整配置树/其余参数以及解析输入等价和主状态旁路校验。当前只准备命令，未提前读取该运行误差或启动分析。

初版core实现只读核对：`enableControlledFeatures(epoch, monotonic_local_ids=false)` 显式选择；was_admitted按旧H验证整批；仅result.accepted后统一提交batch max；新模式不写admitted_ids_，reset清H和模式；默认false原set判断路径保留。尚未编译或执行新有界反例。签名增加参数会改变C++二进制符号；旧冻结二进制必须继续绑定旧冻结库，不能因默认参数相同就把它当ABI兼容。

## Bounded manager/history集成静态结果与待执行测试

新guard为固定1MiB非删除位图，7次确定性hash；从不删除避免已插入ID的假阴性，正查询仅代表potentially seen，可能误拒从未出现的ID。不能把guard正查询当真实重复证明或已校准概率。active ID在admit时插入，仍在exact表的活跃点绕过guard；active退役与候选TTL都输出一次typed事件后删除exact记录/history，容量拒绝不插guard，允许合法首次再试。staging复制vector内guard实现独立位图，不共享可变位。

发现并通知所有者的OFF边界：新增max_candidates>=max_active验证需仅限bounded_memory，否则旧OFF曾可接受的配置被新constructor拒绝。

准备了 `test_bounded_identity_independent.cpp` 和 `test_cache_large_ids_independent.cpp`，未编译/运行。前者覆盖逆序batch按旧H预验、失败batch full x/P和H不变、原模式数值对照、退役/跳过低ID不复活、typed事件、guarded-ID坏packet拒绝与外部ID防重复；后者用最高位非零uint64 ID实际生成candidate TTL、active retirement和identity-guard拒绝并精确重放，针对新cache默认count上限截断风险。需等待主构建稳定/槽授权后执行，不能把准备的用例算通过。

## 独立core/manager结果

所有者已将新增容量约束限定bounded模式。新增独立测试 **build126/test127 PASS**；125仅因构建命令遗漏 `ov_core/src` 头文件路径失败，保留日志，未改断言。测试直接编译core/manager/history/quality稳定源码，未混用新header和旧动态库。

通过项：旧OFF小history配置仍构造成功；bounded模式拒绝容量小于active；逆序birth批次按旧H原子接受；第二birth NaN拒绝不消费H/full x/P，合法同事件重试成功；退役及跳过低ID无法复活；合法输入新旧模式x/P精确一致；typed active/TTL元数据正确；guarded旧ID含NaN必须整包拒绝且不消耗time/位；exact记录删除后外部旧ID仍不可重新seed；staged副本独立，新无关ID可准入。未把这些局部通过宣称完整observer600秒内存或吞吐通过。

高位uint64 cache用例尚未执行，等待统一R3构建完成后运行。

## R3高位uint64缓存独立结果

**build143/test145 PASS**，使用不可变runtime `a62bdfabaf666de8b7b06a00ec65d394fc9fc9d3d53fe91813be534fd3764a70` 的库，并将运行时动态库搜索路径绑定该snapshot。142仅因编译命令遗漏OpenCV头文件路径失败，日志保留；未放宽断言。

66个实际Adapter相机事件，ID均设置uint64最高位。明确断言覆盖candidate TTL、active retirement与identity-guard拒绝三类记录，并对整个生成cache作66事件精确重放；无ID截断/count上限错误。缓存 `/home/he/output/ltv_passive_hardening_v2/independent_high_uint64_R3_a62b.cache`，测试日志 `attempts/0145/stdout.log`。本项是短小工程fixture，不是完整合成或真实序列，不证明长期资源与科学阈值。
