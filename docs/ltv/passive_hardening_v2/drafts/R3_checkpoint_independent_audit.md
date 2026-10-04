# R3开发检查点独立审计

本记录仅支持开发检查点提交，不是最终验收。未修改受测实现或阈值，未运行完整仿真、真实重放、ATE。检查对象为当前R3源码及runtime `cd3bc7f4896d85fd629845e50a87ee56e5768d15672a8cfcbca6cc7760971123`；其观测器库与a62快照相同，新native logger增加真实corrected IDs/substeps字段。

## 本次独立执行

统一账本 `/home/he/output/ltv_passive_hardening_v2/attempts.jsonl` 保存完整命令、源码身份及退出码，各attempt目录保存stdout/stderr。

| 尝试 | 命令/范围 | 退出码与结论 |
|---|---|---|
|172|`cmake --build .../build --target test_ltv_pose_convention test_ltv_gv_jacobian test_ltv_gv_covariance test_ltv_fej test_ltv_joint_layout test_ltv_joint_covariance -j1`|0；六目标编译|
|176|通过budget test串行运行上述六执行文件，`LD_LIBRARY_PATH=.../runtime/cd3.../lib`|0；六项全部通过|
|177|`g++ -std=c++14 -O2 -UNDEBUG -I/usr/include/eigen3 -Iov_msckf/src/ltv scripts/ltv_feature_passive/geometry/test_geometry.cpp`，链接cd3快照库|0；独立composite geometry编译|
|178|通过budget test运行 `.../independent_geometry_cd3`，动态库绑定cd3|0；Jacobian/gauge通过|

172生成的测试flags中`-UNDEBUG`位于`-DNDEBUG`之后。生产库编译选项未改。176涵盖真实PoseJPL扰动/FEJ不变、完整相关P映射、500个姿态/速度样本的GV有限差分、Joseph/full covariance、FEJ传播与clone/visual gauge、联合布局和同线性化批量更新。关键数值：pose Jacobian最大误差2.50073e-9，full cross-P相对差3.31591e-11；GV有限差分最大绝对误差6.66134e-9；联合Joseph相对差2.12637e-14。178的composite seed Jacobian最大差4.90366e-7（原门槛2e-5）、公共world gauge残差2.80621e-13（原门槛1e-9）。这些是有限fixture数学回归，不是全场景稳定性证明；GV更新测试也不授权开启实际注入。

## 已有可引用证据及源码核对

- Release断言遗漏已明确纠正：124/131不能独自支撑assert合同；144重新编译、148八项断言开启集成测试均通过。独立126/127直接编译测试的断言本来开启。未将原失效证据重标为通过。
- R3只在显式hardening/preserve开关下保留有1–14个当前约束的有限x/P，ready仍false；无观测计时、真实输入故障、100m/s和50m/s²模长保护仍可进入Dormant。默认开关false；保留状态不等于可靠估计。initial warmup和后续质量重建使用同事件ack且30秒合同不由该分支旁路。
- core单调内部ID模式显式opt-in；按旧高水位预验整批，合法提交后更新水位。adapter映射只保留现存slots。外部ID防复活依赖manager非删除位图，不能单靠内部高水位证明。位图可能误拒新ID，不声称精确全历史集合；满载误拒性能仍须结合真实数据。
- bounded history/manager/context/IMU上限、typed退役与staging深拷贝，独立126/127和集成148覆盖失败不消费身份、旧ID不复活、默认OFF语义。143/145高位uint64反例精确重放66事件，三种生命周期诊断均覆盖。
- native短134–137的OFF数值cache字节一致、主state/trajectory exact及R3 cache重放是a62版本证据。最新logger字段不据此冒称已在真实短段验过。
- 开发V2_02的147/150/151：2348事件，2266初始化包全raw；历史输入/配置身份逐项核验，主audit/trajectory exact、零注入。单序列固定科学合同通过；2617出生、2587active退役、5516TTL、3882guard拒绝按epoch逐帧守恒，source为STEREO1036/TEMPORAL1581。未推断其他十序列。
- 资源159/168已有12001帧600秒实际Adapter trace，审计报告0错误且平均<50ms/P95≤100ms；先前149场景未触发已退休ID重新出现，其证据缺口与重跑均保留。纯manager600秒循环不能替代该实际Adapter资源证据。
- 173–175当前wrapper构建、10fixture及heavy/light全x/P/slot、corrected IDs/substeps数值一致通过，是当前字段的合成工程证据。

## 必须保留的未验证与适用限制

1. 尚未执行正式冻结后的all11 EuRoC矩阵，未建立≥8有效B序列或全部P_NEW序列合同。开发单序列成功不外推最终验收；formal runner预检通过不等于真实正式执行成功。
2. cd3 native logger新增corrected IDs/substeps的真实短段字段/缓存合同仍未验；a62短段不能覆盖后来新增字段。不得宣称生产源码完全不变：已修改的是明确列表中的LTV及诊断代码，主估计器算法/配置是否不变由限定场景的精确audit支撑。
3. TEMPORAL开发来源有接受与可靠性结果，但没有同输入P_PREV，不能宣称该条件raw回归通过。正式每来源/每条件的数量、机会率及可靠率矩阵仍待齐备。
4. 恢复输入中的30.1–31.5秒供给中断段与31.7秒起持续段必须分列，不跨间断拼成成功，也不将不足5秒的信息窗口单独解释成已证明精度失败。当前严格全段summary仍false；尚非全部恢复条件通过。
5. 正常合成FAST的覆盖/joint5秒是EuRoC目标线附加诊断；严重ready错误与专门恢复仍是合成硬合同。范围解释不修改JSON数值、不撤销R1/R2重复归零负证据，也不能用无ready空集证明安全。
6. 本轮有限差分/协方差/gauge回归不证明全时间PE、任意噪声鲁棒性、真实闭环收益或ATE；不授权G/V反馈。所有真实输出仍要求实际零提交/零接受，而非仅配置开关。

结论：未发现阻止标注为“R3开发检查点”的新增正确性阻断；上述未验项阻止将其标注为最终科学验收完成。

## 后续cd3 native日志短段补证

独立runtime.verify及snapshot身份校验后，`engineering_short.py R3_logger_cd3_short --r3 --hardened-only` 完成179(native)/180(cache)，均exit0。200 receipts、88 raw包，主audit/trajectory exact、实际零注入、cache精确重放。新增`tests/audit_live_corrections.py`在analysis185通过：85个有校正事件、115个无当前校正事件；corrected IDs唯一、属于retained/currentcam0且数量等于observed_features；actual_corrections每次增长对应真实子步和至少15个输入。该字段是实际接受校正的输入ID，不证明每条ray非零增益或准确性。此前“最新native字段短段未验”缺口现有本段证据补齐，不改变其余未验项。

发现当前native每行camera_substeps输出两次，但均来自同一snapshot且实测200行值完全相等；审计强制拒绝冲突重复并显式记录同值重复。当前Python消费结果一致；严格要求唯一JSON对象键的下游仍需去重兼容修复，未修改受测二进制。证据目录 `/home/he/output/ltv_passive_hardening_v2/R3_logger_cd3_short`。

## 后续R3 V2_03开发负结果

按既定候选及cd3 runtime，full186完整exit0：1921事件、1806初始化包且全部raw，实际零注入。独立离线188严格校验历史B/P_PREV输入、配置及主audit/trajectory精确一致，工程通过；科学状态 **NOT_MET**。联合ready最长2.80s不足5s；raw V RMSE从0.3340292818升至0.3528339000，增加0.0188046182超过允许0.0167014641。不得将此单序列从有效分母排除，也不能靠覆盖率通过宣称完整通过。

通过的分项仍分别列出：G/V coverage27.1318%/21.4286%、有效参考24.5000/19.3500s；ready V RMSE/P95为0.0502889/0.0940306m/s；ready G角RMSE/P95为0.847533/1.179854°；严重错误比例均0；raw eta/角回归通过。没有改阈值、warmup或误差窗口。

生命周期189通过逐帧守恒：2947出生（STEREO1619/TEMPORAL1328），2937active退役、5555candidate TTL、3352guard拒绝。full校正字段190通过（仅对旧cd3显式允许同值重复键）；默认新日志检查现严格拒绝所有重复键。证据目录 `/home/he/output/ltv_passive_hardening_v2/real_development/R3_V2_03_01/evaluation_v1`。这补充了开发负证据，不等于正式all11已执行。
