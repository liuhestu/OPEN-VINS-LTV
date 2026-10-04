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
