# Phase 0 测试矩阵：修复后结果

> 2026-10-03：本次重新执行的结果和当前工作区范围见 [Phase 0 复核](phase0_recheck.md)。下文保留早期验收快照；其中“尚未接入”等描述仅适用于该历史阶段，当前实现状态见工程验收与最终报告。

**PASS（本阶段范围）**。已按单独授权修复目标两处 alias；未进行生产 G/V 接入或数据集回放。
原源码交接包、golden 和原容差只读。下面的 PASS 不覆盖后续 updater / adapter。

| 项目 | 状态 | 实际覆盖与结果 |
|---|---|---|
| 现场／只读检查 | PASS | 交接包90/90；原生受保护源文件SHA保持一致 |
| 原始源174事件 | PASS | 原摘要、完整state/P、设计矩阵均通过原compare；退出0 |
| 修复目标174事件 | PASS | 同一输入、golden、atol=1e-8 / rtol=1e-10；离散字段精确匹配；退出0 |
| 旧表达式反例 | PASS（复现失败） | 原测试不变，error=9、asymmetry=9；预期退出1 |
| 目标实际sanitize正向测试 | PASS | 实际9维observer+propagateImu；误差1.137e-13 <1e-10，最终精确对称；退出0 |
| 新测试链接旧核心 | PASS（负对照） | 同一测试在旧核心残留2.665e-15非对称量；预期退出1 |
| 核心谱检查 | PASS | 174个目标P精确对称；未检出负特征值；预设负谱归一化界1e-10 |
| compaction全交叉块 | PASS | 中间ID删除、新ID加入、路标/v/g全部交叉块、state/slot/snapshot/reset；P误差5.330e-14 <1e-10 |
| T1原生约定 | PASS | 非单位旋转JPLQuat/IMU retraction；非零相机lever arm反变换；KALIBR/RPNG、Tg排列；非零bias、固定标定下真实静止传播误差<6e-17 |
| T2 G/V Jacobian | PASS | seed42、500状态、current/不同FEJ、零速度、反向物理重力、位置/bias零列；固定T/测量；H与负residual导数分别检查 |
| T3原生完整协方差 | PASS | 正式set_initial_covariance/augment_clone/EKFUpdate；dense和Joseph；q/v↔p/bias/clone交叉项；奇异clone与非奇异P各含零/非零innovation，共4例 |
| T5 FEJ几何与时序 | PASS | 三积分方式×三帧×五IMU间隔；帧间current修正但FEJ保留；真实Phi、clone、视觉FEJ及nullspace projection |
| 原生传播Q行为 | PASS（限定输入） | 真实predict_and_compute的Q；累计表达式与冻结参考；实际propagate_and_clone与逐步P对照；未修改Propagator |
| 隔离ROS2构建与运行 | PASS | 三包构建退出0，5个安装后ros2 run测试均退出0；链接本次隔离库 |
| T4/T6/T7、adapter非法值、非法R/S、效果评估 | NOT RUN | 后续接入阶段，不在本轮验收范围 |

T2 每个线性化点使用真实 `IMU::update()` 做中心差分，全15列。每档最大绝对误差：

| eps | 预测H误差 | residual导数误差 | 预设界 |
|---|---:|---:|---:|
| 1e-5 | 5.78843e-10 | 5.78843e-10 | 1e-6 |
| 1e-6 | 4.24549e-9 | 4.24549e-9 | 1e-6 |
| 1e-7 | 5.89674e-8 | 5.89674e-8 | 1e-6 |

T3 native P 相对 Joseph 最大误差 `2.46834e-16`，相对 dense `3.85810e-16`（界1e-9）；
完整IMU/clone更新后的值与独立dense delta的原生retraction比较误差<6.4e-17。
非零innovation验证p/bias/clone确实被交叉协方差更新。最小谱舍入负量 `1.70017e-15`，
在 `1e-10*max(1,||P||2)` 内；不对奇异P要求LLT成功，不做PSD截断。

T5 最大归一化Phi gauge误差 `3.01816e-15`、clone `3.26324e-15`、
G/V跨帧 `2.55459e-15`、视觉联合 `8.06983e-17`、视觉消元后 `4.40231e-17`，均小于1e-10。
视觉部分调用真实UpdaterHelper，使用GLOBAL_3D及固定相机标定，clone current也作修正并保留FEJ。
原生整段传播与逐步传播P差 `1.66937e-20`；单步Q非对称量 `1.31605e-24`，累计alias差 `6.63847e-24`。
这些限定输入的数值结果不构成任意Q原地表达式安全证明。

## 复现

从仓库根目录执行；core gate 记录目录会打印到 stdout。两个旧实现反例的预期退出1被runner检查，
其他任何非零退出立即停止。首次golden差异由未修改compare.py报告，不允许继续原生验收。

```bash
python3 ov_msckf/tests/ltv/run_core_gate.py
python3 ov_msckf/tests/ltv/analyze_core_outputs.py /tmp/上述core-gate目录
```

本次隔离构建命令（重新执行时可更换整个输出根目录）：

```bash
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base /tmp/openvins-ltv-core-gate.llf_74db/log build \
  --base-paths /home/he/open_vins_ltv_ws/src/open_vins_ltv \
  --packages-up-to ov_msckf \
  --build-base /tmp/openvins-ltv-core-gate.llf_74db/build \
  --install-base /tmp/openvins-ltv-core-gate.llf_74db/install \
  --executor sequential --cmake-args -DBUILD_LTV_PHASE0_TESTS=ON
python3 ov_msckf/tests/ltv/run_native_tests.py \
  /tmp/openvins-ltv-core-gate.llf_74db/install /tmp/openvins-ltv-core-gate.llf_74db/native
```

已安装测试依次为 `test_ltv_alias_fixed`、`test_ltv_core_lifecycle`、`test_ltv_gv_jacobian`、
`test_ltv_gv_covariance`、`test_ltv_fej`；runner分别保存命令、stdout、stderr、退出码，首次失败即停。
`test_ltv_core_parity` 和旧 `test_ltv_alias` 也可独立运行。默认构建开关仍OFF。

本轮全部执行记录：[commands.json](../evidence/alias_fix_acceptance/commands.json)；
最终原生命令记录：[native](../evidence/alias_fix_acceptance/native)；stdout/stderr 见 [外部产物索引](../evidence/external_artifacts.json)；
数值汇总：[max_metrics.json](../evidence/alias_fix_acceptance/max_metrics.json)；
修复前后差异：[numerical_differences.json](../evidence/alias_fix_acceptance/numerical_differences.json)。
首轮新增测试编译类型错误和检测能力不足的开发样例均保留，不计入最终PASS。
