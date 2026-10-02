# Phase 0 本地适配审查

本轮结果：**STOP / 未通过 Phase 0**。原样源复现和等价移植 parity 通过；
对称化 alias 验收失败，按计划停在步骤 4。没有进入 Passive，没有修改生产估计链路。
本次执行不将计划中“待确认”的 `.eval()` 算法修订视为已经单独获准。

## 冻结现场

- 目标 HEAD：`2a3dcb953cf0b11c6e85a355e4f45090950b97bb`；开始时工作区干净。
- 源 HEAD：`9661610dbcf26374e805634fbd4415ecddf7128d`。
- 源未跟踪项：`docs/01_VINS_LTV_HANDOVER_PROMPT.md`、`docs/ltv_handover/`；已跟踪 diff 为空。
- 交接包 ARTIFACT_HASHES：90/90 匹配；四个核心文件同时匹配实际源文件与 SOURCE_MANIFEST。
- g++ 11.4.0，Eigen 3.4.0，C++14；`-O1 -DNDEBUG -Wextra -Wpedantic`。
- Eigen include：`/usr/include/eigen3`；独立 harness 不链接 ROS/OpenVINS 二进制库。
- 新增 C++ 使用 clang-format 23.1.2、仓库 `-style=file` 格式化。

详细版本、状态、diff 和本地主源码 SHA 见 [freeze.json](evidence/freeze.json)。
未重新获取上游规范版本；此前的“11 文件匹配”不计为本轮重新验证结果。

## 实施范围

新增 `ov_msckf/src/ltv/ltv_observer.{h,cpp}`、`ltv_types.h`，保留持久状态和所有算法表达式。
唯一语义编辑是以 `ov_core::skew_x` 替换 `Utility::skewSymmetric`，两者矩阵元素顺序相同。
排除格式化差异后的补丁见 [core_dependency.diff](evidence/core_dependency.diff)。
交接源副本和 golden 未修改。

ROS2 新增默认 OFF 的 `BUILD_LTV_PHASE0_TESTS`，仅建立独立核心库、parity harness 和 alias 探针。
核心 `.cpp` 不在 `ov_msckf_lib` 源清单中。原生数学测试目标尚未实现，因停止门未通过；
新增 CMake 的隔离 colcon 构建同样 NOT RUN，不能宣称 ROS 链接已验证。

## 阻断证据

源和目标均以原 fixture 重放 174 个事件，摘要与完整 state/P golden 比较均退出 0；
保持 `atol=1e-8, rtol=1e-10` 和离散字段精确匹配。
这验证等价移植，不证明源算法数值安全，也不证明整机轨迹 parity。

`test_ltv_alias` 在非对称 5×5 输入上执行原表达式
`P = 0.5 * (P + P.transpose())`，与冻结旧值参考相比最大误差为 **9**，
输出最大非对称量为 **9**，验收退出 **1**。此探针复现表达式问题；
没有通过篡改私有状态测试完整 sanitize 路径，也没有运行原生传播 Q 的实际数值检查。

未应用 `.eval()`、PSD 截断或 golden 更新。若后续单独确认两处 `.eval()` 修订，
应留存独立 diff、前后数值差异，再按同一 golden/容差运行；超差立即停止。
随后才能继续 T1/T2/T3/T5 与隔离构建。原生 Propagator 保持只读。

原计划中的 ROS2 双目同步检查、平铺配置、adapter、UpdaterLTV 和联合提交均留在后续阶段。
共享输入相关性仍未建模；一次联合 EKF 更新不能消除该近似。

完整临时产物：`/tmp/openvins-ltv-phase0.h7ylvn83/`。
命令、退出码和精简输出已保存于 [commands.json](evidence/commands.json)，
完整临时产物 SHA 见 [artifact_hashes.json](evidence/artifact_hashes.json)。临时目录不保证长期保留。
