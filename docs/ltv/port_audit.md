# Phase 0 本地适配审查：alias 修复后验收

> 2026-10-03：本次重新执行的结果和当前工作区范围见 [Phase 0 复核](phase0_recheck.md)。下文保留早期验收快照；其中“尚未接入”等描述仅适用于该历史阶段，当前实现状态见工程验收与最终报告。

**PASS（Phase 0 范围）**。本轮按用户单独授权，仅修复目标 LTV 核心两处完整 RHS 的 `.eval()`；
原数学公式、参数和其他算法逻辑不变。原 VINS 源码交接副本、golden、容差均未修改。
未接入生产 G/V 更新，未运行数据集，未调参，未进入 Passive。

## 版本与修订

本轮起点 HEAD 为 `a898929`（完整 HEAD 见 scope_check）；此前 Phase 0 冻结版本与环境记录保留在
[freeze.json](evidence/freeze.json)。源核心 HEAD 为 `9661610dbcf26374e805634fbd4415ecddf7128d`。
交接包 90/90 SHA 仍匹配。原生 StateHelper、Propagator、IMU、JPLQuat、UpdaterHelper 和 ov_core 源码保持原 SHA。

目标核心唯一算法补丁见 [alias_fix.diff](evidence/alias_fix.diff)：
`covariance_ = (0.5 * (covariance_ + covariance_.transpose())).eval();`，共两处。
此前等价 skew 依赖替换补丁仍单列于 [core_dependency.diff](evidence/core_dependency.diff)。

g++ 11.4.0、Eigen 3.4.0、ROS Humble；standalone 使用 C++14、`-O1 -DNDEBUG -Wextra -Wpedantic`。
隔离 colcon 使用仓库默认编译参数（包括 O3），`BUILD_LTV_PHASE0_TESTS=ON`、两路编译、顺序构建包。
C++ 已用 clang-format 23.1.2、`-style=file` 格式化。

## 实际路径正向测试与旧证据

旧硬编码表达式探针原样保留，仍输出 error=9 / asymmetry=9，退出 1，作为预期的反例。
旧停止报告见 [initial_stop_audit.md](evidence/alias_fix_acceptance/initial_stop_audit.md)，旧 commands.json 未覆盖。

新 `test_ltv_alias_fixed` 在测试拥有的非 const observer 中，经已有只读 accessor 的 const_cast
注入非对称 9×9 P（一个内部路标），调用真实 `propagateImu()`，执行目标两处实际 sanitize 代码。
未修改生产可见性。独立计算冻结旧值的 Euler P 与对称参考，其特征值严格为正，floor 不起作用。
目标最终误差 `1.13687e-13`（限 `1e-10`），非对称量 **0**，退出 0。

同一新测试链接只读旧核心时，最终非对称量为 `2.66454e-15`，退出 **1**。
测试要求最终 mirrored pairs 完全相等，这是完整 RHS `.eval()` 的精确后置条件；
否则 eigensolver 已把大非对称误差消去，宽松 epsilon 会掩盖旧表达式的残余。
开发期间 6 维样例以及 9 维宽松对称阈值样例确实未检出旧核心，日志如实保留；最终测试已加强。
此测试执行两处修复，但不声称从最终 P 单独区分第一处与第二处的贡献。

## 原 golden 与独立核心

原／目标核心均完成 174 事件，摘要（含完整 state/P、slot、reset、warmup、substeps）及设计矩阵
分别通过只读 compare.py；仍使用 `atol=1e-8, rtol=1e-10`，离散字段精确匹配。
修复相对本轮原样源的最大 state 差 `2.22045e-16`（event 46），最大 P 差 `4.83124e-9`（event 67）。
最大逐元素 P 差／允许误差为 `0.117893`，没有超差。数值变化来自冻结求值消除 alias 后的谱重构与递推舍入。

174 个目标 P 全部精确对称，未检出负特征值；原样源最大非对称量 `1.45519e-11`。
独立 compaction 测试同时删除中间 ID、加入新 ID，验证全部旧路标／v／g 交叉块、新路标初值与零交叉项、
slot 映射、state 保留、snapshot 值拷贝及 reset。最大 P 误差 `5.32907e-14`，state 误差 0。

## 原生验收与构建

T1/T2、T3、T5 均 PASS，阈值与范围见 [test_matrix.md](test_matrix.md)。
FEJ 包括 DISCRETE、RK4、ANALYTICAL 三种积分，三帧与帧间 current 修正，真实 Phi、clone 增广、
真实 `propagate_and_clone()` 与逐步传播对照，以及 UpdaterHelper 视觉 Jacobian 与 nullspace projection。
原生 Q 的本次数值误差在界内；这不证明原生原地对称化对任意非对称输入安全，原传播代码未修改。

隔离构建三包 ov_core/ov_init/ov_msckf 通过；首轮新增测试把 MatrixXd 当 vector 使用导致编译失败，
改为显式 VectorXd 后通过，首次失败与重试日志均保留。依赖库旧 deprecation 警告不影响结果。
原生时间选取在重合端点输出去重提示，完整传播与逐步结果仍通过阈值。

ldd 确认三个 OpenVINS 库均来自本次 `/tmp/.../install`。
生产 `libov_msckf_lib.so` 无 `ltv::` 动态符号，LTV 核心只在独立测试库链接；默认开关仍 OFF。

## 证据与边界

证据入口：[alias_fix_acceptance](evidence/alias_fix_acceptance/commands.json)，包含逐命令退出码、stdout/stderr；
另有数值差异、SHA、编译 flags、库路径、原生完整测试日志和最大指标。
完整二进制、174 事件输出及构建产物位于 `/tmp/openvins-ltv-core-gate.llf_74db/`，不加入仓库；临时目录可能被清理。

本轮 G/V H 是测试参考实现，尚非 UpdaterLTV。后续接入必须直接测试实际 updater。
T4 联合提交、T6 adapter 时序、T7 B/P 回归、非法 R/S 拒绝、真实数据效果均未运行，仍属于后续阶段。
共享输入相关项未建模，一次联合更新不能消除该近似。本轮工程数学验收不代表轨迹效果结论。
