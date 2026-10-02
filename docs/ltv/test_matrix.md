# Phase 0 测试矩阵

状态：**在步骤 4 alias 失败后停止**。未运行数据集；未启动 ROS。

| 项目 | 状态 | 本轮证据 / 后续要求 |
|---|---|---|
| 现场哈希 | PASS | 90/90 交接产物；4/4 核心源/副本/manifest |
| 原始源编译与重放 | PASS | g++ 退出 0；174 事件 |
| 原始源摘要、完整矩阵 golden | PASS | 两个 compare 均退出 0 |
| 目标核心编译与重放 | PASS | g++ 退出 0；174 事件 |
| 目标摘要、完整矩阵 golden | PASS | 两个 compare 均退出 0；原容差 |
| compaction 全交叉块专项 | NOT RUN | 完整 fixture parity 不代替独立映射验证 |
| 非对称输入 alias 专项 | FAIL | error=9、asymmetry=9；退出 1 |
| 核心谱误差专项、原生传播 Q 行为 | NOT RUN | alias 停止门之后不继续 |
| 隔离 colcon 构建 | NOT RUN | 停止门；仅 standalone 核心编译已验证 |
| T1 原生约定 | NOT RUN | retraction、外参、静止重力、bias、固定标定 |
| T2 G/V Jacobian | NOT RUN | seed=42；至少500状态；eps=1e-5/1e-6/1e-7；max abs <=1e-6 |
| T3 原生协方差 | NOT RUN | dense/Joseph 相对阈值1e-9；PSD界1e-10*max(1,norm2(P))；含奇异clone、零innovation |
| T5 FEJ | NOT RUN | 真传播Phi、clone、两帧间current修正而FEJ保留；归一化阈值1e-10 |
| T4/T6/T7、非法R/S、数据集回放 | NOT RUN | 后续接入阶段，非本轮验收 |

T2 还需覆盖 current/不同 FEJ、零速度、位置/bias 零列、反向重力和固定 T/测量。
T3 必须用正式 StateHelper API 构造有 q/v 与 p/bias/clone 交叉项的 PSD P，
比较完整 delta/P，不要求奇异 P 的 LLT 成功。不得放宽阈值或关闭 FEJ。

复现本轮停止门（默认创建新 `/tmp` 目录；预期最终退出 1）：

```bash
python3 ov_msckf/tests/ltv/run_core_gate.py
```

每个子命令分别保存 `.command.json`（含退出码）、stdout、stderr；首次失败立即停止。
本次实际命令记录见 [evidence/commands.json](evidence/commands.json)。
原始输入、golden 和比较脚本来自只读交接目录。没有重写预期答案。

新增 CMake 开关默认 OFF；只包含 `test_ltv_core_parity` 和 `test_ltv_alias`。
计划中的 `test_ltv_gv_jacobian`、`test_ltv_gv_covariance`、`test_ltv_fej` 尚未实现，
不可执行计划中的 ros2 run 命令并将缺失目标解释为数学失败。

恢复工作需先独立处理 `.eval()` 修订，再以原 golden 检查数值变化。
即使 parity 通过，仍必须补齐上述原生测试与隔离 ROS 构建，才能宣称 Phase 0 通过。
