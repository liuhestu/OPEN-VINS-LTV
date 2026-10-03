# 本轮实现与验收记录

本文件对应[value_study_goal.md](value_study_goal.md)，效果结论见[value_study_report.md](value_study_report.md)。输出根：`/home/he/output/openvins_ltv_value_study_20261003`。最终矩阵、复测、输入哈希及预算由`completion_audit.json`统一核验，不能以单个测试通过替代效果结论。

## 新增实现

- `ValueDiagnostics`默认关闭；在真实视觉压缩结果上冻结prior/FEJ/P、视觉及G/V/GV块，调用完整协方差的只读shadow计算，保存实际posterior。无GT、adapter推进或receipt消费。
- MSCKF可选回调覆盖正常与空视觉返回；原生EKF及传播内核保持原样。
- `run_ltv_value`支持既有固定相机时差和配置mask，传感器来自没有GT的派生目录。保留完整轨迹、状态/P/FEJ/clone/视觉集合摘要及输入计数。
- `scripts/ltv_value`提供输入冻结、离线参考、分层/统计、资格判断、逐run账本、固定矩阵、复测、图表及完成核验；不导入旧全量任务runner。
- 独立计时包装器仅用于短段，实际调用冻结库函数；正式效果回放不加载它。

## 已完成构建与单元/数学验收

|验收|状态|证据或最大误差|
|---|---|---|
|隔离ROS2三包构建及格式化后构建|PASS|`build*.stdout.log`、`build_formatted*.log`；隔离install链接记录`linkage.txt`|
|`test_ltv_value_shadow`|PASS|50个GV+视觉案例、奇异clone P；shadow不改state/P、不消费receipt；与原生mean/P一致；Joseph相对误差2.13e-14 <1e-9|
|`test_ltv_gv_jacobian`|PASS|500状态、3个eps；最大绝对误差6.67e-9 <1e-6；current/FEJ、符号/固定标定|
|`test_ltv_gv_covariance`|PASS|完整dense/Joseph、零innovation、奇异/非奇异P；最大dense相对误差3.86e-16 <1e-9|
|`test_ltv_fej`|PASS|真实传播/clone/视觉投影及current变化；保持原1e-10界|
|`test_ltv_joint_covariance`|PASS|完整联合mean/P与Joseph、固定线性批量/顺序对照，原1e-9界|
|`test_ltv_joint_layout`|PASS|重叠order/像素噪声/独立G/V/拒绝、不同FEJ、四元数符号等|
|`test_ltv_innovation`|PASS|小协方差消减、创新检查，原数值界|
|`test_ltv_timing`|PASS|IMU包围/时差/重复时刻、bias一次扣除；T6通过|
|`test_ltv_options`|PASS|YAML方差/角度单位只转换一次|
|Python参考4项|PASS|Decimal/ns、碰撞拒绝、不等距三次轨迹导数、求导窗口、间隙/边界、非单位姿态/符号/杆臂代数、SLERP|
|Python统计3项|PASS|已知20%改善、块抽样/缺失支持、同向/反向/独立/零方差、连续区间间隙|
|Python资格3项|PASS|2/3门槛、参考敏感性翻转拒绝、短支持/P95恶化拒绝|
|独立evaluator自检|PASS|已知刚体变换、无尺度拟合、时间关联|
|C++格式及diff空白检查|PASS|`tests/format.json`及`git diff --check`|
|计时包装器原生合成检查|PASS|50次辅助build与50次EKF确实被截获；未插桩/插桩测试stdout字节相同|

各原生命令、退出码及stdout/stderr位于`tests/`；Python测试日志同目录。正式运行逐事件检查shadow/actual IMU均值相对差≤1e-9，活动模式每事件EKF为0或1；无视觉、拒绝等实际覆盖另见每run工程审计。

既有174事件核心parity和alias正负对照是**前一阶段证据**，见[test_matrix.md](../phase0/test_matrix.md)，本轮未冒充重跑。核心与交接/golden相对本轮起点未改。ROS1构建为NOT RUN；没有全量UZH、随机噪声Monte Carlo或跨平台实时性能验收。

## 可复现命令

首次冻结输入后不要重复覆盖。当前数据已存在，只需使用`--check`。已有运行由账本核验后复用，`repeats`仅复测合同指定三条。

```bash
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon \
  --log-base /home/he/output/openvins_ltv_value_study_20261003/build_log_formatted build \
  --base-paths /home/he/open_vins_ltv_ws/src/open_vins_ltv --packages-up-to ov_msckf \
  --build-base /home/he/output/openvins_ltv_value_study_20261003/build \
  --install-base /home/he/output/openvins_ltv_value_study_20261003/install \
  --parallel-workers 2 --cmake-args -DBUILD_LTV_PHASE0_TESTS=ON

python3 -B scripts/ltv_value/prepare.py --check
python3 -B scripts/ltv_value/study.py diagnostic
python3 -B scripts/ltv_value/study.py freeze
python3 -B scripts/ltv_value/study.py final
python3 -B scripts/ltv_value/study.py repeats
# 可选计时库的完整编译参数保存在输出根profiling/compile_command.json
python3 -B scripts/ltv_value/profile_cost.py
python3 -B scripts/ltv_value/extended_evidence.py
python3 -B scripts/ltv_value/plots.py
python3 -B scripts/ltv_value/report.py
python3 -B scripts/ltv_value/verify_completion.py
```

本机ROS2依赖最终使用C++17（实际flags.make已归档）；新增代码未引入C++17专用语言结构。计时包装器初次以C++14编译因ROS2头要求失败，日志保留，随后匹配冻结目标实际语言级别。未改变正式构建以迎合结果。

## 保留的开发失败与溯源事项

初次输入预检遇到亚纳秒文本尾数，按A001明确舍入及原文双目核对后通过；初版工程审计把core_reset字符串当整数，修正解析后重新审计，估计器未受影响。GT官网Range读取的代理超时与直接读取成功日志均保留。无修改golden、无放宽数学阈值。

两个运行的磁盘runner哈希与已导入源码差异见`provenance/runner_loaded_source_note.json`，原manifest保留；冻结库、二进制和实际数值命令相同。数值分析脚本和Goal合同哈希保持原冻结值，协议仅追加来源/计时证据。

成本分析首轮在UZH短段无GT支持时调用通用参考插值，触发空数组异常；三个短段估计器及字节对照均已成功，原日志保留。成本程序随后直接读取gzip计时及shadow/actual均值，完全去掉GT依赖，复用前三次运行，仅补完第四次。正式九条均有有效GT支持，冻结参考/统计脚本未改。通用reference插值的全无GT查询仍不作为本轮已验证支持的接口；这种成本场景已由GT无关路径处理。
