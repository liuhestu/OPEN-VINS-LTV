# 第一层 observer parity fixture（已实际生成）

最终 **174个事件**。`expected_outputs.jsonl`全部state/v/eta/P/slot/valid/substeps/reset值来自原样源observer，未使用GT、未手填理想结果。初版168事件在同一源二进制两次运行字节一致；补入neutral compaction后，最终174事件的原源构建与本包只读副本构建输出再次字节一致，包括矩阵说明文件。本机C++14、g++11.4、Eigen3.4、`-O1 -DNDEBUG -Wextra -Wpedantic`，不声明跨Eigen/编译器逐位一致。

文件：

- `inputs.jsonl`：事件依序configure/start/imu/camera/reset。第一条含所有core相关配置；未列factor/gate/logger字段保持LtvConfig默认（这些不参与core）。最后的q=0子场景专门隔离compaction，q=0不满足论文SPD Q理论，不能视为生产参数。
- `harness.cpp`：JSON→源observer API适配，只读public state/covariance/slot accessor；无const_cast/私有状态注入、无VINS状态初始化。
- `expected_outputs.jsonl`：每个事件的完整state、v、eta、logical row-major JSON二维P、已知ID→slot（缺席=-1）、时间、healthy/warmup/valid、substeps、reset reason。P维度可由state长度确定。
- `expected_matrices.jsonl`：说明矩阵A/B（IMU事件之前维度）及C/y（camera生命周期之后slots重建）；这些由harness按公式重建，**不是observer内新加getter**，不是用于替代源golden。
- `fixture_manifest.json`：固定seed=42（本fixture无随机采样，seed只是明确记录）、配置输入/源SHA/产物SHA/命令/编译器/字节一致性/compaction证据。
- `compare.py`：实际输出与golden逐项核验。bool/int/string/key/shape严格一致，float默认`atol=1e-8,rtol=1e-10`；可显式收紧。这个工程容差不是统计置信度。

## 覆盖内容与边界

前半段gyro=`[.2,-.15,.3]`、Bg=`[.01,-.02,.03]`，Ba=`[.1,-.03,.05]`，raw acc为固定解析信号且非零；R_BC为0.37rad绕`[1,2,3]/sqrt(14)`旋转，p_BC=`[.08,-.04,.025]`m。合成信号仅用于API/数值parity，未要求它对应一个物理GT轨迹。

正常段含首camera、连续两次以上correction及warmup(3)；max_features=3，IDs10/20/30初次按ID排序；漏20一次时保留、40被容量挡住，漏20第二次删除中间slot并加入age=2的40，留下[10,30,40]。其中M<N、零新点、漏检与global warmup都是源实际行为。正常最大substeps=2。

随后测试无特征→healthy清零、显式reset、IMU时间差+.004s容忍、超tolerance差导致gap、重复frame导致backward、camera gap>.2s、IMU dt>.05s、dt=0、substep cap=1触发CovarianceFailure。`adapter_epoch`仅为**harness计数**（configure/start/显式reset+1），并非core内部字段、也不覆盖自动reset通知；真正目标适配器应独立维护epoch。

最后neutral compaction段q=0/max_missed=0，两步非零IMU传播后单帧删中间slot20并加40。独立比对保留state与P的**每个行列映射，包括所有landmark-v/eta及v-eta交叉项**，最大保留P差`2.000888343900442e-11`（末尾sanitize浮点重构）；旧landmark-motion交叉块非零最大25.01。这证明确实在比对交叉项，非全零P下的空检验；见`../validation/compaction_check.json`。

这个fixture直接调用observer API，不包含ROS同步、getIMUInterval、Ceres或OpenVINS。未来边界样本处理、single-sample区间、动态td/外参、完整目标注入仍未验证。wall time/update_time_ms从golden中排除。G/V辅助本身不在此fixture范围。

## 在新临时目录重放（不覆盖golden）

要求本地g++、Eigen3头、nlohmann/json头，core本身不依赖nlohmann。设置包位置后：

```bash
TASK_BUNDLE=/absolute/path/to/docs/ltv_handover
TASK_BUILD=$(mktemp -d /tmp/ltv-parity.XXXXXX)
g++ -std=c++14 -O1 -DNDEBUG -Wextra -Wpedantic \
  -I"$TASK_BUNDLE/source_core/vins/src" -I/usr/include/eigen3 \
  "$TASK_BUNDLE/source_core/vins/src/ltv/ltv_observer.cpp" \
  "$TASK_BUNDLE/parity_fixture/harness.cpp" -o "$TASK_BUILD/observer_fixture"
"$TASK_BUILD/observer_fixture" "$TASK_BUNDLE/parity_fixture/inputs.jsonl" \
  "$TASK_BUILD/actual_matrices.jsonl" > "$TASK_BUILD/actual_outputs.jsonl"
python3 -B "$TASK_BUNDLE/parity_fixture/compare.py" "$TASK_BUILD/actual_outputs.jsonl"
python3 -B "$TASK_BUNDLE/parity_fixture/compare.py" "$TASK_BUILD/actual_matrices.jsonl" \
  --expected "$TASK_BUNDLE/parity_fixture/expected_matrices.jsonl"
```

默认flags复现本次版本；更改到目标Eigen/平台时保留不同构建身份。首次移植必须先比较源副本，再比较目标适配器对同一组事件的输出；不要通过修改expected来掩盖不一致。`../validation/run_checks.py`是本次创建包时的源构建/既有测试生成器，历史运行记录保留；重复验证用上面的新临时目录命令。
