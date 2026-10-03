# Phase 0 alias 修复复核（2026-10-03）

结论：本次请求的核心对照、原生数学测试和隔离构建均 **PASS**，在 Phase 0 停止。
本轮开始时目标两处完整 RHS `.eval()`、正向测试和后续 G/V 接入改动已存在；
本轮未新增生产接入、未运行数据集、未调参，未回退已有工作。
因此这里的“隔离”指新的 build/install 目录及独立核心测试目标，**不表示当前生产库没有 LTV 源文件**。
早期三个 Phase 0 报告的“尚未接入”描述仅对应其当时快照。

## 修复及只读边界

目标 `sanitizeCovariance()` 的两处修改均为
`covariance_ = (0.5 * (covariance_ + covariance_.transpose())).eval();`。
数学公式、参数和其余核心逻辑不变；独立 [diff](../evidence/phase0_recheck_20261003/alias_fix.diff)
只有这两处。源交接副本、golden、compare.py 均未修改，原容差仍为 atol=1e-8 / rtol=1e-10。
交接包 ARTIFACT_HASHES 的 90 项全部匹配，运行前后 91 个交接文件哈希不变。
StateHelper、Propagator、IMU、JPLQuat、UpdaterHelper 与 HEAD 字节一致。
HEAD、编译器、Eigen、所有被构建源码哈希及实际动态链接路径均保存在证据目录。

## 本次实际结果

| 验收 | 结果 |
|---|---|
| 旧表达式 alias 探针 | 误差9、非对称量9；预期退出1 |
| 实际修复核心正向测试 | 调用 `propagateImu()` 进入真实 sanitize；误差1.137e-13，非对称量0；退出0 |
| 同一正向测试链接只读旧核心 | 非对称量2.665e-15；预期退出1，证明测试能区分实际实现 |
| 源/目标各174事件 | 摘要和完整state/P/设计矩阵四次原compare均退出0；离散字段精确一致 |
| 修复前后差异 | state最大2.22045e-16（事件46）；P最大4.83124e-9（事件67）；最大逐元素容差占比0.117893；无超差 |
| Riccati谱与对称性 | 174个目标P精确对称，未检出负谱；通过既定归一化负谱界1e-10 |
| 核心生命周期 | 删除中间ID、新ID、全交叉块、slot、snapshot、reset；P误差5.32907e-14，state误差0 |
| T1/T2 | seed42、500组、current/不同FEJ、正反重力单位方向、零速度、15列、原生retraction/外参/bias/标定；最大绝对差6.66134e-9 < 1e-6 |
| T3 | 原生StateHelper四种奇异/非奇异、零/非零innovation；完整均值/P和交叉项；相对Joseph最大2.46833e-16、dense最大3.85809e-16 < 1e-9 |
| T5 | 三积分器×三帧×五步；真实Phi、clone、视觉FEJ及帧间current修正；最大Phi/clone/GV归一化误差分别3.01815e-15 / 3.26324e-15 / 2.55458e-15 < 1e-10 |
| 原生Q限定输入 | 单步非对称量最大1.31605e-24；整段/逐步传播P差1.66937e-20；未修改原预测 |
| ROS2隔离构建 | 三包退出0；五个安装后的Phase 0测试均退出0；ldd确认使用本次隔离库 |

T2 三个 eps=1e-5 / 1e-6 / 1e-7 的预测/残差最大绝对误差分别为
7.63820e-11 / 6.66134e-10 / 6.66134e-9。
当前测试调用实际 `UpdaterLTV` 构造 H，再用原生 `IMU::update()` 独立差分，
与早期参考H版本不同，因此不能直接混用两版测试数值。
这里验证单位重力方向；原物理重力符号和静止标定测试仍保留。
共享输入相关噪声仍是未建模近似，本次测试不证明统计一致性。

## 命令与产物

完整命令、退出码、stdout/stderr、环境与链接记录见
[证据目录](../evidence/phase0_recheck_20261003)。完整事件输出和构建 stderr 压缩文件保留在外部归档，位置与 SHA 见 [外部产物索引](../evidence/external_artifacts.json)。
编译产物在 `/tmp/openvins-ltv-phase0-recheck-20261003`，未放入仓库。

```bash
python3 ov_msckf/tests/ltv/run_core_gate.py --output /tmp/openvins-ltv-phase0-recheck-20261003
python3 ov_msckf/tests/ltv/analyze_core_outputs.py /tmp/openvins-ltv-phase0-recheck-20261003
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base /tmp/openvins-ltv-phase0-recheck-20261003/log build \
  --base-paths /home/he/open_vins_ltv_ws/src/open_vins_ltv --packages-up-to ov_msckf \
  --build-base /tmp/openvins-ltv-phase0-recheck-20261003/build \
  --install-base /tmp/openvins-ltv-phase0-recheck-20261003/install \
  --executor sequential --cmake-args -DBUILD_LTV_PHASE0_TESTS=ON
python3 ov_msckf/tests/ltv/run_native_tests.py \
  /tmp/openvins-ltv-phase0-recheck-20261003/install /tmp/openvins-ltv-phase0-recheck-20261003/native
```

本次不验收 T4/T6/T7、生产提交时序、效果或数据集结果；这些不能由上述 PASS 推断。
