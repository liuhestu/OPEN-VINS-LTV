# OPEN-VINS-LTV 第四轮：Active Landmark Consistency 轻量化工程验证方案

**执行对象**：Codex CLI / Work  
**仓库**：`liuhestu/OPEN-VINS-LTV`  
**起点分支**：`feature/ltv-passive-hardening-v2`  
**基线提交**：`258b0b6f7795cd5a5254d9961e178af887cdfe00`  
**任务性质**：第四轮授权；只解决“已进入 LTV 的活动路标后续失去静态一致性”这一问题。  
**最终目标**：在不把 LTV 写成重型前端、不修改 LTV 核心动力学/增益、不进入 G/V 闭环融合的前提下，得到一个能在真实 EuRoC 输入中稳定工作的 Passive LTV：坏活动路标在参与共享 `v/eta` 校正之前被识别并跳过/退休，同时保持足够的可用覆盖与连续 ready。

---

## 0. 可直接发送给 Codex 的启动指令

```text
/goal 按 docs/ltv/active_landmark_consistency_r4/goal.md 执行第四轮 Active Landmark Consistency 任务。以 feature/ltv-passive-hardening-v2@258b0b6 为基线，不重做前3轮，不重新研究 seed、Q/V/P0、readiness 总体窗口或 G/V 融合。本轮只解决：已进入 LTV 的成熟活动路标在后续观测中违反静态路标模型，仍参与 correction 并污染共享 v/eta 的问题。

实现必须轻量：复用 OpenVINS 现有 TrackKLT/RANSAC/FeatureDatabase，不新写前端；复用现有 LtvFeatureHistory 和 LtvSeedEstimator 做 past-only 多视图拟合；生命周期借鉴 OpenVINS SLAM feature 的连续 update failure 后 marginalize 逻辑。新增一个薄的 LtvActiveConsistency 模块，在每个 active landmark 的当前 observation 进入 LtvObserver::updateFeaturesControlled() 之前执行 past-only static-fit + current hold-out consistency check。单次失败只跳过当前 observation 并保留 slot；连续失败达到阈值才 retire。LtvObserver 数学、Riccati、Q/V/P0、离散积分器不改。

先用 76.75 s / ID27537 和 78.90 s / ID29636 两个已知真实反例做精确局部回放和因果验收，再做完整 V2_03 与干净对照序列，冻结后按既有 acceptance.json 做 EuRoC Passive 确认。已有 acceptance.json 的全局成功门槛不得放宽。若现有 LtvSeedEstimator 的 past-only 拟合被证据证明不稳定，才允许增加一个 OpenVINS FeatureInitializer 适配候选；不得同时引入新 RANSAC、robust kernel、回环、NIS 硬门控或新地图系统。

每个阶段完成后自行分析证据并继续，不等待用户逐步批准。允许按需使用最多2个subagents：一个做实现/事件复现，一个做独立正确性与数据泄漏审查；主agent统一共享文件、实验账本和最终合并。达到 PASSIVE_READY_EUROC 或遇到不可解决的正确性/输入/预算阻断才停止。不得进入G/V闭环。
```

---

# 1. 为什么第四轮只做这一件事

前三轮已经把问题收窄：

1. 原始 LTV 在固定、长期 landmark 条件下能够收敛；短寿命、零初始化是主要工程落差。
2. 已建立 LTV 专用候选/种子/生命周期路径，采用“双目优先、历史几何回退”的一次初始化。
3. 新 Passive 路径在多条真实序列中相对旧 LTV 明显改善，但整体仍未满足可用性合同。
4. 最新精确复核证明两处真实尖峰不是单纯 readiness 显示问题，而是 **LTV 当前视觉校正确实把共享状态拉坏**：
   - `76.75 s`：速度误差由校正前 `0.106985 m/s` 升到校正后 `0.251884 m/s`。旧点 `27537` 单点速度贡献模长约 `0.190941 m/s`。
   - `78.90 s`：速度误差 `0.119124 -> 0.119713 m/s`，重力向量误差同时变差。旧点 `29636` 主导校正。
5. `27537` 的过去观测高度自洽，但当前 observation 与过去静态点模型相差约 `0.350 rad`；`29636` 的历史窗口本身已经明显不一致。
6. 现有 R3B/R3C readiness 能在事后不放行部分坏时刻，但这发生在 observation 已经参与 LTV correction 之后，不能防止共享 `v/eta` 被污染；最长 joint-ready 仍只有 `3.05 s`。

因此本轮不再研究“什么时候输出 ready”，而是前移防线：

> **一个已经成为 Active landmark 的点，在每次 observation 进入 LTV correction 之前，必须继续满足静态多视图一致性。**

目标不是增加新前端，而是补齐 LTV 专用路标生命周期中缺失的一环：

```text
Candidate / Seed validation
        ↓
      Active
        ↓
持续观测一致性检查  ← 本轮唯一新增核心
        ↓
  pass ─────────────→ correction
  fail once ────────→ skip current observation, keep slot
  fail repeatedly ──→ retire
```

---

# 2. 本轮明确不做什么

以下全部禁止，除非本文件明确列为条件性备用：

- 不新写 KLT、描述子匹配、RANSAC、FeatureDatabase。
- 不修改 OpenVINS 原 tracker 行为。
- 不修改 MSCKF / SLAM 主滤波更新。
- 不修改 LTV 连续方程、Riccati 方程、离散积分、`Q/V/P0`。
- 不重新扫 C2/C6 或其他 gain。
- 不修改 G/V 融合公式，不开启 G/V 注入。
- 不新增回环、地图重识别、完整 MapPoint 子系统。
- 不新建第二套复杂 feature history。
- 不引入 learned feature / neural network。
- 不把 `P_L` 当作已经标定的统计 covariance 来宣称正式 χ²/NIS。
- 不为了通过而放宽既有 `acceptance.json`。
- 不用 GT、未来 observation、未来 track lifetime 做在线筛选。

---

# 3. OpenVINS 中直接复用什么

## 3.1 Tracker / RANSAC：完全复用，不在 LTV 复制

OpenVINS `TrackKLT` 已经执行：

```text
KLT temporal tracking
→ perform_matching()
→ RANSAC mask
→ 只有通过 mask 的 observation 才写 FeatureDatabase
```

LTV 继续只读这些现有 feature ID / normalized bearing。

**本轮不再添加第二层前端 RANSAC。**

## 3.2 FeatureDatabase / Feature history：继续复用输入，LTV 只保留现有轻量历史

OpenVINS 已有 ID 与观测数据库；LTV 现有：

```cpp
LtvFeatureHistory
FeatureTrackHistory
HistorySample
```

已经足够保存本轮需要的有限 past bearing。

禁止新增另一个 `ActiveLandmarkHistory`。

## 3.3 OpenVINS `FeatureInitializer`：条件性备用，不作为第一实现

OpenVINS 已提供成熟：

```cpp
FeatureInitializer::single_triangulation()
FeatureInitializer::single_gaussnewton()
```

以及距离、baseline、condition 等检查。

但当前 LTV 已有 `LtvSeedEstimator`，它直接接受 `SeedInput`，输出：

- `point_W`
- `landmark_B`
- `condition`
- `max_angle_rad`
- `max_residual_rad`
- `min_ray`

因此第一版 **直接复用 `LtvSeedEstimator` 做 past-only static fit**，避免为 `ov_core::Feature` / `ClonePose` 再写重型 adapter。

只有当实验明确证明：

```text
异常 observation 可识别，
但 past-only LtvSeedEstimator 本身在正常点上频繁不稳定
```

才允许增加一个候选：

```text
OpenVINS FeatureInitializer adapter
```

并做一次 A/B；最终只能保留一套几何拟合实现。

## 3.4 生命周期：复用 OpenVINS SLAM 的“连续 update failure 后 marginalize”思想

OpenVINS 持久 SLAM landmark 已经有：

```text
update failure
→ update_fail_count++
→ 连续失败后 should_marg=true
```

本轮采用同样轻量思想：

```text
当前 consistency fail 一次
→ 本帧 observation 不进入 LTV correction
→ landmark slot 继续 retained

连续 fail 达到阈值
→ retire landmark
```

不先实现复杂 `Suspect/Quarantine` 状态机。

## 3.5 MSCKF χ² Gate：只借鉴思想，不直接作为第一版硬 Gate

OpenVINS MSCKF 使用：

\[
S = HPH^\top + R,\qquad
d^2 = r^\top S^{-1} r
\]

和 χ² 阈值拒绝 feature。

本轮暂不直接把：

\[
P_L
\]

代入并声称“95% NIS”，因为当前 `P_L` 是 LTV Riccati 设计矩阵，尚未证明等价于真实统计误差 covariance。

可以把相关量作为 diagnostic，但第一版硬判据使用纯几何：

\[
\boxed{\text{past-only static fit} + \text{current hold-out angular residual}}
\]

---

# 4. 最小代码结构

本轮期望新增代码只围绕一个薄模块。

```text
ov_msckf/src/ltv/
    LtvActiveConsistency.h        # 新增
    LtvActiveConsistency.cpp      # 新增

    LtvFeaturePipeline.{h,cpp}    # 小改：调用 consistency
    LtvLandmarkManager.{h,cpp}    # 小改：失败计数/skip/retire
    LtvOptions.h                  # 仅增加默认关闭配置
```

**原则上不修改：**

```text
ltv_observer.cpp / .h             # 数学与 Riccati 不改
StateHelper
Propagator
UpdaterMSCKF
UpdaterSLAM
TrackKLT
FeatureDatabase
FeatureInitializer
```

如 build system 需要登记新 cpp，可最小修改对应 CMake。

---

# 5. 新模块接口

建议定义：

```cpp
struct ActiveConsistencyConfig {
  bool enabled = false;

  size_t min_history_frames = 5;
  double min_history_span_s = 0.20;

  // 工程角度阈值；不是95%统计置信区间
  double max_history_residual_rad = 0.04;
  double max_holdout_residual_rad = 0.04;

  int retire_after_consecutive_failures = 2;
};

enum class ActiveConsistencyReason {
  Disabled,
  InsufficientHistory,
  MissingPoseSupport,
  FitFailed,
  HistoryInconsistent,
  HoldoutInconsistent,
  Pass
};

struct ActiveConsistencyResult {
  size_t feature_id = 0;

  bool evaluable = false;
  bool pass = true;

  double history_max_residual_rad = 0.0;
  double holdout_residual_rad = 0.0;

  ActiveConsistencyReason reason = ActiveConsistencyReason::Disabled;
};
```

类：

```cpp
class LtvActiveConsistency {
public:
  explicit LtvActiveConsistency(const ActiveConsistencyConfig &config);

  ActiveConsistencyResult evaluate(
      size_t feature_id,
      double current_time,
      const FeatureTrackHistory &history,
      const FeaturePipelineContext &ctx) const;
};
```

## 5.1 `evaluable=false` 的行为

如果当前缺少足够 past pose / past observations，**不要因为无法判断就惩罚该 landmark**。

第一版行为：

```text
evaluable=false
→ 不增加 fail_count
→ 保持原 observation 行为
→ 记录 reason
```

这样可以避免因为当前 OpenVINS clone 窗口不足而让 coverage 人为崩掉。

若后续证据证明“无法评价本身就对应高风险”，必须另建候选并验证，不能在第一版暗中改变语义。

---

# 6. 核心算法：past-only fit + current hold-out

对于当前时刻 \(t_k\) 的 Active landmark \(i\)：

## 6.1 只取过去观测

从 `FeatureTrackHistory` 中取：

\[
t_j < t_k
\]

的 cam0 observation。

**当前 \(t_k\) observation 严禁用于 past fit。**

只保留能够在当前 `FeaturePipelineContext` 中找到对应 pose 的过去观测。

## 6.2 past-only static fit

构造现有：

```cpp
SeedInput u;
```

使用过去 observation + 对应 pose + camera extrinsic。

调用：

```cpp
LtvSeedEstimator::estimate(u);
```

得到：

\[
\hat p_i^W
\]

以及过去窗口内部：

\[
r_{i,\mathrm{history,max}}
\]

可直接利用 `SeedEstimate::max_residual_rad`。

## 6.3 当前留出预测

利用当前 pose：

\[
R_{WB,k},\quad p_{WB,k}
\]

和 cam0 extrinsic，将 past-only 估计的固定世界点投影到当前相机：

\[
\hat z_{i,k}
=
\operatorname{normalize}
\left(
R_{CW,k}
(\hat p_i^W-p_{C,k}^W)
\right).
\]

当前真实 bearing：

\[
z_{i,k}.
\]

计算：

\[
r_{i,\mathrm{holdout}}
=
\arccos
\left(
\operatorname{clamp}
(\hat z_{i,k}^{\top} z_{i,k},-1,1)
\right).
\]

## 6.4 第一版判据

```text
history_max_residual <= max_history_residual
AND
holdout_residual <= max_holdout_residual
→ PASS
```

否则：

```text
FAIL
```

初始工程值可使用：

```text
0.04 rad
```

因为此前 R3C 开发已有这一尺度，且两个已知异常远高于该量级。

但要明确：

> `0.04 rad` 是开发起点，不是理论置信区间。

允许的阈值候选最多：

```text
0.02 / 0.04 / 0.08 rad
```

不得做更密的网格搜索；最终阈值必须在正式全 EuRoC confirmation 之前冻结。

---

# 7. 与 `LtvLandmarkManager` 的最小集成

不要让 `LtvLandmarkManager` 自己解析 OpenVINS pose。

保持职责：

```text
LtvFeaturePipeline   → 几何/consistency
LtvLandmarkManager  → 生命周期
LtvObserver          → LTV数学
```

因此 `LtvFeaturePipeline::process()` 先生成：

```cpp
std::map<size_t, ActiveConsistencyResult> active_consistency;
```

再传给 manager。

建议 manager 接口变为：

```cpp
LandmarkManagerFrame step(
    uint64_t epoch,
    double time,
    uint64_t context_version,
    const std::vector<HistoryObservation> &observations,
    const std::vector<FeatureSeedCandidate> &candidates,
    const std::map<size_t, ActiveConsistencyResult> &active_consistency,
    bool allow_admission = true);
```

`ManagedLandmark` 只新增：

```cpp
int consistency_fail_count = 0;
uint64_t consistency_reject_count = 0;
```

必要 diagnostic：

```cpp
double last_holdout_residual_rad = 0.0;
double last_history_residual_rad = 0.0;
```

---

# 8. 生命周期语义

对于 retained + 当前可见 Active landmark：

## 8.1 PASS

```cpp
m.consistency_fail_count = 0;
out.observations.push_back(current_observation);
```

正常参与：

```cpp
LtvObserver::updateFeaturesControlled()
```

## 8.2 FAIL 一次

```cpp
m.consistency_fail_count++;
m.consistency_reject_count++;
```

然后：

```text
feature id 仍在 retained_ids
当前 observation 不放进 out.observations
```

即：

> **保留该 landmark 的 LTV slot 和已有交叉 P，但本帧不允许它产生视觉校正。**

不要伪造成 missing tracker observation；日志明确记录 `consistency_rejected_current=true`。

## 8.3 连续 FAIL 达到阈值

```cpp
if (m.consistency_fail_count >= retire_after_consecutive_failures)
    retire(..., "active_consistency");
```

随后按现有受控生命周期真正从 retained set 移除。

同 epoch 内是否允许原 feature ID 重新进入，继续遵守现有 identity guard；不要绕过原 tombstone/guard。

## 8.4 `evaluable=false`

```text
fail_count 不变
按原路径允许当前 observation
```

并记录具体原因：

```text
INSUFFICIENT_HISTORY
MISSING_POSE_SUPPORT
FIT_FAILED
...
```

---

# 9. 两个已知反例：第四轮第一优先级验收

这两个事件属于 **开发反例**，不是最终泛化集。

## 9.1 76.75 s / ID 27537

已知证据：

```text
prediction velocity error: 0.106985 m/s
posterior velocity error:  0.251884 m/s

past fit max residual:     ~0.002366 rad
current holdout residual:  ~0.350263 rad

ID27537 velocity contribution norm:
~0.190941 m/s
```

第四轮必须证明：

1. past-only fit 不包含当前 observation；
2. ID27537 在 76.75 s 被判为 `HoldoutInconsistent`；
3. 当前 observation 没有进入实际 LTV camera correction；
4. ID27537 slot 在第一次失败后仍 retained；
5. 实际重放中该点的原 `~0.1909 m/s` 错误贡献消失；
6. 比较同一时刻 prediction / old posterior / new posterior / OpenVINS reference；
7. 如果移除 27537 后 posterior 仍明显变坏，继续做同帧 top-contributor 诊断，但不得立即扩展新算法；先判断是否存在多个同类坏点。

## 9.2 78.90 s / ID 29636

已知证据：

```text
prediction velocity error: 0.119124 m/s
posterior velocity error:  0.119713 m/s

past-window max residual:
~0.175399 rad
```

第四轮必须证明：

1. 该点历史窗口在/早于目标时刻被判为 `HistoryInconsistent`；
2. 当前 observation 不进入 correction；
3. 连续失败计数工作正确；
4. 达到固定失败次数后，该 landmark 真正 retire；
5. 新生点仍遵守原一次 seed / lifecycle 规则，不被该修改误伤。

---

# 10. 单元与数学测试

必须先通过，才能完整跑序列。

## 10.1 无未来泄漏

构造历史：

```text
t0, t1, t2, CURRENT
```

修改 CURRENT bearing 不得改变 past-only `point_W`。

只允许改变：

```text
holdout_residual
```

## 10.2 当前 observation 不参与拟合

显式断言：

```text
past fit observation count == number of observations with t < current_time
```

## 10.3 坐标/Gauge 一致性

对所有 past poses + landmark 同时施加同一世界刚体变换：

```text
history residual
holdout residual
PASS/FAIL
```

必须保持不变。

## 10.4 生命周期

至少覆盖：

```text
PASS
→ fail_count=0

FAIL once
→ retained
→ current observation absent
→ fail_count=1

PASS next frame
→ fail_count reset to 0

FAIL twice
→ retired

evaluable=false
→ fail_count unchanged
```

## 10.5 Controlled observer 契约

当前 observation 被 consistency reject 时：

- `retained_ids` 仍含该 ID；
- `observations` 不含该 ID；
- state slot 在一次 reject 后仍存在；
- 不生成新 seed；
- 不修改 `v/eta` mean 作为外部补丁；
- 不修改 `P_L` 作为 Gate 结果；
- camera update 可正常完成。

## 10.6 default-off

新模块默认关闭时：

```text
x
P
slot
camera events
ready
trajectory
```

与 `258b0b6` 基线逐值/字节级保持既有允许范围内一致。

---

# 11. 开发与冻结流程

## R4.0 基线审计

记录：

- 当前 HEAD / dirty diff；
- `258b0b6` 的报告和两个事件证据；
- 现有 `acceptance.json`；
- 当前 runtime / config / cache SHA；
- 原有 logging 是否完整。

不要覆盖前三轮结果。

## R4.1 最小实现

实现：

```text
LtvActiveConsistency.{h,cpp}
LtvFeaturePipeline 小接入
LtvLandmarkManager 两字段 + skip/retire
LtvOptions 默认关闭
必要日志
```

先只允许：

```text
past-only fit
history residual
current holdout
1次skip / 连续2次retire
```

## R4.2 两事件精确局部重放

优先复用已有缓存和 exact replay harness。

目标不是看漂亮曲线，而是验证：

```text
“坏 observation 确实在 correction 前被阻断”
```

局部重放必须继续检查原 exact reconstruction：

```text
prediction + correction = posterior
```

以及 x/P/slot 一致性。

## R4.3 小范围阈值开发

最多比较：

```text
0.02
0.04
0.08 rad
```

不得笛卡尔积更多参数。

开发数据至少包含：

- 已知坏事件所在 `V2_03_difficult`
- 一条相对正常的 EuRoC 对照序列

选择标准不是只挡住两个坏点，还要同时看：

- evaluable fraction
- rejection fraction
- active feature count
- retire rate
- raw v/G error
- ready coverage
- longest joint-ready
- camera branch runtime

如果 `0.04 rad` 已同时满足机制和覆盖，不需要继续试完所有候选。

## R4.4 条件性备用：OpenVINS FeatureInitializer

仅当出现以下证据时允许：

> 大量正常 active landmarks 因 `LtvSeedEstimator` past-only fit 本身不稳定被误拒，而不是阈值问题。

才实现一个小 adapter，复用：

```cpp
FeatureInitializer::single_triangulation()
FeatureInitializer::single_gaussnewton()
```

并与现有 past-only fit 做单一 A/B。

不得同时保留两套生产路径。

## R4.5 冻结

冻结：

- 阈值；
- min history；
- fail-count；
- geometry backend；
- source SHA；
- config；
- evaluation scripts。

从此正式 EuRoC confirmation 不能再因结果调参。

## R4.6 EuRoC Passive confirmation

继续沿用现有：

```text
docs/ltv/passive_hardening_v2/acceptance.json
```

作为总合同，不降低门槛。

至少检查有效序列：

- `G-ready fraction >= 20%`
- `V-ready fraction >= 20%`
- G/V 与参考交集各 `>=10 s`
- `V RMSE <=0.20 m/s`
- `V P95 <=0.50 m/s`
- `G angle RMSE <=1 deg`
- `G angle P95 <=2 deg`
- longest joint-ready episode `>=5 s`
- severe V/G fraction `<1%`
- raw regression 不超过原合同
- Passive 主状态 / P / FEJ / clone / visual / trajectory 不变
- G/V injection 始终为 0

`V1_01_easy` 的已有 gravity reference exception 保持原定义，不临时改变。

---

# 12. 失败时 Codex 可以怎样自主调整

Codex 不需要每一步回来询问，但只能在以下授权范围内自适应。

## 情况 A：27537 没被识别

先排查：

1. current observation 是否错误加入 past fit；
2. pose timestamp 是否匹配；
3. camera/body/world 坐标转换；
4. history 是否取错 epoch/version；
5. threshold 是否实际生效。

不得先改阈值。

## 情况 B：27537 被拒绝，但该帧速度仍明显恶化

允许：

1. 输出该帧所有 active landmark 的 holdout residual；
2. 复用已有 correction-contribution 诊断，找 top contributors；
3. 检查是否多个旧点同时违反同一静态一致性规则。

如果同一规则已经能识别它们，修正实现即可；不要立刻增加第二套 Gate。

## 情况 C：29636 没有及时 retire

检查：

- history inconsistency 是否连续；
- fail count 是否被错误 reset；
- evaluable=false 是否太频繁；
- manager retained / retired 顺序是否正确。

## 情况 D：异常点挡住了，但 coverage / joint-ready 更差

**不要先放宽 readiness。**

先报告：

- consistency evaluable fraction；
- false-reject proxy；
- reject reason 分布；
- active/mature feature count；
- retire rate；
- insufficient-history ratio；
- missing-pose-support ratio。

允许在冻结前从以下三项中只改变一个：

1. `min_history_frames/span`
2. `max_*_residual_rad`
3. past-fit backend（只有证据支持时）

不得同时改三项。

## 情况 E：`P_L` normalized score 看起来有用

可以记录 diagnostic。

除非另有专门统计校准证据，不允许本轮把它升级成正式“χ² 95% NIS Gate”。

---

# 13. Subagents 建议

不是必须全程开启。若环境支持，最多 2 个：

### Agent A：实现与事件复现

负责：

- `LtvActiveConsistency`
- pipeline / manager 接入
- 76.75 / 78.90 exact replay

### Agent B：独立审查

只读优先，负责：

- current observation 是否泄漏进 past fit；
- 坐标/Gauge；
- default-off parity；
- 失败分母/coverage；
- 两事件因果复核；
- 正式确认前 code/config freeze 审查。

共享文件、最终合并和完整运行由主 agent 统一负责。

---

# 14. 建议新日志字段

每个 active landmark 不需要记录庞大矩阵，只记录：

```text
timestamp
feature_id
evaluable
history_count
history_span_s
history_max_residual_rad
holdout_residual_rad
consistency_pass
consistency_fail_count
action = UPDATE / SKIP / RETIRE
reason
```

每帧汇总：

```text
active_count
evaluable_count
pass_count
skip_count
retire_count
observations_sent_to_ltv
mature_visible
ready_G
ready_V
```

正式 logger 必须实时完整，不依赖事后 overlay 才能获得核心 consistency 统计。

---

# 15. 预算与停止边界

本轮应保持轻量，不再开大规模搜索。

建议硬上限：

| 资源 | 上限 |
|---|---:|
| 完整真实/缓存 replay | 24 |
| 完整合成运行 | 12 |
| ≤10 s 短段/事件局部 replay | 16 |
| 完整 threshold/机制候选 | 4 |
| subagents | 2 |
| 重计算并发 | 2 |

失败与重试计数，不因换 agent / 目录退款。

满足既有 `PASSIVE_READY_EUROC` 合同即可停止，不为耗满预算继续测试。

若在这些范围内证明：

```text
坏 active landmark 已能被因果阻断，
但整体 Passive 仍无法同时满足精度 + coverage + 5s 连续 ready
```

则输出：

```text
NOT_ACHIEVED_WITH_EVIDENCE
```

并准确说明剩余瓶颈，不自动进入第五轮，不转而调增益。

---

# 16. 最终交付

建议新增：

```text
docs/ltv/active_landmark_consistency_r4/
    goal.md
    protocol.json
    plan_live.md
    decision_log.md
    frozen_config.json
    event_76_75.md
    event_78_90.md
    final_report.md
    evidence/
```

代码：

```text
ov_msckf/src/ltv/
    LtvActiveConsistency.h
    LtvActiveConsistency.cpp
```

最终报告必须回答五件事：

1. `27537` 是否在进入 correction 前被拒绝？实际速度错误修正减少多少？
2. `29636` 是否因持续历史不一致被及时 skip / retire？
3. 全序列 bad correction 是否减少，而不是只修两个指定事件？
4. feature rejection 是否导致 coverage / joint-ready 恶化？
5. 是否达到既有 `PASSIVE_READY_EUROC` 合同？

---

# 17. 本轮核心原则

这一轮不要把 LTV 变成另一个 SLAM 前端。

最终结构应保持：

```text
OpenVINS 原 tracker / RANSAC / FeatureDatabase
                    ↓
      LTV 已有 seed + history 管理
                    ↓
     Active Consistency（薄层）
                    ↓
       LTV controlled correction
```

真正新增的算法只有：

\[
\boxed{
\text{Past-only static landmark fit}
\rightarrow
\text{Current held-out bearing check}
\rightarrow
\text{skip once / retire repeatedly}
}
\]

这正好针对当前已经被实验证实的失效机制，同时最大限度复用 OpenVINS 和现有 LTV 代码。
