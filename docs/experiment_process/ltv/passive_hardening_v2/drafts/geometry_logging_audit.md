# B：几何输入、缓存与在线日志审查

日期：2026-10-04。范围：只读代码审查；没有编译、运行 observer、读新确认标签或修改生产代码。依据为本轮完整 `goal.md`，本文是实施建议与静态证据，不是新测试 PASS。

## 1. 结论与最小实施顺序

1. 先修正式输入 receipt 与日志归属，再改生命周期。上一轮补丁只把整数往返比较换成 double 比较，能修已知遗漏，但没有建立本轮要求的整数事件身份。
2. 旧 P_NEW full cache 足够在原 process 事件区间重新选择候选/路标与重跑种子；它并不是只有旧选择结果。pause 区间没有候选/pose/P，不能用这些缓存声称验证了全时域提前收集、暂停期间预热或一次主 VIO v/η bootstrap。
3. 新合成正常双目使用实际 EuRoC 双相机标定与近距双目共视点，旧 REGULAR/FAST 远点场景保留原样作回归；新增源不能靠降低噪声或风险阈值保证通过。
4. 数据接口尚无逐观测相机时间：异步右目不能可靠判别。正式负样本必须带可核验的时间身份，不能仅把错误 bearing 塞进同步 packet 而期待时间检查发现。

## 2. 当前代码事实

- `LtvAdapter::frame()` 由 `llround(t*1e9)` 生成 `camera_ns`；runner `feature_row()` 与原始 `left[i].ns` 比较。这在大整数纳秒处并不恒等。原 v/η、ready、计数输出位于 guard 外，seeds/tracks/retained 位于 guard 内。已有恢复审计证明的仅是上一轮输出证据可修复，不是 live logger 已修好。
- `scripts/ltv_feature_passive/diagnostic_recovery/runner_logging_only.patch` 只改为 `f.camera_time == camera_ns*1e-9`；应保留为历史补丁，不把它当最终完整 receipt 设计。
- 每次 `track_image_and_update()` 会清空 passive frame；初始化不足或异常路径可走 `pause()`。adapter 的 sequence 不是所有图像 packet 都必然递增的独立 receipt，不能用它单独作包主键。
- `FeaturePipelineContext` 含当前 physical IMU time、version、当前 execution pose、全部存活 clone 均值及完整联合 6N pose covariance，两相机外参和当前各相机 bearing。历史存 bearing，不把不同优化版本的历史位姿当相互独立测量。
- 同步 Stereo 只用当前 execution pose 的两相机观测；Temporal 使用当前联合上下文可找到的过去 clone。HYBRID 首先构造 Stereo，几何/风险未过则构造 Temporal；当前输出只保留最终选用 candidate，未保存被回退覆盖的 Stereo 失败诊断。
- 当前质量门槛仍需要 3 帧与 0.10 s 历史，连同步双目也不能立即入槽。将它改为来源相关机会门槛是可独立检验的机制变更，不能暗中改变机会分母。
- Seed 风险含 pose 交叉块，但固定外参、独立 bearing 噪声，未建模 state-bearing 与跨种子全部相关性。`Sigma_seed` 不是 Riccati `P_L`，风险不是概率。

## 3. 正式 receipt 与日志修复方案

### 3.1 整数身份独立于连续时间

增加只用于输入归属与审计的 `CameraReceipt`：

```cpp
struct CameraReceipt {
  uint64_t receipt_id;       // 每个接收图像包严格递增，pause/拒绝也占号
  int64_t camera_ns;         // 数据源原整数，不从 double 反算
  std::vector<int> camera_ids;
  std::vector<int64_t> camera_ns_by_id;
};
```

入口在原始数据读取后、调用 VIO 前生成；通过只读 envelope 或新增纯元数据参数传递。连续计算继续使用现有 `double camera.timestamp` 与 offset，不重算传播端点。不要修改主 VIO 内部状态/P/视觉更新。记录 `(receipt_id,camera_ns,epoch,observer_sequence,version)`；receipt 在 observer reset 后不归零。非法/重复物理时间依然有 receipt，并记录拒绝原因。

cache 新版 header/version 记录 receipt，process/pause 均带原整数身份；若初始化前不调用 adapter，也写 `NO_OBSERVER` receipt 记录。保留旧 cache parser 与旧日志 schema，不猜造旧原始纳秒。旧缓存只能借助原 features/audit 文件明确绑定为兼容输入。

### 3.2 无静默丢失，raw 与诊断分离

- 每个已消费图像包恰有一条 receipt 行；branch 未启动也写 `availability=false` 与显式原因。
- 从本事件返回对象取得 frame+management，或保存明确的 `management_receipt_id`。不要读取 adapter 最近一次 feature_frame 再凭浮点相等猜归属。
- 当前 raw 不存在则 `raw_valid=false`、v/η=null，并另写 last-valid 档案时刻；不得把旧 snapshot 或零称作当前 raw。
- 诊断总有 `management_present`、缺失/暂停原因；正常事件缺管理记录是工程错误，不能只输出空数组假装没有 birth。
- 逐事件可重建：新生、首次机会、准入、一次 seed、active/coasting/retired、ready 切换与 bootstrap。never-admitted candidate TTL 单独计数，不与曾占槽的 active removal 混用。epoch reset 另记，不伪装逐点失踪。
- HYBRID 新诊断建议保留 attempted-source 及各分支拒绝原因；改变的仅日志，不因记录尝试而再次执行几何求解。
- 重诊断可落盘，内存只维护有界即时信息。不能为“记录完整”把全部历史 tombstone 常驻内存。

### 3.3 必需验证

含 `1450000000000000123 ns` 等不能 double 无损往返的时间样本；双目原整数完全一致与明确不一致样本；pause、未初始化、重复时间、乱序、跨epoch、同ID重现。逐 receipt 唯一且总数等于 consumed packets。日志开关/旧新元数据路径 x/P/slot/物理时间与主轨迹逐项一致。新 live 与同版 cache replay 对照全部 seeds/tracks/counters，而非仅最终误差。原 guard 可正常记录的旧同 schema 事件还可作为序列化回归资产。

## 4. 旧缓存能验证什么

| 输入/上下文 | process record | pause record / 限制 |
|---|---|---|
| 原 IMU | 全部 feed 事件独立序列化，含右 bracket 样本 | IMU仍记录，但需遵守原因果传播端点 |
| 候选 bearing | 当前 frontend 数据库所有同 timestamp 特征；不是 manager 30槽筛选后列表 | 没有暂停时当前候选捕获 |
| 双目 ID/观测 | context 保存当前cam0/1 ID、bearing、match_valid | 没有每相机独立timestamp；match_valid来自现前端身份代理 |
| pose/P | 全部当前 clone 均值、完整联合 pose covariance、execution index | 没有暂停时上下文；更久已边缘化clone无法补回 |
| 标定/偏置 | Da/Dw/Tg、旋转、cam0杆臂、offset、ba/bg；context两相机外参 | pause仅time/reason/version与输出证据 |
| 主VIO velocity/IMU全P | 未保存 | 无法据此测试 POSE_AIDED_BOOTSTRAP；trajectory只有事后值不替代因果输入合同 |
| 输出证据 | 完整x/P/slots与诊断供exact replay验收 | 保存旧期望输出不是新算法输入 |

依据：`VioManager.cpp` 中 feature_context 构造与 `features_containing(message.timestamp)` 循环；`LtvPassiveCache.cpp::context/process/pause` 序列化。新选择器只能使用 cache 原输入段；不能让新 readiness 控制是否读取那些已经缺失的旧 pause 候选。若希望提前收集、停机仍维护候选或用主VIO一次辅助初值，需新捕获：前端后的当前所有观测、可用pose/P/主速度及来源/有效性，即使旧LTV暂停。不能从旧输出逆向填输入。

## 5. 新正常双目合成合同建议

标定读取并冻结 `config/ltv_euroc/kalibr_imucam_chain.yaml` SHA：使用各自 `R_BC,p_BC`，不能把两相机旋转设为相同。原世界固定点与轨迹的标签只在生成器/评价器。

1. 正常共视：事先固定近距约2–4m、覆盖不同方位的世界固定点；通过实际相机投影/FOV判定可见。场景必须维持预登记数量的双目共视机会，而不能从运行结果筛掉坏点。两条 REGULAR/FAST 运动及0.5/1s身份寿命均保留。若原长轨迹/转角使固定点不共视，预先设计沿轨迹的静态地标布局，不移动路标追相机。
2. 原场景回归：原12–17m远点与0.11m平行基线输入不改，明确是困难几何，保留原噪声/标签/身份与失败。新增近距场景不冒充同身份重跑。
3. 噪声不降级：保留0.02加计、0.002陀螺、0.05°bearing和共享时间相关位姿误差/杆臂映射；同一pose的扰动跨特征共享。两目bearing可沿用预登记独立测量扰动，pose必须同一事件共享。正常机会不能读取真实seed误差才能定义。
4. 单独Temporal正常对照关闭右目输入，并给足真实历史视差；双目暂失用预先固定时间段撤去cam1，恢复时使用新合法同步观测。正常→不足→恢复时间保持15/15/30s合同。
5. 负样本：预先固定比例的右目ID置换/重尾bearing/非法相机时间。估计器只看到观测与合法时间信息，不能见错误标签、未来寿命或真实点。若仅改bearing而仍宣称同步时间，只能评几何拒绝，不能评时序拒绝。
6. 输出分母：全birth、合法Stereo机会、合法Temporal历史机会分别冻结；HYBRID源尝试/选择/接受分别统计；每来源至少100接受样本、可靠≥90%、机会接受≥20%是验收目标，不是设计后自动PASS。两目几何只有两条射线，自拟合残差可能低；原held-out历史角误差仍应记录，对错匹配必要时独立检验。

约束量级仅作输入设计解释：角噪声0.05°时，双目相对深度风险随距离/基线增长，0.11m基线下十几米远点很可能被现0.05风险拒绝；不得把这个量级估计写成验证结果。新场景先跑预算内纯几何 sanity 验证输入/投影与来源执行，再运行完整observer。

## 6. 待主agent整合的所有权

本稿不改 adapter、runner、cache、VioManager 或 frozen 工具。主agent分配生产共享文件后，B可实现独立receipt测试/几何生成器专属文件。全部后续编译、短测、完整缓存/合成都使用本轮新账本，最多两项重计算；旧预算不复用。
