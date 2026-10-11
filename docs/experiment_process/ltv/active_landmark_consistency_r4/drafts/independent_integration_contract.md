# R4 独立集成与数学验收合同（实现前审查）

依据新goal，第四轮已获得明确授权；前三轮的禁止第四轮边界不再阻止本轮限定的active consistency。旧结果/预算不可重写。本文件只读检查当前history/pipeline/manager/SeedEstimator/Adapter及此前两事件证据，没有编译、仿真或新几何求解。以下是交付实现前的独立要求，不是通过声明。

## 现成接口与集成次序

`LtvFeaturePipeline::process`在 `manager.step` 之前仍可读取上一事件的 `manager.history()` 与已admitted记录。应在这一边界计算active consistency：只对此前retained、当前cam0存在且match-valid的点评价，包括coasting后再见；同帧新birth仍走原seed路径，不重写均值。若实现只针对“成熟”子集，必须事前明确是否有额外maturity条件；goal第8节的直接语义是此前retained且当前可见，由history min-count/span决定可评价性，不能事后加延迟来绕过两目标。

**不能直接复用原seed构造lambda**：它会把当前cam0加入SeedInput，还可能加入当前cam1。新薄层复用的是 `LtvSeedEstimator::estimate` 数学求解器。execution pose可以是当前pose以支持输出坐标，但SeedInput.observations必须全部来自严格past cam0；anchor_observation也必须指past。

history deque在上一事件时裁窗，因此当前评价仍需显式筛 `ctx.time-history_window <= sample.time < ctx.time`。当前context唯一解析过去poses，1e-9只用于clone查找，不得扩历史窗。没有pose就记missing，不补历史版本均值。`FeatureTrackHistory`没有epoch字段，调用端须绑定 `manager.history().epoch()==ctx.epoch`。统计history_count/span应是实际可解析past支持，不是total_frames，也不是含当前或缺pose的观测总数。

manager的history.update应继续接收完整原始合法观测，包括本次被consistency拒绝的ray；只在out.observations到core的路径skip。这样既不隐藏历史不一致，也不重演R3“删除history改变容量竞争”的兼容问题。已有bounded history/identity guard容量和TTL不动，不加第二套历史。

## FitFailed 与 HistoryInconsistent 必须分开

|条件|evaluable/pass|reason及计数|
|---|---|---|
|开关关闭|false/true|Disabled，不计数、不额外求解|
|原past数量/跨度不足|false/true|InsufficientHistory，计数保持|
|原past足够但当前clone可解支持不足|false/true|MissingPoseSupport，列原count/可用count/missing count，计数保持|
|原solver数值不可解、无效输入、退化、相机中心或NONPOSITIVE_RAY|false/true|FitFailed，保存solver.reason，计数保持|
|solver.valid且history max residual超阈值|true/false|HistoryInconsistent，计数增加|
|history通过且current holdout超阈值|true/false|HoldoutInconsistent，计数增加|
|两者均通过|true/true|Pass，计数清零|

`LtvSeedEstimator.valid`仅验证输入、秩、有限性和正ray，**不检查max_residual**。因此29636这种正ray、数值可解但max residual≈0.1754的例子必须走HistoryInconsistent，不能使用seed quality全部门限先拒为FitFailed而不惩罚。禁止把condition/risk/min-parallax等原seed质量阈值无声复制成active-fit新门限。

秩仍是原 `λmin <= 1e-12 λmax` 不可解；不要给负距离点或不可定义单位向量伪造有限残差。当前预测恰在camera center的处理应作为FitFailed/non-evaluable明确记录，不能制造0角Pass。历史不一致优先于holdout不一致；可记录两个残差，但reason优先级固定。

角度公式：`c_W=p_WB+R_WB p_BC`，`b_W=R_WB R_BC normalize(z)`；当前预测cam向量 `R_BCᵀ[R_WBᵀ(X_W-p_WB)-p_BC]`。点到ray线性模型按原solver，不追加robust核或新RANSAC。pose来自同一主VIO且与bearing相关，current-ray排除不等于统计独立GT；本轮不传播新独立协方差或声称χ²概率。

## 生命周期和事务

一次FAIL：当前tracker条目仍视为可见，last_seen正常、missed_frames不增加；fail_count/reject_count增加，slot retained、当前cam0不给core。计数不能借cam1增加两次。同一包不能因cold预评估/正式提交而消费两次。PASS清fail_count；non-evaluable明确保持，不能清成0。真正tracker缺失按原coasting规则，不伪造consistency失败；“连续失败”应事前固定为连续可评价检查之间未出现PASS，不能事后修改缺失帧语义。

第二次FAIL retire：输出独立reason=active_consistency和原typed ActiveRetired事件；retained移除、普通受控生命周期删除对应state/P块，保留其他完整交叉块。不能同epoch重新birth该ID，guard/tombstone仍生效。不能靠把observation.match_valid改false实现skip，因为旧manager会立即以identity_invalid退役。

当前Adapter有staged pipeline/core/IDs，core拒绝时回滚camera事务；新counter/diagnostic必须一并在副本中，不能在预验证期间修改真实manager。非法/重复时间、ctx、错误calibration或core拒绝不得消费counter/retire/seed。实际IMU预测已提交与相机事务分开，沿用原合同。

一次skip不应人工改x/P或重置共享均值，但**其他点的正常校正仍会经交叉P改变该点及共享状态**。验收是slot/生命周期不丢、没有外部patch，而不是要求skip帧的整个P数值等于前帧。observed与mature_visible必须只计实际送入校正的点，另报raw-visible；不能让被skip的成熟点继续充足ready分母。

## 必需独立反例

- 改CURRENT或追加future/cam1不改变past point_W、history residual和fit count，仅holdout可变；当前pose在u中不等于current ray在u中。
- 严格history左界/当前右界、非零camera/IMU offset、大epoch、缺clone、同一current context重优化且旧pose版本不能补用。
- 世界刚体gauge、非零杆臂/旋转、不规则间隔；残差和分类不变。minimum frame/span边界比较规则先固定，不能从真实结果反调epsilon。
- 高历史残差但solver.valid的反例必须HistoryInconsistent；rank不足/非有限/NONPOSITIVE_RAY必须FitFailed non-evaluable；两种均不靠GT标签判断。
- PASS→FAIL→PASS、FAIL→non-evaluable→FAIL、FAIL→tracker暂缺→再见，以及同包重复/拒绝事务，验证确切counter、skip、retire和ID不复活。
- 一次skip的retained slot、完整交叉P保留、该ID不在corrected_ids；两FAIL后真删除。新birth每点一次seed；现成core数学源码不变。
- 27537精确事件需HoldoutInconsistent；29636在/早于目标HistoryInconsistent并连续计数退休。之前旧路径加法贡献不能直接减掉当新结果；必须实际受控重放，因为移除观测改变H、P、子步与其他贡献。

## Default OFF与live/cache验收

对258b0b6原runtime、相同inputs/config，新增开关默认false时核对完整x/P、active external/internal IDs/slots、retained/birth/retire、camera events/substeps/corrected_ids、ready以及主state/P/FEJ/clone/visual/trajectory。仅比v/η或最终RMSE不够。新日志关闭/开启的数值对照也必须有。原core、主tracker及主更新数学保持SHA不变。

新per-ID日志至少绑定原integer camera_ns、物理time、epoch/version、feature_id、先前active资格、可评价reason、原past数/可用数/跨度/missing、history/holdout角（未计算应null或显式available位）、fail_count/reject_count、UPDATE/SKIP/RETIRE。帧级需active/evaluable/pass/skip/retire与实际sent-to-core/corrected_ids/mature_visible，可逐包核对守恒。startup/pause不得沿用上一帧记录，冷启动无active亦须明确。

缓存必须保存足以精确重放/比较的active config、结果和计数/typed retire，不靠事后overlay补live日志。采用**仅enabled时追加的版本化后缀**保持旧OFF字节；旧reader/writer仍可处理原缓存。大uint64 feature ID不可使用默认1e6 count限制。冷帧、无active、全部non-evaluable、全skip、retire、重置都须覆盖新schema。

运行身份新增源SHA/config/runtime及新mode，不能把旧no-consistency缓存结果当新enabled输出复用。两事件局部目标前需真实history和manager计数状态；若从旧缓存局部锚定，只能说明那一帧first-failure机制，不冒充从序列开始持续启用后的完整轨迹或连续退休证据。

## 第一版冻结与评价注意

默认起点0.04rad、min5帧/0.20s、连续2次；阈值候选仅0.02/0.04/0.08，新增backend仅满足goal条件证据后。readiness使用258b0b6已存在的固定配置，不借此继续R3C窗口搜索。

goal要求沿用旧acceptance.json；正文severe写“<1%”而既有JSON为“≤1%”，执行协议应明确保存原机器合同，报告精确比例，不因边界歧义临时改阈值。evaluable fraction/false-reject proxy不是真值错误率；必须保留全部active/current分母、不可评价原因、rejection/retire率与所有initialized raw/ready覆盖。

新预算由主线程建账，24 real/12 synthetic/16 short/4 candidates，失败重试计数；本审查没有消耗重计算或调用新scheduler。新授权不代表旧报告中的NOT_MET或工具失败已被改写。
