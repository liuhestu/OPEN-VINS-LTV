# C 独立验收矩阵：passive_hardening_v2

本文件是对本轮 goal.md 的只读合同审查和拟执行验证方案，不是通过声明。审查范围包括完整目标、旧 real_evaluate.py、既有 controlled/manager/pipeline/pose/timing 测试、真实 runner、上一轮独立恢复审计。未运行构建、回放、几何批量或新科学评价；不修改受测公式或历史冻结证据。主代理拥有正式 protocol/acceptance 和账本，本文件不另设成功门槛。

## 1. 先冻结的评价语义与风险

1. 旧评价器以相对零初始化 OLD 的20%改善判定，不能用作本轮最终判定。新主对手是 P_PREV，目标是绝对精度、持续性和恢复。旧模块可复用坐标、参考支持和误差计算，但应新增独立新合同判定器，保留旧评价器 SHA。
2. `raw_common` 会自动排除 NEW 缺失。必须同时保留基线初始化后固定 packet 集、逐方法 current/raw 有效或缺失原因、共同支持比例、PREV 有而 NEW 无的时段及反向差集。不能只见共同支持 RMSE 下降就声称 raw 合同通过。goal 未给“允许缺失比例”的新数字，不能自行发明容忍带；正常跟踪的非预声明缺失不得成为合格路径，DORMANT 缺失要连同覆盖、原因和固定时域明确裁决。
3. 5秒连续段须由相邻有效相机事件组成；缺包、非法时间、null、not-ready、epoch/bootstrap 边界均中断。不能用 capped packet weights 求和替代连续长度，也不能跨长输入间隙连接两个 ready 点。图和数值都保留所有 episode。
4. 恢复合同“10秒内出现持续5秒区间”需在新结果前固定端点口径。建议采用完整5秒区间终点不晚于恢复标签+10秒；起点和终点均报告。该建议是保守解释，不追溯改变结果。标签只能来自冻结供给评价器，不能进入 observer。
5. η健康模长范围、质量重建最小间隔、正常/负对照类别、基线有效性规则、连续事件缺口处理及 P95 算法均需在确认前登记。goal 指定重建默认30秒；不可由确认误差反推健康范围。
6. EuRoC有效不足8条只能判覆盖不足；无效B不能按 NEW 表现筛除。V1_01只豁免G方向硬精度，不豁免覆盖、持续性、物理健康、速度与工程旁路。

## 2. 第10节逐条 requirement-to-evidence

|ID/来源|不可移动的合同|最小可核验证据与反例测试|旧资产与缺口|
|---|---|---|---|
|10.1-a|第9节关键测试、坐标/时间/P无未解决错误|测试manifest绑定源码/二进制/参数；失败保留；新旧模式全状态比较|既有测试可重跑，不自动覆盖新状态机|
|10.1-b|B/P_PREV/P_NEW主状态/P/FEJ/clones/视觉集合/时间/轨迹一致，G/V提交和receipt恒0|每包 native digest 与零计数；模式输入SHA相同；故意改变单个P/FEJ/视觉ID必须失败|passive_outputs与native runner可复用，需新增模式与身份|
|10.1-c|新live日志完整，不依赖恢复器|整数输入event ID＋epoch＋单调序号；每消费图像包唯一receipt；在线出生/seed/退出/ready可重建，live与cache一致|旧guard仍有缺陷，必须真正改新runner；旧overlay不能替代|
|10.1-d|声明外部几何来源|逐seed source、执行pose/P来源、数据边界；若辅助bootstrap另显式标记|现有source和joint covariance证据可复用|
|10.2-a|每有效序列 G、V覆盖各≥20%；有效参考交集各≥10s|固定B初始化后全相机包分母，包括启动/恢复；分别列全参考、ready、交集秒数|旧覆盖口径可复用，需完整11序列及固定B锚点|
|10.2-b|V-ready向量RMSE≤0.20 m/s；向量误差P95≤0.50；另报≤0.10比例|同一NEW V-ready支持比较PREV/NEW/OpenVINS；边界、null、单位和P95 fabricated fixture|旧有向量RMSE/wide比例，缺P95及新判定|
|10.2-c|G-ready方向RMSE≤1°；P95≤2°；V1_01预声明例外|方向用原η计算，与η向量/模长分别报告；官方姿态约定保持|已有角度计算，缺P95和例外硬判定|
|10.2-d|全部ready_G时刻η有限、可归一化、满足固定模长健康规则|遍历所有ready声明（不先过滤非法样本）；零η/NaN/过大模长必须失败|旧align_output会过滤invalid-ready，需保留原声明并硬失败|
|10.2-e|共同raw：V增量≤max(5% PREV,0.01m/s)；η向量≤max(5%,0.05m/s²)；方向≤max(5%,0.1°)|各分量规则独立；未就绪也计raw；额外报告所有缺失/支持差集；制造NEW删除最差片段的反例|旧v/η规则可复用，方向与缺失防作弊欠缺|
|10.2-f|每条有效序列至少一个连续5s joint-ready段；最长断档和全部episodes|事件级episode CSV：起止、持续、样本、终止原因、epoch；测试4.95/5.00s、间隙、重建边界|旧只有longest_gap及fraction，不能证明连续性|
|10.2-g|若POSE_AIDED：每episode最多一次；写入5s之后另评价；至少20s无重写有效段|完整v/η外部写入事务列表与独立消融；标签来源严格只读VIO|旧禁止写v/η，新条件模式必须独立测试，不能沿用旧证明|
|10.2-h|raw/G-ready/V-ready/joint四支持，PREV/NEW/VIO列支持相同|输出每个mask SHA、样本数、秒数、误差；不能事后只用joint|旧四支持可复用但改P_PREV身份与新指标|
|10.3-a|正常确认：可靠10%≥90%、机会接受率≥20%、每类来源≥100接受|STEREO/TEMPORAL分开；正确ID＋正ray＋相对3D≤10%，另报5%；逐条件、逐噪声seed表|旧8/8全部TEMPORAL，不能算双目证据|
|10.3-b|机会在最终risk之前，双目/历史机会分开；附全部合法候选与birth分母|独立输入供给计数与manager统计对照；capacity、Gate拒绝不能减少机会|旧raw机会方法是旧规则，需与本轮合法几何机会定义重新核对|
|10.3-c|预定义负对照合理拒绝且有原因，不要求20%接受|同步双目、暂失右目、错ID、错误时刻、距离离群/重尾输入清单；分类在看结果前冻结|旧单测有几何拒绝，缺完整observer负对照|
|10.4-a|REGULAR0.5s压力回归保留，不将12–15m/s误差放行|完整raw/ready/事件时间轴与旧案例对比，零ready不自动是成功|旧失败数据可诊断，不跨新身份冒充重测|
|10.4-b|输入可见断流/非法时间/空观测最多2相机包撤销对应ready|故障receipt序号→首个false的包差；每分支；非法时间不得使用GT触发|需新增精确事件级反例|
|10.4-c|不足区间数值有界或明确DORMANT；不假装当前健康|raw时间、null理由、数值/P健康；最后旧状态不可贴新时间；记录所有不可用与重建|旧pause/available不足以证明新状态机|
|10.4-d|恢复后10s内有完整持续5s ready且v≤0.10、方向≤1°、模长误差≤0.20|输入15s正常→15s不足→30s恢复；独立供给标签、所有episode真实逐样本宽档检查|全新验收；不能以RMSE或ready数量代替|
|10.4-e|每对应分支严重错误ready≤1%：V>1m/s或G方向>5°|合成与可靠真实参考分别统计分子/分母；不把无GT记正确|旧无严重错误率硬判定|
|10.4-f|无reset风暴；质量bootstrap默认间隔≥30s，原因预登记；物理故障单列|连续触发压力、冷却边界、故障真实性对照；每个reset/epoch均有源事件|旧epoch计数可复用，缺原因/冷却合同|
|10.5-a|最终真实runner运行；至少两代表序列NEW复测确定性|重复native审计＋方法事件/输出与cache；完整身份（配置、库、输入）|旧repeat工具可扩展；缓存推论不替代新live复测|
|10.5-b|日志开/关不改数值；估计/缓存/重诊断耗时分开|同输入数值exact；单独计时字段，关闭轻量诊断测observer支路|旧wall时间明确不满足纯模块性能合同|
|10.5-c|完整相机事件均值<一个周期，P95≤两个周期；20Hz为<50ms/≤100ms|受控环境、逐事件耗时原始分布、observer数量/编译/线程/CPU身份；记录性能NOT_MET|新增；不能把平均核心子步代替完整事件|
|10.5-d|至少10分钟资源测试，内存受声明容量限制、不逐帧线性增长|候选/history/tombstone/ID容器计数与RSS时间轴，容量/峰值；老ID再出现不二次seed|现有短单测禁止同epoch复活，但永久tombstone可能无界，需新策略证明|

## 3. 第10节以外的必需证据

|来源|要求|证据/独立验证|
|---|---|---|
|5.1|基准f76f32f…、HEAD/dirtydiff/二进制/库/配置/输入/环境冻结，保留旧可运行产物|启动manifest与只读旧runtime；P_PREV不能被新版库替换|
|3/7|原OpenVINS数学/前端预算/配置/golden不变，LTV方程/C0默认不变|授权文件diff与SHA；状态/P/FEJ原始旁路；非授权差异阻断|
|5.3|REGULAR0.5、FAST1、真实启动/短断档最早失效窗口；H1–H6区分|时间轴：首seed前x/P、birth前后、校正Δv/Δη、新旧贡献、失踪和恢复；每机制决策一页，不把假设写成因果结论|
|6.1/8.3|同步双目、历史、右目暂失、错误匹配/时间负样本；两噪声seed、至少一种时间相关共享pose误差|生成器输入和标签隔离；同pose误差跨特征共享；不能减小噪声帮助候选|
|6.2/7|准入与seed原子；首次校正之前；保留旧均值和完整P交叉块|快照事务前后、随机slot重排、混合合法/非法seed批次；拒绝时core/manager/history/time全部不变；重复调用拒绝/幂等|
|6.2|保留成熟点、临失只预测、不复用旧bearing、不拼ID、不用未来寿命|同ID失踪重现/换ID/TTL/跨epoch重现fixture；旧点不因均匀性被新点无依据逐出|
|6.3|如双observer：最多活动＋预热；整套x/P/ID/epoch/time/cursor一次切换|故意迟到/错误版本切换拒绝；不同observer分量绝不能混合；预热只用当时已到达历史|
|6.4|seed/pool/observer/ready分层，不能只年龄或VIO分歧|每个reason对应可观测在线输入；GT/阶段/序列名不可达；不同G/V门槛独立测试|
|7|P_E/P_L/Sigma_seed区分，完整相关性与真实retraction/gauge|已有Jacobian/PSD秩/原生PoseJPL可复用；新变换独立有限差分与旧交叉块验证；不额外正则化掩错|
|8.1|缓存含所有候选bearing/pose/P/IMU/time，非仅旧选择|schema完整性审查；改捕获后live/cache exact；多observer同遍历至少一次独立单跑exact，工作量记录|
|8.4|全部11 EuRoC B/P_PREV/P_NEW；至少全部NEW真实runner执行；≥8条B有效|固定名单MH_01/02/03/04/05、V1_01/02/03、V2_01/02/03；每项身份/完成/基线有效/参考支持；不能略掉难序列|
|8.4|BASELINE_INVALID依B独立判定；NEW导致失败不得排除；不改原配置救B|先登记B有效规则与所有失败原因；无效B仍保留运行/旁路；<8条不得称EuRoC成功|
|8.5|UZH保留旧不足；最多2条扩展，元数据有效参考≥30s等事先选，不看NEW误差|冻结候选名单/GT版本/区间；不足UZH_NOT_ESTABLISHED，不阻塞已达标EuRoC|
|8.6|最多3轮机制开发、每轮≤4/总≤12候选、≤2轮最终确认|候选/revision/contact账本；失败确认变已见回归，新确认换预登记seed；不得混版本最终表|
|9.2|每包time/event/epoch/state/ready/reason/raw/health/counts/source/bootstrap；每点生命周期与校正次数|schema缺项硬失败、receipt一一对应；null显式原因；异常x/P/slot/输入前后保存，不能NaN改0|
|11|80合成(预留≥24)、72真实/缓存(预留≥40)、24短≤10s、30000几何、12候选、3子代理/2重任务|新共享账本/继承锁；失败不退款；同遍历多observer记录数量和CPU；缓存新执行计数；不拼短段逃预算|
|11|中断可恢复不从头重跑|PID出生身份/遗留reservation/immutable artifact seal测试；checkpoint精确命令；无权限不绕过|
|12|默认OFF可运行代码、启用/恢复/关闭命令、状态机、全验收表、至少2完整时间轴、所有候选/失败SHA|交付CLI端到端检查；日志live完整；GT缺失不裁图；最终状态只用goal规定六种状态；不自动G/V闭环|

## 4. 旧测试复用与新增独立测试计划

可复用但必须按新身份重跑：`test_ltv_controlled.cpp` 的一次seed/旧均值/完整P与低点数状态；`test_ltv_feature_manager.cpp` 的ID、时间拒绝、coast、容量；`test_ltv_feature_pipeline.cpp` 的共享pose、跨pose协方差、未来/缺协方差拒绝、双目/历史路由；`test_ltv_pose_convention.cpp`、timing/旧关闭回归；geometry test；`audit/passive_outputs.py`、repeat与预算PID/身份单测。历史结果只为回归资产，不宣称已测新修改。

旧缺口：manager仍主要年龄成熟、无完整observer状态机；旧manager测试“不能降低min_features”只代表旧实现约束，本轮授权可单独对照修改但不能替代输出证明。controlled批量拒绝需覆盖后续seed失败时前面seed/manager不能半提交。原runner新路径guard比较整数与double往返导致漏记，必须追加直接live receipt和完整诊断测试。旧恢复器只用于历史，不是新验收主入口。

独立新增测试按修复接口到位后实施：

- **事件/原子事务**：大整数ns不同double往返、同double碰撞的不同原始ID、重复/乱序/非法时间；第二个seed故意非法；事务不留半次birth、slot、P、cursor或seed_written。合法批次首次校正前seed，x/P保存完整。
- **readiness/recovery**：在线可见故障2包撤销；DORMANT raw=null而非旧值贴新时间；冷却29.95/30.00秒；恢复宽档连续4.95/5.00秒和10秒边界；中间单包错误/缺口/重建必须断段。GT标签不可流入受测API。
- **评价器防作弊**：绝对阈值边界、角度P95、异常ready在过滤前失败、NEW删除最差raw区间、只有短碎片累计5秒、错误支持、V1_01例外只作用G硬精度、11条缺1和B有效7/8边界、每来源不足100、负对照分类后改拒绝。
- **资源/复现**：限容量下超过旧ID窗口后重现不能重复seed；10分钟滚动输入记录RSS/容器；日志开关exact；真实live/cache exact；至少两完整真实NEW复测；多个observer与独立单跑exact。

以上仅是独立验证设计。实际重计算必须先得到主代理计算槽授权，通过本轮共享账本执行；本轮C不修改受测公式或正式判定阈值。

## 5. 首次 protocol/acceptance 与预算工具复核

初始 protocol.json / acceptance.json 已正确登记第10节数字阈值、11条EuRoC名单、≥8条B有效、预留40/24以及恢复区间终点≤10秒。独立审查向主代理列出需补充的非数字合同：所有ready_G样本物理健康、四支持和宽档比例、全部episodes/断档、raw缺失防支持缩减、合成严重错误率、DORMANT行为、诊断关闭性能口径、辅助bootstrap独立消融及至少一种时间相关pose噪声。只是补齐目标，不改成功门槛。

新共享调度器独立测试 `scripts/ltv_passive_hardening/tests/test_budget.py` 已由本轮预算执行，attempt **0001 PASS，8 tests**。覆盖五类硬上限、40/24最终预留、并发抢开发最后名额、短段≤10秒、启动失败与非零重试计数、两槽拒绝第三任务且不扣费、确认冻结存在和phase记录、日志/source/PID/单线程环境。内部使用临时目录，未占真实/合成完整预算。该调度器只检查冻结文件存在；冻结内容和科学身份的有效性仍须正式dispatcher验证，不能把空文件存在当科学冻结通过。

首次logger修复只读diff未改变数值传播端点：receipt在feed之前建立，原始整数ns作为身份，available frame用物理double时间及epoch/sequence、manager时间一致性检查。向所有者提出补测/修复：原始ns自身单调性（尤其unavailable重复），同double不同原始ns的身份区分，以及写日志后检查ostream避免I/O静默失败。当前raw无效时仍可能写默认数值，完整null/invalid合同待主状态机接口整合，不能据本次header审查宣称整个live日志合同已通过。
