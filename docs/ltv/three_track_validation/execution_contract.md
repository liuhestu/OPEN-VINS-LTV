# OpenVINS + LTV 三线并行：Codex 执行参考与验收合同

版本：2026-10-08
执行方式：主 Codex /goal + 三个 subagents；各自独立 worktree，重任务按资源锁排队。
目的：尽快得到经过限定范围验证的 G/V 与 landmark 工程候选，同时补齐真实联合误差模型，为后续融合改进和论文提供可追溯证据。

> 本文是执行合同。先核对仓库、实际源码与数据，再执行；参考公式必须映射到真实实现，不能把参考模型当作已经实现的模型。本文不要求用户逐阶段确认。主 agent 根据预先冻结的验收标准自动推进、调整或停止，并完成最终报告。

## 1. 当前证据与本轮授权边界

参考冻结点：

- 仓库：<https://github.com/liuhestu/OPEN-VINS-LTV>
- 分支：experiment/ltv-nondegradation-correlation-20261008
- 提交：2417d036002fbfd542ae71d96d0fc734cd6d47e8
- 报告：docs/ltv/nondegradation_validation/report.md
- STOP 说明：docs/ltv/nondegradation_validation/landmark_go_stop.md

上述身份来自已有审阅记录；执行时必须重新核对本地/远端，不得假定当前 HEAD 就是此提交。用户本地若有更新，先记录差异并选择实际基线，保留冻结点作为历史对照。不得丢弃未提交改动。

已有证据支持：限定序列、限定指标下 G/V 容限通过；landmark 有部分预测价值，也存在尾部风险；真实主状态—Observer—历史观测的联合交叉块尚未闭合。不能把历史测试通过表述为生产资格、统计一致性或融合收益已经成立。

**本轮允许任务 B 在完整相关性模型完成前，实施独立开关控制的 landmark 工程近似融合。** 这扩展了上一轮 STOP 的实验范围。目的在于测试工程可用性；它不解除统计一致性证据缺口。生产默认 OFF、历史生产参数及已冻结基线保留。任务 C 的理论结果随后用于替换/改善近似模型。

最终至少交付三个独立结论：

1. G/V：哪些模式在什么范围可用，是否有实际收益。
2. Landmark 工程近似：是否达到限定范围不退化，尾部风险和运行开销怎样。
3. 真实联合模型：推导、实现及验证完成到哪一层，仍有什么缺口。

允许失败、STOP 或收益不显著；不得通过放宽标准、缩小共同样本或隐藏失败来获得 PASS。

## 2. 三条线的职责与依赖

| 任务 | 目的 | 可修改内容 | 主要交付 | 依赖 |
|---|---|---|---|---|
| A：G/V 验收 | 获得可复现的 OpenVINS + LTV G/V 候选 | 评估脚本、独立实验配置、必要的受控修复 | OFF/G/V/GV 对照、ATE/RPE/速度/开销、模式推荐 | 冻结基线；不等待 B/C |
| B：Landmark 门控与工程近似 ON | 判断预测优势能否转为主估计收益，控制尾部风险 | 因果门控、独立 landmark updater、近似噪声配置、日志 | shadow 质量证据、OFF/ON 对照、工程候选或 STOP | 不等待 C；初期 G/V OFF |
| C：真实联合误差与相关性传播 | 建立可信的融合噪声及交叉块，为 B 提供替代模型 | Observer 敏感度、seed/主状态/历史联合传播、相关 updater | 推导、Jacobian 检查、联合矩阵、统计实验 | 先冻结实际离散实现；不改 A/B 在跑的版本 |
| 主 agent | 控制资源、冻结标准、汇总与集成 | 协调脚本、公共合同、最终集成 | 总报告、候选配置、提交与身份清单 | 接收 A/B/C 的冻结结果 |

A/B/C 各自在自己的分支写代码和提交。互相提供“提交 OID + 接口说明 + 数据版本”，不得直接修改另一任务 worktree。共享接口由主 agent 协调，消费方显式 cherry-pick 后重新构建和验证。

初期禁止同时修改 G/V 与 landmark 然后将结果归因于某一个模块。组合模式在独立候选通过后，由主 agent 另开集成 worktree 验证。

## 3. 进程与资源冲突：必须执行的隔离合同

### 3.1 统一目录与任务身份

主 agent 在仓库外创建唯一会话协调目录，并把**同一个绝对路径**传给所有 agent：

- LTV_COORD_ROOT：资源锁、进程登记、调度记录、冻结合同。
- LTV_WORKTREE：各任务独立源码目录。
- LTV_TASK_ROOT：各任务独立 build/install/log/tmp/results。
- LTV_RUN_ID：任务 + 模式 + 序列 + attempt + 唯一编号，不复用。

推荐目录：

~~~text
<coord>/locks/
<coord>/registry/task_a.jsonl
<coord>/registry/task_b.jsonl
<coord>/registry/task_c.jsonl
<coord>/contracts/
<task-a>/build/<source-and-config-id>/
<task-a>/install/<source-and-config-id>/
<task-a>/log/<source-and-config-id>/
<task-a>/results/<run-id>/
<task-b>/...                         # 完全独立
<task-c>/...                         # 完全独立
~~~

示意路径不能原样假定存在。由主 agent 检查磁盘空间、权限和现有任务后设置。不得各自在 worktree 内创建同名锁：那样不能互斥。

### 3.2 冲突矩阵

| 活动组合 | 是否允许同时执行 | 控制措施 |
|---|---|---|
| A/B/C 编辑、读报告、轻量分析 | 可以 | 独立 worktree、独立输出；限制线程 |
| 不同任务的大型编译 | 默认排队 | 共用 host_heavy.lock；有实测预算后才扩大并发 |
| 编译与完整回放 | 默认排队 | 共用 host_heavy.lock |
| Monte Carlo 大批量运行与回放 | 默认排队 | 共用 host_heavy.lock |
| 两次完整回放 | 本轮默认排队 | 防止 CPU/IO 竞争；性能回放必须独占 |
| 同任务编译/清理与运行自己的二进制 | 禁止并发 | task_artifact.lock；冻结构建产物 |
| 性能测量与其他计算任务 | 禁止并发 | 独占调度；轻量分析也暂停 |
| 不同任务 ROS 图 | 满足隔离后才允许 | 独立 ROS_DOMAIN_ID、namespace、日志；本轮重回放仍排队 |
| 同一输出目录写入 | 禁止 | 每次 attempt 新目录；失败输出保留 |
| 同一 build/install/CMake cache | 禁止 | 每任务、每冻结构建独立目录 |
| git worktree add/prune、共享分支管理、最终集成 | 主 agent 串行 | git_admin.lock |
| 不同 worktree 的普通 commit | 可以 | 各自分支；不得重写他人分支或修改共同 refs |

资源锁只约束遵守合同的任务，不会自动阻止用户已有程序。性能运行前检查其他负载；发现无关重任务时记录、等待或判定该次性能数据无效，不擅自杀掉用户进程。

### 3.3 锁顺序、队列与运行保护

统一获取顺序：

1. host_heavy.lock（重任务需要）。
2. 本任务 task_artifact.lock。
3. 创建唯一 run 目录并启动。
4. 完整等待所属进程组退出，再释放锁。

不得先拿任务锁等待全局锁，避免相反顺序造成死锁。提交队列记录；拿不到锁就标记 WAITING_RESOURCE，继续不依赖锁的工作，不能直接绕开。

任务 C 单元测试/小规模 Jacobian 检查可轻量并行；大规模 Monte Carlo 必须进入同一重任务队列。预计时间较长时，主 agent 仍要报告进度，不通过反复短轮询消耗 CPU。

所有 build/clean/run 通过统一 launcher 进入锁。锁应由覆盖整个命令与子进程生命周期的 wrapper 持有；不能只锁住启动命令然后让后台子进程脱离保护。

### 3.4 构建与环境隔离

构建前检查 ROS 版本、现有 overlay、编译器、依赖、colcon 默认配置。新 shell 会继承环境；仅使用 bash --noprofile --norc 不代表干净环境。

- 构建只加载已确认的 ROS underlay，禁止带入其他任务 overlay。
- 运行仅加载本任务对应冻结 install；核对 AMENT_PREFIX_PATH、CMAKE_PREFIX_PATH、LD_LIBRARY_PATH、PYTHONPATH。
- 优先使用本任务 local_setup.bash；检查 setup 的父级链，不能加载历史任务前缀。
- 记录实际执行文件的绝对路径、readlink -f、哈希和动态库来源。
- 禁止共享 /tmp 中的临时静态库、测试二进制和 CMake cache。
- 改头文件/ABI 后必须对相应构建完整重建；必要时新建构建 ID，避免旧对象混入。
- 正在运行的构建产物不能 rebuild、clean 或删除。运行期间也不能改影响 symlink install 的代码/配置。
- 设置独立 TMPDIR、ROS_LOG_DIR；不要修改公共 ROS 配置或系统依赖。
- 分配 CPU/内存预算；线程上限不能默认用整机 nproc。轻量数值分析默认 OMP_NUM_THREADS=1、OPENBLAS_NUM_THREADS=1、MKL_NUM_THREADS=1，其他配置记录理由。

colcon 命令形状如下，先根据仓库真实包结构检查；这是示例，不是可盲跑的完整 launcher：

~~~bash
colcon --log-base "$LTV_BUILD_ROOT/log" build \
  --base-paths "$LTV_WORKTREE" \
  --packages-up-to ov_msckf \
  --build-base "$LTV_BUILD_ROOT/build" \
  --install-base "$LTV_BUILD_ROOT/install" \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
~~~

--log-base 是全局参数，放在 build 前；CMAKE_BUILD_PARALLEL_LEVEL 根据主 agent 分配的预算设置。核查 COLCON_DEFAULTS_FILE 和用户默认参数，避免隐式修改构建目录、包选择或测试选项。

### 3.5 ROS、进程归属与停止权限

若使用 ROS 2：

- 各任务分配不同且检查过的 ROS_DOMAIN_ID，固定 RMW_IMPLEMENTATION。
- 设置独立 namespace，并检查 /clock、/tf、/tf_static、图像和 IMU 等绝对 topic 是否需要 remap。
- domain 隔离消息发现，不隔离 CPU、内存、磁盘和输出文件。
- 对 bag player、estimator、collector 全部登记归属；不得仅登记最外层 shell。
- 无 ROS 的离线 runner 也遵守资源与产物锁；不能把离线测试写成真实 ROS 回放。
- 不全局停止 ROS daemon 或清理共享 ROS 日志。

每任务向自己的 registry 文件追加记录，包含：owner、task、run-id、PID、PGID、启动时间/系统进程起始标识、命令、可执行文件、源码 OID、配置哈希、输出目录、资源锁、结束码。主 agent 汇总读取，不让多个 agent 覆盖同一登记文件。

停止只针对本任务已登记且身份再次核验一致的进程组。先发正常终止，等待并检查，再按需要对同组升级；保留原因、退出状态和日志。不得使用 pkill -f ros、killall python、killall ov_msckf 等宽泛命令。PID 复用时不能凭旧 PID 杀进程。

异常退出后检查遗留子进程和锁状态；不能删除一个仍被活动任务持有的锁文件来“解锁”。

### 3.6 输入与数据输出

原始 bag、真值与已冻结缓存只读共享。任何派生缓存要记录生成提交、schema、输入哈希、因果性；不满足条件就独立重建。

每次运行写独立目录。每个失败/重跑都保留，禁止覆盖旧 attempt。分析结果记录所引用的精确 run-id。公共基线只有主 agent 可以发布为只读版本；不要让三个 agent 同时生成同名 baseline.csv。

## 4. 共同评估规则：先冻结再看测试结果

主 agent 写 contracts/acceptance.yaml、dataset_split.yaml 和 metrics.yaml。已有项目合同若更严格，沿用更严格标准；修改必须留下理由，并对全部受影响模式重新评估。

### 4.1 数据与公平性

- 首先完整复现 V2_02、V2_03；再选择至少两条不同运动/视觉条件的留出序列。
- 门控/噪声设计只使用开发集；留出集禁止逐序列调参。若使用测试结果改参数，原留出集降为开发集，补新的留出验证。
- 若实际具备全部 EuRoC 数据与时间预算，最终候选扩展到全部可用序列；否则明确覆盖范围，不能称全数据集通过。
- 同一输入、时间区间、初始化、随机种子、对齐方式、配置与构建模式。ATE 对齐使用冻结的 SE(3) 规则，不能给不同模式选择不同对齐或随意用尺度吸收误差。
- 比较完整轨迹；失败、提前退出、丢帧、初始化延迟单列，不能只比较成功重叠片段。
- OFF 原样精确复现与“新诊断/新门控但融合 OFF”分别检查；不要混淆纯 OpenVINS baseline、LTV OFF 和历史结果复用。
- 重复运行检查确定性，不等于多个独立样本。置信区间考虑时序相关，使用按时间块或序列的分析，不能把每帧当独立样本。

### 4.2 默认工程不退化门槛

在没有更严格既有合同的情况下，冻结以下起始标准：

| 指标 | 门槛/要求 |
|---|---|
| OFF 回归 | 原状态/协方差、生命周期、readiness 等既有精确合同通过 |
| ATE RMSE | 每序列 ON - OFF ≤ max(0.5% × OFF, 0.001 m) |
| 局部位置误差 | 固定时间窗/分段的 RMSE 增量 ≤ max(2% × OFF, 0.002 m) |
| 失败与异常 | 不新增发散、NaN、异常退出、协方差非有限或无解释的严重异常 |
| 运行开销 | 同一生产诊断级别下，帧处理时延 p95 增量 ≤ 5% |
| 内存 | 峰值 RSS 增量 ≤ 10%；不持续增长 |
| 可复现性 | 候选及 OFF 重复运行通过冻结的确定性合同 |

RPE 平移/旋转、姿态误差、速度误差、偏置、恢复时间、ready 切换、更新频率与实际校正量必须报告；其验收阈值在运行前根据现有合同/单位冻结。不得凭 ATE 一个指标宣布全部状态不退化。

这些门槛是工程选择，不是统计一致性证明，也不证明所有环境安全。实际收益另判：报告效应量与不确定性，不把“处于容限内”写成“精度改善”。

状态用 PASS_WITHIN_SCOPE、FAIL、INCONCLUSIVE、STOP、NOT_RUN。诊断级别改变时重新测量，不能拿重 shadow 的开销代替生产模式开销。

## 5. 任务 A：G/V 独立验收

### A1. 先查接入实际情况

核对 Updater 的坐标系、G/V 量定义、符号、时刻、Jacobian、噪声来源、开关、门限和 reset。列出已实现与未实现项。已有历史数据可复用，但标为 HISTORICAL_REUSED，不能计为本轮新回放。

### A2. 固定参数的四模式矩阵

每条序列至少 OFF、G、V、GV；G/V 之外的算法、门限、landmark 状态保持一致。先完整回放再分析。

报告每个模式的 ATE RMSE、RPE、姿态/速度误差、分段/恢复性能、更新次数、拒绝原因、创新、实际状态校正范数和时延/RSS。G/V 开启却几乎没有作用必须明确，不能仅凭不退化得到“有效融合”的结论。

### A3. 有界自适应实验

先分析来源、readiness、拒绝原因、校正力度，再决定是否开展小规模、预先登记的噪声/门限实验。不能把“不改善”直接解释为“权重太弱”。

允许在独立实验配置中调整；候选使用统一跨序列参数，生产默认不改。开发集没有稳定价值，停止该模式调参；不要耗尽预算扫描。G 不合格而 V 合格时可以交付 V-only，不强求 GV。

### A4. 交付

提交模式表、原始 run manifest、候选配置、推荐理由和限制。结论区分：接入正确、不退化、有效更新、收益、统计一致性。A 不能代替 C 宣称相关性闭合。

## 6. 任务 B：尾部风险门控 → 工程近似 landmark ON

### B1. 先获得同点、同帧、因果 shadow 对照

比较 LTV 与“包含相同历史 bearing 的几何预测”以及较弱 anchor 主位姿传播。只与弱对照比较不足以证明信息增量。

同时报告逐点角度 RMSE、p95/p99、坏点比例、逐帧/分段质量、覆盖率、成熟等待、恢复期和生存长度。保留 ungated、共同通过门控样本、各模型全部可用样本三组统计，防止通过删掉难点制造优势。

角度误差较大不能直接在线用真值过滤：真值仅用于离线评估。不能使用未来轨迹、未来点寿命、全段最优阈值或事后误差作为门控输入。

### B2. 开发有限数量的因果门控

先检查尾部来源：跟踪/身份错误、深度几何、视差、成熟度、预测时长、创新突变、恢复切换。用当时可获取信息选择少量门控：

- track/identity 稳定性、有效几何、深度/视差与成熟度。
- bearing 预测创新及时间一致性。
- 最大预测年龄、输入缺失、异常恢复状态。
- 单帧更新数量和信息贡献上限，防止大量同源点集中更新。

门控若使用依赖近似 R 的 NIS，只能作为工程分数，不能据此声称满足卡方校准。门控会改变条件分布；统计检验要覆盖实际门控后的运行模式。

在开发集冻结少量候选后，留出集只验收。若 p95/p99 仍明显恶化、覆盖率塌缩或恢复段失去价值，停止 ON，报告 STOP。

### B3. 实施工程近似 ON

满足预测质量门槛后允许接入，不等待任务 C 完整结束：

1. 实现独立实验开关与独立配置，默认 OFF；G/V 初期 OFF。
2. 使用与真实坐标系一致的 landmark 残差及 Jacobian；通过有限差分、单位/符号、时刻和 OFF 回归。
3. R_approx 明确标注为工程噪声模型。噪声底值、年龄/质量膨胀及参数选择理由写入合同，单位必须正确。
4. P_Riccati 不能直接冒充真实 landmark 协方差；未知交叉项不能在文档中写成已经证明为零。
5. 近似独立更新可实验，但 manifest 必须标记 correlation_model=approximate，并列出未建模相关性。
6. 优先通过候选门控、噪声膨胀、更新数量限制控制影响。不要事后随意截断 K r 却仍宣称标准 Kalman/Joseph 公式保持一致；若采用非标准更新，单独推导并验证。
7. 输出完整校正与拒绝日志，能检查作用是否真实存在、是否改变主状态和偏置。

先小范围真实 ON 检查，再完整匹配 OFF/ON 回放；发生发散/异常则自动停止该候选，保留证据。修复后新版本、新 attempt，重新运行对应检查。

### B4. 对照与接入判定

固定 G/V OFF，至少比较：

- OFF + 同样诊断；
- gated shadow；
- gated landmark ON，工程近似模型。

shadow 是预测评估，不能替代主状态融合回放。保持既有视觉更新，在报告中明确同源信息可能被重复使用；删除原 MSCKF 观测避免重复属于另一种架构实验，不能悄悄改变基线。

近似候选达到第 4 节标准，只能授予“在所测数据、配置与性能范围内工程不退化”。收益不足也可以交付有限候选，但不能宣称统计一致、理论保证或生产通用可用。

不合格则保留 shadow 和门控工具，交付 STOP 与最小下一步；不强制启用融合。

## 7. 任务 C：真实联合误差模型参考

### C1. 从实际程序建立状态与源信息账本

先核对 Observer 每一步的离散更新顺序、dt、gain、bearing、外参、seed、身份切换、reset、投影/特征值裁剪、readiness。论文连续式不能替代实际代码 Jacobian。

至少记录：

- 主状态误差 δx 与完整 P，clone、bias、calibration/time offset 按实际配置处理。
- Observer 估计误差 e_L，选定 true-est 或 est-true 并全程统一。
- 当前与 anchor 的 Observer 状态及协方差。
- seed 的完整输入及主状态交叉块。
- 历史 bearing 和 IMU 源噪声的复用关系。
- Observer 与当前视觉残差、主状态之间的交叉项。

P_R 是 Observer 内部 Riccati/gain 状态；Σ_L 才是被验证的真实估计误差协方差。若 gain 随 P_R 变化，P_R 的敏感度必须进入计算图，不能固定 gain 后称完整 Observer 模型完成。

### C2. 实际离散敏感度：起始参考

以下仅适用于代码确实采用对应 Euler 步的情形；其他离散化须按真实步骤重推。δ 表示程序估计量的局部扰动，**不是自动等于联合估计误差**。扰动到 true-est/est-true 的映射另行推导。

预测：

$$
\hat l^+=\hat l+h(A\hat l+Ba),\qquad
P_R^+=P_R+h(AP_R+P_RA^\top+V_o).
$$

其一阶扰动：

$$
\delta\hat l^+=(I+hA)\delta\hat l
+h(\delta A)\hat l+hB\delta a+h(\delta B)a,
$$

$$
\delta P_R^+=\delta P_R+h\left(
A\delta P_R+\delta P_RA^\top+
\delta A P_R+P_R\delta A^\top+\delta V_o
\right).
$$

若 dt 也受时间标定/随机时刻影响，加对应 δh 项。以程序校正 IMU 为例，δω=δω_m-δb_hat_g；这是计算扰动关系，不能直接混用主状态 true-est 的 bias 符号。

相机校正若为：

$$
r_o=y-C\hat l,\quad
\hat l^+=\hat l+hqP_RC^\top r_o,\quad
P_R^+=P_R-hqP_RC^\top CP_R,
$$

则冻结 h、q 时：

$$
\delta\hat l^+=(I-hqP_RC^\top C)\delta\hat l
+hq\delta P_RC^\top r_o
+hqP_R\delta C^\top r_o
+hqP_RC^\top(\delta y-\delta C\hat l),
$$

$$
\begin{aligned}
\delta P_R^+=\delta P_R-hq(&\delta P_RC^\top CP_R
+P_R\delta C^\top CP_R\\
&+P_RC^\top\delta C P_R
+P_RC^\top C\delta P_R).
\end{aligned}
$$

若 hq 变化，加 δ(hq) 项。bearing 归一化及投影必须求导：

$$
b=\frac{d}{\|d\|},\quad
\delta b=\frac{I-bb^\top}{\|d\|}\delta d,\quad
\Pi=I-bb^\top,\quad
\delta\Pi=-\delta b\,b^\top-b\,\delta b^\top.
$$

若 y=Π p_BC，则 δy=δΠ p_BC+Π δp_BC；若 bearing 有相机—IMU 旋转，加入外参旋转 Jacobian。不能遗漏归一化、gain、seed、时间和外参依赖。

对裁剪、特征值投影、hard gate、身份/reset 等非光滑步骤：

- 在固定分支内验证局部 Jacobian；
- 边界及切换单独统计/验证；
- 保留触发计数与失效策略；
- 不能用平滑区的有限差分通过证明全部事件正确。

### C3. 联合传播与 seed

令联合误差包含主状态、Observer、anchor 和必要历史变量：

$$
\eta_{k+1}=F_k\eta_k+G_k\epsilon_k,\qquad
\Sigma_k=\operatorname{Cov}(\eta_k),\quad
M_k=\operatorname{Cov}(\eta_k,\epsilon_k).
$$

则：

$$
\Sigma_{k+1}=F_k\Sigma_kF_k^\top+G_kQ_kG_k^\top
+F_kM_kG_k^\top+G_kM_k^\top F_k^\top.
$$

只有当前源噪声与已存状态真正独立时，才可以令 M_k=0。复用历史 bearing/seed 时，可扩增源变量或通过等价敏感度保留其关联；不能每次当成新独立观测注入。

例如 seed 估计 f(x_hat_history,z)，在主状态 true-est、seed est-true 的约定下：

$$
e_s\simeq-J_x\delta x_{\rm history}+J_z\epsilon_z,
$$

$$
\operatorname{Cov}(\delta x_k,e_s)
=-P_{k,\rm history}J_x^\top+
\operatorname{Cov}(\delta x_k,\epsilon_z)J_z^\top.
$$

真实 bias、主滤波视觉更新、误差 reset、clone 扩增/移除、anchor 冻结都要同步变换联合块。以矩阵删行替代应有的历史关联会丢失信息；采用压缩存储时证明其保留所需充分统计量。

### C4. Landmark 残差与交叉相关更新

设 Observer 给出 IMU 坐标系中的点向量 l_hat_a、l_hat_t；R_i 把全局向量变换到 IMU_i，p_i 为全局 IMU 位置。参考残差：

$$
r_L=\hat l_t-
R_t\left(R_a^\top\hat l_a+p_a-p_t\right).
$$

若真实实现采用相机坐标或别的 anchor，则加入外参并重新求 Jacobian。令 A_{ta}=R_tR_a^\top，e_i=l_hat_i-l_i：

$$
r_L\simeq H_L\delta x+n_L,\qquad
n_L=e_t-A_{ta}e_a.
$$

H_L 是对同一误差约定的主状态线性化结果，须解析推导并数值核对，不直接照搬 MSCKF bearing Jacobian。

$$
R_L=\Sigma_{tt}+A_{ta}\Sigma_{aa}A_{ta}^\top
-\Sigma_{ta}A_{ta}^\top-A_{ta}\Sigma_{at},
$$

$$
N_L=\operatorname{Cov}(\delta x,n_L)
=C_{xt}-C_{xa}A_{ta}^\top.
$$

主状态与辅助噪声相关时：

$$
S=H_LPH_L^\top+R_L+H_LN_L+N_L^\top H_L^\top,
$$

$$
K=(PH_L^\top+N_L)S^{-1},\qquad
P^+=P-(PH_L^\top+N_L)S^{-1}(H_LP+N_L^\top).
$$

数值实现用稳定求解，检查对称/PSD，并按实际误差 reset 变换；禁止用普通独立噪声 Joseph 式掩盖 N_L 项。

若与视觉残差合用，还需要：

$$
\operatorname{Cov}(n_L,n_{\rm vis})
=\operatorname{Cov}(e_t,n_{\rm vis})
-A_{ta}\operatorname{Cov}(e_a,n_{\rm vis}).
$$

堆叠更新保留这些块；顺序更新必须相应条件化和更新交叉块。当前 landmark 与 anchor 都来自同一个 Observer，仅建当前 Σ_tt 不够。视角/历史相关性不能通过把 R 乘一个常数就宣称已正确建模。

### C5. 分层实施与自动判定

| 层级 | 实施 | 验证 | 能宣称什么 |
|---|---|---|---|
| C0 | 依赖账本、公式映射、符号与单位合同 | 对实际源码逐项核对 | 设计完整性 |
| C1 | 完整离散敏感度，含 P_R/gain、bearing、seed 等 | 有限差分/自动微分对照，固定分支及切换检查 | 局部计算敏感度正确 |
| C2 | seed、IMU、主更新/reset、anchor 的联合传播 | 有可控真值与同源噪声的 Monte Carlo | 受控模型校准 |
| C3 | 实际运行交叉块与相关 updater | 完整回放、联合 PSD、源关联审计 | 运行模型实现范围 |
| C4 | 用真实模型替换 B 的近似模型 | 同数据同门控 OFF/approx/correlated 对照 | 限定范围模型/融合效果 |

PSD、确定性或矩阵维数正确都不能单独证明统计校准。无独立 landmark 真值时不能标真实校准通过；仿真、创新、真实轨迹证据分别报告。

可先闭合有限状态/固定标定版本，明确限制；不要求本轮证明完整非线性系统全球稳定性。数学推导完成也不等于真实程序相关性传播完成。

## 8. 主 agent 的推进、组合与收尾

1. 核对基线、数据、运行环境；创建 worktree/资源计划/统一 launcher，冻结合同。
2. 启动 A/B/C；轻量工作并行，构建、重 Monte Carlo 和真实回放按队列执行。
3. A/B 各自以冻结结果申请进入下一阶段，主 agent 根据合同自动裁决。
4. C 输出版本化接口；B 可继续工程近似，不等待 C 全闭合，也不混用半成品交叉块。
5. 单独候选通过后，在独立集成 worktree 比较：OFF、最佳 G/V 候选、landmark-only、组合。组合不能自动继承单模块资格。
6. 只将通过对应范围验收的实现作为实验候选交付；全部生产开关继续默认 OFF。
7. 每个任务及最终集成提交并推送用户指定远端；核对远端 OID。不自动合并主分支，不丢弃未提交改动。
8. 无需逐阶段向用户提问。只有缺少必要数据/权限且无法完成、破坏性操作、授权范围不明确才请求补充；其余自动推进并记录。

出现指标失败，先查构建污染、数据匹配、时间与符号、异常状态，再判断算法。构建/环境污染使该次结果无效，不能计为算法失败或成功；失败记录仍保留。

预算不足优先保证 OFF 身份、独立候选完整回放、失败可追溯和总结。未执行的扩展列 NOT_RUN，不能靠复用历史数据假装完成。不要承诺一天内全部理论与生产资格闭合。

## 9. 必须交付的文件与结论

推荐仓库目录 docs/ltv/three_track_validation/：

- execution_contract.md：本文落地版、环境与资源预算。
- process_isolation.md：worktree/锁/ROS domain/launcher/登记与异常清理说明。
- run_manifest.csv：包括所有成功、失败、取消、重跑、历史复用。
- gv_report.md：四模式、逐序列指标、有效作用、推荐候选。
- landmark_gate_report.md：因果性、共同样本、尾部风险、覆盖率。
- landmark_approx_report.md：残差/Jacobian、近似项、OFF/ON、工程结论。
- joint_error_model.md：完整符号、实际离散敏感度、源依赖、联合传播。
- joint_model_validation.md：受控/真实证据与缺口，不混淆校准等级。
- integration_report.md：组合模式资格及影响归因。
- final_usability_and_integration.md：最终可用性与接入逻辑总报告。

大体积原始结果保存在稳定输出目录，报告提供路径、哈希、生成命令；不要把无法访问的本机绝对路径当作已公开数据。仓库至少保留结构化汇总、manifest 和可复现实验命令。

每次 run manifest 至少包含：task/run/attempt、源码/父基线 OID、dirty 状态及补丁哈希、二进制/库/配置/数据哈希、缓存版本、ROS 与构建环境、实际模式、门控和相关性模型、数据区间、开始/结束、进程身份、退出码、确定性检查、指标输出、资源争用、诊断级别、历史复用标识。

最终报告必须逐项回答：

- G、V、GV 哪些通过，哪些有实际收益，覆盖范围是什么？
- Landmark 预测优势是否超过同信息几何对照？门控减少多少尾部风险，损失多少覆盖？
- 工程近似 ON 是否真正改变主估计？不退化/收益/性能结论各是什么？
- 真实联合模型实现到哪一层，缺少哪些交叉块，哪些证明仍依赖假设？
- 组合是否另行通过？哪些默认 OFF？下一步最小工作是什么？

## 10. 可直接交给主 Codex 的 /goal

~~~text
/goal

按 docs/ltv/three_track_validation/execution_contract.md
（或用户提供的 openvins_ltv_three_track_execution_reference.md）执行
OpenVINS + LTV 三线验证。先核对冻结点 2417d03、本地更新、实际源码和数据，
再冻结验收标准。不要丢弃未提交改动，不改生产默认开关。

创建三个 subagents 和独立 worktree：
A：OFF/G/V/GV 验收，先固定参数，再按证据开展有界实验。
B：landmark 因果尾部风险门控与预测质量，随后实施独立开关的工程近似 ON；
不等待完整相关性闭合，明确近似假设，仅授予实测范围工程资格。
C：按实际 Observer 离散代码推导并实现完整敏感度、seed/主状态/历史观测的
真实联合误差及相关性传播，分层验证，向 B 提供冻结模型版本。

所有任务共用仓库外同一个 LTV_COORD_ROOT。源码/build/install/log/tmp/results
分别隔离；大型编译、重 Monte Carlo、完整回放与性能测试共用 host_heavy.lock。
同任务 build/clean/run 共用 task_artifact.lock，按全局锁→任务锁顺序取得。
登记 PID/PGID/起始身份和归属；只停止自己已核验的进程组，不宽泛 pkill。
不同 ROS domain/namespace 不代表计算资源隔离。
严禁运行中重建二进制、混用 overlay、旧对象/共享缓存污染、输出覆盖。

各任务根据冻结合同自动分析、推进或 STOP，无需用户逐阶段确认。
保留所有失败与重跑。工程不退化、实际收益、统计一致性分别裁决。
A/B 独立候选通过后另建集成 worktree，验证单独和组合，不继承资格。
提交并推送实验分支，核对远端 OID，不自动合并主分支。

交付可用性与接入逻辑总报告、各线报告、完整 manifest、候选配置、
公式/实现范围及未闭合项。结果不好也完成报告，禁止把历史复用、
shadow、确定性或 PSD 检查写成真实 ON 收益或统计一致性通过。
~~~

## 11. 官方构建参考

- colcon build 参数：<https://colcon.readthedocs.io/en/released/reference/verb/build.html>
- colcon 全局参数与日志目录：<https://colcon.readthedocs.io/en/released/reference/global-arguments.html>

参考公式是本轮实现的推导起点；真正验收对象是冻结源码、运行环境、真实数据及对应模型。
