# 自主决策记录

## D001 — 启动与优先级（Phase 1）
问题：autonomous_goal.md 与新附件在预算、开发序列和有限调参权限存在差异。
证据：最新附件明确300回放、V1_01/V2_02/V2_03、最多6组额外候选。
选择：最新附件优先；保留文档的其余数学/日志/覆盖/冻结要求。未开始任何候选搜索。
旧值：80次、含MH_04；新值：300次、开发不含MH_04。参数未改，回放0。
否决：按旧预算/开发集隐式执行，因不符合最新授权。

## D002 — Phase 0 复用与初始参数
证据：alias_fix_acceptance 的核心/测试源SHA与当前工作区全部一致，已通过原生测试。
选择：复用Phase 0；接入后将数学测试替换为调用实际UpdaterLTV，并重跑受影响验收。
初值：源observer默认max_features30/min15/warmup20、Q=1e-4、V=1e6、P0=1；G sigma10deg、V sigma1m/s、角度上限30deg、NIS99%、经验quality gate关闭。
旧/新参数：无变更。禁止直接使用旧VINS的ATE作为本任务证据。

## D003 — 确定性输入和初期旁路证据（Phase 1）
问题：ROS transport 的异步队列/丢帧不适合作为严格 OFF/Passive 对照输入。
选择：小型 ASL C++ runner 顺序读取相同原始 IMU 和同步 stereo，直接调用原 VioManager；初始化使用原静态 initializer，无GT输入，所有模式相同资源配置。
原程序的线程选项在所有离线模式统一为状态订阅单线程；原生相机点数、像素/IMU噪声、OpenCV线程4、FEJ及RK4不变。专用配置按范围固定标定、max_slam=0、ArUco/ZUPT关闭，原默认YAML不改。
证据：原版库/runner单独冻结；400包、288条输出的original/OFF/P轨迹与每帧state/P/FEJ/clone/live-feature-ID及接受点坐标digest逐字节一致。P有288 candidate、269 warmup后valid、30内部路标。
参数变化：仅规范必需的固定标定/MSCKF-only实验基础配置；G/V sigma等未变。
否决：只比较ATE接近、或复制B输出作为P，两者均不能证明旁路正确。
构建修复：simulation ZUPT分支无message对象，使用其已有timestamp；不更改ZUPT算法。首次构建失败日志保留，修复后构建通过。

## D004 — 评价口径在读取真实ATE前固定
证据：本地ov_eval/ResultTrajectory.cpp使用nearest association的0.02秒上限；用户要求共同支持、无尺度SE3、99%覆盖门槛。
选择：独立小型Python evaluator统一处理所有模式；SE3位置对齐各自独立，旋转与官方GT速度用同一各自R；物理相机支持以B首次实际输出后全部输入帧为准，无额外初始化排除；同时检查传感器输出覆盖与GT可关联共同支持>=99%。近邻GT容差0.02秒，模式间相机时刻容差1微秒，不插值或改变GT。
初始解析：本实验calib_camimu_dt=0，OpenVINS JPL GtoI四元数分量按Hamilton ItoG解释用于rotation诊断。GT使用官方state CSV的velocity列。
验证：已知刚体变换、旋转/速度、故意2倍尺度以及近邻关联自测通过；evaluator SHA已保存到输出根evaluator_protocol.json，尚未读取真实ATE。
否决：Sim3、按效果改变关联容差、删困难尾段、跨方法共用B对齐旋转。

## D005 — 联合S预检拒绝诊断（Phase 2，进行中）
证据：400包G-only中245次G独立门控通过，212次联合提交，33次因asymmetric_S拒绝；原生P对称、谱在既定舍入界内。
选择：停止完整G回放，捕获第一份实际P/H/R/S，核对当前full-product/LLT下三角预检与原StateHelper的上三角selfadjoint约定。尚不调sigma/NIS，不放宽测试阈值。
候选：实现与native相同的创新矩阵三角求值；若证据显示真实P/H错位则先修索引。最终选择待矩阵证据确定。

### D005 结论 — native三角约定修复（未调参）
实际捕获矩阵：P 78×78，联合H 74×78；raw S norm=9.3706，非对称norm=2.2747e-11，P非对称1.6151e-19，P最小谱-1.06e-23。
上三角mirror S最小谱0.0304656；与long-double乘积最大差3.344e-12，绝对乘积尺度对应舍入界7.45e-9。证据支持有限精度消减，而非错误列映射或非正定主P。
最终选择：新增innovation_matrix小函数，完全复用StateHelper的Upper三角构造/自伴视图约定，供NIS、联合预检和gain诊断统一调用；不改StateHelper，不改变P/噪声/测量公式，不放宽任何阈值。
实际P/H/R已保存为tests/ltv/fixtures/joint_cancellation.txt；新增test_ltv_innovation调用实际validate并与native更新/Joseph对照，通过。原三角不一致产生的短段运行全部保留，不能作正式效果结果。
修复路径1完成；未尝试调sigma、降低权重或Eigen clipping。

## D006 — V-only验收与GV边界（Phase 3–4）
问题：V分支是否实际起作用，以及全视觉chi2拒绝/压缩空输出是否仍一次提交。
证据：V1_01完整V运行2781次实际V更新，NIS均值0.25153、最大4.61382；增益与修正非零，完整P数值正常。phase3_full_audit.json可追溯原运行。
选择：保留sigma_V=1m/s、sigma_G=10deg及所有门限，参数候选仍0；先完成GV三开发序列后冻结，不因修正小而直接减sigma。
边界：新增真实triangulation/refinement成功而像素chi2全拒绝fixture，原生UpdaterMSCKF仍执行一次辅助更新；真实compression对2×0输入给出0行后，实际apply_joint正确提交。正常视觉含clone列时压缩不可能从正行正列得到0行，因此不伪造正常数据声称触达不可达分支。
新增PSD序测试确认增大sigma后完整posterior P不小于小sigma结果。未改变生产算法。
开发记录：新测试首编译因缺少UpdaterHelper/FeatureInitializer完整定义失败，补齐include后构建和测试退出0；失败日志保留。
否决：调大/调小sigma掩盖实现问题、为了覆盖分支修改原视觉压缩算法。

## D007 — V2_03缺帧的精确时间同步输入适配
问题：第7次完整运行在输入检查退出1：UNSUPPORTED stereo counts。完整失败保存在V2_03_difficult-GV-20261002T174649Z-a3c480eb.failed，预算照计。
证据：原ASL与ROS2 bag metadata均有cam0=1922、cam1=2336；1921对原始int64时间戳相同，未配对左图1（最早边界）、右图415。完整差集保存在phase4_sync_diagnosis.json。尚未读取任何真实ATE。
原因：离线runner假设两个CSV数量相同且同索引同步，不能处理原始缺帧；这不是辅助数学或状态损坏。
选择：使用与传感器ExactTime同步含义一致的原始int64相等配对，读取全部CSV时间戳，逐条输出未配对消息；不把不同时间戳强配、不补图、不插图、不修改源CSV/GT。全部模式使用相同规则、同一1921对完整时间范围，所有IMU仍消费。输出明确区分原始图像数、同步包数、未配对数。evaluator仍按原cam0时间支持检查99%覆盖，不放宽覆盖口径。
原生产ROS2输入检查仍拒绝已组成包中的不等header，adapter限制不变。对原先全同步序列配对及顺序完全相同，需原版/B/P精确回归后才冻结。
旧/新效果参数：全部不变，候选仍0。否决：近邻强配左右图、截去困难尾段、改CSV、只把此序列排除来绕过可修复输入问题。
同时增加runner实际读入配置JSON，正式运行检查代码/库/输入清单/evaluator冻结身份和实际sigma/开关，避免仅凭YAML宣称配置生效。

## D008 — 选择唯一初始配置（最终ATE尚未读取）
依据：三开发序列数学/数值/时序验收通过，G/V均有实际非零增益及修正；G NIS均值约0.007–0.082，V约0.252–0.956，有限且无异常强更新。V2_03较低覆盖来自源核心有效性（记录warmup_or_core_invalid），不靠降低门槛强制放行。
选择：不增加参数候选，冻结规范保守初始值，sigma_G=10deg、sigma_V=1m/s，G/V 99% NIS门限、额外quality gate OFF、warmup20及observer初值保持不变。所有序列统一配置。
旧值=新值，候选数量0/6。已确认V分支实际加入，不将小修正误判为未接线。否决降低sigma以追求未知ATE增益；用户允许没有正收益，本轮优先给出可信固定参数结果。
最终11序列只评估，不再按ATE选择参数；任何实现bug需记录、重新冻结并重跑受影响正式实验。

## D009 — 正式输入完整性审计（参数保持冻结）
正式运行发现MH_04有1张未配对左图、V1_02有1张未配对右图，均由已冻结的精确同步规则记录在各run unmatched_camera.csv；不改配对规则，不改99%覆盖门槛，不删困难时间段。
其余实际同步包和所有IMU完整消费。该发现不触发参数选择或evaluator改动；最终报告一并列出原始图像数与未配对数。

## D010 — MH_04全模式发散，保留失败并扩展复测
证据：冻结evaluator在完整共同支持上得到MH_04 B=12999.02697m、G=1162.20422m、V=11911.46926m、GV=1445.40461m；真实GT位置范围仅约19.4×17.4×3.3m。B/P末端速度约783.5m/s，P仅23个packet有接受的视觉行，G/V/GV也仅33/30/33个packet有视觉更新。状态/P保持有限且数值检查通过，但定位明显失败。
进一步证据：使用接入前保存的原版二进制/库，在与冻结runner完全相同的2032个精确同步包和20320条IMU上完整回放；audit.csv及trajectory.csv与冻结B逐字节一致。诊断用派生CSV只列同一精确同步包，原图像/IMU只读链接，未改原始数据/GT；第64次完整运行，证据MH_04_original_diagnostic.json。
结论：这是原生baseline在此固定配置下的定位发散，不是LTV disabled-path变化，也未发现新数学/协方差实现bug。仅凭当前证据不进一步断言是哪一初始化/视觉门控因素造成根因；明确观测到视觉更新稀疏和巨大惯性漂移。按冻结约束不改原初始化、参数或evaluator重跑以改善结果。
报告策略：保留全部原始ATE及11序列原始算术均值作为诊断；主表MH_04标记定位FAIL并列原始数值，不计算相对B有效改善。成功定位共同子集10/11的均值单独明确标注样本数，不冒充全11成功均值。冻结evaluator原始JSON不改、不重写GT、不增加事后优化阈值。
复测策略：原规则选出的V1_01/MH_04/V2_03保留；额外加入有效序列中GV最大相对收益MH_02（3.4154%），四条均复测B/G/V/GV。额外复测为解释失败，不用于调参。预计共80/300次完整回放；候选仍0。
