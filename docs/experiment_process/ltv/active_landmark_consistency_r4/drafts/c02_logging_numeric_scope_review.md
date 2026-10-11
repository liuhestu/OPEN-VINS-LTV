# C02 logging ON/OFF 数值证据边界

独立只读复核 attempt 43：账本指向所选 runtime `943573c...` 的 `replay_ltv_feature_cache`、`C02_V2_03_01` 原配置及原 cache，退出码 0。原配置 `ltv_log_enabled: true`；重放程序加载该配置后明确令 `options.log_enabled=false`。结果 `C02_V2_03_exact_cache.json` 为 PASS：23370 个 IMU 事件、1806 process、115 pause，共 1921 个比较事件，`exact_non_wallclock`。

重放从新建 adapter 开始顺序消费缓存输入，并实际递推状态，没有从 expected 后验安装状态。每个 process/pause 都将重新序列化的 evidence 与原 expected 字节精确比较，包含完整 x、P、feature/core slot 映射、ready/管理和 active consistency 诊断；不比较墙钟计算时长。这支持以下明确结论：

> 在该完整 V2_03 序列固定缓存桥接输入下，C02 原 native 日志开启运行的 LTV 数值与事件输出，与完整缓存重放关闭 adapter 日志配置的输出逐字节一致。

这不是第二次 native logger OFF 实验。`log_enabled` 的实际文件开关在 VioManager 构造与 `record_ltv` 中，缓存重放只构造 adapter；原缓存证据在相机 process 后、`record_ltv` 前记录。缓存提供后续原桥接输入，因而不能独立排除假想的 logger 对主 VIO/前端、后续桥接输入或最终相机事件之后状态的影响。这里没有发现此类副作用，只是重放边界无法验证它。输出 features.jsonl 与缓存记录本身也并未整体关闭。

**判定：43 足以支持固定输入下所选 C02 的 LTV 数值等价，不足以单独把原 goal“日志开启/关闭不改变数值”解释为整个 native 主估计器的已实测 ON/OFF 等价。**验收表若采用 adapter 口径，必须明确上述边界；不能称为第二次 native OFF 完整执行。

若需无歧义补齐完整 native 口径，最小额外验证为一条代表 C02 完整序列，保持输入、runtime、候选、主配置及审计/缓存相同，仅将 `ltv_log_enabled=false`，与已有 ON 的主 audit/trajectory 及缓存非墙钟数值事件精确比较。计入 1 次真实完整预算，使用独立工程身份，明确唯一 logger 配置差异，不修改冻结科学候选或阈值。本审查仅提出补证方式，未创建该配置或执行任务。
