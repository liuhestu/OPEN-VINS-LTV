# EuRoC 六模式 ATE RMSE

main；11序列×6模式，已完成66/66次新完整回放。ATE单位：m。
OFF是纯OpenVINS；全部组max_slam=0、固定C0参数。G/V/L分别开启对应近似融合，L_GV三项同时开启。
SE(3)对齐，无尺度；冻结OFF完整初始化后GT支持，GT最近20ms、输出匹配1µs。独立Horn与SVD交叉核验。
此表不改变历史FAIL/STOP，不等同生产资格、性能或统计一致性。同步ASL原始流，不是ROS transport回放。

| 序列 | OFF | G | V | GV | L | L_GV |
|---|---:|---:|---:|---:|---:|---:|
| MH_01_easy | 0.185424693 | 0.185431293 | 0.185402806 | 0.185408377 | 0.185443428 | 0.185427979 |
| MH_02_easy | 0.097129729 | 0.097078384 | 0.096816983 | 0.096765776 | 0.096836719 | 0.096478773 |
| MH_03_medium | 0.149281646 | 0.149232742 | 0.149010292 | 0.148961349 | 0.149206892 | 0.148887910 |
| MH_04_difficult | 12999.026971521 | 12999.026971521 | 12999.026971521 | 12999.026971521 | 12999.026971521 | 12999.026971521 |
| MH_05_difficult | 0.276242378 | 0.276220920 | 0.275795743 | 0.275772649 | 0.276290360 | 0.275823955 |
| V1_01_easy | 0.064242834 | 0.064240869 | 0.064244019 | 0.064242164 | 0.064238598 | 0.064237676 |
| V1_02_medium | 0.068187958 | 0.068192008 | 0.068190230 | 0.068193828 | 0.068187004 | 0.068192845 |
| V1_03_difficult | 0.041890569 | 0.041891596 | 0.041890508 | 0.041891386 | 0.041890614 | 0.041891209 |
| V2_01_easy | 0.174580656 | 0.174566005 | 0.174574760 | 0.174559534 | 0.174581182 | 0.174558835 |
| V2_02_medium | 0.051975210 | 0.051978425 | 0.051973334 | 0.051977002 | 0.051975137 | 0.051976394 |
| V2_03_difficult | 0.207427615 | 0.207245141 | 0.207234970 | 0.207243632 | 0.207241376 | 0.207247877 |

## 初始化、覆盖与实际融合

| 序列 | 模式 | 状态 | 样本/基准 | G/V/L应用帧 | 条件ATE(m) |
|---|---|---|---:|---:|---:|
| MH_01_easy | OFF | VALID_FULL_MATCHED | 2762/2762 | 0/0/0 | 0.185424693 |
| MH_01_easy | G | VALID_FULL_MATCHED | 2762/2762 | 2351/0/0 | 0.185431293 |
| MH_01_easy | V | VALID_FULL_MATCHED | 2762/2762 | 0/2344/0 | 0.185402806 |
| MH_01_easy | GV | VALID_FULL_MATCHED | 2762/2762 | 2351/2344/0 | 0.185408377 |
| MH_01_easy | L | VALID_FULL_MATCHED | 2762/2762 | 0/0/2344 | 0.185443428 |
| MH_01_easy | L_GV | VALID_FULL_MATCHED | 2762/2762 | 2351/2344/2344 | 0.185427979 |
| MH_02_easy | OFF | VALID_FULL_MATCHED | 2236/2236 | 0/0/0 | 0.097129729 |
| MH_02_easy | G | VALID_FULL_MATCHED | 2236/2236 | 1499/0/0 | 0.097078384 |
| MH_02_easy | V | VALID_FULL_MATCHED | 2236/2236 | 0/1478/0 | 0.096816983 |
| MH_02_easy | GV | VALID_FULL_MATCHED | 2236/2236 | 1499/1478/0 | 0.096765776 |
| MH_02_easy | L | VALID_FULL_MATCHED | 2236/2236 | 0/0/1478 | 0.096836719 |
| MH_02_easy | L_GV | VALID_FULL_MATCHED | 2236/2236 | 1499/1478/1478 | 0.096478773 |
| MH_03_medium | OFF | VALID_FULL_MATCHED | 2289/2289 | 0/0/0 | 0.149281646 |
| MH_03_medium | G | VALID_FULL_MATCHED | 2289/2289 | 2078/0/0 | 0.149232742 |
| MH_03_medium | V | VALID_FULL_MATCHED | 2289/2289 | 0/1924/0 | 0.149010292 |
| MH_03_medium | GV | VALID_FULL_MATCHED | 2289/2289 | 2078/1924/0 | 0.148961349 |
| MH_03_medium | L | VALID_FULL_MATCHED | 2289/2289 | 0/0/1920 | 0.149206892 |
| MH_03_medium | L_GV | VALID_FULL_MATCHED | 2289/2289 | 2078/1924/1920 | 0.148887910 |
| MH_04_difficult | OFF | VALID_FULL_MATCHED | 1896/1896 | 0/0/0 | 12999.026971521 |
| MH_04_difficult | G | VALID_FULL_MATCHED | 1896/1896 | 0/0/0 | 12999.026971521 |
| MH_04_difficult | V | VALID_FULL_MATCHED | 1896/1896 | 0/0/0 | 12999.026971521 |
| MH_04_difficult | GV | VALID_FULL_MATCHED | 1896/1896 | 0/0/0 | 12999.026971521 |
| MH_04_difficult | L | VALID_FULL_MATCHED | 1896/1896 | 0/0/0 | 12999.026971521 |
| MH_04_difficult | L_GV | VALID_FULL_MATCHED | 1896/1896 | 0/0/0 | 12999.026971521 |
| MH_05_difficult | OFF | VALID_FULL_MATCHED | 1826/1826 | 0/0/0 | 0.276242378 |
| MH_05_difficult | G | VALID_FULL_MATCHED | 1826/1826 | 1638/0/0 | 0.276220920 |
| MH_05_difficult | V | VALID_FULL_MATCHED | 1826/1826 | 0/1576/0 | 0.275795743 |
| MH_05_difficult | GV | VALID_FULL_MATCHED | 1826/1826 | 1638/1576/0 | 0.275772649 |
| MH_05_difficult | L | VALID_FULL_MATCHED | 1826/1826 | 0/0/1575 | 0.276290360 |
| MH_05_difficult | L_GV | VALID_FULL_MATCHED | 1826/1826 | 1638/1576/1575 | 0.275823955 |
| V1_01_easy | OFF | VALID_FULL_MATCHED | 2780/2780 | 0/0/0 | 0.064242834 |
| V1_01_easy | G | VALID_FULL_MATCHED | 2780/2780 | 2426/0/0 | 0.064240869 |
| V1_01_easy | V | VALID_FULL_MATCHED | 2780/2780 | 0/2407/0 | 0.064244019 |
| V1_01_easy | GV | VALID_FULL_MATCHED | 2780/2780 | 2426/2407/0 | 0.064242164 |
| V1_01_easy | L | VALID_FULL_MATCHED | 2780/2780 | 0/0/2406 | 0.064238598 |
| V1_01_easy | L_GV | VALID_FULL_MATCHED | 2780/2780 | 2426/2407/2406 | 0.064237676 |
| V1_02_medium | OFF | VALID_FULL_MATCHED | 1592/1592 | 0/0/0 | 0.068187958 |
| V1_02_medium | G | VALID_FULL_MATCHED | 1592/1592 | 953/0/0 | 0.068192008 |
| V1_02_medium | V | VALID_FULL_MATCHED | 1592/1592 | 0/932/0 | 0.068190230 |
| V1_02_medium | GV | VALID_FULL_MATCHED | 1592/1592 | 953/932/0 | 0.068193828 |
| V1_02_medium | L | VALID_FULL_MATCHED | 1592/1592 | 0/0/930 | 0.068187004 |
| V1_02_medium | L_GV | VALID_FULL_MATCHED | 1592/1592 | 953/932/930 | 0.068192845 |
| V1_03_difficult | OFF | VALID_FULL_MATCHED | 1986/1986 | 0/0/0 | 0.041890569 |
| V1_03_difficult | G | VALID_FULL_MATCHED | 1986/1986 | 956/0/0 | 0.041891596 |
| V1_03_difficult | V | VALID_FULL_MATCHED | 1986/1986 | 0/931/0 | 0.041890508 |
| V1_03_difficult | GV | VALID_FULL_MATCHED | 1986/1986 | 956/931/0 | 0.041891386 |
| V1_03_difficult | L | VALID_FULL_MATCHED | 1986/1986 | 0/0/927 | 0.041890614 |
| V1_03_difficult | L_GV | VALID_FULL_MATCHED | 1986/1986 | 956/931/927 | 0.041891209 |
| V2_01_easy | OFF | VALID_FULL_MATCHED | 2166/2166 | 0/0/0 | 0.174580656 |
| V2_01_easy | G | VALID_FULL_MATCHED | 2166/2166 | 1574/0/0 | 0.174566005 |
| V2_01_easy | V | VALID_FULL_MATCHED | 2166/2166 | 0/1563/0 | 0.174574760 |
| V2_01_easy | GV | VALID_FULL_MATCHED | 2166/2166 | 1574/1563/0 | 0.174559534 |
| V2_01_easy | L | VALID_FULL_MATCHED | 2166/2166 | 0/0/1560 | 0.174581182 |
| V2_01_easy | L_GV | VALID_FULL_MATCHED | 2166/2166 | 1574/1563/1560 | 0.174558835 |
| V2_02_medium | OFF | VALID_FULL_MATCHED | 2252/2252 | 0/0/0 | 0.051975210 |
| V2_02_medium | G | VALID_FULL_MATCHED | 2252/2252 | 1606/0/0 | 0.051978425 |
| V2_02_medium | V | VALID_FULL_MATCHED | 2252/2252 | 0/1571/0 | 0.051973334 |
| V2_02_medium | GV | VALID_FULL_MATCHED | 2252/2252 | 1606/1571/0 | 0.051977002 |
| V2_02_medium | L | VALID_FULL_MATCHED | 2252/2252 | 0/0/1570 | 0.051975137 |
| V2_02_medium | L_GV | VALID_FULL_MATCHED | 2252/2252 | 1606/1571/1570 | 0.051976394 |
| V2_03_difficult | OFF | VALID_FULL_MATCHED | 1799/1799 | 0/0/0 | 0.207427615 |
| V2_03_difficult | G | VALID_FULL_MATCHED | 1799/1799 | 706/0/0 | 0.207245141 |
| V2_03_difficult | V | VALID_FULL_MATCHED | 1799/1799 | 0/637/0 | 0.207234970 |
| V2_03_difficult | GV | VALID_FULL_MATCHED | 1799/1799 | 706/637/0 | 0.207243632 |
| V2_03_difficult | L | VALID_FULL_MATCHED | 1799/1799 | 0/0/627 | 0.207241376 |
| V2_03_difficult | L_GV | VALID_FULL_MATCHED | 1799/1799 | 706/637/627 | 0.207247877 |

条件ATE只描述可用片段；失败或覆盖不足不按完整结果计入。没有实际应用帧的分支明确为未生效，不能仅凭开启标志称融合成功。

## 运行身份

二进制源码：`a18f86d9aab0da95f1ca7a86d377425d71857b11`；原main：`300198d08d057d9fb9a0029df6fdb67603722e99`。
原始输出：`/home/he/output/ltv_euroc_six_20261008T172854`（本机路径，不是公开下载）。
[结构化结果](euroc_results_data/results.csv) · [完整结果与失败原因](euroc_results_data/results.json)。

## 结果解释与验收边界

66/66 为完整输入、初始化时间相同且冻结 OFF 评估支持覆盖 100% 的**匹配有效性**，不是精度或工程验收 PASS。Horn/SVD 最大差异为 3.638e-11 m。

MH_04_difficult 六组均为 12999.026971521 m；OFF 轨迹末端位置模长约 42.9 km，六组 G/V/L 实际应用次数均为零。按用户提供的原因，初始静止导致 OpenVINS 基线发散；本次报告更新将此序列排除出收益与不退化汇总，评估范围为其余 10 序列 × 6 模式 = 60 组。该排除在回放后由用户指定，不追溯修改冻结协议、原始 ATE、运行状态或历史结论；原因未在本轮独立验证。MH_04 的六组相同不作为融合不退化证据。本轮没有改参数重跑。

| 目标 | 本轮结论 |
|---|---|
| 预测增量收益 | 冻结的 5% 门槛和历史 FAIL 保留；本轮没有重做 shadow 预测检验。 |
| 工程融合不退化 | 本轮完成真实 ON 的匹配 ATE 对照；尚未完成原冻结的全部不退化、尾部风险和性能验收，不授予 PASS，也不追溯改写历史 STOP。5% 预测门槛不作为工程 ON 的唯一前置条件。 |
| 实际主状态收益 | 有微小 ATE 改善，但方向不一致；排除 MH_04 后，L_GV 比 OFF 低/相同/高分别为 6/0/4 序列，最大相对改善 0.670193%，最大相对退化 0.007167%。单次固定回放不足以证明稳定、统计显著收益。 |
| 相关性模型缺口 | 本次 ATE 矩阵未补齐预测量与视觉更新的交叉相关、相关噪声的一致性、landmark 复用信息的协方差及完整 NEES/NIS 证据；保留原有缺口，不因完整回放宣称解决。 |

排除 MH_04 后的 10 序列描述性比较（较 OFF 的 ATE）：

| 模式 | 降低 | 相同 | 升高 |
|---|---:|---:|---:|
| G | 6 | 0 | 4 |
| V | 8 | 0 | 2 |
| GV | 7 | 0 | 3 |
| L | 6 | 0 | 4 |
| L_GV | 6 | 0 | 4 |

所有六组均关闭原生 SLAM 特征（max_slam=0）；L 是 LTV landmark_approx，不能据此推断原生 SLAM 开启后的表现。本轮没有为跨过 5% 优化 shadow 指标。

## 可核验材料与复现

[协议](protocol.json)、[完整 66 次索引](full_index.json)、[82 次构建/测试/回放尝试](run_manifest.json)、[简表](run_manifest.csv)、[输入内容哈希](data_identity.json)、[运行身份](identity.json)、[独立驱动构建与依赖哈希](runner_build_identity.json)、[完成审计](completion_audit.json)、[归档文件 SHA256](sha256.json)。

实际驱动源码为 a18f86d；其库来自本任务独立构建的 b770d6f，并在驱动重建前校验包源码闭包未变，复制到独立产物后验证库哈希与链接路径。没有使用共享旧 overlay。原算法源码、数据集 YAML、共享 C0 参数与本轮起点 main 300198d 相同；残差、Jacobian、参数、门控和更新顺序未改。

初次驱动构建因 provenance 输出目录缺失退出 1，已保留尝试；修复后重建退出 0，失败产物未用于回放。12 次 40 s 烟测均正常退出，但初始批次校验器错误要求消耗全量数据；[原索引](smoke_index.json)及[限定前缀后重新审计](smoke_corrected_audit.json)同时保留。L_GV 在 V2_02/V2_03 烟测分别有 397/188 帧同时实际应用三分支。

完整回放命令保存在 run_manifest.json 的 command_expanded，每次使用唯一输出目录、单线程、独立安装路径及输入内容标识。本机原始目录已结束全部任务，已登记进程无存活。

在已有相同协调目录与独立构建产物上运行批次（索引有完成记录时跳过，重新实验应创建新目录）：

```bash
python3 docs/experiments/tools/ltv_three_track/run_euroc_six.py /path/to/coord --stage full --data /path/to/ASL
OPENBLAS_NUM_THREADS=1 python3 docs/experiments/tools/ltv_three_track/evaluate_euroc_six.py /path/to/coord docs/euroc_results.mc --data /path/to/ASL
```

协调 launcher、干净环境入口、源码冻结工具与构建规格保存在 coordination_tools/ 及 build.json、driver_fix_build2.json。目录内绝对路径是原机器证据引用；换机器需要按新位置准备协调目录、源码和构建产物。
