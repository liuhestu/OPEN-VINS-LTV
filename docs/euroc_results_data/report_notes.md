## 结果解释与验收边界

66/66 为完整输入、初始化时间相同且冻结 OFF 评估支持覆盖 100% 的**匹配有效性**，不是精度或工程验收 PASS。Horn/SVD 最大差异为 3.638e-11 m。

MH_04_difficult 六组均为 12999.026971521 m；OFF 轨迹末端位置模长约 42.9 km，六组 G/V/L 实际应用次数均为零。此序列存在严重估计发散，按实测保留；不能将六组相同解释为融合不退化证据，也不删除此序列以美化均值。发散原因尚未定位，本轮没有改参数重跑。

| 目标 | 本轮结论 |
|---|---|
| 预测增量收益 | 冻结的 5% 门槛和历史 FAIL 保留；本轮没有重做 shadow 预测检验。 |
| 工程融合不退化 | 本轮完成真实 ON 的匹配 ATE 对照；尚未完成原冻结的全部不退化、尾部风险和性能验收，不授予 PASS，也不追溯改写历史 STOP。5% 预测门槛不作为工程 ON 的唯一前置条件。 |
| 实际主状态收益 | 有微小 ATE 改善，但方向不一致；L_GV 比 OFF 低/相同/高分别为 6/1/4 序列，最大相对改善 0.670193%，最大相对退化 0.007167%。单次固定回放不足以证明稳定、统计显著收益。 |
| 相关性模型缺口 | 本次 ATE 矩阵未补齐预测量与视觉更新的交叉相关、相关噪声的一致性、landmark 复用信息的协方差及完整 NEES/NIS 证据；保留原有缺口，不因完整回放宣称解决。 |

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
