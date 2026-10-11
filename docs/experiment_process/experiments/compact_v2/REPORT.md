> 验证已按用户要求停止，源码仍在编辑。本目录保留阶段性证据：精度回放已完成50组，独立串行测量完成3组；尚未完成最终审计，以下为较早的阶段汇总。

# compact_v2 评估

冻结权重；10条序列×5模式。ATE使用原冻结 OFF 的相同时间支撑，SE(3)对齐、不缩放。
原谱裁剪/Euler实现归档在 prechange；本轮变更不证明统计一致性。

| 序列 | OFF ATE(m) | G | V | L | GVL |
|---|---:|---:|---:|---:|---:|
| MH_01_easy | 0.185425 | 0.183867 | 0.182885 | 0.185451 | 0.182897 |
| MH_02_easy | 0.097130 | 0.096063 | 0.086582 | 0.096266 | 0.085965 |
| MH_03_medium | 0.149282 | 0.148539 | 0.133806 | 0.149011 | 0.133507 |
| MH_05_difficult | 0.276242 | 0.276290 | 0.252662 | 0.276376 | 0.253094 |
| V1_01_easy | 0.064243 | 0.064241 | 0.064247 | 0.064225 | 0.064237 |
| V1_02_medium | 0.068188 | 0.068192 | 0.068192 | 0.068183 | 0.068191 |
| V1_03_difficult | 0.041891 | 0.041891 | 0.041889 | 0.041890 | 0.041891 |
| V2_01_easy | 0.174581 | 0.174566 | 0.174547 | 0.174582 | 0.174561 |
| V2_02_medium | 0.051975 | FAIL/PENDING | FAIL/PENDING | FAIL/PENDING | FAIL/PENDING |
| V2_03_difficult | FAIL/PENDING | FAIL/PENDING | FAIL/PENDING | FAIL/PENDING | FAIL/PENDING |

相对本轮 OFF 的变化（负数改善，正数退化）：

| 序列 | G | V | L | GVL |
|---|---:|---:|---:|---:|
| MH_01_easy | -0.840% | -1.370% | +0.014% | -1.363% |
| MH_02_easy | -1.099% | -10.859% | -0.890% | -11.495% |
| MH_03_medium | -0.498% | -10.367% | -0.181% | -10.567% |
| MH_05_difficult | +0.017% | -8.536% | +0.048% | -8.380% |
| V1_01_easy | -0.003% | +0.006% | -0.027% | -0.008% |
| V1_02_medium | +0.006% | +0.006% | -0.007% | +0.005% |
| V1_03_difficult | +0.002% | -0.004% | -0.002% | +0.001% |
| V2_01_easy | -0.008% | -0.019% | +0.001% | -0.011% |
| V2_02_medium | FAIL/PENDING | FAIL/PENDING | FAIL/PENDING | FAIL/PENDING |
| V2_03_difficult | FAIL/PENDING | FAIL/PENDING | FAIL/PENDING | FAIL/PENDING |

完整 ATE/RPE、覆盖率、尾段、更新次数见 [metrics.json](metrics.json)，10秒局部窗口见 [local_windows.json](local_windows.json)。
拒绝原因、NIS和执行状态见 [accuracy.json](accuracy.json)；独立串行开销见 [performance.json](performance.json)。
串行使用固定 CPU 0,1；OpenCV/BLAS单线程；每组一次独立运行，未估计计时置信区间。V1_01姿态真值来源限制沿用历史报告。
