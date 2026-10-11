# 来源与必要交叉块

主误差 true-minus-estimate，Observer/seed/anchor 为 estimate-minus-true；原生JPL及frame检查见 [seed报告](../../landmark_correlation/actual_seed/report.md)。同一物理来源只登记一次，Observer同帧bearing子步合成B而不重抽。IMU offset六源贯穿IMU/camera；每epoch/local-ID的上一camera anchor保留其物理时间，退休/reset/epoch替换失效。

| 来源/块 | 当前证据 | 统计资格 |
|---|---|---|
| actual seed输入pose/camera/bearing索引与J | 原生Jacobian和受控完整源映射 | 真实posterior历史pose-bearing cross UNKNOWN |
| 主clone完整P | 现有生产边缘及clone交叉保留 | 不提供历史噪声与main/Observer joint |
| IMU校正acc/gyro单位源 | 逐步F/B、持久offset map | 原始插值端点、bias/calibration噪声cross UNKNOWN |
| 同帧bearing源 | epoch/localID/time、完整合成B | nominal gain固定；gain敏感度实测差异约2.94% |
| conditional Sigma_aa/tt/at（含跨点） | 完整可解码源maps及非零跨时刻块 | 仅已知条件贡献，非完整Sigma |
| C_xa/C_xt、LTV-visual cross | UNKNOWN | 不能置零或构造真实R/N/S/K |
| Riccati P_R | 原有Euler/谱处理保留、OFF精确 | 不是未经证明的Cov(estimate−true) |
| 独立landmark真值 | UNAVAILABLE | 真实统计校准NOT_EVALUATED |

主视觉nullspace/compression、EKF校正与JPL注入/reset、clone扩增/边缘化的完整来源映射尚未实现；不得宣称闭合。逐帧来源stream和post-frame global/localID/slot receipt、sha、路径见 [证据索引](../evidence_index.csv)，数学/生命周期范围见 [审计](../../landmark_correlation/observer/model_audit.md)。成功完整runner遇异常即停止；attempt日志通过receipt支持核对，不伪称applied。未来catch-and-continue需transaction lineage与commit/abort marker。

Landmark实际相对残差及其主状态原生JPL/FEJ Jacobian尚未实装核对，GO前仍须补齐；seed Jacobian及归一化小系统H不替代该前提。
