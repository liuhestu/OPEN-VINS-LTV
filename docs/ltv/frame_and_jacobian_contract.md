# G/V 坐标与 Jacobian 合同（Phase 0 原生数值验收通过）

> 2026-10-03：本次重新执行的结果和当前工作区范围见 [Phase 0 复核](phase0_recheck.md)。下文保留早期验收快照；其中“尚未接入”等描述仅适用于该历史阶段，当前实现状态见工程验收与最终报告。

以下合同已通过本轮 T1/T2/T5 原生测试。G/V 预测与 H 仍是测试参考公式；
生产 UpdaterLTV 尚未接入，其后续验收不能由本轮结果替代。详细范围与指标见 test_matrix.md。

令 `R=R_GtoI`，主 IMU 误差顺序 `[theta,p,v,bg,ba]`。
本地 `IMU::update()` 使用归一化 `[delta_theta/2,1]` 左乘 JPL quaternion；
结合 `quat_ops.h`，一阶旋转为 `R+ = (I-[delta_theta]x)R`。

冻结 observer 测量后，以 `res=z-h(x)` 为残差：

```
h_v = R v
H_v = [ [R v]x   0   R   0   0 ]
h_g = T^T R gamma
H_g = [ T^T [R gamma]x   0   0   0   0 ]
```

`H` 是预测函数导数，固定测量的 residual 导数为 `-H`。
`gamma` 是合同指定的世界重力向量或单位方向，必须与测量定义保持同一尺度；
二维切平面基 `T` 在差分期间固定，不对其重新求导。
FEJ 模式 residual 使用 current，H 使用 FEJ 的 R/v。不能对 current residual
直接差分来声称验证不同 FEJ 点的 Jacobian；需在相应线性化点独立差分预测函数。

Propagator 名义加速度为 `R.transpose()*a_corrected - (0,0,+g)`；
物理重力为 `(0,0,-g)`，静止时 `a_corrected + eta = 0`。
本地相机模型为 `p_C=R_ItoC*p_I+p_IinC`，observer 使用
`R_BC=R_ItoC.transpose()`、`p_BC=-R_BC*p_IinC`。

固定标定调用 `State::Dm()` / `State::Tg()`；不重新猜测 KALIBR/RPNG 排列。
先算 `a_hat=R_ACCtoIMU*Da*(am-ba)`，再算
`w_hat=R_GYROtoIMU*Dw*(wm-bg-Tg*a_hat)`。bias 只扣一次。
adapter 需左右包围样本，对校正后端点平均；禁止外推。

已验证 yaw 基姿态与速度块为 `R gamma`、`-[v]x gamma`，故瞬时速度约束满足
`[Rv]x R gamma + R(-[v]x gamma)=0`。平移不出现在 G/V 预测中。
瞬时零空间恒等式不能代替跨帧真实 Phi、clone 增广和视觉 FEJ 检查。

后续紧凑 H_order 应保留 visual order 并追加缺失的 IMU q/v 块；
通过 StateHelper 对完整协方差联合更新，不另建 q/v 小滤波器。
共享输入相关项属于未建模近似。辅助噪声须独立门控并检查合法性，
混合单位的联合矩阵不再次视觉压缩。
