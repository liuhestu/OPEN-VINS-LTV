# A1 实际接入与固定方案

基线为 2417d036002fbfd542ae71d96d0fc734cd6d47e8。独立分支工作目录无起始未提交改动。历史四模式源码 16f02e3，仅标 HISTORICAL_REUSED；本轮运行不得继承历史资格。

G 使用 IMU body 重力单位方向与主状态 R_GtoI gamma、gamma=[0,0,-1] 的切平面二维残差；H_theta=T^T skew(R_fej gamma)，R=(10 deg 转 rad)^2 I。V 使用 body 速度 snapshot.velocity_body-R_GtoI v_G，H_theta=skew(R_fej v_fej)、H_v=R_fej，R=(1 m/s)^2 I。符号遵守原生 JPL 注入；现有 Jacobian、FEJ、联合布局/均值/协方差测试需本轮构建后重跑。

Updater 严格匹配 camera/IMU cursor、epoch/sequence 和 native prior，readiness 按确认20，G angle≤30 deg、NIS 9.21034（2维）、V NIS 11.34487（3维）。质量门、Huber 保持关闭。Observer C0 q=1e-4,V=1e6,P0=1、30/15 features；stereo-then-temporal seed、hardening/active consistency保持历史配置。Landmark ON不存在；max_slam=0；标定固定。辅助与视觉堆叠一次 EKF；未知主/LTV/视觉、G/V交叉块仍按 independence_approximation，统计一致性未成立。

所有模式共用 config/ltv_three_track/task_a/fixed_c0，只有runner显式覆盖G/V开关；生产配置未改。参数字节身份见 fixed_parameters.json。开发 V2_02/03，留出 V1_01/03。历史作用约微度/微米/微m/s，readiness调整10曾无ATE改善；当前证据不支持权重扫描。先以固定候选解决“当前源码完整四模式与生产诊断性能”关键缺口，若没有稳定收益则停止调参。

原 run_ltv_gv_evaluation 强制 passive cache/audit/full matrices。新增 run_ltv_gv_production 编译目标只关闭研究instrumentation，保留生产scalar ltv.csv日志。Updater原算法的全P谱检查、clone快照、第二次K诊断计算仍保留并计入feed_camera_ms。两个runner同源，生产目标外部记录完整state/P digest、tracker/visual digest及Observer x/P_R digest，均在camera计时外；不将digest变为模型校准。

验证顺序：原runner OFF完整精确历史回归→生产runner OFF与原runner逐帧完整state/P/Tracker/visual及Observer digest精确一致→生产四模式完整回放。生产标量日志可重复读取 readiness、innovation、作用norm；拒绝/accepted/applied不得混淆。性能p95只计feed_measurement_camera，RSS /usr/bin/time记录整个进程；这授予离线同步输入范围而非ROS transport queue资格。生产性能需统一诊断等级且独占资源。

最初怀疑audit=false时diagnostics不重置；进一步调用链核对否定：VioManager.cpp Observer处理前无条件重置，提前返回走pause_ltv也重置，因此没有因此修改主代码。生产runner计数严格要求consumed、joint_applied、ekf_calls=1及有效branch行；真实回放继续核验日志和计数。算法状态精确回归若失败先排除构建/overlay，不能放宽零容差。
