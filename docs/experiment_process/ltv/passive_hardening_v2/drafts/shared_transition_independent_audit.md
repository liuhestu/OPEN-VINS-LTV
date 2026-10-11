# 共享状态校正传输独立审查

只读审查geometry修复后的shared_transition.py及四项数学/时间测试，未改其源码。新增独立analysis284 exit0，无observer/Riccati/GT调用。旧280按错误窗口起点产生的分区统计必须保留并标TOOL_WINDOW_ORIGIN_ERROR；本审查仅认可修复后的_02，不用旧分区值解释预登记窗口。

## 时间与窗口

origin现在取首次initialized receipt的物理imu_time；行保留camera time用于缓存关联，新增imu_time=target，relative_time=target-origin。284逐行验证此关系。实际origin=1413394887.3557606，首原始camera=1413394881.6057606，相差5.75秒；本数据全部camera/IMU offset为0。geometry的带.016偏移前缀反例支持坐标工具逻辑，不宣称任意非零offset真实数据已跑验。

固定区间保留[5.80,13.95]及[72.15,81.70]，沿原1e-6浮点端点容差选行：分别164/192行，实际边界5.799999952→13.950000048和72.150000095→81.699999809。首行都已有完整1秒历史，贡献起点分别4.850000143、71.199999809，早于分区起点，证明统计没有在分区边界重新清历史。

每次使用(t-1s,t]贡献集合，先传输已有项、删过期、再追加当前Δ。complete_1s从真实分段起点计时；pause、epoch/bootstrap变化、无current、missing重构或物理camera间隔超0.2会清段。IMU子步仍≤0.05；这些是原observer物理间隔界，不是离线ready episode的0.075秒采样线。合法0.1秒camera稀疏期间可由真实IMU连续传播已有Δ，不创造新的camera校正或ready样本。

## 数学与数值

F的对角块为I-dt[ω]×，η→v块dtI；每子步Phi=F_new Phi_old，按真实顺序左乘。历史校正先被当前Phi传输到同一当前body坐标再求signed sum；η校正累积进入速度，不能把原始不同时间body向量直接相加。加速度为仿射项，在状态差传播中抵消；当前校准/bias仍影响ω，按原顺序使用。

geometry测试覆盖非交换次序、η→v、公共坐标旋转及初始化prefix。284另外用非单位Da/Dw、非交换Racc/Rgyro、非零Tg/bias、插值端点和30组随机状态独立检查 `predict(x+delta)-predict(x)=Phi*delta`，最大差8.881784e-16（门槛2e-14）。这补足仅检查η齐次项无法单独验证v交叉块的缺口。

## 解释限制

结果是固定已发生校正向量的线性预测传输/抵消描述，不重算P、增益、准入或校正，不是移除某个点后的反事实滤波。sum_vector、norm_of_sum与sum_of_norms可描述方向抵消，不能据此证明准确性、放宽ready或分解某个点对共享状态的因果贡献。只有完整1秒窗进入分区统计；跳过段仍在breaks记录。1s持续长度不等于期间始终信息充分，合法稀疏采样不被误标为连续ready。

证据：`/home/he/output/ltv_passive_hardening_v2/diagnostics/R3B_V2_03_shared_transition_02/independent_audit.json`及账本284。未运行任何full。
