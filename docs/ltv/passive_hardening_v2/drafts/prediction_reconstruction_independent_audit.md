# 缓存预测均值重构独立审查

对象：diagnostics/export_prediction_cache.cpp、reconstruct_prediction.py、test_prediction_math.py及276输出。仅只读受测实现，另做当前receipt唯一性离线277（exit0）；未运行observer/full，未修改276工具或数组。

## 可支持的合同

预测均值使用完整旧x构造A/B后一次Euler更新，landmark导数-W×ell-v、v导数-W×v+eta+a、eta导数-W×eta，与core源码一致。标定顺序先am=Racc Da(rawam-ba)，再wm=Rgyro Dw(rawwm-bg-Tg am)，和Adapter一致；使用当前事件缓存标定/bias，按实际已到达IMU左右端点插值、逐子区间端点校正后取中点。不用未来相机状态回填历史，不再次求P/Riccati/增益/准入。

decoder只解析已存原输入、后状态、slot/管理观测及birth均值，P仅被解析跳过；没有调用Adapter.process或重做观测器。previous旧slot取预测路标，当前postslot取后状态，同ID关联，可避免slot重排后用错分量。R_BC bearing与ell-p_BC方向转换符合现有预测角定义；旧点ray>=0.1m才进入角分位数，出生点排除。

276检查1805相机事件，1472个scalar诊断有效；115个pause和首个后状态1次跳过。live速度/重力校正率最大差0，P95角最大差1.337788e-13，小于预设1e-9；重构valid与live validity逐包相同。缓存末6维均值与raw live v/eta精确相等。这支持当前已覆盖序列的scalar诊断对齐。

277独立核验：1921个原始整数camera_ns唯一、forward转换float键也唯一，1805输出诊断的时间/epoch/raw-current均和整数receipt对应；证据为输出目录independent_receipt_audit.json。只是当前20Hz/缺帧数据的关联证明，不证明float反转能恢复原整数，也不证明任意相邻ns输入不会键碰撞。

## 不能越过的解释边界

- signed Δv/Δeta是按已审旧值公式重构的post-minus-predicted向量；norm/dt与live交叉核对。范数相同不单独证明向量方向，不能称为原live signed分量逐值oracle。
- perpoint bearing_residual_body是predicted unit ray减measured body unit bearing，是单位方向差；**不是**core的投影innovation z-Cx，也不是逐点对共享v/eta的独立贡献。P95一致也不能推出所有逐点残差被独立验证。
- birth premean来自实际缓存seed（apply_seed）或零，与core生命周期重建及均值hook源码合同一致；未有独立pre-camera快照逐点核验。出生不进入scalar P95，不能以P95对齐证明出生Δ正确；只作标注后的辅助描述。
- pause/首状态被明确跳过，不能宣称全1921事件都有预测重构；无有效scalar的333个检查事件也不能用rate/P95对齐替代其全部向量正确性证明。当前只有一个epoch，跨epoch/重新bootstrap后的复杂路径未由此序列覆盖。
- 原test_prediction_math验证旧值Euler和简单位姿标定/插值；其标定矩阵为单位阵、Tg为零，尚不是非交换旋转/比例/Tg联合数值反例。当前标定顺序结论另由源码对照支持，不把单元测试范围夸大。
- decoder按既有可信cache格式解析核心前缀，不是新通用完整二进制格式验证器；使用前的cache完整性/全数值证明来自此前固定身份审计。不得把276作为对任意修改缓存的安全校验。

结论：当前1805事件可用于明确标为“离线均值重构”的signed校正与逐点方向差诊断，并以scalar精确对齐为交叉证据；不得当作新增精度结果、重做滤波的反事实，或未观测birth premean的独立证明。此边界已同步主线程及manager/geometry。
