# 已实现与缺失的联合误差模型

实际离散映射、Riccati P_R与真实Sigma的区别、共享输入、生命周期、条件anchor和真实必需块见 [Observer数学审计](../landmark_correlation/observer/model_audit.md)。[总实验合同](../landmark_correlation/experiment_contract.yaml)、[seed合同](../landmark_correlation/actual_seed/experiment_contract.json)、[参考合同](../landmark_correlation/reference/experiment_contract.json)分别限定物理来源、控制条件与统计检验范围。

本轮实现实际Observer调用下的只读误差源传播，不改变原估计均值。其共享offset源跨IMU与camera延续，bearing在Euler子步中不重抽；一般联合映射保留旧误差/源D交叉项。保存滚动上一camera的条件anchor源贡献，产生可复核非零跨时刻与跨点块。Matrix流和receipt身份使测试不只停留在公式。真实main/seed/bearing/visual联合块仍UNKNOWN，真实valid_unconditional恒false。

主误差定义true-minus-estimate；seed/Observer/anchor为estimate-minus-true。seed原生JPL符号和frame映射通过有限差分核对。有限维参考的R/N/S与广义Joseph通过直接源映射和独立SVD核对，真实分布不由该参考自动成立。输入保持由生产路径计算，GT仅进入离线评价。

缺失项、逐LC判定与最小扩展见 [STOP判定](landmark_go_stop.md)。主视觉更新、clone扩增/边缘化、主误差注入/reset的完整联合来源映射未实施，不能宣称阶段C数学闭合。真实landmark校准没有独立真值，仍NOT_EVALUATED。
