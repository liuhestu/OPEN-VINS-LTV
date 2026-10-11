# 集成裁决：NOT_RUN，无候选取得组合准入

A 的 G/V/GV 和 B 的 landmark 近似 ON 均在 V2_03 冻结局部 ATE 门槛失败，并经相同源码/参数完整重复及独立评价确认。没有独立工程候选可进入组合准入，因此未创建或运行集成 worktree，不合并或启用算法。留出集未用于调参；未执行项不记 PASS。

B 已真实接入原有视觉更新，以同 prior/一次原生 EKF 完成独立 landmark ON；该作用经过完整回放与矩阵重建确认，但工程结果失败。hybrid 当前 q/p 与 whole anchor 布局的 G/V metadata 单步测试仅证明接口可合并，不授予组合资格。

C 交付有限条件模型及相关求解核，生产相关版本 C4 STOP；B 未消费未校准 cross，也不继承其理论资格。所有生产默认保持 OFF。

最小后续是预登记完整轨迹 counterfactual，定位辅助引起的视觉增益/协方差及离散选择反馈；同时补记 finite current/anchor mean/identity，使用 GT 两姿态做复合残差诊断，补初始化与共享源模型。任何新候选都须新版本、受影响 OFF/ON 检查、独立留出和组合验收，不能从本轮结果继承资格。
