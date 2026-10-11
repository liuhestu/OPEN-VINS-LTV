# 两点 past-fit/current-holdout 独立审查

只读审查 `export_holdout_context.cpp`、修正后的 `past_fit_holdout.py`、四个fixture源码及305/306输出。没有另做拟合、读取GT、扩大到全域或登记候选。

**源合同可以支持这两次条件性观测/状态诊断。结果不支持把两点统一归为“LTV点估计不准而当前观测可靠”，也不能直接判定前端错配。**

## 排除与坐标合同

当前源严格选择物理时间 `[t-1.0,t)`，同epoch、同ID、match-valid的cam0观测。当前cam0不在过去集合；cam1全部不入拟合。`fit`的A/rhs/point只使用past centers/directions，当前bearing只在点求解之后用于heldout角和沿ray/横向差异报告。改变当前ray不会改变解，已有fixture针对这一点；也覆盖非零相机杆臂/旋转、future排除、缺克隆、退化和窗口边界。

每个过去bearing只查**当前target那一份context的poses**，不读取该过去包的pose均值补缺。当前execution pose时间须对应context.time；位姿列表检查有限及唯一。世界相机中心 `p_WB+R_WB p_BC`，世界单位方向 `R_WB R_BC normalized(z)`，点回body用 `R_WBᵀ(X-p_WB)`，与既有坐标约定一致。随后比较两个body点的差时杆臂相消，投影轴使用当前body bearing，没有把world方向直接与body差相乘。

302使用1.000000001窗口宽度违反事前合同，应保留为TOOL_HISTORY_BOUNDARY_ERROR。修正源将固定窗宽与clone匹配1e-9容差分开，305生成support_02，306仅使用新support。新增窗外5e-10反例防止把匹配容差当作扩窗许可。旧support不能替代新输入证据。

## 实际可用支持和秩

候选历史窗是1s，不代表拥有1s完整克隆支持。ID27537/29636分别有20/18条过去观测，其中9/7条当前克隆已不可解；逐项列出缺失，不补旧版本pose。实际各用全部11条可解析射线，跨度都为0.5s。满足至少两条及原 `λmin > 1e-12 λmax` 数值秩约定；没有新增有利condition门槛，也没有按残差挑子集。

|ID|实际baseline m|A特征值|condition|过去角中位/最大 rad|当前holdout角 rad|LTV pre角 rad|
|---|---:|---|---:|---|---:|---:|
|27537|0.447313|0.0139450 / 10.9861201 / 10.9999349|788.807|0.001914 / 0.002366|0.350263|0.349214|
|29636|0.203751|0.0200934 / 10.9805103 / 10.9993964|547.414|0.104601 / 0.175399|0.297001|0.163584|

两点过去及当前signed ray均正。`FIT_DIAGNOSTIC_ONLY`仅表示数值解可记录，不是可靠seed或静态点模型验收。秩合格不等于不确定度小；尤其29636过去残差本身很大，不应把其拟合点当真值。两个condition必须保留为几何诊断，不能仅凭数值可解称“充分激励”。

## 对机制的有限判断

27537过去11ray在当前VIO pose模型下内部角残差很小，而过去拟合预测和LTV pre都与当前ray相差约0.35rad。该现象不符合“只有LTV点偏了、过去拟合却预测当前ray良好”的模式；它支持检查当前观测与过去静态点/位姿模型的不相容，但不能唯一归因为当前追踪错配。

29636过去拟合本身不满足小残差，当前holdout也很大。它可能涉及观测关联、pose先验误差或静态点假设失效等，当前数据不能区分。不能拿拟合深度替换LTV深度，再声称修复其估计。

两项均只使用OpenVINS已有pose均值，且pose与同一前端历史相关。**从拟合射线集合排除当前ray，不代表生成pose先验的主估计器从未受当前/历史观测影响。** 因此不是统计独立验证、外部GT或标定后的失败概率。当前工具不传播完整joint pose covariance；未声称独立协方差块或置信区间，符合均值诊断范围。

## 剩余工程边界

当前exporter只导出目标邻域并按固定camera-time匹配目标；它不是任意ns通用输入审计器。现两目标与已核验整数receipt对应，未来复用必须保留明确receipt/物理时间和source身份，不能用最近邻。源码的通用数据防御不等于原SeedEstimator全部输入验证（例如determinant、外参和bearing有限性）；本次使用已冻结、经过桥接校验的两份context，无凭据把它提升为生产seed API。

完整结果及missing列表位于 `/home/he/output/ltv_passive_hardening_v2/diagnostics/R3B_holdout_fits_01/results.json`，输入为 `R3B_holdout_support_02/support.json`。实际校正贡献297/298与这些holdout结果可以并列解释，但仍不是删除点反事实或在线剔点授权。R3候选额度及科学门槛不变。

## match_valid 与下一机制边界补审

源码两处 `VioManager.cpp` 桥接均从原tracker数据库的当前timestamp条目构造 `HistoryObservation`，设置ID/camera/bearing，使用 `LtvFeatureHistory.h` 中默认 `match_valid=true`。这没有提供独立的逐帧静态几何验收。当前代码和已读合同也没有证据表明存在一个应传入却被覆盖/漏传的false标志，或旧承诺要求该位包含本次holdout检验。**默认true不能被解释为“真实匹配已获证明”，缺少新检查也不能反过来定义成旧合同实现bug。**

因此当前不能以修复桥接为名加入新门限。若另找到已存在的tracker拒绝标志被错误恢复为true，才可用精确反例讨论原语义修复；本次尚未找到这种证据。新增在线过去拟合—当前观测一致性筛选若改变H中的观测、权重、活动点退出或种子，是新的主要观测/估计机制，不能利用R3剩余一个同机制位置开启第四轮。即使只输出新健康诊断，也不能自动继承准确性，需独立固定合同；本次没有批准这样的新候选。两点条件性诊断并不等于任何特定新Gate已经获得充分机制支持。

## 独立数值复核307

另经统一账本 analysis307 exit0，`tests/audit_holdout_saved_point.py` 对保存的point_W重新计算全部11条过去ray及当前holdout残差，没有重新求点（new_point_solves=0）。从原导出contexts独立重建严格1s观测选择、同ID/cam0/valid过滤、当前版本clone归属及missing列表；逐项核对保存centers/directions，未用旧版本pose。27537/29636分别采用version1409/1452，实际支持约target−0.55至target−0.05 s，跨度0.5 s。

与保存结果最大差分别5.34e-13和4.20e-15；两个heldout角逐值相同。结果在 `R3B_holdout_fits_01/independent_saved_point_audit.json`，含输入及脚本SHA。这里的“独立数值复核”仅指给定保存point_W的支持与预测残差，不是另一个三角化求解器，也不是独立点真值；前面的全量源码审查范围与这项数值复核范围明确不同。
