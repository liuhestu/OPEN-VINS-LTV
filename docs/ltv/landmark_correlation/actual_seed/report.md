# 实际 seed 联合映射与受控扰动

实际 `LtvSeedEstimator::estimate` / `LtvSeedUncertainty` 已执行，保持生产均值算法不变。三种姿态、两种步长（1e-5/1e-6）用原生 `PoseJPL::update` 扰动历史/执行姿态，外参使用 seed 接口规定的右旋转与 body 平移，bearing 使用单位球切平面归一化扰动。最大 Jacobian 元素差为 6.97113e-7，低于预先冻结的2e-5。该核对针对当前 seed 数学映射；时间标定及原生 camera 外参到 SeedCamera 的完整随机依赖尚未在此片段闭合。

12个共同物理源同时驱动全部历史姿态、外参、bearing，主误差取执行姿态估计扰动的负号（true-minus-estimate）。seed误差取 estimate-minus-true。直接源映射 `[M_x; J_seed M_input]` 的联合分布，与实际 seed 边缘传播 `J Q Jᵀ` 和 `C_x_seed=M_x M_inputᵀ Jᵀ` 等价，测试包含非零跨块；主/seed交叉块范数为16.3067（单位幅度）。这是明确给定的受控联合分布，绝非把真实 posterior 历史 pose/bearing 块默认为零。

运行前固定三种扰动幅度，每幅度12000个独立抽样，随机seed20261008。实际C++ seed映射每次重新估计，消费者用同一源draw。线性源参考的所有联合协方差条目落在族显著性0.01、Bonferroni修正的投影χ²同时区间内。实际非线性样本联合统计及线性化偏差保存于 `results.json`；各幅度联合协方差相对偏差约1.42%、0.897%、0.372%，主/seed交叉块偏差约5.20%、2.67%、0.624%。这些差值包括Monte Carlo抽样误差，不能由幅度差异推断线性化随噪声改善。非线性输出只报告描述性统计，未套用Gaussian区间颁发统计校准通过。

原始36000行样本在 `/home/he/output/ltv_nondegradation_correlation_20261008/actual_seed_samples.csv`；预测矩阵在同名 `.predicted.csv`，SHA256、抽样合同与全部线性区间在本目录保存。集成ROS构建版本将再次执行同一合同，并另外记录身份。

LC-01实际 Jacobian/受控联合映射部分 PASS；真实seed联合统计 **BLOCKED**：当前保存的主clone P没有 Cov(main/history pose, reused bearing)、历史bearing互相关、标定/time offset随机依赖及 seed 与当前主/Observer 全部交叉块。生产 pipeline 既有风险协方差把这些来源近似独立，不是可回灌的完整联合分布；本轮没有改变它。最小扩展是在来源首次进入主视觉更新前保留 bearing 误差源，沿clone扩增、视觉消元/更新、JPL注入、边缘化传播源系数，再对实际seed的J做联合映射。

LC-06本片段属于真实实现的受控输入核对。没有独立landmark真值，因此真实Sigma_L校准仍为 **NOT_EVALUATED**，不授予landmark ON资格。

复核：构建后运行 `ros2 run ov_msckf test_ltv_seed_joint /absolute/new/samples.csv`，再运行 `python3 scripts/ltv_landmark_correlation/actual_seed/analyze.py /absolute/new/samples.csv /absolute/new/results.json`。
