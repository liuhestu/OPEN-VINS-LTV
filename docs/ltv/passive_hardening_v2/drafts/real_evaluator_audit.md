# 新真实评价器与首条开发结果

新增 `scripts/ltv_passive_hardening/real_evaluate.py`，只在离线评价中读取GT。导入旧评价器的官方EuRoC速度、四元数/body转换、GT间隙支持、主轨迹转换和误差定义；旧文件不改。扩展序列允许名单为protocol的全部11条，新绝对门槛与P95/持续性判定独立实现。没有改变GT或拟合时间/旋转/尺度。

历史比较路径由旧final manifest定位选定P_NEW原始run，作为本轮P_PREV；旧overlay仅用于来源路径索引，不读取其修复管理字段给新日志补洞。历史CSV与当前原ASL CSV字节SHA不同，但header/行尾归一化后的精确整数ns、IMU数值逐行相同，全部相机文件resolve到同一原始图像。两份原SHA及解析等价记录保留。标定文件SHA一致；B/P_PREV/P_NEW主状态、完整P/FEJ/视觉集合digest及轨迹全部相同，且输入完整、G/V注入零。若无法证明这些条件则拒绝复用。

固定分母是B初始化后的全部相机包，四支持分别报告，每个quantity的PREV/NEW/OpenVINS使用共同物理时刻。NEW自己的绝对误差不受PREV缺失筛选。P95为线性quantile。连续episode遇epoch或>1.5 nominal cadence缺包时断开；不把离散碎片时间相加当连续5秒。raw=null不会替换成0；共同支持和逐方法全raw、支持差集/全部缺失段一起保存。除预声明初始COLLECTING外的新增raw缺失会留下未审查支持损失标志，不允许自动声称raw非退化。该规则没有引入新的百分比门槛。

V1_01 G姿态参考例外显式标记，角度数据仍完整报告；覆盖、物理健康及速度不豁免。单序列判定不是整个EuRoC目标、恢复或性能验收。

测试账本：test76暴露旧NumPy不接受method参数，等价改为interpolation='linear'；test79通过。analysis80暴露把NEW健康规则错误施于历史PREV，已限定为NEW；失败保留。最终test83八个fixture PASS，analysis84完成单序列离线评价。没有重跑完整VIO或合成。

V1_01 R1结论NOT_MET：2800初始化包；PREV raw2800，NEW raw2795，缺失仅初始COLLECTING5包(0.20秒)。raw共同有效参考2775包/138.749735秒。NEW G/V覆盖81.786%/78.964%，参考113.500/109.550秒，joint最长20.750秒。V-ready RMSE0.05516/P95 0.09820m/s，严重错误0。G方向仅诊断RMSE2.486°/P952.996°。raw η向量0.94529→1.02864m/s²，增加0.08335超过0.05容许，故不满足本轮raw回归目标；raw v0.26519→0.27218m/s仍在容许范围内。

完整数值：`/home/he/output/ltv_passive_hardening_v2/real_development/R1_V1_01_01/evaluation_v2/metrics.json` 与 `error_curves.npz`。该负结果不是停止整个目标的理由，由主代理结合其它开发条件决定下一机制。

## 参数复用增强

主代理独立review后增加完整LTV参数复用校验：用P_PREV身份中的estimator SHA定位唯一历史mode_config，再逐个验证完整配置树SHA；对两份estimator YAML所有键进行语义比较，仅允许显式列出的新hardening工程控制键差异，Q/V/P0、原feature来源、风险及成熟规则等均须相同。增加Q/来源/风险变化拒绝fixture，以及历史PREV不受NEW健康规则追溯约束的回归测试。

**test87 PASS（10 tests），analysis88 PASS执行**。新证据在同一开发目录 `evaluation_v3/`，`evaluation_v2/`保留不覆盖。此实际比较唯一配置差异为新增 `ltv_passive_hardening_enabled=true`；原LTV设计与全部其余YAML键一致。结果仍NOT_MET，原始数值未因身份校验改变。

## R2 V1_01 离线评价109

真实运行108完成后，独立analysis109对 `R2_V1_01_01/P_NEW` 评价，输出 `R2_V1_01_01/evaluation_v1/`。复用身份、解析输入/图像、完整配置与主状态旁路检查全部通过。固定2800初始化包，NEW/PREV raw均2800，无新增缺失；共同raw v/η/角RMSE逐值相同，分别0.267756m/s、1.031995m/s²、7.36827°（角仍为V1参考诊断）。NEW G/V覆盖81.929%/79.107%，参考113.700/109.750秒，joint最长20.750秒；V-ready RMSE0.055919/P950.098186m/s，严重率0。结果为单序列MEETS_SEQUENCE_CONTRACT，不能扩张为整个EuRoC、恢复、Temporal或资源验收通过。没有新增full运行。

## 全11序列新运行接口（仅fixture验收，未运行正式确认）

新增成对CLI参数 `--baseline /path/B --previous /path/P_PREV`，与 `--new /path/P_NEW` 同用；省略这两个参数则保留历史manifest来源路径。新显式路径要求三份identity中的sequence/mode分别精确匹配；inputs可使用 `inputs.sensors` 或旧 `sensor_csv_sha`，配置可使用 `config_sha`字典或 `config_tree_sha`；`config_path`可显式指向estimator_config，否则默认run父目录config。NEW仍需 `inputs.reference_metadata_only`，GT只在离线读取。

新增配置白名单仅增加 `ltv_hardening_preserve_constrained_state`；其余原参数仍拒绝差异。显式三模式解析输入/相机文件相同；完整配置树SHA验证；PREV/NEW全部非hardening参数相同；B全部非ltv原生参数相同，且标定/include SHA一致；之后执行原有完整主状态/FEJ/轨迹旁路检查。

新增 `assess_all11(records)` 要求全11唯一序列，NEW真实完整与工程旁路证据齐备；B无效仅接受预登记原生初始化/估计器/输入失败，不允许以NEW误差排除；少于8条B有效不通过；每条有效序列按固定合同。其 `OUTPUT_CONTRACT_MET` 只表示输出层，并不替代种子、恢复、资源、性能、冻结和复测验收。主dispatcher必须将records与真实运行、评价文件SHA绑定，不能直接信任任意外部布尔摘要。

预算test128通过初始14项，新增显式三模式fixture后 **test130 PASS（15项）**。fixture覆盖全11缺项、B有效7/8边界、NEW失败不能排B、未完整NEW、R3键允许、其余参数拒绝与显式配置SHA篡改拒绝；没有读取正式确认结果或执行完整运行。
