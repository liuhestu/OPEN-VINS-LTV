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
