# 正式科学聚合独立复核

**维持冻结评价结果 NOT_MET。10/11 单序列通过不能写成总体通过，也不能依据 NEW 表现差剔除 MH_04。**本次逐份读取 11 个 metrics，小文件中的 sequence/status/decisions 与 `formal_all11_assessment.json` 一致；全部记录 complete/engineering/reference 为真，唯一输出合同失败是 MH_04_difficult。

## MH_04 的 B 有效性边界

工程证据确认 B 本身 exit0、2032 相机包与 20320 IMU 全量消耗、1919 初始化后轨迹行、零注入。三方法主 audit/trajectory 精确相同，因此主状态异常不能归为 active consistency 对主估计器的数值侵入。

但 **baseline_valid=true 目前只支持冻结评价器所实现的运行有效判定，不足以证明 B 定位科学有效**。`real_evaluate.evaluate` 在工程检查和存在初始化包后写入 baseline_valid=true，没有按 GT 检查主定位是否失效；原 goal §11 要求 Baseline 本身定位失败标 BASELINE_INVALID。

既有 MH_04 metrics 在共同 raw 的 180 个样本上报告 OpenVINS 速度 RMSE 410.070332 m/s、峰值 742.434636 m/s、重力方向 RMSE 124.918877°。这是必须保留的主基线异常信号，不能用“正常完成”掩盖。该共同支持受 NEW raw 可用性限制，故它本身不能替代全 B、独立于 NEW 的定位失败裁定。本审查没有扫描 B 全时域或新增筛选门槛。

建议表述为“冻结工具计入 11 个 B 运行有效序列；MH_04 的 B 定位有效性另有明确异常、需 B-only 复核”。当前 NOT_MET 与完整 11 条记录保留；不得因这条 NEW 不通过而临时修改有效名单或宣称 10 条子集总体成功。

## 零 ready 与真实 raw 失败分开

MH_04 的 G/V/joint-ready 均没有样本。对应 ready severe fraction、RMSE、P95 为 null；`threshold(None, limit)` 返回 false 是无法建立通过证据，**不是观测到坏 ready 比例为 100%，也不是零坏输出率**。

独立的 raw 失败则有数值证据：NEW raw 180/1919 包，缺失 1739；同支持速度 RMSE 9.116316→13.778768 m/s，超过允许增加 0.455816；eta RMSE 5.399331→6.634044，超过允许增加 0.269967。NEW 另损失 PREV 的 4 个 raw 包，状态可解释但仍触发原非启动支持损失检查。以上与零 ready 的不可评价情况不能混写。

## 重复性及剩余合同

`formal_repeats_exact.json` 记录冻结 SHA 匹配，V1_03 与 V2_03 各自 original/repeat 的 cache.bin、audit.csv、trajectory.csv SHA 全相同，支持两次完整 native 确定性重复。此处只复核保存的摘要，没有再次读取大文件计算 SHA。确定性不意味着估计准确，不能抵消 MH_04 输出合同失败。

本页不改冻结评价公式、不删除任何序列、不读取新增 GT 或启动重计算。合成、初始化/恢复、资源、logging ON/OFF 与最终完整验收仍须各自据证据判定；真实输出合同仅为其中一部分。
