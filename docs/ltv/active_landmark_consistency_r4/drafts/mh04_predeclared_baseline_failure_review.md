# MH_04 预声明 B-only 失败证据复核

本页补充 `formal_scientific_aggregation_review.md` 的待查问题，保留该页和原冻结 metrics/assessment 不变。

**接纳另列 B-only 契约视图：MH_04 为原基线定位失败，其余 10 个有效序列的输出合同 10/10 通过。**这不是改变冻结输出、按 NEW 误差选序列或整体任务成功。

## 预声明与历史原基线

- `docs/ltv/feature_readiness_passive/goal.md:429` 在 R4 前已明确：“MH_04已有原基线定位失败，本轮不修基线也不加入它。”这是旧轮的范围声明，不能用来免除 R4 已要求的完整执行；本轮确实执行了该序列。
- `docs/ltv/passive_hardening_v2/goal.md:342` 要求基线本身定位失败标 BASELINE_INVALID，方法新导致失败不得据此排除。原 protocol 的 baseline_invalid_contract 允许 native_estimator_failure，不要求另外发明 ATE 门槛。
- `docs/ltv/evidence/final/MH_04_original_diagnostic.json` 保存独立原始基线二进制、配置、输入规则、完整执行与退出码 0。`MH_04_failure_analysis.json` 在本轮之前已记录原 B 数值失效：最大速度 783.530313 m/s，末位置约 [-27824.957,-27658.795,-17357.811] m，而对应 GT 最大速度 2.870395 m/s。它也明确输出有限，说明 exit0/finite 不能等同定位成功。

## 本次轻量身份桥接

历史原 B 目录为 `/home/he/output/openvins_ltv_autonomous_20261002/runs/MH_04_difficult-B-original-20261002T183009Z-568614d1.done`。新 B 为 `CONF_CONTROLS_C02_01_B_MH_04_difficult/B`。

1. 两者 `trajectory.csv` 长度均 663996 bytes，完整 SHA-256 均为 `a5e0a0708c7599538dde0501000332696a35a4296915e3b65448b55d399dbdb0`。本轮 B 数值轨迹逐字节重现历史原基线失败，不是仅凭序列名推断。
2. 主 YAML 非 `ltv_` 项只存在四处行尾说明注释差别，参数值相同。两份 IMU/相机校准文件 SHA 分别一致为 `408ea8b60b5f9e7c8251e6d302f04c0675bfefdd31229bb1afd139bc8f4a0287` 和 `b9e11b7bcda102f7c8c384c97318d67f3916b58942f9073722f83c22bd7073f7`。
3. 旧 diagnostic exact-pairs cam0/cam1 的 2032 行逐项等于当前原始 CSV 按共同精确时间戳选择的列表；IMU 文件和两相机图像目录 resolve 到相同原始源。本次只读这些小 CSV/路径，不扫描图像。
4. 历史和新 audit.csv 格式、长度及 SHA 不同，本页不称其字节相同。数值复现结论由完整轨迹相同、输入/主配置桥接及此前各轮独立原基线证据支持。

## 应同时保留的两种结果

| 视图 | MH_04 | 聚合结论 |
|---|---|---|
| 原冻结评价器输出 | 11 个技术运行有效记录之一；输出合同失败 | NOT_MET，保持原 metrics 与 assessment |
| 原 B-only 定位有效契约解释 | BASELINE_INVALID / native_estimator_failure，仍保留实际执行及旁路检查 | 10 个 B 定位有效序列的输出合同 10/10 通过，满足至少 8 条的数量要求 |

不能将第二行写为“11/11 科学通过”，也不能忽略 MH_04 的 NEW raw 回归与零 ready 事实。预声明失效以及本轮 B 对旧失败轨迹的完整重现独立于 NEW 好坏，因此本页的分类依据不是事后筛选候选结果。

全任务仍需独立汇总初始化、恢复、合成、资源、logging 和重复性合同；有效真实输出的 10/10 不替代这些义务。本次未更改评价器、阈值、冻结清单或原记录，未执行重放或重计算。
