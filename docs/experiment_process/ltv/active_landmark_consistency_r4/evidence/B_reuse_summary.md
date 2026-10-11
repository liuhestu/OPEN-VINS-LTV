# 正式确认前：原始 B 基线复用清单

只读analysis **57** exit0，工具 `scripts/ltv_active_consistency_r4/prepare_baseline_views.py`。外部完整清单为 `/home/he/output/ltv_active_landmark_consistency_r4/B_reuse_manifest.json`；记录工具/源manifest/metrics/原run文件/新identity、传感器CSV、完整配置树及runtime manifest SHA。工具成功完成核查，不表示11条基线均存在。

原指定 `ltv_feature_readiness_passive/final_manifest_recovered.json` 的real_metrics仅包含4条EuRoC和2条UZH。11条EuRoC逐名结果：

| 序列 | 结果 |
|---|---|
|MH_01_easy|MISSING_METRIC|
|MH_02_easy|MISSING_METRIC|
|MH_03_medium|MISSING_METRIC|
|MH_04_difficult|MISSING_METRIC|
|MH_05_difficult|MISSING_METRIC|
|V1_01_easy|VERIFIED_B_VIEW|
|V1_02_medium|MISSING_METRIC|
|V1_03_difficult|VERIFIED_B_VIEW|
|V2_01_easy|MISSING_METRIC|
|V2_02_medium|VERIFIED_B_VIEW|
|V2_03_difficult|VERIFIED_B_VIEW|

4条view在外部 `evidence_views/formal/<sequence>/B`。原文件均只读symlink，只有view内identity为新文件，显式`mode=B`、`alias_origin`、原identity SHA与`config_path`。不改原始报告或输出；没有把旧P_NEW改名成R3B前驱。

每条先用原metrics的B identity SHA定位唯一run，避免挑不同runtime、short或好看结果，再逐项核验metrics记录的原文件SHA。replay必须complete、相机包数完整、零G/V提交；native audit行数/整数receipt唯一性/零注入以及原metrics B完整输入与主audit相同证据均核对。原sensor CSV SHA重新读取；mode_configs中estimator SHA及全部配置树必须唯一精确匹配。4条均解析到 `mode_configs/STEREO_THEN_TEMPORAL/ltv_euroc/B/estimator_config.yaml`，配置内容没有按新候选改写；原runtime manifest及核心库SHA亦核验。

剩余7条是该指定来源缺证据，不能用现4条外推，也未擅自从其它命名输出择优替代。主线程可为这7条安排具名新B或提供另一个可审计原来源；新增运行须单独预算。两份R3B前驱alias由主线程另建，本工具不混入其身份。
