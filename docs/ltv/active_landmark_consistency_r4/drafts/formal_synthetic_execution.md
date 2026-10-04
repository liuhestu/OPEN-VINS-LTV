# 冻结C02：正式8项synthetic执行清单（尚未执行）

只读检查冻结protocol、frozen_config、R4 producer/runner/evaluator/budget。本页不修改冻结源码，不生成输入或启动计算。现两个重计算槽均由主线程占用，必须等主调度释放。

冻结8个具名案例如下，每个60 s，仅运行 `P_NEW_ACTIVE`（wrapper ABI5），沿用冻结source与参数。

| 名称/目录 | scene | lifetime | condition | source | seed |
|---|---|---:|---|---|---:|
|STEREO_REGULAR_1_201|REGULAR|1|STEREO|HYBRID|201|
|STEREO_FAST_05_202|FAST|0.5|STEREO|HYBRID|202|
|STEREO_FAST_1_201|FAST|1|STEREO|HYBRID|201|
|TEMPORAL_FAST_1_202|FAST|1|TEMPORAL|TEMPORAL|202|
|WRONG_MATCH_REGULAR_1_201|REGULAR|1|WRONG_MATCH|HYBRID|201|
|STEREO_DROPOUT_REGULAR_1_202|REGULAR|1|STEREO_DROPOUT|HYBRID|202|
|TIME_ERROR_REGULAR_1_201|REGULAR|1|TIME_ERROR|HYBRID|201|
|PRESSURE_REGULAR_05_202|REGULAR|0.5|PRESSURE|HYBRID|202|

## 固定工具与预算

输出根 `OUT=/home/he/output/ltv_active_landmark_consistency_r4`。冻结库 `OUT/synthetic_wrapper_C02_01.so` SHA256 `88684ca34b42c5c349e02e94f5b36a9eabc3a7bec50d4d92ab3b226ec4a520de`，链接冻结runtime `943573c164f9f6fb7c1f265cda71d7630902d2b07a7e4be63cd27748d44b7ab0`。ABI5固定history/holdout角0.02、min5/span0.20、两失败退休，R3B grace开启；ABI4是同库active OFF。

既有synthetic消耗4次，正式预留8，总上限12。下面每个runner调用计1次synthetic，共8；输入生成与离线评价计analysis。失败/重跑同样计数，没有预留第9次正式运行。所有阶段必须 `--phase confirmation`，单进程BLAS及至多2槽由现工具维持。不要把完整60 s运行登记为test/analysis。

## 可直接执行的命令模板

在仓库根目录按下列设置（此处仅说明，不执行）：

```bash
OUT=/home/he/output/ltv_active_landmark_consistency_r4
R4=scripts/ltv_active_consistency_r4
FREEZE=docs/ltv/active_landmark_consistency_r4/frozen_config.json
FROZEN_SHA=$(sha256sum "$FREEZE" | cut -d ' ' -f1)
CAL=/home/he/output/ltv_passive_hardening_v2/real_development/R3B_grace_V2_03_01/config/kalibr_imucam_chain.yaml
LIB="$OUT/synthetic_wrapper_C02_01.so"
```

对表中前7项逐项设置NAME/SCENE/LIFETIME/CONDITION/SOURCE/SEED并串行执行，或由主线程按两槽调度；不要运行未登记并发shell循环：

```bash
python3 "$R4/budget.py" analysis --phase confirmation --identity "formal-input:$NAME:freeze=$FROZEN_SHA" -- \
  python3 "$R4/synthetic_inputs.py" --out "$OUT/inputs/formal/$NAME" \
  --calibration "$CAL" --scene "$SCENE" --lifetime "$LIFETIME" --condition "$CONDITION" \
  --seed "$SEED" --duration 60 --confirmation-sha "$FROZEN_SHA"

python3 "$R4/budget.py" synthetic --phase confirmation --identity "formal-C02:$NAME:ABI5:freeze=$FROZEN_SHA" -- \
  python3 "$R4/synthetic_runner.py" --input-dir "$OUT/inputs/formal/$NAME" \
  --out "$OUT/synthetic_confirmation/$NAME/P_NEW_ACTIVE" --library "$LIB" \
  --mode P_NEW_ACTIVE --source "$SOURCE"

python3 "$R4/budget.py" analysis --phase confirmation --identity "formal-evaluate:$NAME:freeze=$FROZEN_SHA" -- \
  python3 "$R4/synthetic_evaluate.py" --input-dir "$OUT/inputs/formal/$NAME" \
  --run-dir "$OUT/synthetic_confirmation/$NAME/P_NEW_ACTIVE" \
  --out "$OUT/synthetic_confirmation/$NAME/evaluation_v1"
```

第8项设置`NAME=PRESSURE_REGULAR_05_202`、`SOURCE=HYBRID`，生成命令**必须替换**为：

```bash
python3 "$R4/budget.py" analysis --phase confirmation --identity "formal-far-pressure:$NAME:freeze=$FROZEN_SHA" -- \
  python3 "$R4/pressure_inputs.py" --out "$OUT/inputs/formal/$NAME" \
  --seed 202 --duration 60 --confirmation-freeze-sha "$FROZEN_SHA"
```

之后使用同一runner/evaluator模板。pressure producer固定原far REGULAR/.5s公式与新随机种子，经诚实midpoint桥接并保存source；它不是near几何，也不是旧seed101数值复现。不要对普通producer传未支持的`--condition PRESSURE`，不要把midpoint IMU冒充endpoint。

## OFF复用与评价要求

目前没有查到这8个新201/202输入的完整同身份OFF结果。现seed42开发OFF或前三轮旧桥输入不能作为同输入raw-regression基线。只有input SHA完全相同、完整消费、相同模式/参数和可核验runtime来源的现成OFF才可增加`--baseline <path>`；读取后仍须审查评价器要求，不凭名称复用。额外执行8个OFF将超当前剩余synthetic预算，不能隐计；没有同身份OFF时raw回归对照明确UNVERIFIED，不等于退化，也不等于通过。主线程已有同输入开发对照应单独保留，不能改称正式未见支持。

每项先核验run.json完整消费和1201相机事件、输入/label SHA及退出码，再评价raw/ready_G/ready_V/joint、严重误ready、全段峰值、seed来源≥100样本/机会分母及可靠率。输入仅inputs.npz进入估计器，labels只进入evaluator。WRONG_MATCH/TIME_ERROR保留预登记负对照语义，不强求正常机会接受率。PRESSURE完整启动坏段保留。

STEREO_DROPOUT已生成15–30 s退化阶段和30 s后的phase2及独立sufficient-supply标签，可执行原恢复口径，不能以“30 s后”直接替代稳定可靠供给起点；TEMPORAL没有可证独立双目充分供给时按原评价器标UNVERIFIED。其他正常场景没有恢复标签不能假报恢复通过。synthetic的coverage/最长5 s是完整诊断，不能冒充EuRoC真序列合同；负对照严重误ready仍按原合同报告。原evaluator可能给总体状态，最终需按合同适用范围解释，不能只摘PASS字段。

最后8项只补当前冻结方法的预登记合成确认；资源600 s已有独立预算/结果，不能把这些60 s案例合并称600 s资源验收，也不自动新增案例或扫阈值。
