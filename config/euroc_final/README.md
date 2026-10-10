# 手动选择 EuRoC 序列、融合模式与强度

编辑同目录的 [manual_run.yaml](manual_run.yaml)，然后在仓库根目录运行：

```bash
python3 docs/experiments/tools/ltv_gain_scan/run_manual_euroc.py
```

入口使用最终实验已经编译好的 `run_ltv_gv_production`，修改选择或权重后无需重新编译。它直接读取原始 ASL 图像和IMU完成离线回放；输出轨迹CSV，不启动ROS话题或RViz。默认安装位置从 `docs/euroc_final_results_data/identity.json` 读取；安装被移动时修改 `install_root`。

## 最常修改的字段

```yaml
sequence: MH_02_easy
mode: GVL
weights:
  G: null
  V: null
  L: null
```

`null` 自动选择对应场景、模式的最终冻结强度。可以填写下面10条序列中的任意一条：

| 场景 | 序列 | G模式 | V模式 | L模式 | GVL模式（G/V/L） |
|---|---|---:|---:|---:|---|
| MH | MH_01_easy、MH_02_easy、MH_03_medium、MH_05_difficult | 16 | 49 | 6 | 2/49/6 |
| Vicon | V1_01_easy、V1_02_medium、V1_03_difficult、V2_01_easy、V2_02_medium、V2_03_difficult | 1 | 6 | 6 | 1/1/1 |

模式支持 `OFF`、`G`、`V`、`L`、`GVL`。OFF关闭Observer及辅助更新；单分支模式仅启用对应分支。MH_04未纳入这套最终实验配置。

## 手动填写强度

例如在MH_01上仅使用G6：

```yaml
sequence: MH_01_easy
mode: G
weights:
  G: 6
  V: null
  L: null
```

例如三路组合使用G4/V9/L2：

```yaml
sequence: MH_01_easy
mode: GVL
weights:
  G: 4
  V: 9
  L: 2
```

启用分支的强度必须为有限正数；未启用分支填写 `null` 或 `0`。手动覆盖后的配置会标记为非冻结配置，保存在新的结果目录中。

这里的强度是融合信息权重，不是Observer内部Riccati增益。换算为：

- G标准差：`10 / sqrt(G)` 度。
- V标准差：`1 / sqrt(V)` m/s。
- L标准差下限：`0.5 / sqrt(L)` m；信息上限：`0.01 * L`；最多2个点。

Observer内部参数沿用本轮冻结值。模式由二进制原生入口应用，因此生成的估计器YAML保留其初始化开关，运行后的 `effective_options.json` 才是最终实际开关和参数。

## 输入、输出与短段检查

`dataset_root` 是包含 `MH_01_easy/mav0` 等目录的原始ASL根目录。它不是ROS bag路径。`cpu_affinity` 指定允许使用的逻辑CPU，OpenCV及常用数值库均使用单线程。

`short_seconds: 0` 回放完整序列；填写 `(0,40]` 秒可检查短段。也可以临时覆盖：

```bash
python3 docs/experiments/tools/ltv_gain_scan/run_manual_euroc.py --seconds 40
```

MH_01的前40秒可能尚未完成初始化；检查初始化及辅助更新时推荐 `MH_05_difficult` 或 `V2_02_medium`。如果日志提示没有初始化轨迹，改用完整回放，不通过改变初始化参数绕过。

只生成估计器YAML和选择记录，暂不回放：

```bash
python3 docs/experiments/tools/ltv_gain_scan/run_manual_euroc.py --prepare-only
```

也可以另存一份自己的选择文件：

```bash
python3 docs/experiments/tools/ltv_gain_scan/run_manual_euroc.py /absolute/path/my_run.yaml
```

每次运行在 `output_root` 下新建目录，终端打印其路径。默认根目录为 `/tmp/openvins_ltv_manual`；需要长期保留结果时将其改到自己的持久目录。

| 文件 | 内容 |
|---|---|
| manual_run.yaml | 本次手动选择文件副本 |
| selection.json | 解析后的序列、模式、权重、预期参数及二进制/配置哈希 |
| config/estimator_config.yaml | 本次生成的估计器配置及同目录标定文件 |
| effective_options.json | 二进制实际生效的参数 |
| trajectory.csv | 估计轨迹 |
| replay.json | 输入消费与实际G/V更新次数 |
| landmark_approx.csv | L分支收据 |
| stdout_stderr.log | 初始化及运行日志 |
| resources.json | 本次CPU/墙钟时间和峰值RSS |
| manual_validation.json | 运行退出码、实际参数、线程与初始化检查 |

入口会尝试取得现有实验的共享锁；若已有实验运行，会提示稍后重试。手动运行的开销记录不自动纳入正式性能表，手动轨迹也不会覆盖已归档的最终实验。
