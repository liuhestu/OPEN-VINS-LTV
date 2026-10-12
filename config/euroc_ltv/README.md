# EuRoC LTV 定版入口

统一使用工作空间的 `build` 和 `install`：

```bash
cd /home/he/open_vins_ltv_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ov_msckf
source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py ltv_mode:=GVL dobag:=true dolivetraj:=true rviz:=true
```

默认 `config:=euroc_ltv`，统一从配置目录读取 `estimator_config.yaml` 和 `ltv.yaml`。仅选择 `ltv_mode:=OFF/G/V/L/GVL` 即可，不必输入权重或 LTV 配置路径；不指定模式时使用 YAML 顶层 `mode`，默认 OFF。原生 EuRoC 配置可显式使用 `config:=euroc_mav ltv_mode:=OFF`。`config_path` 可直接指定 estimator 文件，LTV 配置仍从该文件的同目录读取。
修改同目录 `ltv.yaml` 调权重，每个场景、每个模式独立：

| 场景 | OFF | G | V | L | GVL |
|---|---|---|---|---|---|
| MH | 0/0/0 | 16/0/0 | 0/49/0 | 0/0/6 | 2/49/6 |
| Vicon（V1/V2） | 0/0/0 | 1/0/0 | 0/6/0 | 0/0/6 | 1/1/1 |

表中依次为 G/V/L，初始值来自历史最终实验 `docs/experiment_process/euroc_final_results_data/protocol.json`。例如改变 MH 的 GVL，修改 `profiles.MH.GVL.weights`；不会影响单独 G 或 V 模式。

- bag 回放：根据 `bag` 的序列目录名前缀 `MH_`、`V1_`、`V2_` 自动选择 MH/Vicon。默认 bag 为 V1_01；显式启用回放后会选择 Vicon。默认 `dobag:=false` 使用实时订阅。
- 实时订阅或无法识别的 bag 名：使用 YAML 的 `scene`，默认 MH。可用 `ltv_scene:=MH` 或 `ltv_scene:=Vicon` 显式指定。
- `config` 统一选择宿主与 LTV 配置；不再提供独立 `ltv_config` launch 参数。启用 LTV 时，该目录必须有 `ltv.yaml`。
- launch 的 `ltv_mode` 优先于 YAML 顶层 `mode`；顶层默认 OFF。
- 权重换算统一在 C++：G 标准差 `10°/sqrt(G)`，V 标准差 `1 m/s/sqrt(V)`，L 噪声下限 `0.5 m/sqrt(L)`、信息上限 `0.01*L`。只有启用分支的权重参与计算。
- 使用上述 `--symlink-install` 构建后，install 中的配置链接到源码，直接修改本目录 YAML 并重启即可，无需重新编译。若自行使用普通复制安装，需同步安装副本，或用 `config_path` 指向源码目录中的 `estimator_config.yaml`。

例子：

```bash
ros2 launch ov_msckf subscribe.launch.py ltv_mode:=V dobag:=true bag:=/home/he/datasets/euroc/MH_05_difficult_db
ros2 launch ov_msckf subscribe.launch.py ltv_mode:=GVL dobag:=false
```

原来的 `config/euroc_final`、`ltv_euroc`、`ltv_three_track`、`ltv_value` 已移除。历史配置和研究脚本按历史 Git 版本重现；冻结报告与数据不改写。当前统一通过 `subscribe.launch.py` 回放 ROS bag；旧 ASL 手动回放脚本和选择文件已移除。

真值显示与轨迹保存沿用 ROS 1 的独立节点：

- `rviz:=true` 启动 RViz，`dolivetraj:=true` 启动 `ov_eval/live_align_trajectory`；二者默认 false。
- 对齐默认 `alignment_type:=posyaw`；`alignment_type:=se3` 使用截至当前全部匹配轨迹拟合固定尺度 SE3。节点复用原生 20 ms 最近邻匹配及对齐实现，不改变估计器或保存的估计轨迹。
- `path_gt:=/path/to/gt.csv` 或 `.txt` 指定真值；省略时根据 bag 路径查找对应 EuRoC ASL CSV。`dolivetraj:=false` 时不要求真值文件。
- `dosave:=true` 启动 `ov_eval/pose_to_file`，`path_est` 默认 `/tmp/traj_estimate.txt`。保存未对齐的原始估计位姿及协方差。
- `dotime:=true` 记录估计器耗时，`path_time` 默认 `/tmp/traj_timing.txt`。
- `dataset` 默认 V1_01_easy；未指定 `bag` 时使用 `/home/he/datasets/euroc/<dataset>_db`。`bag_start` 默认 0 秒，`bag_rate` 默认 1。
- 参数统一为 `rviz`、`dobag`、`bag`、`dolivetraj`、`alignment_type`；旧参数名称已移除，不保留兼容入口。bag 结束后保留 RViz，Ctrl+C 结束全部节点。

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=euroc_mav ltv_mode:=OFF \
  dobag:=true bag:=/home/he/datasets/euroc/V1_01_easy_db \
  dolivetraj:=true alignment_type:=se3 rviz:=true \
  dosave:=true dotime:=true
```
