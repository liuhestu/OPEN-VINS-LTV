# compact_v2

本轮修改离散实现，保留FeatureManager策略与原冻结融合权重。旧报告和旧安装目录不覆盖。

- 修改前的源码：`prechange/source.tar.gz`；安装副本：`prechange/install/`（本地归档，不提交生成产物）。
- 独立构建与安装：工作空间的 `build_compact_v2`、`install_compact_v2`。
- 数学核心：Observer 只拥有动态 `[路标…, v_B, g_B]` 和设计矩阵P；重排保留所有已有交叉块。
- Runtime：拥有IMU/相机时序、epoch、预热、有效性和L锚点；失败重置。
- MSCKF：在最终更新作用域内构造G/V/L，合并视觉量测，由原生EKFUpdate更新状态；无视觉可仅辅助更新，辅助拒绝继续视觉更新。

数学为一阶均值传播、`F=I+ΔtA`、`P=FPFᵀ+ΔtV`；相机噪声 `R_c=I/(q_landmark·Δt)`，LLT求K，Joseph更新P。首次事件、零Δt和空观测不修正。P不是辅助噪声，精简代码不证明精度或统计一致性。

旧Euler安全阈值、谱裁剪阈值仅为旧YAML兼容字段，在compact_v2中不参与计算。原无受控生命周期入口已删除；ON需启用FeatureManager。

构建：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /home/he/open_vins_ltv_ws/log_compact_v2 build \
  --build-base /home/he/open_vins_ltv_ws/build_compact_v2 \
  --install-base /home/he/open_vins_ltv_ws/install_compact_v2 \
  --packages-up-to ov_msckf --parallel-workers 2 \
  --cmake-args -DBUILD_LTV_PHASE0_TESTS=ON -DBUILD_LTV_EXPERIMENTS=ON -DCMAKE_BUILD_TYPE=Release
```

基础测试证据写入 `tests.json` 后，运行：

```bash
OPENBLAS_NUM_THREADS=1 python3 docs/experiment_process/experiments/compact_v2/tools/evaluate.py all
```

此脚本先检查MH_05/V2_02五模式的40秒段，再执行50次完整精度回放与50次独立串行开销回放。不扫描参数，性能计时固定CPU 0,1，精度并发计时不用于开销结论；每组只测一次。基础数学、输入或运行有效性失败即停止扩展，保留失败证据。MH_04沿用原实验排除范围。

结果：[REPORT.md](REPORT.md)；明细包含ATE、1/5秒RPE、覆盖、10秒局部窗口、最后10%误差、实际分支更新次数和独立串行开销。V1_01姿态GT来源限制沿用历史说明。

手动入口仍为 `config/euroc_final/manual_run.yaml`，默认安装切换到compact_v2。
