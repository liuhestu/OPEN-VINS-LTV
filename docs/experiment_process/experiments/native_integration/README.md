# 原生 LTV 接入验证

构建成功，独立安装在 `/home/he/open_vins_ltv_ws/install_ltv_native`；ov_core/ov_init 沿用未修改的 compact_v2 安装依赖。

- 16 项必要测试通过：数学、特征几何/生命周期/有界历史、宿主恢复与融合确认、共享 IMU 区间、G/V/L FEJ Jacobian、联合更新和模式权重配置。
- MH_05 ROS 2 bag，OFF/GVL 各约40秒：均初始化成功并保存轨迹；OFF 366行，GVL 365行；节点均正常退出。
- GVL 实际分支更新：G 279次、V 265次、L 264次。同一EKF可包含多个分支，次数不能相加当作EKF调用数。
- `bag_play:=false` 的 GVL 节点启动成功，确认 `/imu0`、`/cam0/image_raw`、`/cam1/image_raw` 订阅。
- 当前离线手动入口 prepare-only 通过；未启用分支的权重可保留，Python不重复换算噪声。

初次短段发现原生ROS2退出崩溃。修正异步IMU的栈引用和后台线程回收后，GDB进一步定位到 image_transport 发布器在全局析构阶段销毁过晚；主函数现在显式释放可视化对象。最初失败尝试留在 `/tmp/ltv_native_launch_smoke*`，不计入验收。最终检查会核对节点退出日志，不能仅凭轨迹文件判定通过。

本轮未做完整精度矩阵、调参或串行开销评估。实时检查只验证启动与订阅，未连接真实传感器。

详细命令和结果：[validation.json](validation.json)。使用方法：[euroc_ltv](../../../../config/euroc_ltv/README.md)。

重现必要测试：

```bash
python3 ov_msckf/tests/ltv/run_native_tests.py /home/he/open_vins_ltv_ws/install_ltv_native /tmp/ltv-tests-new
python3 ov_msckf/tests/ltv/smoke_native_launch.py /home/he/open_vins_ltv_ws/install_ltv_native /home/he/datasets/euroc/MH_05_difficult_db /tmp/ltv-smoke-new
```
