# 76.75 s / 27537：实际核心单帧干预

**固定0.04 rad规则在校正前拒绝目标观测；相同完整预测快照下，新后验速度误差由旧0.251884降至0.090604 m/s。** 这是单事件受控干预，不是完整序列改善声明。

R4 build20通过；首次short22测试锚点安装没有提供controlled birth要求的visible ID，核心正确拒绝，尚未执行目标校正。该失败和预算保留。仅测试安装阶段补齐dummy观测后build23通过，short24成功；首次相机零校正时长，随后全x/P由验证过的快照覆盖，dummy观测不进入被比较的真实更新。未改生产核心。

真实 `LtvActiveConsistency` 使用当前同版本克隆支持的11条过去cam0射线、跨度0.5 s：history max=0.002365515 rad，current holdout=0.350262531 rad，结果 `HOLDOUT_INCONSISTENT`。当前观测不参与拟合，GT不在模块/harness输入中。初次失败仅从实际校正观测集合去掉27537，完整retained slots不变。

真实 `LtvObserver::updateFeaturesControlled` 从相同after-lifecycle完整x/P分别执行原始和过滤观测，原结果对实际cache完整x/P相对差为8.32e-15/7.39e-14，原5子步一致；过滤后子步数由实际算法选择为2，完整P重新递推。不是从旧Δ减去单点贡献。新路径没有27537观测行，因此其原直接观测贡献不再出现，但其slot、均值及交叉P仍可通过其他观测自然耦合更新，不能声称该点状态完全不动。

| 同时刻离线参考误差 | prediction | 原posterior | 新posterior | OpenVINS |
|---|---:|---:|---:|---:|
| v（m/s） | 0.106985 | 0.251884 | 0.090604 | 0.016983 |
| eta（m/s²） | 0.159881 | 0.203312 | 0.153943 | 0.139866 |
| G方向（deg） | 0.850307 | 1.144770 | 0.817436 | 0.816904 |

实际速度校正模长0.172563→0.022528 m/s。参考在core结果保存后由analysis26按原整数camera_ns=1413394964105760512关联，未用于筛选/改阈值。

证据：`/home/he/output/ltv_active_landmark_consistency_r4/local_events/27537_02/`含input、result及SHA/provenance；同级`error_comparison.json`含完整数值。工具为`scripts/ltv_active_consistency_r4/local_event_replay.cpp/.py`及`evaluate_local_events.py`。

限制：测试用非const实例的公开const引用经test-only const_cast注入完整已验证after-lifecycle快照，不新增生产setter。快照来自旧轨迹局部重构，实际cache最终完整x/P为复原oracle。只干预指定目标并人为设为首次失败，未声称其他点已检查、真实先前失败计数为零、或长期启用能保持此输入状态；出生/身份guard与连续失败退休由独立生命周期小测覆盖，仍需真实序列确认。
