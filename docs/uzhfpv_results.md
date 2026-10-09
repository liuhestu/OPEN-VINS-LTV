# UZH-FPV 六模式 ATE RMSE

原 LTV 配置；max_slam=0；完整原始流同步回放。位置 SE(3) 无尺度对齐，Horn/SVD 交叉核验。
保留 UZH 标定、噪声和初始化；秒制 GT 位置插值，不外推、不跨超过 10 ms 的间隙。
冻结 OFF 初始化后有效 GT 支持。GT 姿态不用于验收。历史 5% 预测 FAIL 和工程结论保留。
本轮每进程 OpenCV 实际为 4 线程，限制在独立四 CPU 内；构造覆盖问题已在专用入口修正并完成七组字节一致回归。原耗时不作单线程性能验收，见 runtime_threading.json。
完整输入回放不代表全程都有 GT：ATE 仅覆盖冻结的有效 GT 支持。尾部诊断为该支持最后 10% 的位置 RMSE，并非缺少 GT 的完整输入末尾。


## ATE (m)

| 序列 | OFF | G | V | GV | L | L_GV |
|---|---:|---:|---:|---:|---:|---:|
| indoor_forward_5 | 0.685600726 | 0.685877057 | 0.685621679 | 0.685897656 | 0.685551993 | 0.685846243 |
| indoor_forward_6 | 0.412670352 | 0.412664888 | 0.412677168 | 0.412673132 | 0.412603711 | 0.412603014 |
| indoor_forward_7 | 0.484900421 | 0.484900421 | 0.484900421 | 0.484900421 | 0.484900421 | 0.484900421 |
| indoor_45_2 | 0.669980575 | 0.670024166 | 0.669980250 | 0.670024178 | 0.669979832 | 0.670022801 |
| outdoor_forward_1 | 0.770249791 | 0.770343526 | 0.770097915 | 0.770192007 | 0.770252899 | 0.770196817 |
| outdoor_forward_5 | 0.906907276 | 0.906751313 | 0.906908477 | 0.906759319 | 0.906902935 | 0.906725252 |

## 相对 OFF (%)

| 序列 | OFF | G | V | GV | L | L_GV |
|---|---:|---:|---:|---:|---:|---:|
| indoor_forward_5 | +0.000000% | +0.040305% | +0.003056% | +0.043310% | **-0.007108%** | +0.035811% |
| indoor_forward_6 | +0.000000% | **-0.001324%** | +0.001652% | +0.000673% | **-0.016149%** | **-0.016318%** |
| indoor_forward_7 | +0.000000% | +0.000000% | +0.000000% | +0.000000% | +0.000000% | +0.000000% |
| indoor_45_2 | +0.000000% | +0.006506% | **-0.000049%** | +0.006508% | **-0.000111%** | +0.006303% |
| outdoor_forward_1 | +0.000000% | +0.012169% | **-0.019718%** | **-0.007502%** | +0.000403% | **-0.006878%** |
| outdoor_forward_5 | +0.000000% | **-0.017197%** | +0.000132% | **-0.016314%** | **-0.000479%** | **-0.020071%** |

| 序列 | 模式 | 状态 | 初始化差(s) | GT覆盖 | OFF支持覆盖 | G/V/L实际次数 | 原因 |
|---|---|---|---:|---:|---:|---|---|
| indoor_forward_5 | OFF | VALID_FULL_MATCHED | 0.0 | 0.13375474083438685 | 1.0 | 0/0/0 |  |
| indoor_forward_5 | G | VALID_FULL_MATCHED | 0.0 | 0.13375474083438685 | 1.0 | 1519/0/0 |  |
| indoor_forward_5 | V | VALID_FULL_MATCHED | 0.0 | 0.13375474083438685 | 1.0 | 0/693/0 |  |
| indoor_forward_5 | GV | VALID_FULL_MATCHED | 0.0 | 0.13375474083438685 | 1.0 | 1519/693/0 |  |
| indoor_forward_5 | L | VALID_FULL_MATCHED | 0.0 | 0.13375474083438685 | 1.0 | 0/0/692 |  |
| indoor_forward_5 | L_GV | VALID_FULL_MATCHED | 0.0 | 0.13375474083438685 | 1.0 | 1519/693/692 |  |
| indoor_forward_6 | OFF | VALID_FULL_MATCHED | 0.0 | 0.45816956765861877 | 1.0 | 0/0/0 |  |
| indoor_forward_6 | G | VALID_FULL_MATCHED | 0.0 | 0.45816956765861877 | 1.0 | 279/0/0 |  |
| indoor_forward_6 | V | VALID_FULL_MATCHED | 0.0 | 0.45816956765861877 | 1.0 | 0/127/0 |  |
| indoor_forward_6 | GV | VALID_FULL_MATCHED | 0.0 | 0.45816956765861877 | 1.0 | 290/138/0 |  |
| indoor_forward_6 | L | VALID_FULL_MATCHED | 0.0 | 0.45816956765861877 | 1.0 | 0/0/126 |  |
| indoor_forward_6 | L_GV | VALID_FULL_MATCHED | 0.0 | 0.45816956765861877 | 1.0 | 290/138/137 |  |
| indoor_forward_7 | OFF | VALID_FULL_MATCHED | 0.0 | 0.5984822352535357 | 1.0 | 0/0/0 |  |
| indoor_forward_7 | G | VALID_FULL_MATCHED | 0.0 | 0.5984822352535357 | 1.0 | 0/0/0 |  |
| indoor_forward_7 | V | VALID_FULL_MATCHED | 0.0 | 0.5984822352535357 | 1.0 | 0/0/0 |  |
| indoor_forward_7 | GV | VALID_FULL_MATCHED | 0.0 | 0.5984822352535357 | 1.0 | 0/0/0 |  |
| indoor_forward_7 | L | VALID_FULL_MATCHED | 0.0 | 0.5984822352535357 | 1.0 | 0/0/0 |  |
| indoor_forward_7 | L_GV | VALID_FULL_MATCHED | 0.0 | 0.5984822352535357 | 1.0 | 0/0/0 |  |
| indoor_45_2 | OFF | VALID_FULL_MATCHED | 0.0 | 0.7089175546831183 | 1.0 | 0/0/0 |  |
| indoor_45_2 | G | VALID_FULL_MATCHED | 0.0 | 0.7089175546831183 | 1.0 | 139/0/0 |  |
| indoor_45_2 | V | VALID_FULL_MATCHED | 0.0 | 0.7089175546831183 | 1.0 | 0/127/0 |  |
| indoor_45_2 | GV | VALID_FULL_MATCHED | 0.0 | 0.7089175546831183 | 1.0 | 139/127/0 |  |
| indoor_45_2 | L | VALID_FULL_MATCHED | 0.0 | 0.7089175546831183 | 1.0 | 0/0/126 |  |
| indoor_45_2 | L_GV | VALID_FULL_MATCHED | 0.0 | 0.7089175546831183 | 1.0 | 139/127/126 |  |
| outdoor_forward_1 | OFF | VALID_FULL_MATCHED | 0.0 | 0.5264586160108549 | 1.0 | 0/0/0 |  |
| outdoor_forward_1 | G | VALID_FULL_MATCHED | 0.0 | 0.5264586160108549 | 1.0 | 714/0/0 |  |
| outdoor_forward_1 | V | VALID_FULL_MATCHED | 0.0 | 0.5264586160108549 | 1.0 | 0/567/0 |  |
| outdoor_forward_1 | GV | VALID_FULL_MATCHED | 0.0 | 0.5264586160108549 | 1.0 | 714/567/0 |  |
| outdoor_forward_1 | L | VALID_FULL_MATCHED | 0.0 | 0.5264586160108549 | 1.0 | 0/0/565 |  |
| outdoor_forward_1 | L_GV | VALID_FULL_MATCHED | 0.0 | 0.5264586160108549 | 1.0 | 714/567/565 |  |
| outdoor_forward_5 | OFF | VALID_FULL_MATCHED | 0.0 | 0.1962864721485411 | 1.0 | 0/0/0 |  |
| outdoor_forward_5 | G | VALID_FULL_MATCHED | 0.0 | 0.1962864721485411 | 1.0 | 605/0/0 |  |
| outdoor_forward_5 | V | VALID_FULL_MATCHED | 0.0 | 0.1962864721485411 | 1.0 | 0/547/0 |  |
| outdoor_forward_5 | GV | VALID_FULL_MATCHED | 0.0 | 0.1962864721485411 | 1.0 | 605/540/0 |  |
| outdoor_forward_5 | L | VALID_FULL_MATCHED | 0.0 | 0.1962864721485411 | 1.0 | 0/0/537 |  |
| outdoor_forward_5 | L_GV | VALID_FULL_MATCHED | 0.0 | 0.1962864721485411 | 1.0 | 605/540/530 |  |

## 异常与解释边界

实际应用次数为零的分支没有生效，不能把相同 ATE 当作融合收益或压力测试通过。
本次所有输入均完整回放；条件 ATE 不替代完整有效结果；未调整参数、代表序列或评估支持。
构建准备曾有两次失败：未加载 ROS underlay 导致依赖检查退出 1；环境入口缺少执行权限导致退出 126。修正为 bash 入口加载 underlay 后独立构建成功；失败记录保留，失败产物未用于回放。
indoor_forward_7 / G：G 未实际应用；门控诊断 `{"G_reason:disabled": 278, "V_reason:disabled": 3177, "G_reason:time_or_token": 2899}`。
indoor_forward_7 / V：V 未实际应用；门控诊断 `{"G_reason:disabled": 3177, "V_reason:disabled": 278, "V_reason:time_or_token": 2899}`。
indoor_forward_7 / GV：G,V 未实际应用；门控诊断 `{"G_reason:disabled": 278, "V_reason:disabled": 278, "G_reason:time_or_token": 2899, "V_reason:time_or_token": 2899}`。
indoor_forward_7 / L：L 未实际应用；门控诊断 `{"G_reason:disabled": 3177, "V_reason:disabled": 3177}`。
indoor_forward_7 / L_GV：G,V,L 未实际应用；门控诊断 `{"G_reason:disabled": 278, "V_reason:disabled": 278, "G_reason:time_or_token": 2899, "V_reason:time_or_token": 2899}`。

源码 `21564dda04e4fd59013682e9bd95dc21d7fcc467`；原始输出 `/tmp/ltv_gain_scan_20261009`。
[结果及诊断](uzhfpv_results_data/results.json) · [运行身份与命令](uzhfpv_results_data/run_manifest.json) · [位置证据及复算](uzhfpv_results_data/evaluation_archive.json)
