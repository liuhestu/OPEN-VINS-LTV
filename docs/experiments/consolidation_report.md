# experiment/main 收敛与等价整理验收

已汇集 A/B/C 到完整 experiment；main 从共同整合代码起点复制后仅整理入口、文件位置和配置。保留数值计算、参数、门控、更新顺序、数值保护与异常分支。此前实验 FAIL/STOP、生产默认 OFF 均不改写，不宣称获得新组合资格。

## 分支与恢复

- 本地/远端仅保留 main、experiment，旧分支按归档 tip 做 lease 保护删除。
- experiment：`e9a14e6a2ae8963112322ed3b69b68d17fb288e1`，共同算法基线：`66d9eb327182ecbb741a1137d35d0df62b9aab3c`。
- 原37个本地/远端引用已归档为远端标签并逐个核验；Git bundle 已verify。恢复映射见 branches.json。main最终OID由交付核验记录，避免文档自引用其commit hash。
- 两个有修改的审计worktree及只读/有忽略内容的历史目录保留为detached，未强制清理。可移除的干净worktree已移除；大原始结果未删除。只读基线移除失败后恢复了原登记并核对source clean。

## 必要整理

研究脚本在docs/experiments/tools，原包层次与参数格式保留。A/B指标判定及评价入口去重；三个旧运行器退出当前构建，源码仍可通过归档准确复现。缓存shadow有独立用途，保留而非合并成行为不同的入口。

四个研究驱动由默认OFF的BUILD_LTV_EXPERIMENTS控制；原数值单测开关保留。必要缓存/矩阵记录与研究挂点仍按原条件运行，涉及估计器策略的诊断没有删除。六个只读联合诊断文件归入diagnostics/joint；不移动执行挂点或改变先后顺序。

A/B三套完全同字节的C0 YAML合为一套，生产数据集配置路径不变。原科学报告/合同/manifest不改写；旧命令指向其原归档版本。

## 实际验收

- 整合基线及整理main全新独立ROS Humble Release构建通过（j1、数值优化flags相同，g0仅控制调试信息）；无混用overlay或运行中重建。
- 两套各18个原生契约检查通过，涵盖Jacobian、布局、协方差、FEJ、状态生命周期及完整敏感度。默认OFF配置的CMake目标列表确认不含研究运行器。
- 80个估计器C++源/头文件在include重定位与格式化归一后相同；函数体、常量、接口和数值流程未重写。
- V2_02/V2_03 × OFF/G/V/GV/L_OFF/L_ON：24次完整实际回放、12组整理前后配对全部零容限一致。主状态/P、Tracker、Observer、生命周期、readiness、receipt、轨迹与所有scalar LTV列逐项/逐字节一致。只排除feature compute_time_ms和明确的诊断输出路径，integrated_seconds没有排除。
- 两条DEV × 两源码：4次完整finite shadow；原始输入、完整cache、主状态/P/Observer以及finite scalar/matrix/bin/JSON字节精确一致。
- 可复核证据：equivalence.json、shadow_equivalence.json、native_checks.json及consolidation_runs.csv。运行权限预检的退出126保留为FAILED，不当构建成功；更正可执行权限后的新attempt构建通过。

这些是结构整理等价性，非新的精度收益、性能或统计校准资格。数据仍为同步ASL真实原始流，不冒充ROS transport回放。
