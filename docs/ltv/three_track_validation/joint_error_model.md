# 联合误差模型：已实现范围与接入限制

实现包含实际离散 Observer 的 mean/P_R/gain、bearing 归一化、seed、固定分支谱投影敏感度，以及主状态/历史像素/共享 IMU 端点/anchor 的源因子传播。主状态程序扰动、真实误差 chart、内部 P_R 与存储 P 分开；QR 同时保留主状态、Observer mean/Riccati、anchor 和活跃源块。

本轮真实运行闭合到独立单槽、固定标定/时钟、常真 bias、声明初始化先验与 nominal 主 H/K 的条件模型。生产30槽耦合、实际初始化源、随机主增益/选择、未分类 missing 事件和真实均值仍未闭合，不能消费为生产相关更新。

完整公式和代码映射见 [C 模型](task_c/joint_error_model.md)、[有限模型合同](task_c/finite_model_contract.md) 与 [冻结接口 v2](task_c/model_interface_v2.json)。接口的相关核使用显式同 chart 的 R_L/N_L/visual cross 和残差减声明均值，检查输入有限/对称/joint PSD及后验 PSD；没有 State mutation，也未授权真实 correlated ON。

运行源码为 `6982e0473b6963ab95dfb169b585c09f464bfa8e`，最终交付分支为 `experiment/ltv-three-track-c-20261008`、OID `46d0a9df8b8438a9f6146e0861e05c9a312929bc`。分析、核函数和纯格式化版本单独在接口/manifest 登记；不修改或重建在跑产物。
