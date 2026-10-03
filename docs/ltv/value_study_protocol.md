# LTV value study 评价协议

以[value_study_goal.md](value_study_goal.md)为执行合同。状态：输入与实现验收中，尚未运行效果实验或解封验证结果。

时间、坐标、GT导数窗口、分层、置信区间、权重候选与门槛已在合同冻结。后续仅追加源码/数据核验证据和明确实现细节；不得根据效果改写协议。

- 原则：同机体系比较不做ATE旋转拟合；ATE无尺度SE3，RPE为1s/5s。
- 现有证据：本地UZH16条GT均为8列姿态位置；图像/IMU格式区别已确认。
- 官方来源：https://fpv.ifi.uzh.ch/ 、https://github.com/uzh-rpg/uzh_fpv_open 、https://rpg.ifi.uzh.ch/uzh-fpv/ICRA2020/reports/open-vins.pdf 。

## A001：亚纳秒文本时间（输入预检，尚未效果回放）

部分UZH文本含浮点输出尾数，Decimal乘1e9并非整数。采用nearest-ns / ties-to-even，最大舍入误差0.5ns，保留原文本SHA及每序列最大误差；单流舍入后严格递增，任何碰撞拒绝。双目在整数键相等后还检查原Decimal值必须相等，禁止舍入制造同步。不是放宽LTV的时序容差。

## A002：参考量与配置来源（效果分析前）

仓库三类UZH相机标定所有几何、内参、畸变、时差字段与本地官方副本一致。IMU原配置噪声及update_rate与校准采集设置不同，按合同保留仓库估计器噪声、不以ATE调整；实际时间积分使用样本时间。各模式一致，calibration_audit.json保存两套值。

官方2022 GT生成方法为视觉/IMU/全局位置批优化；姿态与导出速度不能当作完全独立的传感器真值。官方方法B=IMU、W z轴与重力对齐；原始GT姿态相对旋转与机体陀螺同轴的数值检查通过。gt_preflight.json记录原始IMU与GT姿态角速度误差及p二阶导-Ra的重力符号；这只是约定检查，不证明亚度精度，也不消除bias/GT误差。

来源：https://rpg.ifi.uzh.ch/docs/RAL2021_Cioffi.pdf 第III节；https://raw.githubusercontent.com/uzh-rpg/uzh_fpv_open/master/scripts/compare_gyro.py 。GT不做对齐以修正重力轴，不优化时差来迎合结果。

ATE继承旧evaluator最近GT<=20ms、无尺度SE3；机体系机制用SLERP/导数的严格10ms间隙规则。RPE以1/5s最近输出对，时间差<=25ms。独立标记巨大定位漂移：ATE>max(5m,GT包围盒对角线)，不隐藏原数值；此工程标记在任何新真实ATE分析前规定。

分层时长按原始事件时间步长计，稀疏筛选不把事件间空档算作有效观察时长。B/P逐事件证据涵盖状态/P/FEJ/clone和视觉集合。

## A003：V1_01原始姿态GT限制

OpenVINS官方支持数据集文档明确指出V1_01_easy原始姿态GT不准确（https://docs.openvins.com/gs-datasets.html）。本轮按只读合同继续保留原GT和既有ATE，V1_01的G直接误差与姿态指标仅作为参考敏感性结果，不能单独支持G有效/无效结论。V的body-frame参考也依赖姿态，需同样说明。候选资格仅由UZH三开发序列决定，因此此来源限制不改变选参门槛。

## A004：本地UZH GT发布版本逐字节验证

官方当前datasets页面链接v3 ZIP。通过HTTP Range只读取ZIP目录和groundtruth.txt（未重下载图像），6条选定UZH GT的SHA256均与本地原文件一致；ZIP GT时间为2021-12-20，与官网2022新GT发布相容。完整URL、SHA、ZIP成员时间和下载字节记录在输出根provenance/gt_release_verification.json。原始GT未替换。此证据确认版本来源，不证明重建GT没有误差或信息独立。

## A005：独立短段成本插桩

正式矩阵保持冻结的估计器源码、库和二进制。为分开辅助构造/原生EKF/压缩写入耗时，另外构建可选LD_PRELOAD计时包装器，原函数仍按原参数执行；仅用于V1_01与indoor_forward_3的GV短段，分别诊断ON/OFF，共4次计入20次短段预算。先用原生shadow测试核验包装器，再与既有未插桩GV短段逐字节比较轨迹和完整审计。不开新权重，不据此筛选正式效果结果。

包装器最初按C++14编译遇到本机ROS2头要求C++17，失败日志保留；随后匹配已冻结ROS2目标实际使用的C++17（flags.make），C++源码未使用新增语言特性。ROS1未构建。正式构建配置未变。

实验恢复脚本的可选profile参数曾短暂写入磁盘，而正在运行的final进程仍使用原已导入模块。两条run中runner_sha因此表示启动时磁盘文件而非已导入源码；原manifest未改，provenance/runner_loaded_source_note.json逐条注明，保存已导入源码且SHA与final首条一致。数值命令和冻结估计器身份未变；磁盘脚本已恢复，插桩参数待final退出后再启用。
