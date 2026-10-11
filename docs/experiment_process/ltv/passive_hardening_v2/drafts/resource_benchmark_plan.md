# R3 流式资源验证设计（尚未运行）

600秒、200Hz IMU、20Hz相机，直接调用实际C++ Adapter ABI。此运行按主账本计完整合成，不以“资源测试”规避预算。生成器与评价器可知道阶段，Adapter不接收阶段/未来寿命。不是新的独立精度样本。

被测进程不能调用会收集全部states/P的常规 `synthetic_runner.run()`。只使用其 `Adapter` 封装，每帧维护至多21个pose/噪声状态、固定点池与小型累加器，输入与事件以JSONL逐帧外写。诊断关闭通过 `ph_set_diagnostics(false)` 实际跳过全seed/全track展开和全矩阵快照；保留轻量birth/typed retirement/拒绝ID和容量字段，足够离线重建一次seed。

输入计划：可重复、固定标定的近距同步双目供给，0.5秒身份更替；加入确定性旧退休ID重现、一直未达准入条件并最终TTL的候选，及预登记有限空观测和恢复区间。静态或可重复有限轨迹只作资源压力，不声称姿态/速度PE或新精度证据。对应GT/negative/phase不进C++接口。固定容量拒绝与Bloom保守误拒必须如实计数。

记录分层：

- `compute_time_ms` 是 Adapter::processHardened 内部计时，不含桥接构造/cache/logger，不能单独代替完整相机支路耗时。
- 外层另测输入pose联合P构造、整次C ABI相机调用（含轻量序列化）、Python解析与磁盘写入；分别输出，不能将重日志耗时记为纯估计耗时或反过来。
- 每分钟RSS、固定容量/最大容器规模、CPU均值/极值；每事件耗时流式落盘，精确P95由独立离线分析读日志，不在被测进程保存600秒对象。
- 容器检查：manager/history≤256、每轨sample≤21、adapter映射≤retained≤30、core旧admission集合为空、本地highwater单调、IMU≤4096、guard固定1MiB且epoch内bit只增；显式epoch切换另记。
- 完整历史去重和typed事件核对由第二个离线阶段执行，不在被测进程建立无限 `(epoch,id)` 集合。每个epoch身份不得再次seed；TTL未准入与已准入退休分别核对计数。
- 前/后一分钟资源趋势同时报告绝对容量界限，不能仅凭RSS曲线平坦断言无界容器不存在。

先用短fixture比较诊断开/关x和IDs、raw/ready/birth数值完全相同，再由主分配600秒预算。新版wrapper只链接同版冻结runtime，不链接旧库冒充验证R3。
