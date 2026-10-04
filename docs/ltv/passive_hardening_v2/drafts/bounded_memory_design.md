# 有界身份与生命周期内存：最小可实施方案

本稿是只读设计；未改manager、adapter、core或运行实验。前提是本轮固定成功合同，不以清空身份历史、周期性重启或降低15点门槛制造覆盖。

## 1. 推荐组合及不推荐的简化

**推荐：manager固定容量Bloom身份保护＋有限精确活体表；adapter仅保留活动映射；core使用明确授权的本地单调ID模式。** 不要求外部tracker ID单调。

目前无界结构：manager `landmarks_ / retired_`、adapter `ids_`、core `admitted_ids_`。history虽限制候选数和时间窗，`samples`没有显式最大条数：仅凭1秒时间窗不能限制任意高频有效时间戳下的条数，也应补一个固定每点样本上限（20Hz目标对应21条含端点，桥接允许的更高频率需显式配置）。每帧的原始输入包需来自已知有限前端预算；如果API允许任意大小vector，还须验证输入包最大记录数而非复制后才拒绝。

不推荐单独“退休TTL后删记录”：删完无法区别同epoch旧ID重现与新ID，可能二次seed。不推荐默认为maxID窗口：当前KLT新检测确实`++currid`，但输入缓存、异步相机首次出现顺序、其他tracker/地图ID并没有统一声明该合同。窗口会拒绝合法低ID；不能把这种损失伪装成匹配不可信。

## 2. manager的固定空间身份保护

初始实现建议参数（须主agent纳入候选配置后使用）：

- 当前精确记录总上限256，含候选、SEED_READY、ACTIVE和COASTING；active≤30。保留原history容量/TTL，活动点受保护，不为均匀分布逐出。
- 每个observer epoch一份 **2^23 bit（1 MiB）、7位置** 的非删除Bloom集合，记录“本epoch已经准入或已明确永久退出的原始ID”。固定64位混合函数与salt写入版本；不使用进程随机hash。
- 查询顺序为精确活体表优先，然后Bloom：已有活体（包括暂时COASTING）不因Bloom命中被拒绝。陌生ID若命中则拒绝，理由 `identity_seen_or_conservative_collision`，不能宣称每次都是真旧ID。
- 新准入在原子事务成功后插入Bloom；同frame先完整验证全部birth，再一起提交，避免排序影响。失败事务不能占用身份bit、递增累计准入或写seed。
- 未准入候选TTL/不可恢复身份错误退出时也插入Bloom，输出一次typed退出事件，然后删除精确记录和对应history。已准入退休输出 `ACTIVE_RETIRED`；从未准入TTL输出 `CANDIDATE_TTL`，不能混报。
- 未入缓存的容量溢出ID不曾seed，不必全部插入Bloom；输出容量拒绝receipt，后续若重新出现可作为候选。这与“已接纳又退休”严格分开。
- 累计birth/opportunity/admitted/active_retired/candidate_ttl等在线值在状态迁移时更新标量。完整身份历史只进事件日志；全部raw候选的去重分母由离线日志计算，不再以无限在线集合实现统计。

非删除Bloom对已经插入的ID没有假阴性（假设内存未损坏且所有路径都使用相同hash/epoch），所以误判只会额外拒绝、不会允许重复seed。不能通过定时清bit、轮换两张Bloom、饱和时重置或退役计数递减来保持低误判率——这些操作会产生遗忘。记录set-bit比例/插入数/由Bloom拒绝包数；饱和造成覆盖不足是需要报告的负结果，不是触发质量重启的理由。

近似假阳性率 `(1-exp(-7*n/8388608))^7` 仅是均匀hash模型下的容量规划，不是实测概率保证：

| 插入身份数n | 例子/压力尺度 | 近似假阳性率 |
|---:|---|---:|
| 36,000 | 30活动点、0.5秒全部更替、10分钟 | 1.99e−11 |
| 76,800 | 256缓存、2秒TTL、10分钟的简化候选周转 | 3.55e−9 |
| 440,000 | 高周转压力设计点；不是通用数学上界 | 0.0259% |
| 1,000,000 | 极高周转/长episode | 1.86% |
| 3,072,000 | 256全新ID/帧×20Hz×600s的raw输入压力 | 57.1% |

最后一行解释为什么不能把所有容量拒绝ID也无差别插入集合。真实10分钟测试应报告实际插入数、拒绝率和coverage，而非引用该公式宣称无损。1MiB在pipeline staging中可先直接复制，20Hz约20MiB/s数据复制量；先测完整支路开销，再决定是否对固定bit数组做事务增量，不为节省复制引入共享写入破坏原子性。

## 3. adapter与core：只对自己生成的ID声明单调

adapter保留原始ID→core ID映射只到该点离开retained集合，不保存退休映射。新映射由单调、本episode不回绕的本地计数分配；这是adapter自己能证明的合同，不是对外部feature ID的假设。

core新增**默认关闭**的受控本地单调birth-ID模式。该模式不保留无限`admitted_ids_`，改为一个`max_committed_birth_id`：

1. 所有新增birth ID都必须大于事务开始前高水位；birth集合内部必须唯一。
2. 原来已经存在的低ID槽继续合法，不参与新birth高水位比较。
3. 完整控制事务成功才将高水位更新为整批birth最大值。风险/象限排序可以任意，不要求同批按ID递增。
4. 退休core ID再次出现，因≤高水位且不在当前槽表而拒绝。旧核心/旧控制模式保持原行为，新模式单独验证。
5. adapter counter溢出明确报错，不重用int值；不能借溢出定时reset掩盖覆盖。10分钟600birth/s也仅36万，远低于int32上限，但接口必须有边界测试。

这样manager负责“不把同一个原始ID重新包装成新core ID”，core负责“同一个本地ID不重复birth”；两道保护缺一不可。仅core高水位不能阻止adapter给旧原始ID发新编号。

## 4. epoch与恢复：不能用30秒重启清理内存

分别记录frontend身份epoch、observer bootstrap episode、相机receipt序号。Bloom的key必须明确为 `(observer_episode, original_feature_id)`；合法完整observer重建允许同一原始ID在新episode一次初始化，但必须由readiness触发、同事件原子提交并记录bootstrap来源。

- 不允许因Bloom饱和、ID表容量或时间达到30秒主动重建。
- 30秒是质量重建的**最小间隔**，不是定时周期。供给不足时保持DORMANT；没有恢复供给不发bootstrap请求。
- COLLECTING/短缺测不能反复reset history/Bloom。真正物理输入故障单独统计，不能为了清身份保护而伪造故障。
- 在DORMANT准备新episode时，可使用独立待提交的有限candidate历史与新Bloom；直到完整bootstrap成功才切换episode和身份集合。失败时保留原episode的保护与错误日志。
- 断点恢复须保存固定Bloom bit数组、高水位、活体表、计数、时钟及pending事务身份；只从新日志重播恢复也必须明确最后提交边界。不能只恢复x/P而遗失身份保护。

## 5. 必测不变量与10分钟验收

小测试覆盖：任意非单调原始ID、同帧无序birth、低ID长寿命活体、COASTING短返、退休后低/高ID重现、候选TTL后重现、容量拒绝后再出现、hash碰撞导致额外拒绝但无重复seed、失败事务不置bit/不推进高水位、不同episode同名ID、同episode重复ack、counter溢出。

10分钟20Hz恒定有界输入流至少包含：0.5秒全更替、间歇重复退休ID、从未获准TTL轨迹、恢复段；阶段标签仅在生成/评价器。每帧检查：

- active≤30、全部精确manager/history≤声明上限、每点sample≤固定上限、adapter映射≤retained规模、core无限admitted容器在新模式为空。
- Bloom内存恒定，bit只增不减；旧episode历史只在明确完整重建处切换。
- 每个 `(episode,id)`最多一次seed，退休与TTL typed事件计数可由日志精确重建；false-positive拒绝另计。
- 质量bootstrap间隔≥30秒且没有“每30秒自动重启”；正常不缺测输入不能为减内存重建。
- 同样有界活动规模下，最后一分钟RSS/每帧CPU不呈总历史条数线性增长；记录容器容量本身，而不只依赖分配器RSS曲线。
- 全量日志流式写外部文件，测试工具不得把10分钟所有诊断对象留在同一进程内再声称被测模块内存有界。

本设计提供有限内存与不会重复seed的工程路径，不保证Bloom零误拒绝，也不替代LTV输出精度、readiness及主VIO旁路验收。
