# 三线进程与资源隔离

唯一共享协调根为 `/home/he/output/ltv_three_track_20261008_1827`，会话身份在 session.json。A/B/C worktree和tasks目录各自独立；git管理由主线程持git_admin.lock串行进行。用户未提交文件留在原工作树，实验源从2417d03新分支开始。

所有编译、MC、回放、性能均用 `tools/launcher_v2.py spec.json`；既有v1的原始尝试保留、不能事后补时序RSS资格。重任务host_heavy独占，然后性能gate，然后task_artifact；轻量检查取共享性能gate和本task锁。性能取独占gate，暂停数值轻量任务。整个子进程组退出之前不释放锁；中断wrapper会等待所属组终止，不让后台脱离保护。停止仅用stop_owned_group.py按owner、runID、PID/start_ticks、PGID再核验后SIGTERM，不宽泛pkill、不操作其他任务/用户组。

freeze_source.py从明确commit git archive发布task/source/OID只读树；构建指向该树，避免运行中工作树编辑改变构建/安装。每个新ABI使用fresh构建ID，build/install/log/tmp/results互不共享，禁止symlink install/公共tmp静态库。默认j1/所有数值线程1、单任务8GiB预算；实测A首次fresh Release构建约6.36min/RSS2.53GiB。

launcher清除继承的工作区前缀和LTV开关、固定空colcon defaults；env_exec.sh仅加载ROS Humble underlay和本任务local_setup，并拒绝foreign prefix/not-found libraries。实际环境与ldd在每run environment.json，所有进程包括exec-ready声明和/proc识别在registry。每次attempt时间+UUID目录，拒绝覆盖；stdout/资源/源码OID/dirty补丁hash/配置/输入/动态库身份保留。

A/B/C固定ROS domain171/172/173、namespace对应task，RMW fastdds；offline ASL命令不能标ROS消息传输回放。domain隔离不代替重任务锁。v2外部/proc每2秒采集RSS/HWM及进程身份，配合time峰值报告时序趋势，不把单峰值当无增长证明。
