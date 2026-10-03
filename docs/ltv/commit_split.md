# LTV 提交拆分与实验版本对应

本次只整理 Git 历史、报告与证据归档。实验冻结代码仍是 `c7fc29b11352da3104c80462bcdd32553e785085`，备份引用为 `archive/ltv-frozen-c7fc29b`。原实验 manifest、指标、GT、golden 和容差不变。

## 提交对应

```text
3facbf4 fix(ltv): avoid covariance symmetrization aliasing
eada1b9 feat(ltv): add persistent observer adapter and options
de711f5 feat(ltv): build and validate gravity velocity measurements
0327313 feat(msckf): integrate LTV joint updates into VIO
c50ba16 feat(eval): add reproducible LTV EuRoC experiments
4955ca8 docs(ltv): record alias fix and native acceptance
a076838 docs(ltv): document integration decisions and acceptance
```

随后文档提交 `docs(eval): report frozen EuRoC results and limitations` 收录本记录和最终结果。

前五个提交的最终源码、配置、构建文件及实验脚本与旧冻结提交逐文件相同；拆分只改变提交历史。Phase 0/1 的历史报告不代表在这些新中间提交上曾完成旧全量实验。

## 验证

五个代码提交分别在独立目录构建 ROS2 ov_msckf，复用未修改的 ov_core/ov_init 已安装依赖。按依赖运行相关测试，最终执行全部十个原生测试；核心新旧 alias 对照及 174 事件完整 state/P 原容差对照通过。没有新跑数据集，也没有宣称重新验证 ROS1。

本次逐命令记录、构建和测试 stdout/stderr：`/home/he/output/ltv_commit_split_20261003`。结构化结果见 [拆分验证记录](evidence/commit_split_validation.json)。

## 日志与复现

生成日志与压缩原始输出不纳入此次提交，原字节保留在外部归档；路径与 SHA 见 [外部产物索引](evidence/external_artifacts.json)。已有历史哈希文件描述的是当时产物，未重写。拆分前全部未提交报告与证据的原件位于 `/home/he/output/ltv_commit_split_20261003/snapshot`，对应哈希为其上级目录的 `snapshot_hashes.json`；原 deliverable manifest 中的报告哈希应对照这些历史原件。仓库保留结构化结果、命令、测试源码和失败记录。

现有运行器严格匹配 HEAD，因此不能在拆分后的分支直接续跑旧冻结实验。需要复现时，在独立 worktree 检出 `archive/ltv-frozen-c7fc29b`，使用原输出根、输入和安装库运行原命令。不得把旧 manifest 的 git_head 换成新提交来绕过检查。本次未执行任何新回放。
