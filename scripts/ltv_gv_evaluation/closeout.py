"""Archive reviewable summaries and bind external raw evidence without committing large files."""
import argparse
import json
from pathlib import Path
import subprocess
from run_matrix import exact, write

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out",type=Path);ap.add_argument("doc",type=Path)
    args=ap.parse_args();out=args.out.resolve();doc=args.doc.resolve()
    metrics=json.loads((doc/"metrics.json").read_text())
    events=json.loads((doc/"events.json").read_text())
    evidence=doc/"evidence";evidence.mkdir(exist_ok=True)
    names=["build_attempt_01.json","build_attempt_02.json","build_attempt_03.json","build_attempt_04.json","build_attempt_05.json","build_attempt_06.json","build_artifact_repair.json","engineering_attempts.json","build_identity.json","tests.json",
           "matrix_status.json","budget_before.json","budget_after.json","compiled_source.json","freeze.json"]
    names += [p.name for p in out.glob("parity_*.json")]+[p.name for p in out.glob("repeat_*.json")]
    for name in names:
        if (out/name).exists():
            (evidence/name).write_bytes((out/name).read_bytes())
    write(evidence/"final_runs.json", {p.name: str(p.resolve()) for p in (out/"final_runs").iterdir()})
    (evidence/"attempts.jsonl").write_bytes((out/"attempts.jsonl").read_bytes())
    manifest={}
    for path in sorted(out.rglob("*")):
        if path.is_file() and not path.is_symlink() and not any(x in path.relative_to(out).parts for x in ("build","install","log")):
            manifest[str(path.relative_to(out))]=dict(bytes=path.stat().st_size,sha256=exact.sha(path))
    write(evidence/"external_manifest.json",dict(external_root=str(out),files=manifest))
    lines=["# G/V 融合评测收尾","","冻结基线 83c71cbc585123381f033c8dbcd625c14e8e7fac。"]
    status=json.loads((out/"matrix_status.json").read_text())
    tests=json.loads((out/"tests.json").read_text())
    passed=len(status)==12 and all(status.values()) and all(r["exit_code"]==0 for r in tests)
    lines += ["",f"工程验证：{'通过' if passed else '存在失败/未完成'}。实际 ROS/colcon 构建及 {len(tests)} 个相关原生测试完成；十二次完整真实回放及预算记录见 evidence。",
              "生产 G/V 默认关闭。所有配置未调参。收益、工程有效性和统计一致性分开判断。",
              "", "| 序列 | 模式 | ATE m | 1s RPE m / deg | 5s RPE m / deg | 覆盖 / 样本 | 实际 G / V 更新 |",
              "|---|---|---:|---|---|---|---|"]
    for m in metrics:
        key=m["sequence"]+"/"+m["mode"];count=events[key]["counts"];rpe=m.get("rpe",{})
        def rp(dt):
            v=rpe.get(dt,{})
            return f'{v.get("translation_rmse_m",float("nan")):.6f} / {v.get("rotation_rmse_deg",float("nan")):.6f}'
        lines.append(f'| {m["sequence"]} | {m["mode"]} | {m.get("ate_rmse_m",float("nan")):.6f} | {rp("1.0")} | {rp("5.0")} | {m.get("coverage",0):.2%} / {m.get("samples",0)} | {count["G"]["actual_updates"]} / {count["V"]["actual_updates"]} |')
    attempts = [json.loads(line) for line in (out/"attempts.jsonl").read_text().splitlines()]
    finished = [r for r in attempts if r["event"] == "finish"]
    failed = [r for r in finished if r["exit_code"]]
    lines += ["",f"本轮共 {len(finished)} 次真实回放进程启动，{len(finished)-len(failed)} 次完整成功，{len(failed)} 次早期工具失败。原预算从38次真实回放增加到{json.loads((out/'budget_after.json').read_text())['real']}次。所有失败、修复和重验均留档。",
              "八次 G/V 早期启动在两轮中被配置校验和 observer 构造器内的旧 Passive 注入断言拦截，未读取传感器输入。完整检索并将配置、adapter、manager 中全部仅 Passive 注入断言接入同一仪器开关，保留其他校验。两次修复各以新目录重跑两序列原 Passive 与融合 OFF，最终执行 G/V/GV 和重复。",
              "编译时诊断结构补充与 UpdaterLTV 对象编译重叠，曾导致空视觉行计数测试失败；显式重编译该对象后24测试通过。修复配置断言后重新构建、24测试再次通过。首次沙箱构建在子进程退出后等待停滞而中断；所有构建尝试/原生测试/评价器自测日志保留。没有按误差调参。",
              "", "原 Passive OFF 对已有冻结 OFF、新工具 OFF 对原 Passive 的逐字段/逐字节比较均保留在 parity 文件；只排除既有顶层 compute_time_ms。GV 重复的主状态、完整 P 摘要、完整 LTV x/P/输入缓存、生命周期、Consistency、readiness、融合事件和原始矩阵要求精确一致，见 repeat 文件。",
              "", "仪器差异：原 Passive 保留零注入断言；融合工具关闭该断言并启用只读完整矩阵捕获，OFF/G/V/GV 覆盖相同。详细数学/相关性/异常合同见 [update_contract.md](update_contract.md)。"]
    for seq in sorted({m["sequence"] for m in metrics}):
        base=next(m for m in metrics if m["sequence"]==seq and m["mode"]=="OFF")
        lines += ["",f"## {seq}","","| 模式 | 相对 OFF ATE 变化 | 首次主均值 / P / LTV 分歧时间 ns | 首次主分歧对应实际融合 |",
                  "|---|---:|---|---|"]
        for mode in ("G","V","GV"):
            m=next(m for m in metrics if m["sequence"]==seq and m["mode"]==mode)
            d=events[seq+"/"+mode]["first_divergence"]
            times=" / ".join(str(d[k]["camera_ns"]) if d[k] else "无" for k in ("main_digest","covariance_digest","observer_digest"))
            first=min((d[k] for k in ("main_digest","covariance_digest") if d[k]),key=lambda v:v["camera_ns"],default=None)
            change=(m["ate_rmse_m"]/base["ate_rmse_m"]-1)*100 if m.get("ate_rmse_m") is not None else float("nan")
            lines.append(f'| {mode} | {change:+.3f}% | {times} | {first["actual_fusion"] if first else "无数值分歧"} |')
        lines += ["",f"![完整轨迹误差曲线]({seq}_trajectory.png)",
                  "",f"![GV 全时间事件曲线]({seq}_GV_events.png)",
                  "",f"G-only/V-only/OFF 全时间事件曲线分别见 [{seq}_G_events.png]({seq}_G_events.png)、[{seq}_V_events.png]({seq}_V_events.png)、[{seq}_OFF_events.png]({seq}_OFF_events.png)。"]
    lines += ["", "## 证据和边界", "",
              f"外部原始输出根目录：{out}。完整矩阵、缓存、输入与运行日志留在外部；external_manifest.json 记录身份，compiled_source.json 绑定实际编译源。每次运行 identity.json 含命令、退出码、输入/二进制/源身份和单线程环境；本轮 attempts.jsonl 关联原 R4 账本 id。",
              "完整逐时刻数据见各 run 的 fusion.jsonl、features.jsonl、audit.csv、matrices.jsonl 和 cache.bin；轨迹曲线 CSV 位于外部根目录。events.json 保留每分支请求、原因、门控尝试/拒绝、通过门控、提交拒绝与实际更新计数，以及首次分歧的完整事件。",
              "指标使用固定 OFF 初始化支持，20ms GT 最近邻、1μs 输出匹配、SE(3) 无尺度对齐和原覆盖/定位失败规则。metrics.json 保留样本、覆盖、RPE 对数、状态及原始指标；GT 仅离线读取。",
              "R4 NOT_ACHIEVED_WITH_EVIDENCE 保留：30–31.5s 恢复供给中断导致总恢复验收未通过，201/202 合成案例缺同身份 OFF 和每条件两种子的成对确认；未在本轮补做。",
              "融合采用块对角伪测量噪声，没有 LTV G/V、LTV-main、LTV-visual 交叉相关性。主完整 P 的使用及 ATE 收益均不证明统计一致性；两序列结果不扩展为所有 EuRoC 或飞行上线验收。",
              "下一阶段仅交付 [landmark_contract.md](landmark_contract.md)，不实现或运行 landmark 融合。当前 max_slam=0；启用 SLAM 必须另立匹配基线。",
              "", "## 复现命令", "", "先按 evidence/build_identity.json 中的实际 colcon 命令创建独立 install，然后：", "", "~~~bash",
              "python3 scripts/ltv_gv_evaluation/verify_install.py /path/to/fresh/output",
              "python3 scripts/ltv_gv_evaluation/run_matrix.py /path/to/fresh/output",
              "OPENBLAS_NUM_THREADS=1 python3 scripts/ltv_gv_evaluation/evaluate.py /path/to/output docs/ltv/gv_fusion_evaluation",
              "~~~", "", "fresh/output 必须预先具有同输入身份的 freeze.json、build.sh 和独立构建；运行目录不允许复用。失败/重跑使用新目录与新账本 id。冻结文件包含源哈希、原 OFF 哈希、校准、CSV 与全部图片聚合身份。"]
    (doc/"report.md").write_text("\n".join(lines)+"\n")

if __name__=="__main__":
    main()
