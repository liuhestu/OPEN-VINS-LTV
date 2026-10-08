"""Frozen full-input matrix; append-only attempts, two single-threaded compute jobs."""
import argparse
import concurrent.futures
import importlib.util
import json
import fcntl
import os
from pathlib import Path
import subprocess
import threading
import time
ROOT = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location("exact", ROOT / "docs/experiments/tools/ltv_integration_cleanup/check_real_outputs.py")
exact = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exact)
budget_spec = importlib.util.spec_from_file_location("r4_budget", ROOT / "docs/experiments/tools/ltv_active_consistency_r4/budget.py")
budget = importlib.util.module_from_spec(budget_spec)
budget_spec.loader.exec_module(budget)
LOCK = threading.Lock()
ENV = {k: v for k, v in os.environ.items() if k not in (
    "CMAKE_PREFIX_PATH", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "LD_LIBRARY_PATH", "PYTHONPATH")}
ENV.update(OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1")

def write(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")

def command(out, binary, args):
    shell = 'source /opt/ros/humble/setup.bash && source "$1/setup.bash" && exec "$2" "$' + '{@:3}"'
    return ["bash", "--noprofile", "--norc", "-c", shell, "gv",
            str(out / "install"), str(binary), *map(str, args)]

def append(out, row):
    with LOCK, (out / "attempts.jsonl").open("a") as f:
        f.write(json.dumps(row, allow_nan=False) + "\n")
        f.flush()

def run(out, seq, mode, index=1, off_index=1):
    frozen = json.loads((out / "freeze.json").read_text())
    inp = frozen["inputs"][seq]
    for name, digest in inp["config_sha"].items():
        assert exact.sha(Path(inp["config"]).parent / name) == digest
    for v in inp["sensors"].values():
        assert exact.sha(v["path"]) == v["sha256"]
    binary = out / "install/ov_msckf/lib/ov_msckf" / (
        "run_ltv_feature_passive" if mode == "PASSIVE" else "run_ltv_gv_evaluation")
    dest = out / "runs" / f"{seq}_{mode}_{index:02d}"
    dest.mkdir(parents=True, exist_ok=False)
    cmd = command(out, binary, [inp["config"], inp["root"], dest, "P_NEW" if mode == "PASSIVE" else mode])
    identity = dict(sequence=seq, mode=mode, index=index, command=cmd, cwd=str(dest),
                    binary_sha=exact.sha(binary), input_identity=inp, source_manifest_sha=exact.sha(out / "compiled_source.json"),
                    threads={k: ENV[k] for k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS")},
                    started=time.time(), prior_ledger=frozen["prior_ledger"])
    slot = None
    for i in range(2):
        handle = (budget.OUT / f"heavy_{i}.lock").open("a")
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            slot = handle
            break
        except BlockingIOError:
            handle.close()
    if slot is None:
        raise RuntimeError("Both existing heavy slots occupied")
    attempt = budget.reserve("real", cmd, dict(phase="engineering", study="gv_fusion_evaluation", **identity))
    identity["existing_ledger_id"] = attempt
    attempt_dir = budget.OUT / "attempts" / f"{attempt:04d}"
    attempt_dir.mkdir()
    write(attempt_dir / "source_sha256.json", budget.base.source_identity())
    write(attempt_dir / "command_files_sha256.json", {str(binary): exact.sha(binary), inp["config"]: exact.sha(inp["config"])})
    append(out, dict(event="start", **identity))
    write(dest / "identity.json", identity)
    with (dest / "stdout.log").open("w") as stdout, (dest / "stderr.log").open("w") as stderr:
        p = subprocess.Popen(cmd, cwd=dest, env=ENV, stdout=stdout, stderr=stderr, pass_fds=(slot.fileno(),))
        with budget.locked():
            budget.append(dict(event="started", id=attempt, pid=p.pid, process=budget.process_identity(p.pid),
                               owner=budget.process_identity(os.getpid()), time=time.time()))
        identity["pid"] = p.pid
        write(dest / "identity.json", identity)
        identity["exit_code"] = p.wait()
    identity["seconds"] = time.time() - identity["started"]
    with budget.locked():
        budget.append(dict(event="finished", id=attempt, exit_code=identity["exit_code"],
                           seconds=identity["seconds"], time=time.time()))
    slot.close()
    write(dest / "identity.json", identity)
    append(out, dict(event="finish", **identity))
    print(json.dumps(dict(run=str(dest), exit_code=identity["exit_code"], seconds=identity["seconds"])), flush=True)
    if identity["exit_code"]:
        return False
    if mode == "PASSIVE":
        write(out / f"parity_{seq}_PASSIVE.json", exact.compare(frozen["off"][seq]["path"], dest))
    if mode == "OFF":
        write(out / f"parity_{seq}_OFF.json", exact.compare(out / "runs" / f"{seq}_PASSIVE_{off_index:02d}", dest))
    if mode == "GV" and index == 2:
        before = out / "runs" / f"{seq}_GV_01"
        result = dict(audit=exact.csv_exact(before / "audit.csv", dest / "audit.csv"),
                      trajectory=exact.csv_exact(before / "trajectory.csv", dest / "trajectory.csv"),
                      unmatched=exact.csv_exact(before / "unmatched_camera.csv", dest / "unmatched_camera.csv"),
                      features=exact.jsonl_exact(before / "features.jsonl", dest / "features.jsonl"),
                      cache=exact.cache_exact(before / "cache.bin", dest / "cache.bin"))
        for name in ("fusion.jsonl", "matrices.jsonl", "effective_options.json", "replay.json", "instrumentation.json"):
            assert exact.sha(before / name) == exact.sha(dest / name), name
            result[name] = exact.sha(dest / name)
        write(out / f"repeat_{seq}_GV.json", result)
    view = out / "final_runs"
    view.mkdir(exist_ok=True)
    alias = view / f"{seq}_{mode}_{2 if mode == 'GV' and index == 2 else 1:02d}"
    alias.symlink_to(dest)
    return True

def main():
    p = argparse.ArgumentParser()
    p.add_argument("out", type=Path)
    p.add_argument("--round", type=int, default=1, help="New attempt index after a documented blocking tool repair")
    args = p.parse_args()
    out = args.out.resolve()
    round_ = args.round
    assert round_ >= 1
    sequences = list(json.loads((out / "freeze.json").read_text())["inputs"])
    write(out / "budget_before.json", budget.usage())
    statuses = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        for mode in ("PASSIVE", "OFF", "G", "V", "GV", "GV_REPEAT"):
            jobs = []
            for seq in sequences:
                if mode == "GV" and not all(statuses.get((seq, m), False) for m in ("G", "V")):
                    append(out, dict(event="blocked_GV", sequence=seq, reason="G_or_V_numerical_failure"))
                    continue
                if mode == "GV_REPEAT" and not statuses.get((seq, "GV"), False):
                    continue
                if mode not in ("PASSIVE", "OFF") and not statuses.get((seq, "OFF"), False):
                    continue
                jobs.append((seq, pool.submit(run, out, seq, "GV" if mode == "GV_REPEAT" else mode,
                                               2 if mode == "GV_REPEAT" else (1 if mode == "GV" else round_), round_)))
            for seq, future in jobs:
                statuses[(seq, mode)] = future.result()
            write(out / "matrix_status.json", {f"{seq}/{m}": v for (seq, m), v in statuses.items()})
    write(out / "budget_after.json", budget.usage())
    if not all(statuses.values()) or len(statuses) != 12:
        raise SystemExit("Matrix contains failure or blocked cells; preserve attempts and investigate.")

if __name__ == "__main__":
    main()
