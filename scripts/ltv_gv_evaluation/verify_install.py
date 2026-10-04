"""Bind real ROS build, frozen toolchain flags, installed libraries and relevant tests."""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess
import time
from run_matrix import ROOT, ENV, command, exact, write

def normalize(command_line, source, ws):
    parts = shlex.split(command_line)
    parts = [p.replace(str(source), "SOURCE").replace(str(ws), "WORKSPACE") for p in parts]
    result, i = [], 0
    while i < len(parts):
        if parts[i] in ("-o", "-c"):
            i += 2
        else:
            result.append(parts[i]); i += 1
    return result

def main():
    p = argparse.ArgumentParser()
    p.add_argument("out", type=Path)
    out = p.parse_args().out.resolve()
    source = {}
    for folder in ("ov_core", "ov_init", "ov_msckf", "config", "scripts/ltv_gv_evaluation"):
        for path in sorted((ROOT / folder).rglob("*")):
            if path.is_file() and path.suffix in (".cpp", ".h", ".cmake", ".txt", ".yaml", ".py"):
                source[str(path.relative_to(ROOT))] = exact.sha(path)
    write(out / "compiled_source.json", source)
    baseline_ws = Path("/home/he/output/ltv_integration_cleanup_real_20261005/current")
    flagsets, caches = {}, {}
    for label, ws in (("frozen", baseline_ws), ("revised", out)):
        records = json.loads((ws / "build/ov_msckf/compile_commands.json").read_text())
        lib = [r for r in records if "CMakeFiles/ov_msckf_lib.dir/" in r["command"]]
        flagsets[label] = sorted({json.dumps(normalize(r["command"], ROOT, ws)) for r in lib})
        for r in records:
            if "tests/ltv/CMakeFiles/test_" in r["command"]:
                assert "-UNDEBUG" in r["command"], r
        cache = {}
        for package in ("ov_core", "ov_init", "ov_msckf"):
            text = (ws / "build" / package / "CMakeCache.txt").read_text()
            cache[package] = {}
            for key in ("CMAKE_BUILD_TYPE", "CMAKE_CXX_COMPILER", "ENABLE_ROS", "BUILD_LTV_PHASE0_TESTS", "Eigen3_DIR", "OpenCV_DIR", "Boost_DIR", "Ceres_DIR"):
                m = re.search("^" + key + r":[^=]+=(.*)$", text, re.M)
                if m: cache[package][key] = m[1]
        caches[label] = cache
    assert flagsets["frozen"] == flagsets["revised"], "changed production flags"
    assert caches["frozen"] == caches["revised"], "changed toolchain/dependency settings"
    installed = out / "install/ov_msckf/lib/ov_msckf"
    runtime = {}
    for name in ("run_ltv_feature_passive", "run_ltv_gv_evaluation"):
        cmd = command(out, Path("/usr/bin/ldd"), [installed / name])
        dep = subprocess.run(cmd, env=ENV, text=True, capture_output=True)
        assert dep.returncode == 0 and "not found" not in dep.stdout
        assert str(out / "install/ov_msckf/lib/libov_msckf_lib.so") in dep.stdout
        runtime[name] = dict(binary_sha=exact.sha(installed / name), command=cmd, ldd=dep.stdout)
    libs = {str(p): exact.sha(p) for pkg in ("ov_core","ov_init","ov_msckf") for p in (out / "install" / pkg / "lib").glob("*.so")}
    write(out / "build_identity.json", dict(status="PASS_SAME_ROS_FLAGS_TOOLCHAIN",flags=flagsets,caches=caches,runtime=runtime,libraries=libs,
        compiler=subprocess.check_output(["g++","--version"],text=True),
        cmake=subprocess.check_output(["cmake","--version"],text=True),
        colcon_build_command=(out / "build.sh").read_text()))
    prior = json.loads((ROOT / "docs/ltv/integration_cleanup/layout_and_openvins_input/real_20261005/current_tests.json").read_text())
    results = []
    (out / "tests").mkdir(exist_ok=True)
    for record in prior:
        name = record["name"]
        cmd = command(out, installed / name, [])
        start = time.time()
        with (out / "tests" / f"{name}.log").open("w") as log:
            ret = subprocess.run(cmd, cwd=out, env=ENV, stdout=log, stderr=subprocess.STDOUT)
        results.append(dict(name=name,command=cmd,exit_code=ret.returncode,seconds=time.time()-start))
        write(out / "tests.json", results)
        print(name, ret.returncode, flush=True)
        if ret.returncode: raise SystemExit("Installed test failed")
    for seq, inp in json.loads((out / "freeze.json").read_text())["inputs"].items():
        images = {}
        for cam in ("cam0","cam1"):
            for line in (Path(inp["root"]) / cam / "data.csv").read_text().splitlines():
                if line and not line.startswith("#"):
                    _, file = line.split(",", 1)
                    relative = str(Path(cam) / "data" / file.strip())
                    images[relative] = exact.sha(Path(inp["root"]) / relative)
        import hashlib
        aggregate = hashlib.sha256(json.dumps(images, sort_keys=True, separators=(",",":")).encode()).hexdigest()
        assert aggregate == inp["images_aggregate_sha"]
        write(out / f"images_{seq}.json", dict(files=images, aggregate_sha=aggregate))

if __name__ == "__main__":
    main()
