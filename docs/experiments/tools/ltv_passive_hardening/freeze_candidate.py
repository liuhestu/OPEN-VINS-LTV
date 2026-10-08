"""Freeze the reviewed candidate before opening confirmation seeds.

This command archives source/config and records immutable runtime identities;
it does not build, run a simulation, or change production YAML.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import budget
from legacy import artifacts
from real_confirm import HARDENING_FLAGS


def freeze(revision, runtime, wrapper, ready_soft_grace=False):
    if type(ready_soft_grace) is not bool:
        raise ValueError('ready_soft_grace must be an explicit boolean')
    destination = budget.DOC / 'frozen_config.json'
    if destination.exists():
        raise FileExistsError('Preserve the previous frozen revision; never overwrite it')
    if revision not in ('confirmation_1', 'confirmation_2'):
        raise ValueError('Only two predeclared confirmation revisions are permitted')
    runtime, wrapper = Path(runtime).resolve(), Path(wrapper).resolve()
    artifacts.verify(runtime)
    dependencies = artifacts.dependencies(wrapper)
    if artifacts.sha(dependencies['libov_msckf_lib.so']) != artifacts.sha(runtime / 'lib/libov_msckf_lib.so'):
        raise ValueError('Wrapper observer library does not match frozen native runtime')
    protocol = json.loads((budget.DOC / 'protocol.json').read_text())
    if hashlib.sha256((budget.DOC / 'goal.md').read_bytes()).hexdigest() != protocol['goal_sha256']:
        raise ValueError('Original goal changed')
    root = budget.OUT / 'confirmation_freeze' / revision
    root.mkdir(parents=True, exist_ok=False)
    base = root / 'base_config'
    shutil.copytree(budget.OUT / 'logger_short/config', base)
    wrapper_copy = root / wrapper.name
    shutil.copy2(wrapper, wrapper_copy)
    # Capture working-tree versions, including authorized new tools, never an
    # inferred git HEAD-only identity. Exclude generated arrays and output logs.
    source_roots = ['ov_core', 'ov_init', 'ov_msckf',
                    'docs/experiments/tools/ltv_passive_hardening', 'docs/experiments/tools/ltv_feature_passive', 'docs/experiments/tools/ltv_standalone_study',
                    'docs/ltv/passive_hardening_v2', 'config/ltv_euroc']
    paths = set()
    for prefix in source_roots:
        for path in (budget.ROOT / prefix).rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix not in ('.pyc', '.npz', '.png', '.svg'):
                paths.add(path)
    for pattern in ('CMakeLists.txt', 'cmake/*.cmake'):
        paths.update(p for p in budget.ROOT.glob(pattern) if p.is_file())
        paths.update(p for p in (budget.ROOT / 'ov_msckf').glob(pattern) if p.is_file())
    source = {str(p.relative_to(budget.ROOT)): artifacts.sha(p) for p in sorted(paths)}
    archive = root / 'source.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        for path in sorted(paths):
            tar.add(path, arcname=str(path.relative_to(budget.ROOT)), recursive=False)
    (root / 'source_manifest.json').write_text(json.dumps(source, indent=2) + '\n')
    (root / 'working_tree.patch').write_bytes(subprocess.check_output(['git', 'diff', 'HEAD', '--binary'], cwd=budget.ROOT))
    frozen = dict(revision=revision, runtime=str(runtime), manifest_sha=artifacts.sha(runtime / 'manifest.json'),
                  wrapper=str(wrapper_copy), wrapper_sha=artifacts.sha(wrapper_copy),
                  wrapper_dependencies={k: {'path':str(p), 'sha256':artifacts.sha(p)} for k,p in dependencies.items()},
                  base_config=str(base), base_config_tree_sha=artifacts.tree(base),
                  input_metadata_sha=artifacts.sha(budget.OUT / 'input_metadata.json'),
                  hardening_flags={key:(ready_soft_grace if key == 'ltv_hardening_ready_soft_grace' else True) for key in HARDENING_FLAGS},
                  git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=budget.ROOT,text=True).strip(),
                  source_manifest=str(root/'source_manifest.json'), source_manifest_sha=artifacts.sha(root/'source_manifest.json'),
                  source_archive=str(archive), source_archive_sha=artifacts.sha(archive),
                  protocol_sha=artifacts.sha(budget.DOC/'protocol.json'), acceptance_sha=artifacts.sha(budget.DOC/'acceptance.json'),
                  confirmation_seeds=protocol['confirmation_seed_groups'][int(revision[-1])-1],
                  notes='Same C0 model, zero G/V injection; source/config frozen before confirmation outcomes.')
    # Last operation: a failed archive cannot accidentally authorize confirmation.
    destination.write_text(json.dumps(frozen, indent=2) + '\n')
    return frozen


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ready-soft-grace',action='store_true');p.add_argument('revision');p.add_argument('--runtime',required=True);p.add_argument('--wrapper',required=True)
    print(json.dumps(freeze(**vars(p.parse_args())),indent=2))
