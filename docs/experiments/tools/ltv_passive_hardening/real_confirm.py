"""Frozen all-EuRoC native confirmation entry; never loads GT or runs ATE.

Main must create the frozen manifest. This tool cannot freeze a candidate,
change selection, retry an existing output, or execute before that manifest.
"""
import argparse
import json
from pathlib import Path
import re
import shutil
import budget
from legacy import artifacts, passive_outputs
from check_hardened_log import check as check_hardened

HARDENING_FLAGS = ('ltv_passive_hardening_enabled', 'ltv_hardening_initial_warmup',
                   'ltv_hardening_health_readiness', 'ltv_hardening_preserve_constrained_state',
                   'ltv_hardening_ready_soft_grace')


def load(path):
    return json.loads(Path(path).read_text())


def preflight(name, sequence, mode, doc=None, output=None):
    doc = Path(doc) if doc is not None else budget.DOC
    output = Path(output) if output is not None else budget.OUT
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,100}', name):
        raise ValueError('Safe unique run name required')
    protocol = load(doc / 'protocol.json')
    if sequence not in protocol['real_final'] or mode not in ('B', 'P_PREV', 'P_NEW'):
        raise ValueError('Sequence or mode outside frozen all11 contract')
    freeze_path = doc / 'frozen_config.json'
    if not freeze_path.is_file():
        raise RuntimeError('Main-authored frozen_config.json required before confirmation')
    frozen = load(freeze_path)
    # Scientific contract and executable scripts are part of the frozen identity.
    # Reporting prose may evolve, but estimator/evaluator code cannot drift.
    for name in ('protocol', 'acceptance'):
        if artifacts.sha(doc / (name + '.json')) != frozen[name + '_sha']:
            raise ValueError('Frozen scientific contract changed: ' + name)
    source_manifest = Path(frozen['source_manifest'])
    if artifacts.sha(source_manifest) != frozen['source_manifest_sha']:
        raise ValueError('Frozen source manifest changed')
    for relative, digest in load(source_manifest).items():
        if relative.startswith(('ov_core/', 'ov_init/', 'ov_msckf/', 'docs/experiments/tools/ltv_passive_hardening/', 'docs/experiments/tools/ltv_feature_passive/', 'docs/experiments/tools/ltv_standalone_study/')):
            if artifacts.sha(budget.ROOT / relative) != digest:
                raise ValueError('Frozen execution source changed: ' + relative)
    runtime = Path(frozen['runtime'])
    base = Path(frozen['base_config'])
    if not runtime.is_absolute() or not base.is_absolute() or not frozen.get('revision'):
        raise ValueError('Frozen runtime/base_config must be absolute and revision explicit')
    if artifacts.sha(runtime / 'manifest.json') != frozen['manifest_sha']:
        raise ValueError('Frozen runtime manifest SHA mismatch')
    artifacts.verify(runtime)
    if artifacts.tree(base) != frozen['base_config_tree_sha']:
        raise ValueError('Frozen base config tree SHA mismatch')
    flags = frozen['hardening_flags']
    if set(flags) != set(HARDENING_FLAGS) or any(type(value) is not bool for value in flags.values()) or not flags['ltv_passive_hardening_enabled']:
        raise ValueError('Explicit uniform frozen hardening boolean flags required')
    metadata_path = output / 'input_metadata.json'
    if artifacts.sha(metadata_path) != frozen['input_metadata_sha']:
        raise ValueError('Frozen input metadata SHA mismatch')
    inputs = load(metadata_path)['sequences'][sequence]
    # Sensor CSV only. Reference metadata is copied without opening GT.
    for sensor in ('cam0', 'cam1', 'imu0'):
        value = inputs['sensors'][sensor]
        if artifacts.sha(Path(value['path'])) != value['sha256']:
            raise ValueError('Frozen sensor CSV changed: ' + sensor)
    root = output / 'real_confirmation' / name
    if root.exists():
        raise FileExistsError('Existing run is preserved; no automatic restart')
    return dict(root=root, runtime=runtime, base=base, frozen=frozen, freeze_sha=artifacts.sha(freeze_path), inputs=inputs)


def configure(base, destination, mode, frozen_flags):
    shutil.copytree(base, destination)
    path = destination / 'estimator_config.yaml'
    updates = {key: (frozen_flags[key] if mode == 'P_NEW' else False) for key in HARDENING_FLAGS}
    updates.update(ltv_enabled=mode != 'B', ltv_feature_readiness_enabled=mode != 'B',
                   ltv_enable_gravity=False, ltv_enable_velocity=False, ltv_passive_audit_enabled=True,
                   ltv_value_diagnostics_enabled=False)
    lines = path.read_text().splitlines()
    # Replace existing instances, not append ambiguous duplicate YAML keys.
    for key, value in updates.items():
        pattern = re.compile(r'^\s*' + re.escape(key) + r'\s*:')
        locations = [i for i, line in enumerate(lines) if pattern.match(line)]
        if len(locations) > 1:
            raise ValueError('Ambiguous duplicate YAML option: ' + key)
        text = key + ': ' + str(value).lower()
        if locations:
            lines[locations[0]] = text
        else:
            lines.append(text)
    path.write_text('\n'.join(lines) + '\n')
    return path


def run(name, sequence, mode):
    context = preflight(name, sequence, mode)
    root = context['root']
    root.mkdir(parents=True, exist_ok=False)
    frozen = context['frozen']
    config = configure(context['base'], root / 'config', mode, frozen['hardening_flags'])
    output = root / mode
    output.mkdir()
    # P_PREV is the existing managed Passive algorithm: native P_NEW plus all
    # new hardening options disabled, never the old zero-seed native P_OLD mode.
    native_mode = 'B' if mode == 'B' else 'P_NEW'
    identity = dict(sequence=sequence, mode=mode, native_mode=native_mode, phase='confirmation',
                    revision=frozen['revision'], freeze_sha=context['freeze_sha'],
                    runtime=str(context['runtime']), runtime_manifest_sha=frozen['manifest_sha'],
                    config_path=str(config), config_sha=artifacts.tree(config.parent),
                    inputs=context['inputs'], input_metadata_sha=frozen['input_metadata_sha'],
                    source='STEREO_THEN_TEMPORAL', hardening_flags={key: frozen['hardening_flags'][key] if mode == 'P_NEW' else False for key in HARDENING_FLAGS})
    # Bind the selected source to the actual frozen YAML; do not invent a mode.
    source_lines = [line for line in config.read_text().splitlines() if re.match(r'^\s*ltv_feature_seed_source\s*:', line)]
    if len(source_lines) != 1:
        raise ValueError('Frozen feature source must be explicit and unique')
    identity['source'] = source_lines[0].split(':', 1)[1].split('#', 1)[0].strip().strip('"\'')
    if identity['source'] not in ('STEREO', 'TEMPORAL_POSE', 'STEREO_THEN_TEMPORAL'):
        raise ValueError('Unsupported frozen feature source')
    (output / 'identity.json').write_text(json.dumps(identity, indent=2) + '\n')
    (root / 'identity.json').write_text(json.dumps(identity, indent=2) + '\n')
    command = artifacts.command(context['runtime'], 'run_ltv_feature_passive') + [str(config), context['inputs']['root'], str(output), native_mode]
    try:
        attempt = budget.run('real', command, identity, phase='confirmation')
        (root / 'attempt.json').write_text(json.dumps(attempt, indent=2) + '\n')
        if attempt['exit_code']:
            raise RuntimeError('Native confirmation failed; attempt preserved')
        engineering = {'passive': passive_outputs.inspect(output), 'accuracy': 'NOT_EVALUATED_NO_GT_ACCESS'}
        if mode == 'P_NEW':
            engineering['live_log'] = check_hardened(output)
        if not engineering['passive']['complete']:
            raise RuntimeError('Native confirmation did not consume complete inputs')
        (root / 'engineering.json').write_text(json.dumps(engineering, indent=2) + '\n')
    except Exception as error:
        (root / 'failure.json').write_text(json.dumps({'status': 'FAILED_PRESERVED', 'error': repr(error), 'command': command}, indent=2) + '\n')
        raise
    return {'output': str(output), 'attempt': attempt, 'engineering': engineering}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name')
    parser.add_argument('sequence')
    parser.add_argument('mode', choices=['B', 'P_PREV', 'P_NEW'])
    print(json.dumps(run(**vars(parser.parse_args())), indent=2))
