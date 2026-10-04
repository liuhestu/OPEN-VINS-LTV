"""Reuse the verified immutable snapshot implementation with the R4 output root."""
import importlib.util
from pathlib import Path
import budget
spec = importlib.util.spec_from_file_location('_r4_verified_artifacts', budget.ROOT / 'scripts/ltv_feature_passive/artifacts.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
base.budget = budget
sha = base.sha
tree = base.tree
dependencies = base.dependencies
snapshot = base.snapshot
verify = base.verify
command = base.command
if __name__ == '__main__':
    print(snapshot())
