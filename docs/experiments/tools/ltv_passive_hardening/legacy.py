"""Load read-only previous helpers while using this task's explicit budget root."""
import importlib.util
from pathlib import Path
import budget

def load(relative,name):
    path=budget.ROOT/'docs/experiments/tools/ltv_feature_passive'/relative
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module
artifacts=load('artifacts.py','_prior_artifact_helpers')
passive_outputs=load('audit/passive_outputs.py','_prior_passive_output_audit')
