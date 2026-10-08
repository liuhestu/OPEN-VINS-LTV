"""Build logging-only recovery against the immutable selected runtime, via budget."""
import hashlib,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import budget
import freeze
HERE=Path(__file__).resolve().parent

def build():
    frozen,freeze_sha=freeze.check()
    runtime=Path(frozen['runtime']);lib=runtime/'lib'
    source=HERE/'recover.cpp'
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    binary=budget.OUT/'diagnostic_recovery'/('recover_'+digest)
    binary.parent.mkdir(parents=True,exist_ok=True)
    if binary.exists():raise RuntimeError('Refusing overwrite prior recovery binary')
    command=['g++','-std=c++14','-O2','-I/usr/include/eigen3','-I/usr/include/opencv4',
             '-Iov_msckf/src','-Iov_core/src','-Iov_init/src',str(source),
             str(lib/'libov_msckf_lib.so'),str(lib/'libov_core_lib.so'),
             str(lib/'libopencv_core.so.4.5d'),str(lib/'libboost_filesystem.so.1.74.0'),
             '-Wl,--disable-new-dtags','-Wl,-rpath,'+str(lib),'-Wl,-rpath-link,'+str(lib),'-o',str(binary)]
    result=budget.run('build',command,'postfreeze_diagnostic_recovery_'+digest[:16])
    proof={'freeze_sha256':freeze_sha,'source_sha256':digest,'runtime':str(runtime),'command':command,'build':result}
    if result['exit_code']==0:proof['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
    (binary.parent/('build_'+digest+'.json')).write_text(json.dumps(proof,indent=2)+'\n')
    print(json.dumps(proof));return result['exit_code']
if __name__=='__main__':raise SystemExit(build())
