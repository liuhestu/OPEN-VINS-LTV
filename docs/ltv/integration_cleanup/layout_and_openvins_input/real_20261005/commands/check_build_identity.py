import hashlib,json,re,shlex,subprocess
from pathlib import Path
OUT=Path(__file__).resolve().parent;ID=json.loads((OUT/'initial_identity.json').read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
results={};flags={}
for label in ('baseline','current'):
 ws=OUT/label;root=Path(ID[label]);records=json.loads((ws/'build/ov_msckf/compile_commands.json').read_text());library=[r for r in records if 'CMakeFiles/ov_msckf_lib.dir/' in r['command']]
 def normalize(command):
  parts=shlex.split(command);parts=[p.replace(str(root),'${SOURCE}').replace(str(ws),'${WORKSPACE}') for p in parts]
  stripped=[];i=0
  while i<len(parts):
   if parts[i] in ('-o','-c'):i+=2
   else:stripped.append(parts[i]);i+=1
  return stripped
 flagsets={json.dumps(normalize(r['command'])) for r in library};assert len(flagsets)==1,flagsets
 flags[label]=next(iter(flagsets));assert '-DROS_AVAILABLE=2' in flags[label]
 for r in records:
  if 'tests/ltv/CMakeFiles/test_' in r['command']:assert '-UNDEBUG' in r['command'],r
 caches={}
 for package in ('ov_core','ov_init','ov_msckf'):
  text=(ws/'build'/package/'CMakeCache.txt').read_text();cache={}
  for key in ('CMAKE_BUILD_TYPE','CMAKE_CXX_COMPILER','ENABLE_ROS','BUILD_LTV_PHASE0_TESTS','Eigen3_DIR','OpenCV_DIR','Boost_DIR','Ceres_DIR'):
   match=re.search('^'+key+r':[^=]+=(.*)$',text,re.M)
   if match:cache[key]=match[1]
  caches[package]=cache
 build=json.loads((ws/'build_attempt.json').read_text());assert build['exit_code']==0
 stderr=(ws/'build.stderr').read_text().replace(str(root),'${SOURCE}').replace(str(ws),'${WORKSPACE}')
 warnings=[l for l in stderr.splitlines() if 'warning:' in l];assert 'error:' not in stderr
 results[label]=dict(cache=caches,library_sources=[str(Path(r['file']).relative_to(root)) for r in library],all_test_targets_assertions_enabled=True,warnings=warnings,compiler_flags=json.loads(flags[label]),build=build,files={n:sha(ws/n) for n in ('build.stdout','build.stderr','build/ov_msckf/compile_commands.json')})
assert flags['baseline']==flags['current']
assert results['baseline']['cache']==results['current']['cache']
assert results['baseline']['warnings']==results['current']['warnings']
source=(Path(ID['current'])/'ov_msckf/src/ltv/sources.cmake').read_text();expected=re.findall(r'\s+(src/ltv/[^\s)]+\.cpp)',source)
actual=[p.removeprefix('ov_msckf/') for p in results['current']['library_sources'] if p.startswith('ov_msckf/src/ltv/')]
assert sorted(expected)==sorted(actual) and len(actual)==len(set(actual))
report=dict(status='PASS_SAME_ROS_TOOLCHAIN_FLAGS_AND_SOURCE_LIST',results=results,source_list_entries=len(expected),source_list_each_compiled_once=True)
(OUT/'build_identity.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'status':report['status'],'source_list_entries':len(expected),'warning_count_each':len(results['current']['warnings'])}))
