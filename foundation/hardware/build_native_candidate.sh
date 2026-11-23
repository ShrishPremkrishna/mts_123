#!/bin/bash
# Isolated target build, never deployment. SOURCE is an already assembled package.
# Manifest JSON: {"package": NAME, "source_commit": ID, "files": {relative: sha256}}.
# Caller reviews the exact test sources before opting into --test-no-devices.
set -e -o pipefail
if [[ $# -lt 6 || $# -gt 7 ]]; then
  echo 'Usage: build_native_candidate.sh PACKAGE SOURCE MANIFEST SHA256 NEW_OUTPUT PARENT_SOURCE_ID [--test-no-devices]' >&2
  exit 2
fi
case "$1" in neural_controller|joy_utils|control_board_hardware_interface) ;; *) exit 2;; esac
if [[ $# == 7 && $7 != --test-no-devices ]]; then exit 2; fi
if [[ -z ${PUPPER_CLEAN_BUILD_SHELL:-} ]]; then
  exec env -i HOME="$HOME" USER="$USER" LANG=C.UTF-8 PATH=/usr/local/bin:/usr/bin:/bin \
    PYTHONDONTWRITEBYTECODE=1 PUPPER_CLEAN_BUILD_SHELL=1 \
    nice -n 19 ionice -c 3 /bin/bash --noprofile --norc "$0" "$@"
fi
PKG=$1; SOURCE=$2; MANIFEST=$3; DIGEST=$4; OUTPUT=$5; SOURCE_ID=$6
test "$(uname -m)" = aarch64
test -f /opt/ros/jazzy/setup.bash
python3 - "$PKG" "$SOURCE" "$MANIFEST" "$DIGEST" "$OUTPUT" "$SOURCE_ID" <<'VERIFY'
import hashlib,json,sys
from pathlib import Path
pkg,source,manifest,digest,output,source_id=sys.argv[1:]
source=Path(source); manifest=Path(manifest); output=Path(output)
assert source.is_absolute() and output.is_absolute(), 'Use absolute paths'
assert not output.exists(), 'Output must be new; preserve earlier evidence'
assert not output.resolve().is_relative_to(source.resolve()), 'Output cannot be within source'
assert hashlib.sha256(manifest.read_bytes()).hexdigest()==digest, 'Manifest digest mismatch'
meta=json.loads(manifest.read_text())
assert meta['package']==pkg and meta['source_commit']==source_id
paths=list(source.rglob('*'))
assert not any(p.is_symlink() for p in paths), 'Source closure must not contain symlinks'
files={p.relative_to(source).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()}
assert files==meta['files'], 'Source closure mismatch'
assert 'CMakeLists.txt' in files and 'package.xml' in files
print(f'Verified {len(files)} source files for {pkg} at {source_id}')
VERIFY
check_idle() {
python3 - <<'IDLE'
import os
from pathlib import Path
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit() or proc.name==str(os.getpid()): continue
 try:
  name=Path(os.fsdecode((proc/'cmdline').read_bytes().split(b'\0')[0])).name
  if name in {'ros2_control_node','estop_controller'}:
   raise SystemExit(f'Hardware process {name} pid={proc.name}; defer this build/test')
  if proc.stat().st_uid!=os.getuid(): continue
  env=(proc/'environ').read_bytes().split(b'\0')
  if any(f'ROS_DOMAIN_ID={d}'.encode() in env for d in (164,165,166,190)):
   raise SystemExit(f'Reserved test domain occupied by pid={proc.name}')
 except (FileNotFoundError,ProcessLookupError,PermissionError): continue
IDLE
}
check_idle
mkdir "$OUTPUT"
exec > >(tee "$OUTPUT/transcript.log") 2>&1
trap 'status=$?; printf "%s\n" "$status" > "$OUTPUT/exit-code"' EXIT
cp "$MANIFEST" "$OUTPUT/source-closure.json"
sha256sum "$0" > "$OUTPUT/recipe.sha256"
date -u; cat /proc/sys/kernel/random/boot_id
source /opt/ros/jazzy/setup.bash
test "$AMENT_PREFIX_PATH" = /opt/ros/jazzy
UNDERLAY_LIBRARY_PATH=${LD_LIBRARY_PATH:-}
BUILD="$OUTPUT/build"; PREFIX="$OUTPUT/prefix"
export ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PUPPER_ISOLATED_ROS_TEST=1
python3 - "$OUTPUT" <<'ENVIRONMENT'
import hashlib,json,shutil,subprocess,sys
from pathlib import Path
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
r={'tools':{},'packages':{}}
for name in ('cmake','g++','python3','strace'):
 p=shutil.which(name)
 if p:r['tools'][name]={'path':p,'sha256':sha(Path(p))}
for p in sorted(Path('/opt/ros/jazzy/share').glob('*/package.xml')):
 r['packages'][p.parent.name]={'path':str(p),'sha256':sha(p)}
Path(sys.argv[1],'environment.json').write_text(json.dumps(r,indent=2)+'\n')
ENVIRONMENT
cmake -S "$SOURCE" -B "$BUILD" -DBUILD_TESTING=ON \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo '-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-O2 -g -UNDEBUG' \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_INSTALL_PREFIX="$PREFIX" 2>&1 | tee "$OUTPUT/configure.log"
cmake --build "$BUILD" --parallel 1 2>&1 | tee "$OUTPUT/build.log"
cmake --install "$BUILD" 2>&1 | tee "$OUTPUT/install.log"
export LD_LIBRARY_PATH="$PREFIX/lib:$UNDERLAY_LIBRARY_PATH"
python3 - "$PREFIX" "$OUTPUT" <<'ELFS'
import hashlib,json,subprocess,sys
from pathlib import Path
prefix=Path(sys.argv[1]); out=Path(sys.argv[2]); files={}; elfs={}
for p in sorted(prefix.rglob('*')):
 if not p.is_file():continue
 key=p.relative_to(prefix).as_posix();data=p.read_bytes();files[key]=hashlib.sha256(data).hexdigest()
 if data[:4]!=b'\x7fELF':continue
 header=subprocess.check_output(['readelf','--file-header',str(p)],text=True)
 assert 'AArch64' in header
 links=subprocess.check_output(['ldd',str(p)],text=True)
 assert 'not found' not in links
 elfs[key]=links.splitlines()
assert elfs, 'No target ELF produced'
(out/'installed-files.json').write_text(json.dumps(files,indent=2)+'\n')
(out/'resolved-dependencies.json').write_text(json.dumps(elfs,indent=2)+'\n')
print(f'Checked {len(elfs)} AArch64 ELF dependencies, {len(files)} installed files')
ELFS
if [[ $PKG == joy_utils ]]; then
  BINDINGS=$(find "$PREFIX" -type d \( -name dist-packages -o -name site-packages \) -print -quit)
  test -n "$BINDINGS"
  PYTHONPATH="$BINDINGS:${PYTHONPATH:-}" /usr/bin/python3 - "$PREFIX" <<'BINDINGS'
from pathlib import Path
import sys,joy_utils.srv
from joy_utils.srv import StartMotion
from rclpy.type_support import check_for_type_support
assert Path(joy_utils.srv.__file__).is_relative_to(Path(sys.argv[1]))
check_for_type_support(StartMotion)
if (Path(sys.argv[1])/'share/joy_utils/srv/FcStartMotion.srv').is_file():
 from joy_utils.srv import FcStartMotion
 check_for_type_support(FcStartMotion)
print('Target service bindings resolved from candidate prefix')
BINDINGS
fi
if [[ ${7:-} == --test-no-devices ]]; then
  command -v strace >/dev/null
  check_idle
  export LD_LIBRARY_PATH="$BUILD:$UNDERLAY_LIBRARY_PATH"
  ctest --test-dir "$BUILD" --show-only=json-v1 -LE linter > "$OUTPUT/tests-selected.json"
  # User requested tasks finish without elapsed-time cancellation. Keep test
  # assertions/command-expiry semantics, remove only CTest process time budgets.
  python3 - "$OUTPUT" "$BUILD" <<'NO_TASK_TIMER'
import json,sys
from pathlib import Path
out,build=map(Path,sys.argv[1:])
names=[row['name'] for row in json.loads((out/'tests-selected.json').read_text())['tests']]
with (build/'CTestTestfile.cmake').open('a') as f:
 for name in names:
  assert all(c.isalnum() or c in '_-.' for c in name)
  f.write(f'\nset_tests_properties({name} PROPERTIES TIMEOUT 0)\n')
NO_TASK_TIMER
  ctest --test-dir "$BUILD" --show-only=json-v1 -LE linter > "$OUTPUT/tests-selected.json"
  # Test environment scoped to child: the idle/closure check must not see itself.
  set +e
  ROS_DOMAIN_ID=190 strace -ff -o "$OUTPUT/test-syscalls" -e trace=%file \
    ctest --test-dir "$BUILD" --no-tests=error --output-on-failure -LE linter \
      --output-junit "$OUTPUT/tests.xml" 2>&1 | tee "$OUTPUT/ctest.log"
  TEST_EXIT=${PIPESTATUS[0]}
  set -e
  check_idle
  python3 - "$OUTPUT" <<'TESTS'
import json,re,sys,xml.etree.ElementTree as ET
from pathlib import Path
out=Path(sys.argv[1]);selected=json.loads((out/'tests-selected.json').read_text())['tests']
assert selected, 'No selected tests'
tree=ET.parse(out/'tests.xml'); cases=tree.findall('.//testcase')
assert len(cases)==len(selected), 'Missing tests'
assert not tree.findall('.//skipped'), 'Skipped tests do not pass'
devices=set()
for p in out.glob('test-syscalls.*'):
 for name in re.findall(r'"(/dev/[^"\n]+)"',p.read_text()):
  if name not in {'/dev/null','/dev/random','/dev/urandom'} and not name.startswith('/dev/shm/'):
   devices.add(name)
assert not devices, f'Unexpected device paths: {sorted(devices)}'
print(f'Enumerated {len(cases)} tests; no unexpected /dev paths in traced file operations')
TESTS
  test "$TEST_EXIT" = 0
fi
echo "Isolated candidate complete: $OUTPUT (no physical qualification)"
