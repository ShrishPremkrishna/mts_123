"""ctypes wrapper that runs the driver's own rt/fc_imu_filter.hpp (taken from
fc/robot/imu/stock-imu-filter.patch, the single source) through fcimu_shim.cpp."""
import ctypes, os, subprocess, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
PATCH = os.path.join(HERE, '..', '..', 'imu', 'stock-imu-filter.patch')
CACHE = os.environ.get('FCIMU_CACHE', os.path.join(os.path.expanduser('~'), '.cache', 'fcimu_build'))
os.makedirs(CACHE, mode=0o700, exist_ok=True)
_st = os.stat(CACHE)
if _st.st_uid != os.getuid() or (_st.st_mode & 0o022):
    raise RuntimeError(f'refusing to build/load from {CACHE}: not owned by you or writable by others')
INC = os.path.join(CACHE, 'include'); HDR = os.path.join(INC, 'rt', 'fc_imu_filter.hpp'); LIB = os.path.join(CACHE, 'libfcimu.so')


def _extract_header():
    out, on = [], False
    for line in open(PATCH):
        if line.startswith('+++ '):
            on = line.split()[1].endswith('include/rt/fc_imu_filter.hpp'); continue
        if on and line.startswith('diff '): break
        if on and line.startswith('+'): out.append(line[1:])
    text = ''.join(out)
    os.makedirs(os.path.dirname(HDR), exist_ok=True)
    if not os.path.exists(HDR) or open(HDR).read() != text:
        open(HDR, 'w').write(text)


def _build():
    _extract_header()
    src = os.path.join(HERE, 'fcimu_shim.cpp')
    if not os.path.exists(LIB) or os.path.getmtime(LIB) < max(os.path.getmtime(src), os.path.getmtime(HDR)):
        subprocess.check_call(['c++', '-O2', '-std=c++17', '-shared', '-fPIC', '-I', INC, src, '-o', LIB])


_build()
_L = ctypes.CDLL(LIB)
FIELDS = ('kp', 'ki', 'quiet_rate', 'spike_thresh', 'freeze_s', 'kp_frozen')


class _P(ctypes.Structure):
    _fields_ = [(k, ctypes.c_double) for k in FIELDS]


_L.fcimu_new.restype = ctypes.c_void_p; _L.fcimu_new.argtypes = [ctypes.POINTER(_P)]
_L.fcimu_free.argtypes = [ctypes.c_void_p]
_dp = np.ctypeslib.ndpointer(dtype=np.float64, flags='C'); _ip = np.ctypeslib.ndpointer(dtype=np.int32, flags='C')
_L.fcimu_batch.argtypes = [ctypes.c_void_p, ctypes.c_int, _dp, _dp, _dp, _ip, _dp, _dp, _ip]
_L.fcimu_stats.argtypes = [ctypes.c_void_p, _dp]
DEFAULTS = dict(kp=2.0, ki=0.3, quiet_rate=0.03, spike_thresh=0.015, freeze_s=0.04, kp_frozen=2.0)  # = header defaults


class Filter:
    def __init__(self, **kw):
        p = dict(DEFAULTS, **kw); self.h = _L.fcimu_new(ctypes.byref(_P(**p)))

    def __del__(self):
        if getattr(self, 'h', None): _L.fcimu_free(self.h); self.h = None

    def run(self, dt, gyro, acc, acc_new):
        """Arrays of n steps: dt, gyro (n,3) sensor rad/s, acc (n,3) m/s^2, acc_new (n,). Returns
        published quaternion xyzw (n,4), published rate (n,3), flags (1 frozen, 2 despiked, 4 quiet, 8 init)."""
        n = len(dt); q = np.zeros((n, 4)); r = np.zeros((n, 3)); fl = np.zeros(n, np.int32)
        _L.fcimu_batch(self.h, n, np.ascontiguousarray(dt, float), np.ascontiguousarray(gyro, float),
                       np.ascontiguousarray(acc, float), np.ascontiguousarray(acc_new, np.int32), q, r, fl)
        return q, r, fl

    def stats(self):
        o = np.zeros(6); _L.fcimu_stats(self.h, o)
        return dict(zip(['samples', 'despiked', 'frozen_samples', 'accel_frozen_samples', 'accel_skipped', 'longest_freeze_s'], o.tolist()))
