"""
Minimal fisheye to equirectangular converter for Hailo detection
"""
import cv2
import numpy as np
import yaml


class DoubleSphereModel:
    """Double sphere fisheye camera model"""
    def __init__(self, cx, cy, fx, fy, xi, alpha, width=None, height=None):
        self.cx = cx
        self.cy = cy
        self.fx = fx
        self.fy = fy
        self.xi = xi
        self.alpha = alpha
        self.width = width
        self.height = height

    def project(self, x, y, z, eps=1e-9):
        r2 = x * x + y * y
        d1 = np.sqrt(r2 + z * z)
        k2 = self.xi * d1 + z
        d2 = np.sqrt(r2 + k2 * k2)
        denom_raw = self.alpha * d2 + (1.0 - self.alpha) * k2

        valid = denom_raw > 0
        denom = np.maximum(denom_raw, eps)

        mx = x / denom
        my = y / denom

        u = self.fx * mx + self.cx
        v = self.fy * my + self.cy

        return u, v, valid


def create_equirectangular_rays(width, height, h_fov_deg=220.0):
    """Generate 3D rays for equirectangular projection"""
    h_fov_rad = np.deg2rad(h_fov_deg)

    lon = (np.linspace(0, width - 1, width) / (width - 1)) * h_fov_rad - (h_fov_rad / 2)
    lat = (np.linspace(0, height - 1, height) / (height - 1)) * np.pi - (np.pi / 2)
    lon_grid, lat_grid = np.meshgrid(lon, lat)

    x = np.cos(lat_grid) * np.sin(lon_grid)
    y = np.sin(lat_grid)
    z = np.cos(lat_grid) * np.cos(lon_grid)

    return x, y, z


# The remap tables only depend on the lens model, the output size and the FOV,
# none of which change from frame to frame. Building them costs a meshgrid plus
# a handful of trig passes over every output pixel, so keep the last few around
# instead of paying for that on every frame.
_MAP_CACHE = {}
_MAP_CACHE_LIMIT = 8


def equirectangular_maps(model, out_width, out_height, h_fov_deg=220.0):
    """Cached (map_x, map_y, invalid) remap tables for one projection setting."""
    key = (
        model.cx, model.cy, model.fx, model.fy, model.xi, model.alpha,
        out_width, out_height, round(float(h_fov_deg), 4),
    )
    maps = _MAP_CACHE.get(key)
    if maps is None:
        x, y, z = create_equirectangular_rays(out_width, out_height, h_fov_deg)
        u, v, valid = model.project(x, y, z)
        maps = (u.astype(np.float32), v.astype(np.float32), ~valid)
        if len(_MAP_CACHE) >= _MAP_CACHE_LIMIT:
            _MAP_CACHE.clear()
        _MAP_CACHE[key] = maps
    return maps


def equirect_height(out_width, h_fov_deg=220.0):
    """Output height that keeps degrees-per-pixel equal in x and y.

    The projection always spans 180 deg vertically but only h_fov horizontally,
    so sizing the height off the source image's aspect squashes everything
    vertically -- people come out short and wide, which is exactly the shape a
    detector was *not* trained on. Both detection nodes size their output with
    this, which also means they publish boxes in the same reference frame.
    """
    return max(2, int(round(out_width * 180.0 / max(h_fov_deg, 1e-6))) // 2 * 2)


def fisheye_to_equirectangular(img, model, out_width, out_height=None, h_fov_deg=220.0):
    """Convert fisheye image to equirectangular projection"""
    if out_height is None:
        out_height = out_width // 2

    map_x, map_y, invalid = equirectangular_maps(model, out_width, out_height, h_fov_deg)

    equirect = cv2.remap(
        img, map_x, map_y,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0)
    )

    if equirect.ndim == 3:
        equirect[invalid] = (0, 0, 0)
    else:
        equirect[invalid] = 0

    return equirect


def load_camera_model(config_path, img_width, img_height):
    """Load camera parameters and create model"""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    params = config.get("camera_params", {})
    return DoubleSphereModel(
        cx=params["cx"],
        cy=params["cy"],
        fx=params["fx"],
        fy=params["fy"],
        xi=params["xi"],
        alpha=params["alpha"],
        width=img_width,
        height=img_height,
    )



