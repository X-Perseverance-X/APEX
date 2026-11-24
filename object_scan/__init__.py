"""Hand-held camera + planar LiDAR object reconstruction."""

from .core import (
    ColoredVoxelCloud,
    ICPResult,
    ObjectScanSession,
    ScanUpdate,
    estimate_rigid_transform,
    icp_register,
    lidar_polar_to_camera,
    transform_points,
)
from .config import (
    DEFAULT_SENSOR_RIG,
    camera_distortion,
    camera_from_lidar,
    camera_matrix,
    load_sensor_rig,
    with_translation_overrides,
)

__all__ = [
    "ColoredVoxelCloud",
    "ICPResult",
    "ObjectScanSession",
    "ScanUpdate",
    "estimate_rigid_transform",
    "icp_register",
    "lidar_polar_to_camera",
    "transform_points",
    "DEFAULT_SENSOR_RIG",
    "camera_from_lidar",
    "camera_matrix",
    "camera_distortion",
    "load_sensor_rig",
    "with_translation_overrides",
]
