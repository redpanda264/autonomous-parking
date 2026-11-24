"""Read the Town05 parking handoff from the camera's latest report."""

import carla
import math
from pathlib import Path
from typing import TYPE_CHECKING, TypedDict

from .target_selection import available_slots



PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_STATUS_PATH = PACKAGE_DIR / 'parking_status.json'
DEFAULT_LAYOUT_PATH = PACKAGE_DIR / 'parking_slots.json'
DEFAULT_MAP_NAME = 'Carla/Maps/Town05'
DEFAULT_MAX_AGE_SECONDS = 3.0

_SLOT_PRIORITY = {
    f'row_{row}-{number}': row_rank * 10 + number
    for row_rank, row in enumerate('FEDCBA')
    for number in range(1, 11)
}


class ParkingPose(TypedDict):
    x: float
    y: float
    yaw_deg: float


class ParkingTarget(ParkingPose):
    slot_id: str


class StagingPose(ParkingPose):
    z: float


class ParkingHandoff(TypedDict):
    start: ParkingPose
    target: ParkingTarget
    staging: StagingPose


_START_POSE: ParkingPose = {'x': 15.8, 'y': -22.10, 'yaw_deg': 90.0}


def get_parking_target(
    ego_x: float | None = None,
    ego_y: float | None = None,
    *,
    map_name: str = DEFAULT_MAP_NAME,
    status_path: str | Path = DEFAULT_STATUS_PATH,
    layout_path: str | Path = DEFAULT_LAYOUT_PATH,
    max_age_seconds: float = DEFAULT_MAX_AGE_SECONDS,
) -> ParkingTarget | None:
    """Return the highest-priority confirmed-free bay in CARLA world meters.

    The optional ego coordinates are accepted for existing callers. Bay priority
    determines the selection. Return None if a fresh report has no free bay.
    """
    if any(value is not None and not math.isfinite(value) for value in (ego_x, ego_y)):
        raise ValueError('Ego coordinates must be finite')
    if not math.isfinite(max_age_seconds) or max_age_seconds <= 0:
        raise ValueError('Maximum report age must be finite and positive')

    try:
        slots = available_slots(
            Path(status_path), Path(layout_path), map_name, max_age_seconds
        )
        slot = min(slots, key=lambda item: _SLOT_PRIORITY[item['id']], default=None)
        if slot is None:
            return None
        return {
            'slot_id': slot['id'],
            'x': slot['x'],
            'y': slot['y'],
            'yaw_deg': slot['yaw_deg'],
        }
    except (AttributeError, KeyError, TypeError) as error:
        raise ValueError('Invalid parking status or Town05 layout format') from error


def get_parking_handoff(
    *,
    map_name: str = DEFAULT_MAP_NAME,
    status_path: str | Path = DEFAULT_STATUS_PATH,
    layout_path: str | Path = DEFAULT_LAYOUT_PATH,
    max_age_seconds: float = DEFAULT_MAX_AGE_SECONDS,
) -> ParkingHandoff | None:
    """Return the fixed start, final bay, and intermediate A* staging pose."""
    target = get_parking_target(
        map_name=map_name, status_path=status_path, layout_path=layout_path,
        max_age_seconds=max_age_seconds,
    )
    if target is None:
        return None

    row = target['slot_id'].removeprefix('row_').split('-', 1)[0]
    x_offset = -5.0 if row in ('B', 'D', 'F') else 5.0
    return {
        'start': _START_POSE.copy(),
        'target': target,
        'staging': {
            'x': target['x'] + x_offset,
            'y': target['y'] - 5.0,
            'z': 1.0,
            'yaw_deg': 270.0,
        },
    }



def make_staging_transform(staging: StagingPose) -> 'carla.Transform':
    """Build a CARLA transform from the staging pose in a parking handoff."""

    location = carla.Location(x=staging['x'], y=staging['y'], z=staging['z'])
    rotation = carla.Rotation(yaw=staging['yaw_deg'])
    return carla.Transform(location, rotation)
