"""Print or return the latest staging transform from the parking monitor."""

from pathlib import Path
import sys
from typing import TYPE_CHECKING
import carla

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from parking.target import get_parking_handoff, make_staging_transform


def get_staging_transform() -> 'carla.Transform | None':
    """Return the current staging transform, or None if no bay is free."""
    handoff = get_parking_handoff()
    if handoff is None:
        return None
    return make_staging_transform(handoff['staging'])


def return_staging_transform() -> 'carla.Transform | None':
    try:
        handoff = get_parking_handoff()
    except (OSError, ValueError) as error:
        print(f'Parking target unavailable: {error}', file=sys.stderr)
        raise SystemExit(2) from error

    if handoff is None:
        print('null')
        return None

    slot_label = handoff['target']['slot_id'].removeprefix('row_').replace('-', '', 1)
    staging = handoff['staging']
    print(f'The next available spot is {slot_label}\n', file=sys.stderr)
    location = carla.Location(x=staging['x'], y=staging['y'], z=0.5)
    rotation = carla.Rotation(yaw=staging['yaw_deg'])
    transform = carla.Transform(location, rotation)
    return transform

def get_target_transform() -> 'carla.Transform | None':
    try:
        handoff = get_parking_handoff()
    except (OSError, ValueError) as error:
        print(f'Parking target unavailable: {error}', file=sys.stderr)
        raise SystemExit(2) from error

    if handoff is None:
        print('null')
        return None
    
    """Return the CARLA transform for the final parking target."""
    target = handoff['target']
    location = carla.Location(x=target['x'], y=target['y'], z=0.5)
    rotation = carla.Rotation(yaw=target['yaw_deg'])
    transform = carla.Transform(location, rotation)
    return transform
