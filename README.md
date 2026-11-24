# YOLO parking slot handoff

This folder contains the Town05 camera monitor, the 60-bay layout, and the YOLO11s weights. The handoff selects the highest-priority confirmed-free bay and computes the staging pose for A*.

## Run the camera

Start CARLA 0.9.16 on Town05. If CARLA is running in synchronous mode, keep the client that ticks the world running. Open PowerShell in this folder and use the existing `carla-sim312` environment if available. To set up a new Windows x64 Python 3.12 environment, install the bundled CARLA client wheel and the vision dependencies.

```powershell
python -m pip install .\wheels\carla-0.9.16-cp312-cp312-win_amd64.whl -r requirements.txt
```

Start the monitor with the bundled model, layout, and SAHI inference.

```powershell
python -m parking_slot_handoff.parking_monitor
```

The monitor opens three windows for **A/B**, **C/D**, and **E/F**. Each camera looks straight down at −90° pitch and is centered on its own bays. With the default image size and field of view, it sits 35 m above the lot. The three previews preserve the aspect ratio of each cropped camera image. Use `--no-display` to run the same three cameras without preview windows.

The default connection is `localhost:2000`. Supply `--host` and `--port` if your simulator uses a different address. Each camera needs three consistent observations to confirm occupancy. All three feed one report, with the image and polygon coordinates associated with their camera. Press Q or Esc in the preview, or Ctrl+C in the terminal, to stop.

### Full-lot view and camera angle

Use `--single-camera` for the original 55 m view. Passing an explicit position or angle also selects the single camera. Use `--pitch` to tilt it, `--yaw` to turn it, and `--fov` to change the horizontal field of view.

```powershell
python -m parking_slot_handoff.parking_monitor --single-camera --pitch -75 --yaw 0
```

## Read the parking handoff

Open a second terminal in this folder.

```powershell
python -m parking_slot_handoff
```

The command returns one JSON value with the fixed start, the selected bay, and the staging pose. The priority order is F1–F10, E1–E10, D1–D10, C1–C10, B1–B10, then A1–A10. Only confirmed-free bays are eligible. Coordinates are in CARLA world meters and headings are in degrees.

| Rows | Staging X | Staging Y | Staging yaw |
| --- | --- | --- | --- |
| B, D, F | target X − 5 m | target Y − 5 m | 270° |
| A, C, E | target X + 5 m | target Y − 5 m | 270° |

This is the result when F1 is free.

```json
{
  "start": {"x": 15.8, "y": -22.1, "yaw_deg": 90.0},
  "target": {"slot_id": "row_F-1", "x": -29.0435, "y": -18.7866, "yaw_deg": 180},
  "staging": {"x": -34.0435, "y": -23.7866, "z": 1.0, "yaw_deg": 270.0}
}
```

A fresh report with zero confirmed-free bays yields `null`. A missing, stale, or mismatched report yields exit code `2` and a diagnostic on stderr. The default maximum report age is three seconds. Use `--max-age` to change it. To reuse an existing SAHI camera process, pass its report with `--status` and its layout with `--slots`.

### Read the staging transform

With the monitor running, use this command in the second terminal to print Python code for the selected bay's staging transform.

```powershell
python staging_transform.py
```

The terminal shows the selected bay as a label above the code. For F1, it reads “The next available spot is F1”. The location uses the staging X and Y, and the rotation uses yaw 270°.

```python
location = carla.Location(x=-34.0435, y=-23.7866)
rotation = carla.Rotation(yaw=270.0)
transform = carla.Transform(location, rotation)
```

From Python, import `get_staging_transform` from `staging_transform` and call it to get the CARLA object. The function returns `None` if a fresh report has no confirmed-free bay.

## Call from A*

Add this folder to the planner's `PYTHONPATH`. With your teammate's initialized `AStarPlanner` instance, plan from the fixed start to staging.

```python
from parking_slot_handoff import get_parking_handoff

handoff = get_parking_handoff(map_name=world.get_map().name)
if handoff is not None:
    start = handoff['start']
    staging = handoff['staging']
    path_x, path_y = planner.planning(
        start['x'], start['y'], staging['x'], staging['y']
    )
```

The grid planner consumes the world `x` and `y` values. Use the start pose to set the vehicle's CARLA transform through your simulator setup. The vehicle controller can use `staging['yaw_deg']` for alignment and `handoff['target']` for the final parking maneuver.
