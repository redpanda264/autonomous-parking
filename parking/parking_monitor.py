"""Monitor Town05 parking with three close overhead cameras.

Run from the handoff folder with python -m parking.parking_monitor.
Press Q or Escape in a preview to exit.
"""

from contextlib import ExitStack
from dataclasses import dataclass, field
from functools import partial
import json
import math
from pathlib import Path
import queue
import sys
import time

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from parking.target import DEFAULT_LAYOUT_PATH, DEFAULT_STATUS_PATH, PACKAGE_DIR


PREVIEW_WIDTH = 600
PREVIEW_HEIGHT = 300
GROUP_CAMERA_MIN_HEIGHT = 35


@dataclass
class MonitorSettings:
    weights: Path = PACKAGE_DIR / 'yolo11s.pt'
    compare_weights: Path | None = None
    comparison_dir: Path = Path('parking_comparison')
    inference: str = 'sahi'
    slice_size: int = 512
    slice_overlap: float = 0.25
    confidence: float = 0.15
    slots: Path = DEFAULT_LAYOUT_PATH
    ground_z: float = 0.0
    host: str | None = None
    port: int | None = None
    single_camera: bool = False
    x: float | None = None
    y: float | None = None
    z: float | None = None
    pitch: float | None = None
    yaw: float | None = None
    width: int = 1600
    height: int = 1200
    fov: float = 90.0
    interval: float = 0.5
    output: Path = DEFAULT_STATUS_PATH
    target_status: Path | None = None
    stop_file: Path | None = None
    no_display: bool = False


@dataclass
class CameraView:
    name: str
    slots: list[dict]
    x: float
    y: float
    z: float
    pitch: float = -90.0
    yaw: float = 0.0
    frames: queue.Queue = field(default_factory=lambda: queue.Queue(maxsize=1))

    @property
    def window(self):
        return f'Parking monitor - {self.name}'


def camera_views(settings, layout):
    from parking.mapped_vision import slot_corners

    pose = (settings.x, settings.y, settings.z, settings.pitch, settings.yaw)
    if settings.single_camera or any(value is not None for value in pose):
        defaults = (-9.6, -31.35, 55.0, -90.0, 0.0)
        return [CameraView('Lot', layout, *(default if value is None else value
                                          for value, default in zip(pose, defaults)))]

    pairs = {name: [] for name in ('A/B', 'C/D', 'E/F')}
    pair_for_row = {row: name for name in pairs for row in name.split('/')}
    for slot in layout:
        row = slot['id'].removeprefix('row_').partition('-')[0]
        if row not in pair_for_row:
            raise ValueError(f"Cannot group bay {slot['id']!r} by row A–F. Set single_camera=True for this layout.")
        pairs[pair_for_row[row]].append(slot)

    horizontal = math.tan(math.radians(settings.fov) / 2)
    vertical = horizontal * settings.height / settings.width
    views = []
    for name, slots in pairs.items():
        if not slots:
            continue
        xs, ys = zip(*(corner for slot in slots for corner in slot_corners(slot)))
        # At pitch -90 and yaw 0, world Y spans the image width.
        height = 1.4 * max((max(ys) - min(ys)) / (2 * horizontal),
                           (max(xs) - min(xs)) / (2 * vertical))
        altitude = settings.ground_z + max(GROUP_CAMERA_MIN_HEIGHT, math.ceil(height))
        views.append(CameraView(name, slots, (min(xs) + max(xs)) / 2,
                                (min(ys) + max(ys)) / 2, altitude))
    return views


def receive_latest(frames, frame):
    item = (frame, time.time())
    # Keep only the latest frame so inference cannot build a queue of old frames.
    try:
        frames.put_nowait(item)
    except queue.Full:
        try:
            frames.get_nowait()
        except queue.Empty:
            pass
        try:
            frames.put_nowait(item)
        except queue.Full:
            pass


def stop_camera(camera):
    try:
        camera.stop()
    finally:
        camera.destroy()


def write_report(path, report):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(report, indent=2, default=lambda value: value.item()),
                         encoding='utf-8')
    # A planner reading the file can briefly block the replacement on Windows.
    for attempt in range(20):
        try:
            temporary.replace(path)
            return
        except PermissionError:
            if attempt == 19:
                raise
            time.sleep(.05)


def combine_observations(observations, map_name, inference, target, single_camera):
    newest = max(observations.values(), key=lambda item: item['frame'])
    slots = [slot for item in observations.values() for slot in item['slots']]
    lists = {'available': [], 'occupied': [], 'unknown': []}
    report_keys = {'free': 'available', 'occupied': 'occupied', 'unknown': 'unknown'}
    for slot in slots:
        lists[report_keys[slot['status']]].append(slot['id'])
    report = dict(
        target=target, map=map_name, frame=newest['frame'],
        simulation_timestamp=newest['simulation_timestamp'],
        updated_at_unix=min(item['received_at'] for item in observations.values()),
        polygon_coordinates='full_camera_image', inference=inference, id_scope='parking_layout',
        **lists, slots=slots,
    )
    if single_camera:
        report.update(camera=newest['camera'], monitor_roi=newest['monitor_roi'],
                      comparison=newest['comparison'])
    else:
        report['cameras'] = {
            name: dict(item['camera'], frame=item['frame'], monitor_roi=item['monitor_roi'],
                       simulation_timestamp=item['simulation_timestamp'])
            for name, item in observations.items()
        }
        comparisons = {name: item['comparison'] for name, item in observations.items()
                       if item['comparison'] is not None}
        report['comparison'] = {'cameras': comparisons} if comparisons else None
    return report, lists


def main():
    settings = MonitorSettings()
    if settings.inference not in ('crop', 'sahi'):
        raise ValueError('Inference must be crop or sahi')
    if settings.pitch is not None and not -90.0 <= settings.pitch <= 90.0:
        raise ValueError('Camera pitch must be between -90 and 90 degrees')
    if not settings.weights.expanduser().is_file():
        raise ValueError('Checkpoint does not exist: %s' % settings.weights)
    if settings.compare_weights and not settings.compare_weights.expanduser().is_file():
        raise ValueError('Comparison checkpoint does not exist: %s' % settings.compare_weights)
    if settings.slice_size < 64 or not 0 <= settings.slice_overlap < 1 or not 0 < settings.confidence <= 1:
        raise ValueError('Use slice_size >=64, slice_overlap in [0,1), confidence in (0,1]')
    if settings.width < 64 or settings.height < 64 or not 1 < settings.fov < 179 or settings.interval <= 0:
        raise ValueError('Use image dimensions >=64, FOV between 1 and 179, interval >0')
    import carla
    import cv2
    import numpy as np
    from parking.mapped_vision import MappedParkingDetector
    from parking.target_display import draw_target, load_target
    from utils.connection_config import load_connection_config

    detector_options = dict(inference=settings.inference, slice_size=settings.slice_size,
                            slice_overlap=settings.slice_overlap, confidence=settings.confidence)
    detector = MappedParkingDetector(settings.weights.expanduser(), settings.slots, settings.ground_z,
                                     **detector_options)
    comparison = (MappedParkingDetector(settings.compare_weights.expanduser(), settings.slots, settings.ground_z,
                                         **detector_options) if settings.compare_weights else None)
    views = camera_views(settings, detector.layout)
    if not views:
        raise ValueError('The parking layout has no bays to monitor')

    single_camera = views[0].name == 'Lot'
    host, port = load_connection_config()
    client = carla.Client(settings.host or host, settings.port or port)
    client.set_timeout(10.0)
    settings.output.parent.mkdir(parents=True, exist_ok=True)

    def camera_transform(view):
        return carla.Transform(carla.Location(x=view.x, y=view.y, z=view.z),
                               carla.Rotation(pitch=view.pitch, yaw=view.yaw))

    def process(view, frame, received_at, target):
        rgb = np.frombuffer(frame.raw_data, np.uint8).reshape(frame.height, frame.width, 4)[:, :, :3][:, :, ::-1]
        started = time.perf_counter()
        annotated, slots = detector.process_frame(rgb, frame, settings.fov, slots=view.slots)
        primary_seconds = time.perf_counter() - started
        if settings.target_status:
            annotated = draw_target(annotated, slots, detector.monitor_roi, target)
        camera_metadata = dict(x=view.x, y=view.y, z=view.z, pitch=view.pitch, yaw=view.yaw,
                               width=frame.width, height=frame.height, fov=settings.fov)
        if not single_camera:
            slots = [dict(slot, camera_id=view.name) for slot in slots]

        comparison_report = None
        if comparison is not None:
            started = time.perf_counter()
            alternative, alternative_slots = comparison.process_frame(rgb, frame, settings.fov, slots=view.slots)
            alternative_seconds = time.perf_counter() - started
            if settings.target_status:
                alternative = draw_target(alternative, alternative_slots, comparison.monitor_roi, target)
            if not single_camera:
                alternative_slots = [dict(slot, camera_id=view.name) for slot in alternative_slots]
            first = {slot['id']: slot['status'] for slot in slots}
            differences = [dict(id=slot['id'], primary=first[slot['id']], comparison=slot['status'])
                           for slot in alternative_slots if first[slot['id']] != slot['status']]
            comparison_report = dict(
                primary_model=str(settings.weights), comparison_model=str(settings.compare_weights),
                primary_seconds=primary_seconds, comparison_seconds=alternative_seconds,
                summary=comparison.summary, differences=differences, slots=alternative_slots,
            )
            directory = settings.comparison_dir if single_camera else settings.comparison_dir / view.name.replace('/', '')
            directory.mkdir(parents=True, exist_ok=True)
            for name, image in [('primary', annotated), ('comparison', alternative), ('raw', rgb)]:
                cv2.imwrite(str(directory / (name + '.png')), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            cv2.putText(annotated, settings.weights.name, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2)
            cv2.putText(alternative, settings.compare_weights.name, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2)
            annotated = np.concatenate((annotated, alternative), axis=1)

        if not settings.no_display:
            preview = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)
            if not single_camera:
                height, width = preview.shape[:2]
                scale = min(PREVIEW_WIDTH / width, PREVIEW_HEIGHT / height, 1.0)
                if scale < 1.0:
                    size = (max(1, round(width * scale)), max(1, round(height * scale)))
                    preview = cv2.resize(preview, size, interpolation=cv2.INTER_AREA)
            cv2.imshow(view.window, preview)
        return dict(frame=frame.frame, simulation_timestamp=frame.timestamp, received_at=received_at,
                    camera=camera_metadata, monitor_roi=detector.monitor_roi,
                    slots=slots, comparison=comparison_report)

    try:
        with ExitStack() as cleanup:
            world = client.get_world()
            map_name = world.get_map().name
            blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
            for name, value in [('image_size_x', settings.width), ('image_size_y', settings.height),
                                ('fov', settings.fov), ('sensor_tick', settings.interval)]:
                if blueprint.has_attribute(name):
                    blueprint.set_attribute(name, str(value))

            if not settings.no_display:
                cleanup.callback(cv2.destroyAllWindows)
            cameras = {}
            for index, view in enumerate(views):
                camera = world.spawn_actor(blueprint, camera_transform(view))
                cleanup.callback(stop_camera, camera)
                cameras[view.name] = camera
                camera.listen(partial(receive_latest, view.frames))
                print(f'{view.name}: camera {camera.id}, x={view.x:.2f}, y={view.y:.2f}, z={view.z:g}, pitch={view.pitch:g}')
                if not settings.no_display:
                    if single_camera:
                        cv2.namedWindow(view.window, cv2.WINDOW_NORMAL)
                        cv2.resizeWindow(view.window, 430, 320)
                    else:
                        cv2.namedWindow(view.window, cv2.WINDOW_AUTOSIZE)
                    cv2.moveWindow(view.window, 15 + index % 2 * (PREVIEW_WIDTH + 15),
                                   40 + index // 2 * (PREVIEW_HEIGHT + 55))

            print(f'Output: {settings.output.resolve()}. Press Ctrl+C to stop.')
            if not settings.no_display and single_camera:
                print('W/S and A/D move the camera, R/F change height, Z/X turn it, Q or Escape exits.')

            changes = {ord('w'): ('x', 2), ord('s'): ('x', -2), ord('a'): ('y', -2), ord('d'): ('y', 2),
                       ord('r'): ('z', 5), ord('f'): ('z', -5), ord('z'): ('yaw', -5), ord('x'): ('yaw', 5)}
            pending = {}
            last_summary = None
            next_wait_notice = time.monotonic() + 5
            while not (settings.stop_file and settings.stop_file.exists()):
                target = load_target(settings.target_status)
                for view in views:
                    try:
                        frame, received_at = view.frames.get(timeout=.05)
                    except queue.Empty:
                        continue
                    pending[view.name] = process(view, frame, received_at, target)

                # A partial sweep must not make an older camera reading look fresh.
                if len(pending) == len(views):
                    report, lists = combine_observations(pending, map_name, settings.inference, target, single_camera)
                    write_report(settings.output, report)
                    pending.clear()
                    next_wait_notice = time.monotonic() + 5
                    if lists != last_summary:
                        print(f"Frame {report['frame']}: {json.dumps(lists)}", flush=True)
                        last_summary = lists
                elif time.monotonic() >= next_wait_notice:
                    missing = ', '.join(view.name for view in views if view.name not in pending)
                    print(f'Waiting for camera frames from {missing}. Check rendering and simulation ticks.', flush=True)
                    next_wait_notice = time.monotonic() + 5

                if not settings.no_display:
                    key = cv2.waitKey(1) & 0xff
                    if key in (ord('q'), 27):
                        break
                    if single_camera and key in changes:
                        view = views[0]
                        name, delta = changes[key]
                        setattr(view, name, getattr(view, name) + delta)
                        detector.history.clear()
                        if comparison is not None:
                            comparison.history.clear()
                        cameras[view.name].set_transform(camera_transform(view))
                        try:
                            view.frames.get_nowait()
                        except queue.Empty:
                            pass
                        pending.clear()
                        print(f'Camera x={view.x:g}, y={view.y:g}, z={view.z:g}, '
                              f'pitch={view.pitch:g}, yaw={view.yaw:g}', flush=True)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    try:
        main()
    except (ImportError, OSError, ValueError, RuntimeError) as error:
        print(f'Parking monitor unavailable: {error}', file=sys.stderr)
        raise SystemExit(2) from error
