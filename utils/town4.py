import carla
import time

try:
    client = carla.Client('host.docker.internal', 2000)
    client.set_timeout(10.0)
    
    # Load Town04 which has the square parking lot layout
    world = client.load_world('Town04')
    spectator = world.get_spectator()

    for _ in range(20):
        transform = spectator.get_transform()
        loc = transform.location
        print(f"Current Position -> X: {loc.x:.2f}, Y: {loc.y:.2f}, Z: {loc.z:.2f}")
        time.sleep(2.0)

except Exception as e:
    print(f"Error: {e}")
