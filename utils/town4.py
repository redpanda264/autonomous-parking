import carla
import time

if __package__:
    from .connection_config import load_connection_config
else:
    from connection_config import load_connection_config

try:
    host, port = load_connection_config()
    client = carla.Client(host, port)
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
