import carla
import time
from parking.setup import setup

try:
    client = carla.Client('host.docker.internal', 2000)
    client.set_timeout(10.0)
except Exception as e:
    print(f"Error: {e}")
