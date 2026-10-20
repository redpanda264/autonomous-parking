import carla

if __package__:
    from .connection_config import load_connection_config
else:
    from connection_config import load_connection_config

host, port = load_connection_config()
client = carla.Client(host, port)
client.set_timeout(10.0)

world = client.get_world()

spectator = world.get_spectator()
print(f"Spectator Coordinates: {spectator.get_transform()}")
