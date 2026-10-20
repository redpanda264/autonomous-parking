import carla

if __package__:
    from .connection_config import load_connection_config
else:
    from connection_config import load_connection_config

host, port = load_connection_config()
client = carla.Client(host, port)
client.set_timeout(10.0)

world = client.load_world('Town05')

print("Loaded:", world.get_map().name)
