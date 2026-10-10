import carla

client = carla.Client("host.docker.internal", 2000)
client.set_timeout(10.0)

world = client.get_world()
