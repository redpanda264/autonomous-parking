import carla

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.load_world('Town05')

print("Loaded:", world.get_map().name)