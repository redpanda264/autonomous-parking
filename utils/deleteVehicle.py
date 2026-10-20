import carla

if __package__:
    from .connection_config import load_connection_config
else:
    from connection_config import load_connection_config

host, port = load_connection_config()
client = carla.Client(host, port)
client.set_timeout(10.0)
world = client.get_world()
vehicles = world.get_actors().filter('vehicle.*')

for vehicle in vehicles:
    print(
        "ID:", vehicle.id,
        "Type:", vehicle.type_id,
        "Position:", vehicle.get_location()
    )

vehicle_id = int(input("Enter the ID of the vehicle to delete: "))

vehicle = world.get_actor(vehicle_id)

if vehicle is not None:
    vehicle.destroy()
    print(f"Vehicle {vehicle_id} deleted!")
