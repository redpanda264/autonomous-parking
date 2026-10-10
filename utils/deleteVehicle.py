import carla

client = carla.Client('localhost', 2000)
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