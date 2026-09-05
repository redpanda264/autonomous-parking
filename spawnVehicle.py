import carla
import random

# 1. Connect to CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 2. Get the current world
world = client.get_world()

# 3. Get the blueprint library
blueprint_library = world.get_blueprint_library()

# 4. Select a vehicle
vehicle_bp = blueprint_library.find('vehicle.tesla.model3')

# 5. Get available spawn points
spawn_points = world.get_map().get_spawn_points()

# 6. Pick one spawn point
spawn_point = random.choice(spawn_points)

# 7. Spawn the vehicle
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

print("Vehicle spawned!")
print(vehicle)