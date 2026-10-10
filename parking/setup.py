import carla
import time

# world setup
try:
    client = carla.Client('host.docker.internal', 2000)
    client.set_timeout(10.0)
    world = client.load_world('Town05')
    spectator = world.get_spectator()
except Exception as e:
    print(f"Error: {e}")


# Spawn ego vehicle (tesla model 3)
blueprints = world.get_blueprint_library()
ego_bp = blueprints.find("vehicle.tesla.model3")
ego_bp.set_attribute("role_name", "hero")
spawn_points = world.get_map().get_spawn_points() # list of all spawn points in map
ego_spawn_point = carla.Transform(
    carla.Location(x=-42, y=-28, z=0.5), carla.Rotation(yaw=0.0)
)

world.wait_for_tick()

try:
    ego = world.spawn_actor(ego_bp, ego_spawn_point)
except Exception as e:
    print(f"Failed to spawn ego vehicle: {e}")


world.wait_for_tick() # Wait for the world to spawn vehicle before teleporting, doesn't work otherwise.
# Teleport spectator to a position above the ego vehicle
spectator.set_transform(
    carla.Transform(
        ego.get_location() + carla.Location(x=2, y=0, z=2),
        carla.Rotation(yaw=180) # turn around
    )
)

print(f"Ego location: {ego.get_location()}")
print(f"Spectator location: {spectator.get_location()}")

# print all actors
# actors = world.get_actors()
# for actor in actors:
#     print(f"Actor ID: {actor.id}, Type: {actor.type_id}, Location: {actor.get_location()}")