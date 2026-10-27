import carla
import time
from utils.connection_config import load_connection_config


def setup():
    # world setup
    try:
        host, port = load_connection_config()
        client = carla.Client(host, port)
        client.set_timeout(10.0)
        world = client.get_world()

        # don't reload the map if it is already loaded
        if not world.get_map().name.endswith("Town05"):
            world = client.load_world("Town05")

            # only move spectator if it's in the default map location
            world.wait_for_tick()
            spectator = world.get_spectator()
            spectator.set_transform(carla.Transform(
                carla.Location(x=-13, y=-27.2, z=10),
                carla.Rotation(yaw=90)
            ))

        # clean up existing spawned actors
        else:
            print("Map already loaded")
            world.wait_for_tick()
            spectator = world.get_spectator()
            existing_vehicles = world.get_actors().filter('vehicle.*')
            # spectator.set_transform(carla.Transform(
            #     carla.Location(x=-13, y=-27.2, z=10),
            #     carla.Rotation(yaw=90)
            # ))

            for vehicle in existing_vehicles:
                if vehicle.attributes.get("role_name") == "hero":
                    vehicle.destroy()
                    print(f"Destroyed ego vehicle: {vehicle.id}")

    except Exception as e:
        print(f"Error: {e}")


    # Spawn ego vehicle (tesla model 3)
    blueprints = world.get_blueprint_library()
    ego_bp = blueprints.find("vehicle.tesla.model3")
    ego_bp.set_attribute("role_name", "hero")
    spawn_points = world.get_map().get_spawn_points() # list of all spawn points in map
    ego_spawn_point = carla.Transform(
        carla.Location(x=0, y=0, z=0.5),
        carla.Rotation(yaw=180),
    )

    world.wait_for_tick()

    try:
        ego = world.spawn_actor(ego_bp, ego_spawn_point)
    except Exception as e:
        print(f"Failed to spawn ego vehicle: {e}")

    # spawn vehicles to left and right of the target space (for testing)
    # parked_bp = blueprints.find("vehicle.audi.tt")
    # left_vehicle = world.try_spawn_actor(
    #     parked_bp,
    #     carla.Transform(
    #         carla.Location(x=-13, y=-24.5, z=0.5),
    #         carla.Rotation(yaw=180)
    #     )
    # )

    # right_vehicle = world.try_spawn_actor(
    #     parked_bp,
    #     carla.Transform(
    #         carla.Location(x=-13, y=-30, z=0.5),
    #         carla.Rotation(yaw=180)
    #     )
    # )

    #spawned_vehicles = [left_vehicle, right_vehicle]
    spawned_vehicles = []

    #print(f"Ego location: {ego.get_location()}")
    #print(f"Spectator location: {spectator.get_location()}")

    # print all actors
    # actors = world.get_actors()
    # for actor in actors:
    #     print(f"Actor ID: {actor.id}, Type: {actor.type_id}, Location: {actor.get_location()}")

    return client, world, ego, spectator, spawned_vehicles
