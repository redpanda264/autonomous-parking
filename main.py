import carla
import time
from parking.navigation_controller import NavigationController
from parking.setup import setup
from parking.controller import Controller

def main():
    # call setup script and get necessary actor references
    try:
        client, world, ego, spectator, spawned_vehicles = setup()
    except Exception as e:
        print(f"Setup failed: {e}")
        return

    navigation_controller = NavigationController(ego, world)
    # navigation logic loop
    try:
        while True:
            navigation_controller.control()
            world.wait_for_tick()
            
    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        # destroy actors, stop navigation script
        ego.destroy()
        for vehicle in spawned_vehicles:
            vehicle.destroy()
        print("Actors destroyed")

    # once navigation is complete, transition to precise parking controller
    
    parking_controller = Controller(ego, world)
    # parking logic loop
    try:
        while True:
            parking_controller.control()
            world.wait_for_tick()
            
    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        # destroy actors, stop parking script
        ego.destroy()
        for vehicle in spawned_vehicles:
            vehicle.destroy()
        print("Actors destroyed")


# run main loop when file is executed directly
if __name__ == "__main__":
    main()
