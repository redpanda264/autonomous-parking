import carla
import time
from parking.setup import setup
from parking.controller import Controller

def main():
    # call setup script and get necessary actor references
    try:
        client, world, ego, spectator = setup()
    except Exception as e:
        print(f"Setup failed: {e}")
        return

    # TODO: create navigation controller
    
    parking_controller = Controller(ego, world)

    # parking logic loop
    try:
        while True:

            # TODO: add parking logic here
            parking_controller.control()
            

            # print locations for debugging
            #print(f"Ego location: {ego.get_location()}")
            #print(f"Spectator location: {spectator.get_location()}")

            world.wait_for_tick()
            
    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        # destroy actors, stop parking script
        ego.destroy()
        print("Actors destroyed")


# run main loop when file is executed directly
if __name__ == "__main__":
    main()
