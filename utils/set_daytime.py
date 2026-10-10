import carla

try:
   
    client = carla.Client('host.docker.internal', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    print("Setting weather to a clear, bright noon...")
    # Cloudiness = 0.0
    # Sun altitude = 90.0 
    weather = carla.WeatherParameters(cloudiness=0.0, sun_altitude_angle=90.0)
    world.set_weather(weather)

    print("The sky is daytime.")

except Exception as e:
    print(f"Failed to change weather: {e}")
