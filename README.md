# autonomous-parking
This project uses CARLA to simulate an autonomous parking system.

## Credits & Requirements
- CARLA Simulator
- Python
- CARLA Python API
- Path Planning: The core Reeds-Shepp and A* algorithms used in this project is adapted from the open-source PythonRobotics repository by Atsushi Sakai. Licensed under the MIT License.
- Docker: If machine does not support CARLA natively

## Workflow
### Configure the CARLA connection (once per machine)

From the project root in PowerShell, copy the shared configuration:

```powershell
Copy-Item config.example.json config.local.json
```

Edit `config.local.json` to set your CARLA server address and port:

```json
{
  "carla_host": "localhost",
  "carla_port": 2000
}
```

Use `localhost` when CARLA is reachable on the same machine from your Python
environment. When the CARLA server runs inside Docker and the CARLA client runs on the host machine,
use `host.docker.internal` if your Docker setup provides that hostname. You can
also specify the CARLA server's IP address or hostname. Running on a non-Windows machine
may require additional steps to get working, including running the parking_monitor module headless.

`config.local.json` is ignored by Git, so each teammate keeps their own settings.
Commit `config.example.json` and `utils/connection_config.py` with the shared code.
If no local file exists, scripts use `config.example.json` (localhost:2000).
Configuration paths are resolved relative to the project, not the terminal's
current directory. No additional Python packages are required for configuration.

## Run the project

Start CARLA first, activate your Python environment with the project dependencies,
then run these commands from the project root:

### Run the camera

Start carla server and load Town05. Setup required environments. 

Start the monitor with the bundled model, layout, and SAHI inference.

```powershell
python -m parking.parking_monitor
```

### Ready for running main.py script
After monitors popup, YOLO camera system will acquire coordinates of target and staging area. Therefore, the vehicle is ready for autonomous parking.

```powershell
python main.py
```

Utility scripts can run directly or as modules, for example:

```powershell
python utils/set_daytime.py
# Equivalent module command:
python -m utils.set_daytime
```

All custom scripts in `parking/setup.py` and `utils` use the same connection
configuration. The original CARLA examples in `examples/` retain their own options.

After downloading CARLA, you should find examples folder under ROOT\CARLA\PythonAPI\examples.\
This examples folder contains some python scripts we can play around with.
<img width="849" height="688" alt="explorer_CRC8oJmcj9" src="https://github.com/user-attachments/assets/6931512f-9d07-4670-a75a-367d39523d41" />

We're running a conda environment called whatever under examples folder by using Anaconda Prompt.
<img width="1113" height="626" alt="WindowsTerminal_WrlR8GhmZc" src="https://github.com/user-attachments/assets/0706902b-f6c1-4869-bc94-cf2b34b6f03f" />

## How to create virtual environment
$ conda create --name ENV_NAME python=3.12

### Active virtual environment
$ activate ENV_NAME

### Additional packages recommended after activting an environment
$ pip install carla\
$ pip install jupyter\
$ pip install pygame\
$ pip install shapely\
$ pip install networkx

## YOLO camera system

YOLO sahi is a object detection models with SAHI (Slicing Aided Hyper Inference) to detect small or distant objects in large, high-resolution images.

In our project, YOLO sahi works as three separate cameras for monitoring parking lot rows A/B, C/D, and E/F.

YOLO camera system can detect and update available parking slots by requests.  
