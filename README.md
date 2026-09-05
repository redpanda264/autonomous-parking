# autonomous-parking
This project uses CARLA to simulate an autonomous parking system.

## Requirements
- CARLA
- Python
- CARLA Python API

## Workflow
After downloading CARLA, you should find examples folder under ROOT\CARLA\PythonAPI\examples.\
This examples folder contains some python scripts we can play around.
<img width="849" height="688" alt="explorer_CRC8oJmcj9" src="https://github.com/user-attachments/assets/6931512f-9d07-4670-a75a-367d39523d41" />

We're running a conda environment called whatever under examples folder by using Anaconda Prompt.
<img width="1113" height="626" alt="WindowsTerminal_WrlR8GhmZc" src="https://github.com/user-attachments/assets/0706902b-f6c1-4869-bc94-cf2b34b6f03f" />

### How to create virtual environment
$ conda create --name ENV_NAME python=3.12\

### Active virtual environment
$ activate ENV_NAME\

### Additional libraries recommended after activting an environment
$ pip install carla\
$ pip install jupyter\
