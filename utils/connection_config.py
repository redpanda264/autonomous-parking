import json
from pathlib import Path


def load_connection_config():
    project_root = Path(__file__).resolve().parents[1]
    config_path = project_root / "config.local.json"

    # Use the shared example when no local configuration exists.
    if not config_path.exists():
        config_path = project_root / "config.example.json"

    with config_path.open(encoding="utf-8") as config_file:
        config = json.load(config_file)

    return config["carla_host"], int(config["carla_port"])
