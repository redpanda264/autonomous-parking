"""Bounded retries for JSON snapshots replaced concurrently on Windows."""
import json
import time


def read_status(path, attempts=10, delay=.02):
    for attempt in range(attempts):
        try:
            return json.loads(path.read_text(encoding='utf-8'))
        except (PermissionError, FileNotFoundError, json.JSONDecodeError):
            if attempt==attempts-1:
                raise
            time.sleep(delay)
