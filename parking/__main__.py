"""Print the current parking handoff as JSON."""

import json
from pathlib import Path
import sys

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from parking.target import get_parking_handoff


def main() -> None:
    try:
        handoff = get_parking_handoff()
    except (OSError, ValueError) as error:
        print(f'Parking target unavailable: {error}', file=sys.stderr)
        raise SystemExit(2) from error

    print(json.dumps(handoff))


if __name__ == '__main__':
    main()
