"""Read mapped SAHI-free slots and reject stale or mismatched output."""
import json
import math
import time
from .status_io import read_status


def available_slots(status_path, layout_path, map_name, max_age=30):
    status=read_status(status_path)
    age=time.time()-status['updated_at_unix']
    if not math.isfinite(age) or age < -2 or age > max_age:
        raise ValueError('SAHI output is stale; run the parking monitor and retry')
    if status.get('inference')!='sahi':
        raise ValueError('Target selection requires SAHI camera inference')
    if status.get('map')!=map_name:
        raise ValueError('Monitor map differs or is missing; restart the updated monitor')
    layout=json.loads(layout_path.read_text(encoding='utf-8'))['parking_slots']
    available=set(status['available'])
    records={s['id']:s for s in status['slots']}
    matches=[]
    for slot in layout:
        observed=records.get(slot['id'],{})
        if slot['id'] not in available or observed.get('status')!='free':
            continue
        if observed.get('world')!=slot:
            raise ValueError('Monitor layout differs from selected layout')
        matches.append(slot)
    return matches
