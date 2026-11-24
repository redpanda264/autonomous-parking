"""Parking target telemetry and monitor overlay (independent of occupancy)."""
import json
import time
import math


def publish_target(path, vehicle, slot, phase):
    location=vehicle.get_location()
    report=dict(vehicle_id=vehicle.id,slot_id=slot['id'] if slot else None,phase=phase,
                distance_m=math.hypot(location.x-slot['x'],location.y-slot['y']) if slot else None,
                updated_at_unix=time.time())
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(report),encoding='utf-8')
    for attempt in range(5):
        try:
            temporary.replace(path)
            return
        except PermissionError:
            if attempt==4:
                return  # Display telemetry must not interrupt vehicle control.
            time.sleep(.01)


def load_target(path):
    if path is None:
        return None
    try:
        report=json.loads(path.read_text(encoding='utf-8'))
        if time.time()-report['updated_at_unix']>5:
            return None
        return report
    except (OSError, ValueError, KeyError):
        return None


def draw_target(image,slots,roi,target):
    import cv2
    import numpy as np
    if target is None:
        text='Target: waiting for parking controller'
    else:
        sid=target.get('slot_id')
        text='Target: %s | %s' % (sid or 'selecting',target['phase'])
        if target.get('distance_m') is not None:
            text+=' | %.2f m' % target['distance_m']
        for slot in slots:
            if slot['id']==sid and slot['polygon']:
                points=np.asarray(slot['polygon'],np.int32)-np.array(roi[:2],np.int32)
                cv2.polylines(image,[points],True,(0,220,255),5)
                break
    # Separate banner preserves the occupancy counts below it.
    image=cv2.copyMakeBorder(image,42,0,0,0,cv2.BORDER_CONSTANT,value=(15,15,15))
    cv2.putText(image,text,(8,28),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,220,255),1)
    return image
