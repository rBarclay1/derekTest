"""Color blob detection helpers for JetRover retrieve."""

import cv2
import numpy as np

# OpenCV HSV: H is 0-180. Red wraps around 0.
HSV_RANGES = {
    'red': [
        (np.array([0, 120, 70]), np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255])),
    ],
    'green': [
        (np.array([35, 80, 70]), np.array([85, 255, 255])),
    ],
    'blue': [
        (np.array([95, 80, 70]), np.array([130, 255, 255])),
    ],
}

DRAW_COLORS = {
    'red': (0, 0, 255),
    'green': (0, 255, 0),
    'blue': (255, 0, 0),
}


def color_mask(hsv, color_name):
    mask = None
    for lower, upper in HSV_RANGES[color_name]:
        part = cv2.inRange(hsv, lower, upper)
        mask = part if mask is None else cv2.bitwise_or(mask, part)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask


def largest_blob(mask, min_area, previous_center=None, max_jump_px=100.0):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    if previous_center is not None:
        candidates = []
        for contour in contours:
            moments = cv2.moments(contour)
            if cv2.contourArea(contour) < min_area or moments['m00'] == 0:
                continue
            cx = moments['m10'] / moments['m00']
            cy = moments['m01'] / moments['m00']
            distance = np.hypot(cx - previous_center[0], cy - previous_center[1])
            if distance <= max_jump_px:
                candidates.append((distance, contour))
        if not candidates:
            return None
        largest = min(candidates, key=lambda item: item[0])[1]
    else:
        largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < min_area:
        return None
    moments = cv2.moments(largest)
    if moments['m00'] == 0:
        return None
    cx = int(moments['m10'] / moments['m00'])
    cy = int(moments['m01'] / moments['m00'])
    return cx, cy, area, largest


def detect_largest_object(bgr_frame, target_color, min_area,
                          previous_center=None, max_jump_px=100.0):
    """Return (cx, cy, area, color_name, vis) or None.

    target_color may be red, green, blue, or any.
    """
    hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
    vis = bgr_frame.copy()
    colors = HSV_RANGES.keys() if target_color == 'any' else [target_color]

    best = None
    for name in colors:
        if name not in HSV_RANGES:
            continue
        blob = largest_blob(color_mask(hsv, name), min_area, previous_center, max_jump_px)
        if blob is None:
            continue
        cx, cy, area, contour = blob
        if best is None or area > best[2]:
            best = (cx, cy, area, name, contour)

    if best is None:
        return None

    cx, cy, area, name, contour = best
    cv2.drawContours(vis, [contour], -1, DRAW_COLORS.get(name, (255, 255, 255)), 2)
    cv2.circle(vis, (cx, cy), 6, (0, 255, 255), -1)
    cv2.putText(vis, f'{name} {area:.0f}px', (cx + 8, cy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, DRAW_COLORS.get(name, (255, 255, 255)), 1)
    return cx, cy, area, name, vis


def depth_to_meters(depth_image, cx, cy, encoding=None,
                    min_depth=0.05, max_depth=5.0):
    if depth_image is None:
        return None
    h, w = depth_image.shape[:2]
    if not (0 <= cy < h and 0 <= cx < w):
        return None
    # Median of a small window is more stable than a single pixel.
    y0, y1 = max(0, cy - 2), min(h, cy + 3)
    x0, x1 = max(0, cx - 2), min(w, cx + 3)
    window = depth_image[y0:y1, x0:x1].astype(np.float32)
    # Use the declared ROS encoding, never infer units from the distance.
    if encoding is None:
        encoding = '16UC1' if depth_image.dtype == np.uint16 else '32FC1'
    if encoding.upper() == '16UC1':
        window /= 1000.0
    elif encoding.upper() != '32FC1':
        return None
    valid = window[np.isfinite(window) & (window >= min_depth) & (window <= max_depth)]
    if valid.size == 0:
        return None
    return float(np.median(valid))
