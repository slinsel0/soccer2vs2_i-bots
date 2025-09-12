#!/usr/bin/env python3
import time
import numpy as np
import cv2
from picamera2 import Picamera2
try:
    import libcamera
except Exception:
    libcamera = None

# ====== Lade Kalibrierung ======
SAVE_BASENAME = "fisheye_charuco_calib_1280x800"  # ggf. anpassen!
data = np.load(SAVE_BASENAME + ".npz", allow_pickle=True)
K = data["K"]; D = data["D"]; img_size = tuple(data["image_size"])
WIDTH, HEIGHT = img_size


EXPOSURE_TIME_US = 10000           # z.B. 10 ms; ggf. anpassen

# ====== Kamera ======
picam2 = Picamera2(1)
picam2.video_configuration.controls.FrameRate = 50

controls = {
    "FrameDurationLimits": (EXPOSURE_TIME_US, EXPOSURE_TIME_US),
    "ExposureTime": EXPOSURE_TIME_US,
    "AeEnable": False, "AwbEnable": False,
    "AnalogueGain": 1.0, "Sharpness": 1.0, "Contrast": 1.0, "Saturation": 1.0

}
config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (WIDTH, HEIGHT)},
    
    controls=controls
)
picam2.configure(config)
picam2.start()
time.sleep(0.3)

cv2.namedWindow("Fisheye Undistort (balance)")

def make_maps(balance_percent=50):
    balance = np.clip(balance_percent/100.0, 0.0, 1.0)
    R = np.eye(3)
    Knew = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (WIDTH, HEIGHT), R, balance=balance, fov_scale=1.0
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, R, Knew, (WIDTH, HEIGHT), m1type=cv2.CV_16SC2
    )
    return map1, map2, Knew

balance_val = 50
map1, map2, Knew = make_maps(balance_val)
cv2.createTrackbar("balance", "Fisheye Undistort (balance)", balance_val, 100,
                   lambda v: None)

print("[i] 'q' = quit")
while True:
    # evtl. Maps neu berechnen, wenn Slider geändert wurde
    new_val = cv2.getTrackbarPos("balance", "Fisheye Undistort (balance)")
    if new_val != balance_val:
        balance_val = new_val
        map1, map2, Knew = make_maps(balance_val)

    frame = picam2.capture_array()
    undist = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # Anzeige (BGR)
    vis = cv2.cvtColor(undist, cv2.COLOR_RGB2BGR)
    cv2.putText(vis, f"balance={balance_val}  Knew fx={Knew[0,0]:.1f} fy={Knew[1,1]:.1f}",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)
    cv2.imshow("Fisheye Undistort (balance)", undist)
    if (cv2.waitKey(1) & 0xFF) == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
