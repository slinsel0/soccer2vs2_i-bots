#!/usr/bin/env python3
import sys, time, os, math
import numpy as np
import cv2

# --- Picamera2 & libcamera ---
from picamera2 import Picamera2
try:
    import libcamera
    HAS_LIBCAMERA_DRAFT = hasattr(libcamera.controls, "draft")
except Exception:
    libcamera = None
    HAS_LIBCAMERA_DRAFT = False

# ========= USER SETTINGS =========
WIDTH, HEIGHT = 640, 280          # Passe an deine Auflösung an
EXPOSURE_TIME_US = 15000           # z.B. 10 ms; ggf. anpassen
MIN_CORNERS_FOR_CAPTURE = 10       # Mindestanzahl ChArUco-Ecken pro Frame
TARGET_CAPTURES = 60               # Anzahl guter Ansichten vor Kalibrierung
SAVE_BASENAME = f"fisheye_charuco_calib_{WIDTH}x{HEIGHT}"
# ChArUco-Board: 7x5, 40 mm Square, 30 mm Marker, DICT_5X5_1000 (aus deiner PDF)
SQUARE_LEN_M = 0.040
MARKER_LEN_M = 0.030
SQUARES_X = 7
SQUARES_Y = 5
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
# =================================

# Erzeuge ChArUco-Board
board = cv2.aruco.CharucoBoard_create(
    squaresX=SQUARES_X, squaresY=SQUARES_Y,
    squareLength=SQUARE_LEN_M, markerLength=MARKER_LEN_M,
    dictionary=ARUCO_DICT
)

# ArUco Detector-Parameter
aruco_params = cv2.aruco.DetectorParameters_create()
# (Optional) Robustheit erhöhen:
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

# Picamera2 konfigurieren
picam2 = Picamera2(1)
controls = {
    "FrameDurationLimits": (EXPOSURE_TIME_US, EXPOSURE_TIME_US),
    "ExposureTime": EXPOSURE_TIME_US,
    "AnalogueGain": 1.0,
    "AeEnable": False,
    "AwbEnable": False,
    "Sharpness": 1.0,
    "Contrast": 1.0,
    "Saturation": 1.0
}


if HAS_LIBCAMERA_DRAFT:
    controls["NoiseReductionMode"] = libcamera.controls.draft.NoiseReductionModeEnum.Off

config = picam2.create_video_configuration(main={"format": "RGB888", "size": (WIDTH, HEIGHT)},
                                           controls=controls)
    # Falls deine Kalibrierung ohne Flip gemacht wurde: keine Transforms setzen!
transform = libcamera.Transform(hflip=True, vflip=True)
config["transform"] = transform





picam2.configure(config)
picam2.start()
time.sleep(0.5)  # kurze Warmup-Zeit

print("[i] Steuerung: 'c' = Frame übernehmen, 'u' = sofort kalibrieren, 'q' = Abbruch")

# Listen für Fisheye-Kalibrierung
objpoints_list = []  # List of (Ni,1,3)
imgpoints_list = []  # List of (Ni,1,2)
frames_used = 0

def capture_frame():
    frame = picam2.capture_array()  # RGB888
    return frame

def detect_charuco(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=aruco_params)
    if ids is None or len(ids) == 0:
        return None

    # Subpixel-Verfeinerung für Marker-Ecken verbessert ChArUco-Interpolation
    for c in corners:
        cv2.cornerSubPix(gray, c, (3,3), (-1,-1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

    # Interpolierte Schachbrettecken (ChArUco)
    retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        markerCorners=corners, markerIds=ids, image=gray, board=board
    )
    if retval is None or charuco_ids is None or len(charuco_ids) == 0:
        return None

    return (corners, ids, charuco_corners, charuco_ids)






def draw_feedback(frame, detection):
    if detection is None: 
        return frame, 0
    corners, ids, charuco_corners, charuco_ids = detection
    out = frame.copy()
    cv2.aruco.drawDetectedMarkers(out, corners, ids)
    cv2.aruco.drawDetectedCornersCharuco(out, charuco_corners, charuco_ids, (0,255,0))
    n = len(charuco_ids)
    cv2.putText(out, f"ChArUco corners: {n}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2, cv2.LINE_AA)
    return out, n

def _charuco_objpoints(board, charuco_ids):
    """
    Liefert die 3D-Objektpunkte zu den gefundenen ChArUco-IDs
    im Format (Ni, 1, 3) als float32 – robust gegen unterschiedliche
    OpenCV-Board-Layouts.
    """
    objp_all = np.asarray(board.chessboardCorners, dtype=np.float32)
    # Erwartet (N, 3). Falls deine OpenCV-Version (N,1,3) liefert, behandeln wir das ebenfalls.
    if objp_all.ndim == 2 and objp_all.shape[1] == 3:
        objp = objp_all[charuco_ids.flatten(), :]          # (Ni, 3)
        objp = objp.reshape(-1, 1, 3)                      # -> (Ni, 1, 3)
    elif objp_all.ndim == 3 and objp_all.shape[1:] == (1, 3):
        objp = objp_all[charuco_ids.flatten(), :, :]       # (Ni, 1, 3)
    else:
        raise ValueError(f"Unerwartete chessboardCorners-Form: {objp_all.shape}")
    return objp

def add_sample(detection):
    global frames_used
    corners, ids, charuco_corners, charuco_ids = detection

    # Bildpunkte sicher als (Ni,1,2) float32
    imgp = np.ascontiguousarray(
        charuco_corners.reshape(-1, 1, 2).astype(np.float32)
    )

    # Passende 3D-Objektpunkte (Ni,1,3) float32
    objp = _charuco_objpoints(board, charuco_ids)

    objpoints_list.append(objp)
    imgpoints_list.append(imgp)
    frames_used += 1


def run_calibration(img_size):
    flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
        cv2.fisheye.CALIB_CHECK_COND |
        cv2.fisheye.CALIB_FIX_SKEW
    )
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

    K = np.zeros((3,3), dtype=np.float64)
    D = np.zeros((4,1), dtype=np.float64)
    rvecs, tvecs = [], []

    print("[i] Starte Fisheye-Kalibrierung...")
    rms, _, _, _, _ = cv2.fisheye.calibrate(
        objectPoints=objpoints_list,
        imagePoints=imgpoints_list,
        image_size=img_size,
        K=K, D=D, rvecs=rvecs, tvecs=tvecs,
        flags=flags, criteria=criteria
    )
    print(f"[i] RMS-Fehler: {rms:.6f}")
    print("[i] K=\n", K)
    print("[i] D=\n", D.T)

    # Speichern
    np.savez(SAVE_BASENAME + ".npz", K=K, D=D, rvecs=np.array(rvecs, dtype=object), tvecs=np.array(tvecs, dtype=object),
             image_size=np.array(img_size))
    # YAML für andere Tools
    fs = cv2.FileStorage(SAVE_BASENAME + ".yml", cv2.FILE_STORAGE_WRITE)
    fs.write("K", K); fs.write("D", D); fs.write("width", img_size[0]); fs.write("height", img_size[1])
    fs.release()

    print(f"[i] Gespeichert: {SAVE_BASENAME}.npz und {SAVE_BASENAME}.yml")
    return K, D, rms

# --- Main Loop: sammeln & kalibrieren ---
img_size = (WIDTH, HEIGHT)
while True:
    frame = capture_frame()
    det = detect_charuco(frame)
    vis, n_corners = draw_feedback(frame, det)

    # Auto-Hinweis: Frame wird automatisch "grün", wenn genug Ecken da sind
    color = (0, 255 if n_corners >= MIN_CORNERS_FOR_CAPTURE else 255, 0)
    msg = f"{frames_used}/{TARGET_CAPTURES}  ('c' capture, 'u' calibrate, 'q' quit)"
    cv2.putText(vis, msg, (20, HEIGHT-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

    cv2.imshow("ChArUco Fisheye Calibration", vis)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):
        if det is not None and n_corners >= MIN_CORNERS_FOR_CAPTURE:
            add_sample(det)
            print(f"[i] Sample übernommen ({frames_used}/{TARGET_CAPTURES})")
        else:
            print("[!] Zu wenige ChArUco-Ecken – näher ran, kippen/rollen, andere Perspektive.")
    elif key == ord('u'):
        if frames_used >= max(12, TARGET_CAPTURES//2):
            K, D, rms = run_calibration(img_size)
            print("[i] Fertig. Nutze danach das Undistortion-Demo (siehe 2).")
            break
        else:
            print("[!] Zu wenige Samples für stabile Kalibrierung.")
    elif key == ord('q'):
        print("[i] Abbruch.")
        break

    if frames_used >= TARGET_CAPTURES:
        K, D, rms = run_calibration(img_size)
        break

picam2.stop()
cv2.destroyAllWindows()
