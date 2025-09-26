#!/usr/bin/env python3
# file: dual_cam_serial_tracking.py

import threading
import queue
import time
import signal
import sys
import struct
import zlib
import serial
from threading import Event, Lock

import cv2
import numpy as np

# ============================================================
# -----------  KONFIGURATION  -------------------------------
# ============================================================

# --- Serielle Schnittstelle (COBS+CRC32, wie pi_tx.py) ---
SER_PORT = "/dev/ttyACM0"     # ggf. ACM1
SER_BAUD = 2_000_000          # USB-CDC, unkritisch aber ok
SER_RATE_HZ = 200             # Sende-Frequenz

# --- Kalibrierung (pro Kamera separate Datei empfohlen) ---
CALIB_CAM0 = "ioi.npz"   # K, D, image_size
CALIB_CAM1 = "oioio.npz"   # K, D, image_size
FISHEYE_BALANCE = 0.80                      # 0..1

# --- Kamera-Steuerung ---
EXPOSURE_TIME_US = 50000
FPS = 30

# --- BallTracker-Konfig (extern aus JSON) ---
CONFIG_PATH = 'ball_tracker_config.json'

# ============================================================
# -----------  FISHEYE-ENTZERRUNG  --------------------------
# ============================================================

class FisheyeUndistorter:
    def __init__(self, calib_path: str, balance: float = 0.5):
        self.valid = False
        self.map1 = self.map2 = None
        self.Knew = None
        self.WIDTH = self.HEIGHT = None
        try:
            data = np.load(calib_path, allow_pickle=True)
            K = data["K"]; D = data["D"]
            img_size = tuple(data["image_size"])
            self.WIDTH, self.HEIGHT = img_size
            R = np.eye(3)
            bal = float(np.clip(balance, 0.0, 1.0))
            self.Knew = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, D, (self.WIDTH, self.HEIGHT), R, balance=bal, fov_scale=1.0
            )
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                K, D, R, self.Knew, (self.WIDTH, self.HEIGHT), m1type=cv2.CV_16SC2
            )
            self.valid = True
            print(f"[Fisheye] '{calib_path}' geladen: {self.WIDTH}x{self.HEIGHT}, balance={bal:.2f}")
        except Exception as e:
            print(f"[Fisheye] WARN: Kalibrierung '{calib_path}' nicht nutzbar: {e}")

    def undistort(self, frame_bgr: np.ndarray) -> np.ndarray:
        if not self.valid:
            return frame_bgr
        return cv2.remap(frame_bgr, self.map1, self.map2,
                         interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)

# ============================================================
# -----------  SERIELLER SENDER (pi_tx integriert) ----------
# ============================================================

# Frame-Layout: <BBIfff  (msg_id:uint8, seq:uint8, t_us:uint32, vx:float, vy:float, cam_id:float)
FMT_BODY = "<BBIfff"
MSG_ID_VELOCITY = 1  # Felder transportieren x,y (px) + Kamera-ID (als float)

def _cobs_encode(buf: bytes) -> bytes:
    # Kleine, schnelle COBS-Implementierung genügt hier; du kannst auch 'from cobs import cobs' nutzen
    try:
        from cobs import cobs
        return cobs.encode(buf)
    except Exception:
        # Notfall-COBS (einfach, nicht super-optimiert)
        out = bytearray()
        idx = 0
        while idx < len(buf):
            block_start = idx
            idx += 1
            code_pos = len(out)
            out.append(0)  # Platzhalter für code
            code = 1
            while idx <= len(buf) and code < 0xFF:
                if idx == len(buf) or buf[idx-1] == 0:
                    break
                out.append(buf[idx-1])
                idx += 1
                code += 1
            out[code_pos] = code
            if idx <= len(buf) and (idx == len(buf) or buf[idx-1] == 0):
                idx += 0  # der Null-Byte trennt Frames; hier im Block nicht abspeichern
        return bytes(out)

class SerialCOBSSender(threading.Thread):
    def __init__(self, shared_state, stop_event: Event,
                 port=SER_PORT, baud=SER_BAUD, rate_hz=SER_RATE_HZ):
        super().__init__(name="SerialSender", daemon=True)
        self.shared = shared_state
        self.stop_event = stop_event
        self.port = port
        self.baud = baud
        self.period = 1.0 / float(rate_hz)
        self.seq = 0
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0)
        print(f"[Serial] geöffnet: {self.port} @ {self.baud}")

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _make_frame(self, vx: float, vy: float, cam_id_f: float) -> bytes:
        t_us = time.perf_counter_ns() // 1000
        body = struct.pack(FMT_BODY,
                           MSG_ID_VELOCITY,
                           self.seq & 0xFF,
                           t_us & 0xFFFFFFFF,
                           float(vx), float(vy), float(cam_id_f))
        crc = zlib.crc32(body) & 0xFFFFFFFF
        full = body + struct.pack("<I", crc)
        self.seq = (self.seq + 1) & 0xFF
        return _cobs_encode(full) + b"\x00"

    def run(self):
        try:
            self.open()
        except Exception as e:
            print(f"[Serial] Konnte Port nicht öffnen: {e}")
            return

        next_t = time.perf_counter()
        while not self.stop_event.is_set():
            st = self.shared.get()
            if st["camera_id"] >= 0:
                # --- WERTE SCHICKEN ---
                # Option 1 (aktuell): Pixel direkt übertragen + Kamera-ID (als float)
                vx, vy, cam_id_f = float(st["x"]), float(st["y"]), float(st["camera_id"])

                # Option 2 (alternativ): auf −1..1 normieren:
                # vx = np.clip(st["x"] / (self.shared.frame_width/2.0), -1.0, 1.0)
                # vy = np.clip(st["y"] / (self.shared.frame_height), 0.0, 1.0)
                # cam_id_f = float(st["camera_id"])

                try:
                    self.ser.write(self._make_frame(vx, vy, cam_id_f))
                except Exception:
                    # Ignorieren, nächster Versuch
                    pass

            # Takt halten
            next_t += self.period
            dt = next_t - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.perf_counter()

        self.close()

# ============================================================
# -----------  BALL TRACKER (unverändert, kurz integriert) ---
# ============================================================

class BallTracker:
    """
    Erkennt einen orangefarbenen Golfball auf grünem Untergrund.
    Parameter kommen aus JSON (CONFIG_PATH).
    Liefert (vis, mask_inv, transformed_xy, kp_size).
    """
    def __init__(self, config_path: str = CONFIG_PATH):
        try:
            import json
            with open(config_path, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
        except FileNotFoundError:
            raise RuntimeError(f"Konfigurationsdatei nicht gefunden: {config_path}")

        self.orange_lower = np.array([cfg['h_lower'], cfg['s_lower'], cfg['v_lower']])
        self.orange_upper = np.array([cfg['h_upper'], cfg['s_upper'], cfg['v_upper']])

        k = max(1, cfg.get('kernel_size', 2))
        self.erosion_kernel = np.ones((k, k), np.uint8)
        self.dilation_kernel = np.ones((k, k), np.uint8)

        self.min_blob_area = cfg['min_blob_area']
        self.max_blob_area = cfg['max_blob_area']
        self.min_circularity = cfg['min_circularity']
        self.min_convexity = cfg['min_convexity']
        self.min_inertia_ratio = cfg['min_inertia_ratio']
        self.minDistBetweenBlobs = cfg.get('min_dist_between_blobs', 10)
        self.minThreshold = cfg.get('min_threshold', 10)
        self.maxThreshold = cfg.get('max_threshold', 200)
        self.thresholdStep = cfg.get('threshold_step', 10)

        self._exclusion_masks = {}
        self.detector = self._make_detector()

        self.last_detection_time = {0: 0.0, 1: 0.0}

    def _make_detector(self):
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = self.minThreshold
        params.maxThreshold = self.maxThreshold
        params.thresholdStep = self.thresholdStep
        params.filterByArea = True
        params.minArea = self.min_blob_area
        params.maxArea = self.max_blob_area
        params.filterByCircularity = True
        params.minCircularity = self.min_circularity
        params.filterByConvexity = True
        params.minConvexity = self.min_convexity
        params.filterByInertia = True
        params.minInertiaRatio = self.min_inertia_ratio
        params.minDistBetweenBlobs = self.minDistBetweenBlobs
        return cv2.SimpleBlobDetector_create(params)

    @staticmethod
    def _poly(pts): return np.array(pts, dtype=np.int32)

    def _exclusion(self, cam_id: int, shape) -> np.ndarray:
        if cam_id in self._exclusion_masks:
            return self._exclusion_masks[cam_id]
        h, w = shape[:2]
        mask = np.full((h, w), 255, dtype=np.uint8)
    
         
        self._exclusion_masks[cam_id] = mask
        return mask

    def detect_ball(self, frame_bgr: np.ndarray, cam_id: int):
        H, W = frame_bgr.shape[:2]
        excl = self._exclusion(cam_id, frame_bgr.shape)

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        base_mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        base_mask = cv2.bitwise_and(base_mask, excl)
        eroded = cv2.erode(base_mask, self.erosion_kernel, iterations=1)
        dilated = cv2.dilate(eroded, self.dilation_kernel, iterations=2)
        closed  = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, self.dilation_kernel, iterations=1)

        mask_inv = cv2.bitwise_not(closed)
        keypoints = self.detector.detect(mask_inv)

        vis = frame_bgr.copy()
        vis[excl == 0] = (0, 0, 0)

        coords = None
        size = 0.0
        if keypoints:
            kp = max(keypoints, key=lambda k_: k_.size)
            size = kp.size
            vis = cv2.drawKeypoints(vis, [kp], None, (0, 0, 255),
                                    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            x_cv, y_cv = kp.pt
            cx, cy = (W * 0.5), (H * 0.5)
            x_rel = x_cv - cx
            y_rel = cy - y_cv
            coords = (x_rel, y_rel)
            self.last_detection_time[cam_id] = time.time()

        return vis, mask_inv, coords, size

# ============================================================
# -----------  RAM-SHARED STATE -----------------------------
# ============================================================

class SharedState:
    def __init__(self):
        self._lock = Lock()
        self._state = dict(camera_id=-1, x=0.0, y=0.0, kp_size=0.0, timestamp=0.0)
        self.frame_width = 1.0
        self.frame_height = 1.0

    def set_frame_size(self, w: int, h: int):
        with self._lock:
            self.frame_width = float(w)
            self.frame_height = float(h)

    def update(self, camera_id: int, x: float, y: float, kp_size: float, ts: float):
        with self._lock:
            self._state.update(camera_id=int(camera_id), x=float(x), y=float(y),
                               kp_size=float(kp_size), timestamp=float(ts))

    def clear(self):
        with self._lock:
            self._state.update(camera_id=-1, x=0.0, y=0.0, kp_size=0.0, timestamp=time.time())

    def get(self):
        with self._lock:
            return dict(self._state)

# ============================================================
# -----------  KAMERA-THREAD --------------------------------
# ============================================================

def camera_readout(camera_index: int,
                   frame_queue: queue.Queue,
                   stop_event: Event,
                   undistorter: FisheyeUndistorter):

    # Picamera2 nur importieren, wenn vorhanden
    try:
        from picamera2 import Picamera2
        import libcamera
    except Exception as e:
        print(f"[Camera{camera_index}] Picamera2 nicht verfügbar: {e}")
        return

    picam = Picamera2(camera_index)

    # Auflösung aus Kalibrierung übernehmen (Fallback: 640x480)
    if undistorter and undistorter.valid:
        W, H = undistorter.WIDTH, undistorter.HEIGHT
    else:
        W, H = 640, 480

    controls = {
        "FrameDurationLimits": (EXPOSURE_TIME_US, EXPOSURE_TIME_US),
        "ExposureTime": EXPOSURE_TIME_US,
        "AnalogueGain": 1.0,
        "AeEnable": False,
        "AwbEnable": False,
        "Sharpness": 1.0,
        "Contrast": 1.0,
        "Saturation": 1.2,
        "NoiseReductionMode": libcamera.controls.draft.NoiseReductionModeEnum.Off
    }

    cfg = picam.create_video_configuration(main={"format": "RGB888", "size": (W, H)},
                                           controls=controls)
    # Falls deine Kalibrierung ohne Flip gemacht wurde: keine Transforms setzen!
    transform = libcamera.Transform(hflip=True, vflip=True)
    cfg["transform"] = transform

    picam.configure(cfg)
    picam.start()
    time.sleep(0.2)

    try:
        while not stop_event.is_set():
            frame_rgb = picam.capture_array()
            frame_bgr = frame_rgb
            if undistorter and undistorter.valid:
                frame_bgr = undistorter.undistort(frame_bgr)

            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
            frame_queue.put(frame_bgr)
    except Exception as e:
        print(f"[Camera{camera_index}] Fehler: {e}")
    finally:
        try:
            picam.stop()
        except Exception:
            pass

# ============================================================
# -----------  PROCESSING-THREAD -----------------------------
# ============================================================

def dual_camera_processor(queue_cam0: queue.Queue,
                          queue_cam1: queue.Queue,
                          stop_event: Event,
                          shared: SharedState):

    cv2.namedWindow('Cam0', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mask0', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Cam1', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mask1', cv2.WINDOW_NORMAL)

    tracker = BallTracker()

    # Framegröße (für optionale Normierung im Sender)
    # wird dynamisch beim ersten Bild gesetzt
    first_w_h_set = False

    while not stop_event.is_set():
        f0 = f1 = None
        try:
            f0 = queue_cam0.get(block=False)
        except queue.Empty:
            pass
        try:
            f1 = queue_cam1.get(block=False)
        except queue.Empty:
            pass

        best = None  # (size, cam_id, (x,y), ts, vis, mask)

        if f0 is not None:
            if not first_w_h_set:
                shared.set_frame_size(f0.shape[1], f0.shape[0])
                first_w_h_set = True
            vis0, m0, c0, s0 = tracker.detect_ball(f0, 0)
            cv2.imshow('Cam0', vis0)
            cv2.imshow('Mask0', m0)
            if c0 is not None:
                best = (s0, 0, c0, tracker.last_detection_time[0])

        if f1 is not None:
            if not first_w_h_set:
                shared.set_frame_size(f1.shape[1], f1.shape[0])
                first_w_h_set = True
            vis1, m1, c1, s1 = tracker.detect_ball(f1, 1)
            cv2.imshow('Cam1', vis1)
            cv2.imshow('Mask1', m1)
            if c1 is not None and (best is None or s1 > best[0]):
                best = (s1, 1, c1, tracker.last_detection_time[1])

        if best is not None:
            size_px, cam_id, (x, y), ts = best
            shared.update(cam_id, x, y, size_px, ts)
        else:
            shared.clear()

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            stop_event.set()
            print("[Proc] Stop-Signal – beende …")
            break

# ============================================================
# -----------  HAUPTKLASSE ----------------------------------
# ============================================================

class CameraSystem:
    def __init__(self, use_serial=True):
        self.stop_event = Event()
        self.threads = []
        self.shared = SharedState()

        self.queue_cam0 = queue.Queue(maxsize=3)
        self.queue_cam1 = queue.Queue(maxsize=3)

        # Undistorter laden (pro Kamera)
        self.und0 = FisheyeUndistorter(CALIB_CAM0, FISHEYE_BALANCE)
        self.und1 = FisheyeUndistorter(CALIB_CAM1, FISHEYE_BALANCE)

        self.use_serial = use_serial
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig, frame):
        self.stop()

    def start(self):
        # Kamera-Threads
        t_cam0 = threading.Thread(
            name="Camera0",
            target=camera_readout,
            args=(0, self.queue_cam0, self.stop_event, self.und0),
            daemon=True
        )
        t_cam1 = threading.Thread(
            name="Camera1",
            target=camera_readout,
            args=(1, self.queue_cam1, self.stop_event, self.und1),
            daemon=True
        )
        self.threads += [t_cam0, t_cam1]
        t_cam0.start(); t_cam1.start()

        # Processing
        t_proc = threading.Thread(
            name="BallTracker",
            target=dual_camera_processor,
            args=(self.queue_cam0, self.queue_cam1, self.stop_event, self.shared),
            daemon=True
        )
        self.threads.append(t_proc)
        t_proc.start()

        # Serieller Sender
        if self.use_serial:
            t_serial = SerialCOBSSender(self.shared, self.stop_event,
                                        port=SER_PORT, baud=SER_BAUD, rate_hz=SER_RATE_HZ)
            self.threads.append(t_serial)
            t_serial.start()

        try:
            while not self.stop_event.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        if not self.stop_event.is_set():
            self.stop_event.set()
            for t in self.threads:
                if t.is_alive():
                    t.join(timeout=2.0)
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

# ============================================================
# -----------  MAIN -----------------------------------------
# ============================================================

def main():
    import argparse
    p = argparse.ArgumentParser(description="Dual camera system (no network, with serial COBS send)")
    p.add_argument('--no-serial', action='store_true', help='Serielle Ausgabe deaktivieren')
    args = p.parse_args()

    system = CameraSystem(use_serial=not args.no_serial)
    try:
        system.start()
    except Exception as e:
        print(f"[Main] Fehler: {e}")
        system.stop()
        sys.exit(1)

if __name__ == "__main__":
    main()
