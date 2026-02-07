import cv2
import numpy as np
import json
import time
import sys
import os
from picamera2 import Picamera2
import libcamera

# Add src/cam to path to import geometry
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../cam')))
from geometry import GeometryTransformer

CONFIG_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '../cam/config.json'))

# Globale Variablen
roi_points = []
drawing_roi = False
roi_selected_for_processing = False
current_frame_for_roi = None

def load_config(path):
    try:
        with open(path, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Fehler beim Laden von {path}: {e}")
        return None

def save_config(path, config):
    try:
        with open(path, 'w') as f:
            json.dump(config, f, indent=4)
        print(f"Konfiguration gespeichert in {path}")
    except Exception as e:
        print(f"Fehler beim Speichern: {e}")

def empty_callback(x):
    pass

def mouse_events_for_roi(event, x, y, flags, param):
    global roi_points, drawing_roi, roi_selected_for_processing

    if event == cv2.EVENT_LBUTTONDOWN:
        roi_points = [(x, y)]
        drawing_roi = True
        roi_selected_for_processing = False
    elif event == cv2.EVENT_LBUTTONUP:
        if drawing_roi:
            roi_points.append((x, y))
            drawing_roi = False
            if roi_points[0] != roi_points[1]:
                roi_selected_for_processing = True
            else:
                roi_points = []

def setup_trackbars(config):
    cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Controls', 500, 700)
    
    ball_conf = config['vision']['ball']
    
    # HSV
    cv2.createTrackbar('H_min', 'Controls', ball_conf.get('h_min', 0), 179, empty_callback)
    cv2.createTrackbar('H_max', 'Controls', ball_conf.get('h_max', 179), 179, empty_callback)
    cv2.createTrackbar('S_min', 'Controls', ball_conf.get('s_min', 0), 255, empty_callback)
    cv2.createTrackbar('S_max', 'Controls', ball_conf.get('s_max', 255), 255, empty_callback)
    cv2.createTrackbar('V_min', 'Controls', ball_conf.get('v_min', 0), 255, empty_callback)
    cv2.createTrackbar('V_max', 'Controls', ball_conf.get('v_max', 255), 255, empty_callback)
    
    # Area & Shape
    cv2.createTrackbar('Min_Area', 'Controls', ball_conf.get('min_area_px', 0), 10000, empty_callback)
    cv2.createTrackbar('Max_Area', 'Controls', ball_conf.get('max_area_px', 50000), 50000, empty_callback)
    
    # Circularity (Default 0.6 -> 60)
    circ_val = int(ball_conf.get('min_circularity', 0.6) * 100)
    cv2.createTrackbar('Min_Circularity', 'Controls', circ_val, 100, empty_callback)

def get_params_from_trackbars(config):
    ball_conf = config['vision']['ball']
    
    ball_conf['h_min'] = cv2.getTrackbarPos('H_min', 'Controls')
    ball_conf['h_max'] = cv2.getTrackbarPos('H_max', 'Controls')
    ball_conf['s_min'] = cv2.getTrackbarPos('S_min', 'Controls')
    ball_conf['s_max'] = cv2.getTrackbarPos('S_max', 'Controls')
    ball_conf['v_min'] = cv2.getTrackbarPos('V_min', 'Controls')
    ball_conf['v_max'] = cv2.getTrackbarPos('V_max', 'Controls')
    
    ball_conf['min_area_px'] = cv2.getTrackbarPos('Min_Area', 'Controls')
    ball_conf['max_area_px'] = cv2.getTrackbarPos('Max_Area', 'Controls')
    
    ball_conf['min_circularity'] = cv2.getTrackbarPos('Min_Circularity', 'Controls') / 100.0
    
    return config

def main_calibration():
    global roi_points, roi_selected_for_processing, current_frame_for_roi
    
    config = load_config(CONFIG_PATH)
    if not config:
        return

    # Camera Setup
    cam_conf = config['camera']
    width = cam_conf['width']
    height = cam_conf['height']
    fps = cam_conf['fps']
    exposure = cam_conf['exposure_us']

    print(f"Starte Kamera: {width}x{height} @ {fps}fps, Exposure: {exposure}us")
    
    picam2 = Picamera2(cam_conf.get('camera_index', 0))
    vid_config = picam2.create_video_configuration(
        main={"format": "RGB888", "size": (width, height)},
        controls={
            "ExposureTime": exposure,
            "AnalogueGain": 1.0,
            "AeEnable": False,
            "AwbEnable": False,
            "Saturation": 1.2
        }
    )
    # WICHTIG: Transform muss zu vision_process.py passen (dort nicht explizit gesetzt, aber camera_process.py hat vflip=True)
    # camera_process.py Line 72: vid_config["transform"] = libcamera.Transform(hflip=False, vflip=True)
    vid_config["transform"] = libcamera.Transform(hflip=False, vflip=True)

    picam2.configure(vid_config)
    picam2.start()
    
    # Init Geometry / Mask
    geo = GeometryTransformer(config)
    static_mask = geo.get_donut_mask((height, width))
    
    setup_trackbars(config)
    
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('Original', mouse_events_for_roi)
    cv2.namedWindow('Mask', cv2.WINDOW_NORMAL) # Debug View

    kernel = np.ones((3, 3), np.uint8)
    
    print("--- Controls ---")
    print("s: Save Config")
    print("r: Reset ROI")
    print("q: Quit")
    
    try:
        while True:
            frame = picam2.capture_array()
            current_frame_for_roi = frame
            display_frame = frame.copy()
            
            # Auto-ROI Logic
            if roi_selected_for_processing:
                x1, y1 = roi_points[0]
                x2, y2 = roi_points[1]
                rx_min, rx_max = min(x1, x2), max(x1, x2)
                ry_min, ry_max = min(y1, y2), max(y1, y2)
                
                roi = frame[ry_min:ry_max, rx_min:rx_max]
                if roi.size > 0:
                    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    
                    # Simple percentile based auto-tuning
                    h = hsv_roi[:,:,0].flatten()
                    s = hsv_roi[:,:,1].flatten()
                    v = hsv_roi[:,:,2].flatten()
                    
                    cv2.setTrackbarPos('H_min', 'Controls', int(np.percentile(h, 5)))
                    cv2.setTrackbarPos('H_max', 'Controls', int(np.percentile(h, 95)))
                    cv2.setTrackbarPos('S_min', 'Controls', int(np.percentile(s, 5)))
                    cv2.setTrackbarPos('S_max', 'Controls', int(np.percentile(s, 95)))
                    cv2.setTrackbarPos('V_min', 'Controls', int(np.percentile(v, 5)))
                    cv2.setTrackbarPos('V_max', 'Controls', int(np.percentile(v, 95)))
                    
                    print("Auto-Tuned HSV based on ROI")
                
                roi_selected_for_processing = False
                roi_points = []
            
            # Update Config from Trackbars
            config = get_params_from_trackbars(config)
            c_vis = config['vision']['ball']
            
            # Processing (Match vision_process.py)
            lower = np.array([c_vis['h_min'], c_vis['s_min'], c_vis['v_min']])
            upper = np.array([c_vis['h_max'], c_vis['s_max'], c_vis['v_max']])
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.bitwise_and(mask, mask, mask=static_mask) # Donut Mask
            
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw Donut
            cv2.circle(display_frame, (geo.cx, geo.cy), geo.r_inner, (255, 0, 0), 1)
            cv2.circle(display_frame, (geo.cx, geo.cy), geo.r_outer, (255, 0, 0), 1)
            
            # Find Best Ball
            best_cnt = None
            max_area = 0
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < c_vis['min_area_px'] or area > c_vis['max_area_px']:
                    continue
                
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0: continue
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                
                if circularity < c_vis.get('min_circularity', 0.6):
                    # Draw rejected
                    cv2.drawContours(display_frame, [cnt], -1, (0, 0, 255), 1)
                    continue
                
                # Draw candidate
                cv2.drawContours(display_frame, [cnt], -1, (0, 255, 255), 1)
                
                if area > max_area:
                    max_area = area
                    best_cnt = cnt
            
            if best_cnt is not None:
                cv2.drawContours(display_frame, [best_cnt], -1, (0, 255, 0), 2)
                M = cv2.moments(best_cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(display_frame, (cx, cy), 5, (0, 0, 255), -1)
                    
                    dist, ang = geo.pixel_to_polar(cx, cy)
                    cv2.putText(display_frame, f"D:{dist:.1f} A:{np.degrees(ang):.1f}", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            if drawing_roi and len(roi_points) > 0:
                pt = roi_points[0]
                # Mouse pos is not readily available in loop without callback passing it, 
                # but we can draw the start point
                cv2.circle(display_frame, pt, 3, (0, 255, 0), -1)

            cv2.imshow('Original', display_frame)
            cv2.imshow('Mask', mask)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                save_config(CONFIG_PATH, config)
            elif key == ord('r'):
                roi_points = []
    
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main_calibration() 