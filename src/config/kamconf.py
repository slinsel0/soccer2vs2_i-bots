import cv2
import numpy as np
import json
import time
from picamera2 import Picamera2
import libcamera # Für libcamera.Transform

CONFIG_PATH = 'ball_tracker_config.json'

# Kameraeinstellungen
WIDTH = 1280
HEIGHT = 800
EXPOSURE_TIME_US = 50000
CAMERA_INDEX = 1

# Globale Variablen für ROI-Auswahl
roi_points = []
drawing_roi = False
roi_selected_for_processing = False
current_frame_for_roi = None # Wird in der Schleife aktualisiert

def load_config(path):
    default_params = {
        "h_lower": 10, "s_lower": 150, "v_lower": 150,
        "h_upper": 30, "s_upper": 255, "v_upper": 255,
        "erode_iter": 1, "dilate_iter": 1,
        "kernel_size": 3,
        "min_blob_area": 100, "max_blob_area": 10000,
        "min_circularity": 0.6, "min_convexity": 0.8, "min_inertia_ratio": 0.4,
        "minDistBetweenBlobs": 10, "minThreshold": 10,
        "maxThreshold": 200, "thresholdStep": 10
    }
    try:
        with open(path, 'r') as f:
            loaded_params = json.load(f)
            for key, value in default_params.items():
                if key not in loaded_params:
                    loaded_params[key] = value
            loaded_params['min_circularity'] = max(0.0, min(1.0, loaded_params.get('min_circularity', 0.6)))
            loaded_params['min_convexity'] = max(0.0, min(1.0, loaded_params.get('min_convexity', 0.8)))
            loaded_params['min_inertia_ratio'] = max(0.0, min(1.0, loaded_params.get('min_inertia_ratio', 0.4)))
            return loaded_params
    except (FileNotFoundError, json.JSONDecodeError):
        print(f"Konfigurationsdatei {path} nicht gefunden oder fehlerhaft. Verwende Standardwerte.")
        return default_params

def save_config(path, params):
    params_to_save = params.copy()
    params_to_save['min_circularity'] = float(params_to_save.get('min_circularity_track', params_to_save.get('min_circularity',0.6)*100)) / 100.0
    params_to_save['min_convexity'] = float(params_to_save.get('min_convexity_track', params_to_save.get('min_convexity',0.8)*100)) / 100.0
    params_to_save['min_inertia_ratio'] = float(params_to_save.get('min_inertia_ratio_track', params_to_save.get('min_inertia_ratio',0.4)*100)) / 100.0
    params_to_save.pop('min_circularity_track', None)
    params_to_save.pop('min_convexity_track', None)
    params_to_save.pop('min_inertia_ratio_track', None)
    with open(path, 'w') as f:
        json.dump(params_to_save, f, indent=4)
    print(f"Konfiguration gespeichert in {path}")

def empty_callback(x):
    pass

def mouse_events_for_roi(event, x, y, flags, param):
    global roi_points, drawing_roi, roi_selected_for_processing, current_frame_for_roi

    if event == cv2.EVENT_LBUTTONDOWN:
        roi_points = [(x, y)]
        drawing_roi = True
        roi_selected_for_processing = False # Neue Auswahl beginnt
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing_roi:
            # Temporäres Rechteck im Originalfenster zeichnen (optional, aber gut für UX)
            # Dies wird in der Hauptschleife gehandhabt, um das Bild nicht im Callback zu ändern
            pass
    elif event == cv2.EVENT_LBUTTONUP:
        if drawing_roi: # Nur wenn wir tatsächlich gezeichnet haben
            roi_points.append((x, y))
            drawing_roi = False
            if roi_points[0] != roi_points[1]: # Stelle sicher, dass es eine gültige Region ist
                roi_selected_for_processing = True
            else: # Einzelklick, keine Region
                roi_points = []


def setup_trackbars(initial_params):
    cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Controls', 500, 700)
    # HSV
    cv2.createTrackbar('H_lower', 'Controls', initial_params['h_lower'], 179, empty_callback)
    cv2.createTrackbar('S_lower', 'Controls', initial_params['s_lower'], 255, empty_callback)
    cv2.createTrackbar('V_lower', 'Controls', initial_params['v_lower'], 255, empty_callback)
    cv2.createTrackbar('H_upper', 'Controls', initial_params['h_upper'], 179, empty_callback)
    cv2.createTrackbar('S_upper', 'Controls', initial_params['s_upper'], 255, empty_callback)
    cv2.createTrackbar('V_upper', 'Controls', initial_params['v_upper'], 255, empty_callback)
    # Morphology
    cv2.createTrackbar('Kernel_Size', 'Controls', initial_params['kernel_size'], 31, empty_callback)
    cv2.createTrackbar('Erode_Iter', 'Controls', initial_params['erode_iter'], 10, empty_callback)
    cv2.createTrackbar('Dilate_Iter', 'Controls', initial_params['dilate_iter'], 10, empty_callback)
    # Blob
    cv2.createTrackbar('Min_Area', 'Controls', initial_params['min_blob_area'], 50000, empty_callback)
    cv2.createTrackbar('Max_Area', 'Controls', initial_params['max_blob_area'], 50000, empty_callback)
    cv2.createTrackbar('Min_Circularity', 'Controls', int(initial_params['min_circularity'] * 100), 100, empty_callback)
    cv2.createTrackbar('Min_Convexity', 'Controls', int(initial_params['min_convexity'] * 100), 100, empty_callback)
    cv2.createTrackbar('Min_Inertia', 'Controls', int(initial_params['min_inertia_ratio'] * 100), 100, empty_callback)
    cv2.createTrackbar('Min_Dist_Blobs', 'Controls', initial_params['minDistBetweenBlobs'], 200, empty_callback)
    cv2.createTrackbar('Min_Threshold', 'Controls', initial_params['minThreshold'], 255, empty_callback)
    cv2.createTrackbar('Max_Threshold', 'Controls', initial_params['maxThreshold'], 255, empty_callback)
    cv2.createTrackbar('Threshold_Step', 'Controls', initial_params['thresholdStep'], 50, empty_callback)

def get_params_from_trackbars():
    params = {}
    params['h_lower'] = cv2.getTrackbarPos('H_lower', 'Controls')
    params['s_lower'] = cv2.getTrackbarPos('S_lower', 'Controls')
    params['v_lower'] = cv2.getTrackbarPos('V_lower', 'Controls')
    params['h_upper'] = cv2.getTrackbarPos('H_upper', 'Controls')
    params['s_upper'] = cv2.getTrackbarPos('S_upper', 'Controls')
    params['v_upper'] = cv2.getTrackbarPos('V_upper', 'Controls')
    ks = cv2.getTrackbarPos('Kernel_Size', 'Controls')
    params['kernel_size'] = max(1, ks if ks % 2 != 0 else ks + 1)
    cv2.setTrackbarPos('Kernel_Size', 'Controls', params['kernel_size'])
    params['erode_iter'] = cv2.getTrackbarPos('Erode_Iter', 'Controls')
    params['dilate_iter'] = cv2.getTrackbarPos('Dilate_Iter', 'Controls')
    params['min_blob_area'] = cv2.getTrackbarPos('Min_Area', 'Controls')
    params['max_blob_area'] = cv2.getTrackbarPos('Max_Area', 'Controls')
    params['min_circularity_track'] = cv2.getTrackbarPos('Min_Circularity', 'Controls')
    params['min_circularity'] = params['min_circularity_track'] / 100.0
    params['min_convexity_track'] = cv2.getTrackbarPos('Min_Convexity', 'Controls')
    params['min_convexity'] = params['min_convexity_track'] / 100.0
    params['min_inertia_ratio_track'] = cv2.getTrackbarPos('Min_Inertia', 'Controls')
    params['min_inertia_ratio'] = params['min_inertia_ratio_track'] / 100.0
    params['minDistBetweenBlobs'] = cv2.getTrackbarPos('Min_Dist_Blobs', 'Controls')
    params['minThreshold'] = cv2.getTrackbarPos('Min_Threshold', 'Controls')
    params['maxThreshold'] = cv2.getTrackbarPos('Max_Threshold', 'Controls')
    ts = cv2.getTrackbarPos('Threshold_Step', 'Controls')
    params['thresholdStep'] = max(1, ts) # Muss >= 1 sein
    cv2.setTrackbarPos('Threshold_Step', 'Controls', params['thresholdStep'])
    return params

def main_calibration_auto_hsv():
    global roi_points, drawing_roi, roi_selected_for_processing, current_frame_for_roi
    global current_params # Wird benötigt, um von Trackbars und ROI-Logik aktualisiert zu werden

    picam2 = Picamera2(CAMERA_INDEX)
    config = picam2.create_video_configuration(
        main={"format": "RGB888", "size": (WIDTH, HEIGHT)},
        controls={
            "FrameDurationLimits": (EXPOSURE_TIME_US, EXPOSURE_TIME_US),
            "ExposureTime": EXPOSURE_TIME_US, "AnalogueGain": 1.0, "AeEnable": False,
            "AwbEnable": False, "NoiseReductionMode": libcamera.controls.draft.NoiseReductionModeEnum.Off,
            "Sharpness": 1.0, "Contrast": 1.0, "Saturation": 1.2 # Sättigung leicht erhöht
        }
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    initial_config = load_config(CONFIG_PATH)
    current_params = initial_config.copy() # Wichtig für save_config
    setup_trackbars(initial_config)

    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Processed Mask for Blob', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('Original', mouse_events_for_roi)

    print("--- Bedienung ---")
    print("1. Ziehen Sie mit der Maus ein Rechteck um das Zielobjekt im 'Original'-Fenster.")
    print("2. Die HSV-Regler werden automatisch angepasst.")
    print("3. Führen Sie Feinjustierungen mit allen Reglern durch.")
    print("4. Drücken Sie 's', um die Konfiguration zu speichern.")
    print("5. Drücken Sie 'r', um die ROI-Auswahl zurückzusetzen und neu zu wählen.")
    print("6. Drücken Sie 'q', um das Programm zu beenden.")
    print("-----------------")


    blob_params_obj = cv2.SimpleBlobDetector_Params()

    while True:
        frame_rgb = picam2.capture_array()
        current_frame_for_roi = frame_rgb
        display_frame = current_frame_for_roi.copy() # Kopie für Zeichnungen

        if roi_selected_for_processing:
            x1, y1 = roi_points[0]
            x2, y2 = roi_points[1]
            roi_x_start, roi_x_end = min(x1, x2), max(x1, x2)
            roi_y_start, roi_y_end = min(y1, y2), max(y1, y2)

            if roi_x_end > roi_x_start and roi_y_end > roi_y_start:
                selected_roi_bgr = current_frame_for_roi[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
                if selected_roi_bgr.size > 0:
                    selected_roi_hsv = cv2.cvtColor(selected_roi_bgr, cv2.COLOR_BGR2HSV)
                    
                    h_vals = selected_roi_hsv[:, :, 0].flatten()
                    s_vals = selected_roi_hsv[:, :, 1].flatten()
                    v_vals = selected_roi_hsv[:, :, 2].flatten()

                    # Verwende Percentile für Robustheit gegen Ausreißer
                    margin_h, margin_sv = 5, 20 # Kleinere Margin für H, größere für S/V

                    h_min = max(0, int(np.percentile(h_vals, 5)) - margin_h)
                    h_max = min(179, int(np.percentile(h_vals, 95)) + margin_h)
                    s_min = max(0, int(np.percentile(s_vals, 5)) - margin_sv)
                    s_max = min(255, int(np.percentile(s_vals, 95)) + margin_sv)
                    v_min = max(0, int(np.percentile(v_vals, 5)) - margin_sv)
                    v_max = min(255, int(np.percentile(v_vals, 95)) + margin_sv)

                    # Sicherstellen, dass lower <= upper
                    if h_min > h_max: h_min, h_max = h_max, h_min if h_max > 0 else (0,10) # Fallback
                    if s_min > s_max: s_min, s_max = s_max, s_min if s_max > 0 else (0,10)
                    if v_min > v_max: v_min, v_max = v_max, v_min if v_max > 0 else (0,10)


                    cv2.setTrackbarPos('H_lower', 'Controls', h_min)
                    cv2.setTrackbarPos('S_lower', 'Controls', s_min)
                    cv2.setTrackbarPos('V_lower', 'Controls', v_min)
                    cv2.setTrackbarPos('H_upper', 'Controls', h_max)
                    cv2.setTrackbarPos('S_upper', 'Controls', s_max)
                    cv2.setTrackbarPos('V_upper', 'Controls', v_max)
                    print(f"Auto HSV: H({h_min}-{h_max}), S({s_min}-{s_max}), V({v_min}-{v_max})")
            roi_selected_for_processing = False # Flag zurücksetzen

        # Parameter von Trackbars holen (könnten gerade durch ROI-Logik aktualisiert worden sein)
        current_params = get_params_from_trackbars()

        # Verarbeitung basierend auf current_params
        orange_lower = np.array([current_params['h_lower'], current_params['s_lower'], current_params['v_lower']])
        orange_upper = np.array([current_params['h_upper'], current_params['s_upper'], current_params['v_upper']])
        k_size = current_params['kernel_size']
        erosion_kernel = np.ones((k_size, k_size), np.uint8)
        dilation_kernel = np.ones((k_size, k_size), np.uint8)

        hsv = cv2.cvtColor(current_frame_for_roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, orange_lower, orange_upper)
        
        eroded = mask
        if current_params['erode_iter'] > 0:
            eroded = cv2.erode(mask, erosion_kernel, iterations=current_params['erode_iter'])
        dilated = eroded
        if current_params['dilate_iter'] > 0:
            dilated = cv2.dilate(eroded, dilation_kernel, iterations=current_params['dilate_iter'])
        closed_morph = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, dilation_kernel, iterations=1)
        mask_for_blob = cv2.bitwise_not(closed_morph)

        blob_params_obj.minThreshold = current_params['minThreshold']
        blob_params_obj.maxThreshold = current_params['maxThreshold']
        blob_params_obj.thresholdStep = current_params['thresholdStep']
        blob_params_obj.filterByArea = True
        blob_params_obj.minArea = current_params['min_blob_area']
        blob_params_obj.maxArea = current_params['max_blob_area']
        blob_params_obj.filterByCircularity = True
        blob_params_obj.minCircularity = current_params['min_circularity']
        blob_params_obj.filterByConvexity = True
        blob_params_obj.minConvexity = current_params['min_convexity']
        blob_params_obj.filterByInertia = True
        blob_params_obj.minInertiaRatio = current_params['min_inertia_ratio']
        blob_params_obj.minDistBetweenBlobs = current_params['minDistBetweenBlobs']
        detector = cv2.SimpleBlobDetector_create(blob_params_obj)
        keypoints = detector.detect(mask_for_blob)

        frame_with_keypoints = display_frame # display_frame ist Kopie von current_frame_for_roi
        if keypoints:
            frame_with_keypoints = cv2.drawKeypoints(display_frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Zeichne das ROI-Rechteck, wenn es ausgewählt wird oder wurde
        if drawing_roi and len(roi_points) == 1: # Während des Ziehens
             # Holen der aktuellen Mausposition ist hier schwierig ohne sie zu übergeben
             # Einfacher: Rechteck erst nach Loslassen zeichnen oder Callback muss auch frame malen
             pass # Temporäres Zeichnen kann komplex sein, wir zeichnen das finale Rechteck
        if len(roi_points) == 2: # ROI ist ausgewählt
            cv2.rectangle(frame_with_keypoints, roi_points[0], roi_points[1], (0, 255, 0), 2)


        cv2.imshow('Original', frame_with_keypoints)
        cv2.imshow('Mask', mask)
        cv2.imshow('Processed Mask for Blob', mask_for_blob)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            save_config(CONFIG_PATH, current_params)
        elif key == ord('r'): # ROI zurücksetzen
            roi_points = []
            drawing_roi = False
            roi_selected_for_processing = False
            print("ROI-Auswahl zurückgesetzt. Bitte neu auswählen.")


    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main_calibration_auto_hsv()