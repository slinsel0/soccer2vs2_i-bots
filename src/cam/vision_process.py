import multiprocessing
from multiprocessing import shared_memory
import numpy as np
import cv2
import time
import traceback
from geometry import GeometryTransformer
from collections import deque

def run_vision(config, stop_event, frame_ready_event, result_queue):
    """
    Process that reads from SharedMemory, detects the ball.
    Includes DEBUG VIEW logic.
    """
    try:
        width = config['camera']['width']
        height = config['camera']['height']
        shm_name = config['camera']['shm_name']
        
        # Debug Modus auslesen
        debug_mode = config['vision'].get('debug_view', False)
        
        # --- FIX: Warten auf Shared Memory (Retry Loop) ---
        shm = None
        existing_shm = False
        print("[VIS] Warte auf Shared Memory Verbindung...")


  
        
        while not stop_event.is_set():
            try:
                shm = shared_memory.SharedMemory(name=shm_name)
                existing_shm = True
                print("[VIS] Shared Memory verbunden!")
                break
            except FileNotFoundError:
                # Kamera ist noch nicht bereit, kurz warten und nochmal versuchen
                time.sleep(0.5)
                continue
            except Exception as e:
                print(f"[VIS] Unerwarteter Fehler beim Verbinden: {e}")
                time.sleep(1.0)

        if stop_event.is_set():
            return

        # Frame Buffer als Numpy Array wrappen
        frame_buffer = np.ndarray((height, width, 3), dtype=np.uint8, buffer=shm.buf)
        
        geo = GeometryTransformer(config)
        
        # Statische Maske (Donut)
        static_mask = geo.get_donut_mask((height, width))
        
        c_vis = config['vision']['ball']
        lower_color = np.array([c_vis['h_min'], c_vis['s_min'], c_vis['v_min']])
        upper_color = np.array([c_vis['h_max'], c_vis['s_max'], c_vis['v_max']])
        
        kernel = np.ones((3, 3), np.uint8)

        # GUI Initialisierung (nur wenn debug_view an ist)
        if debug_mode:
            try:
                print("[VIS] DEBUG VIEW ENABLED (Drücke 'q' im Fenster zum Beenden)")
                cv2.namedWindow("Debug View", cv2.WINDOW_NORMAL)
            except cv2.error:
                print("[VIS] WARNUNG: Konnte kein Fenster öffnen (kein Display?). Debug-Modus deaktiviert.")
                debug_mode = False

        print("[VIS] Vision Loop gestartet.")


        ball_history = deque(maxlen=5)
        PREDICTION_TIME_SEC = 0.15
        while not stop_event.is_set():
            # Warten auf Signal von der Kamera (Timeout damit wir stop_event prüfen können)
            if not frame_ready_event.wait(timeout=1.0):
                continue
            
            frame_ready_event.clear()
            
            # Direktzugriff auf den Puffer (Zero-Copy)
            frame = frame_buffer 
            
            # --- Verarbeitung ---
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_color, upper_color)
            mask = cv2.bitwise_and(mask, mask, mask=static_mask)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            best_ball = None
            max_score = 0
            
            # Ergebnis Variablen
            res_dist = 0
            res_angle = 0
            found = False

            M = None # Initialisieren für Visualisierung später

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < c_vis['min_area_px'] or area > c_vis['max_area_px']:
                    continue
                
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0: continue
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                
                if circularity < c_vis.get('min_circularity', 0.6): 
                    continue

                if area > max_score:
                    max_score = area
                    best_ball = cnt

            if best_ball is not None:
                M = cv2.moments(best_ball)
                if M['m00'] != 0:
                    ccx = M['m10'] / M['m00']
                    ccy = M['m01'] / M['m00']
                    res_dist, res_angle = geo.pixel_to_polar(ccx, ccy)
                    current_time = time.perf_counter()
                    #print(f"[VIS] Ball gefunden: D={res_dist:.2f}px, A={res_angle:.2f}°")
                    #print(cx, cy)
                    ball_history.append((current_time, ccx, ccy))
                    vx, vy = 0.0, 0.0
                    if len(ball_history) >= 2:
                        # Wir vergleichen aktuellen Punkt mit dem ältesten im Puffer
                        dt = current_time - ball_history[0][0]
                        if dt > 0:
                            vx = (ccx - ball_history[0][1]) / dt
                            vy = (ccy - ball_history[0][2]) / dt
            
                    pred_x = ccx + (vx * PREDICTION_TIME_SEC)
                    pred_y = ccy + (vy * PREDICTION_TIME_SEC)
                    cx, cy = pred_x, pred_y

                    pred_dist_px = np.sqrt(pred_x**2 + pred_y**2)
                    pred_angle_rad = np.arctan2(pred_x, pred_y)

                    found = True

                    payload = {
                        'found': True, 'cx': cx, 'cy': cy,
                        't_us': time.perf_counter_ns() // 1000
                    }
                    if not result_queue.full():
                        result_queue.put(payload)
            else:
                ball_history.clear()
                if not result_queue.full():
                    result_queue.put({'found': False})

            # --- Visualisierung (Nur wenn aktiviert) ---
            if debug_mode:
                try:
                    vis_frame = frame
                    
                    
                    # Zeichne Donut-Grenzen
                    cv2.circle(vis_frame, (geo.cx, geo.cy), geo.r_inner, (255, 0, 0), 1)
                    cv2.circle(vis_frame, (geo.cx, geo.cy), geo.r_outer, (255, 0, 0), 1)

                    if found and best_ball is not None and M is not None:

                        draw_pred_x = int(geo.cx + pred_x)
                        draw_pred_y = int(geo.cy - pred_y)
                        ball_center = (int(cx), int(cy))
                        
                        # Rote Linie: Wohin rollt der Ball?
                        cv2.line(vis_frame, ball_center, (draw_pred_x, draw_pred_y), (0, 0, 255), 2)
                        # Roter Kreis: Vorhergesagte Position
                        cv2.circle(vis_frame, (draw_pred_x, draw_pred_y), 5, (0, 0, 255))
                        
                        # Zeichne Ball Kontur
                        cv2.drawContours(vis_frame, [best_ball], -1, (0, 255, 0), 2)
                        # Zeichne Linie zum Ball
                        ball_center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
                        cv2.line(vis_frame, (geo.cx, geo.cy), ball_center, (0, 255, 255), 1)
                        # Text Info
                        info_txt = f"Dist: {res_dist:.1f}px | Ang: {np.degrees(res_angle):.1f}deg"
                        cv2.putText(vis_frame, info_txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Maske klein einblenden
                    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                    mask_small = cv2.resize(mask_bgr, (160, 120))
                    vis_frame[0:120, 0:160] = mask_small
                    cv2.imshow("Debug View", vis_frame)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("[VIS] Beende durch Benutzer (q)...")
                        stop_event.set()
                except Exception as e:
                    print(f"[VIS] Fehler in GUI Loop: {e}")
                    debug_mode = False # Bei Fehler GUI abschalten

    except Exception as e:
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print(f"[VIS] KRITISCHER ABSTURZ: {e}")
        print(traceback.format_exc())
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    finally:
        if 'existing_shm' in locals() and existing_shm and shm:
            try:
                shm.close()
            except:
                pass
        if debug_mode:
            try:
                cv2.destroyAllWindows()
            except:
                pass
        print("[VIS] Prozess beendet.")
