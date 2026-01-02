import multiprocessing
from multiprocessing import shared_memory
import numpy as np
import cv2
import time
from geometry import GeometryTransformer

def run_vision(config, stop_event, frame_ready_event, result_queue):
    """
    Process that reads from SharedMemory, detects the ball,
    calculates polar coordinates, and pushes results to Queue.
    """
    width = config['camera']['width']
    height = config['camera']['height']
    shm_name = config['camera']['shm_name']
    
    # Connect to Shared Memory
    try:
        shm = shared_memory.SharedMemory(name=shm_name)
        existing_shm = True
    except FileNotFoundError:
        print("[VIS] Wait for Cam...")
        time.sleep(1)
        # Retry logic usually needed here in prod
        return

    frame_buffer = np.ndarray((height, width, 3), dtype=np.uint8, buffer=shm.buf)
    
    # Geometry Helper
    geo = GeometryTransformer(config)
    
    # Precompute Masks (The 'Donut')
    # Use bitwise logic to exclude center and outer edges
    static_mask = geo.get_donut_mask((height, width))
    
    # HSV Parameters
    c_vis = config['vision']['ball']
    lower_color = np.array([c_vis['h_min'], c_vis['s_min'], c_vis['v_min']])
    upper_color = np.array([c_vis['h_max'], c_vis['s_max'], c_vis['v_max']])
    
    kernel = np.ones((3, 3), np.uint8)

    print("[VIS] Vision Processing Started")

    last_time = time.time()
    
    while not stop_event.is_set():
        if not frame_ready_event.wait(timeout=1.0):
            continue
        
        frame_ready_event.clear()
        
        # 1. Get Frame (Copy to avoid race condition while reading? 
        # Usually fine if processing is faster than FPS, or we just accept tearing)
        # For absolute safety: frame = frame_buffer.copy() 
        # For speed: Use directly (risk of tearing)
        frame = frame_buffer 
        
        # 2. Pre-Processing
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # 3. Masking
        # Color Threshold
        mask = cv2.inRange(hsv, lower_color, upper_color)
        
        # Apply Geometry Mask (Donut)
        mask = cv2.bitwise_and(mask, mask, mask=static_mask)
        
        # Morphological Ops
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # 4. Blob/Contour Detection
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_ball = None
        max_score = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Simple Area Filter
            if area < c_vis['min_area_px'] or area > c_vis['max_area_px']:
                continue
            
            # Calculate Circularity
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            
            if circularity < 0.6: # Filter non-round objects
                continue

            # --- Geometry Aware Scoring ---
            # Objects closer to the mirror edge (closer to robot usually) might look different
            # M = cv2.moments(cnt)
            # cx = int(M['m10'] / M['m00'])
            # cy = int(M['m01'] / M['m00'])
            # r_px = np.sqrt((cx - geo.cx)**2 + (cy - geo.cy)**2)
            # Here you could check if 'area' matches expected area for 'r_px'
            
            if area > max_score:
                max_score = area
                best_ball = cnt

        # 5. Result Extraction
        if best_ball is not None:
            # Get precise center
            M = cv2.moments(best_ball)
            if M['m00'] != 0:
                cx = M['m10'] / M['m00']
                cy = M['m01'] / M['m00']
                
                # Transform to Polar (Distance in mm, Angle in Rad)
                dist, angle = geo.pixel_to_polar(cx, cy)
                
                # Push to Queue
                payload = {
                    'found': True,
                    'dist': dist,
                    'angle': angle,
                    't_us': time.perf_counter_ns() // 1000
                }
                
                if not result_queue.full():
                    result_queue.put(payload)
        else:
             if not result_queue.full():
                result_queue.put({'found': False})

        # FPS Calculation (Optional logging)
        # now = time.time()
        # fps = 1.0 / (now - last_time)
        # last_time = now

    if existing_shm:
        shm.close()