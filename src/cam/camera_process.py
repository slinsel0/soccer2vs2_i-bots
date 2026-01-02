import time
import multiprocessing
from multiprocessing import shared_memory
import numpy as np
import cv2

def run_camera(config, stop_event, frame_ready_event):
    """
    Process to capture frames from Picamera2 and write to SharedMemory.
    This decouples capture rate from processing rate.
    """
    # Import here to avoid pickling issues with multiprocessing
    from picamera2 import Picamera2
    import libcamera

    width = config['camera']['width']
    height = config['camera']['height']
    fps = config['camera']['fps']
    shm_name = config['camera']['shm_name']

    # Initialize Shared Memory
    frame_size = width * height * 3 # RGB888
    try:
        shm = shared_memory.SharedMemory(name=shm_name, create=True, size=frame_size)
    except FileExistsError:
        shm = shared_memory.SharedMemory(name=shm_name)
    
    # Wrap buffer as numpy array
    shared_frame = np.ndarray((height, width, 3), dtype=np.uint8, buffer=shm.buf)

    # Initialize Camera
    picam2 = Picamera2(0) # Index 0 or 1
    
    # Configure Controls
    controls = {
        "FrameDurationLimits": (int(1000000 / fps), int(1000000 / fps)),
        "ExposureTime": config['camera']['exposure_us'],
        "AnalogueGain": 1.0,
        "AeEnable": False,
        "AwbEnable": False,
        # Optimize for CV
        "Sharpness": 1.0,
        "Contrast": 1.1, 
        "Saturation": 1.5 
    }
    
    vid_config = picam2.create_video_configuration(
        main={"format": "RGB888", "size": (width, height)},
        controls=controls
    )
    # Add Flip if necessary (mirror mounted upside down?)
    # vid_config["transform"] = libcamera.Transform(hflip=True, vflip=True)
    
    picam2.configure(vid_config)
    picam2.start()

    print(f"[CAM] Camera started at {width}x{height} @ {fps} FPS")

    try:
        while not stop_event.is_set():
            # Zero-copy capture directly into shared memory buffer
            # Note: Picamera2 capture_array usually allocates new memory. 
            # We use capture_request to get the buffer or copy efficiently.
            # For simplicity in this wrapper, we capture and copy, 
            # but picam2.capture_file/array can be optimized further with request objects.
            
            temp_frame = picam2.capture_array()
            
            # Write to shared memory
            np.copyto(shared_frame, temp_frame)
            
            # Signal vision process
            frame_ready_event.set()
            
            # Optional: yield slightly to prevent CPU hogging if FPS limit handles it
            # time.sleep(0.001) 

    except Exception as e:
        print(f"[CAM] Error: {e}")
    finally:
        picam2.stop()
        shm.close()
        shm.unlink() # Cleanup shared memory
        print("[CAM] Stopped.")