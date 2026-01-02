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

    # Lese Index aus Config. Wenn leer oder nicht vorhanden -> None (Auto-Detect)
    camera_idx = config['camera'].get('camera_index') 
    
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
    try:
        if camera_idx is not None:
            print(f"[CAM] Initializing Picamera2 (Index: {camera_idx})...")
            picam2 = Picamera2(camera_idx)
        else:
            print(f"[CAM] Initializing Picamera2 (Auto-Detect)...")
            picam2 = Picamera2() # Leere Klammer -> sucht erste verfügbare Kamera
    except Exception as e:
        print(f"[CAM] CRITICAL ERROR: Could not open camera. Error: {e}")
        stop_event.set()
        return
    
    # Configure Controls
    controls = {
        "FrameDurationLimits": (int(1000000 / fps), int(1000000 / fps)),
        "ExposureTime": config['camera']['exposure_us'],
        "AnalogueGain": 1.0,
        "AeEnable": False,
        "AwbEnable": False,
        # Optimize for CV
        "Sharpness": 1.0,
        "Contrast": 1.0, 
        "Saturation": 1.2,
        "NoiseReductionMode": libcamera.controls.draft.NoiseReductionModeEnum.Off


        


    }
    
    vid_config = picam2.create_video_configuration(
        main={"format": "RGB888", "size": (width, height)},
        controls=controls
    )
    
    # Optional: Transform (Flip/Rotation)
    vid_config["transform"] = libcamera.Transform(hflip=False, vflip=True)
    
    picam2.configure(vid_config)
    picam2.start()

    print(f"[CAM] Camera started at {width}x{height} @ {fps} FPS")

    try:
        while not stop_event.is_set():
            # Zero-copy capture directly into shared memory buffer
            temp_frame = picam2.capture_array()
            
            # Write to shared memory
            np.copyto(shared_frame, temp_frame)
            
            # Signal vision process
            frame_ready_event.set()
            
            # Optional: yield slightly to prevent CPU hogging
            # time.sleep(0.0001)

    except Exception as e:
        print(f"[CAM] Error during loop: {e}")
    finally:
        try:
            picam2.stop()
            picam2.close()

        except:
            pass
        try:
            shm.close()
            shm.unlink() # Cleanup shared memory
        except:
            pass
        print("[CAM] Stopped.")