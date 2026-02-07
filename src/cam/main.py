#!/usr/bin/env python3
import multiprocessing
import json
import time
import sys
import os

# Import modules
from camera_process import run_camera
from vision_process import run_vision
from serial_process import run_serial

CONFIG_PATH = "config.json"

def load_config():
    with open(CONFIG_PATH, 'r') as f:
        return json.load(f)

def main():
    print("--- RoboCup Junior Optimized Vision System ---")
    print(f"--- CPU Cores: {multiprocessing.cpu_count()} ---")

    # Load Config
    try:
        config = load_config()
    except Exception as e:
        print(f"Error loading config: {e}")
        return

    # Synchronization Primitives
    stop_event = multiprocessing.Event()
    frame_ready_event = multiprocessing.Event()
    
    # Communication Queues
    # Results from Vision -> Serial (small data, Queue is fine)
    result_queue = multiprocessing.Queue(maxsize=10)

    # Processes
    # 1. Camera (Writes to Shared Memory)
    p_cam = multiprocessing.Process(name="Camera", target=run_camera, 
                                    args=(config, stop_event, frame_ready_event))
    
    # 2. Vision (Reads Shared Memory, Writes to Queue)
    p_vis = multiprocessing.Process(name="Vision", target=run_vision, 
                                    args=(config, stop_event, frame_ready_event, result_queue))
    
    # 3. Serial (Reads Queue, Writes to UART)
    p_ser = multiprocessing.Process(name="Serial", target=run_serial, 
                                    args=(config, stop_event, result_queue))

    processes = [p_cam, p_vis, p_ser]

    # Start
    for p in processes:
        p.start()
        print(f"Started Process: {p.name} (PID: {p.pid})")

    try:
        while True:
            time.sleep(1)
            # Optional: Watchdog logic here
            for p in processes:
                p.join(timeout=5.0)
                if not p.is_alive():
                    print(f"Process {p.name} died! Shutting down...")
                    raise KeyboardInterrupt
                    
    except KeyboardInterrupt:
        print("\nStopping system...")
    finally:
        stop_event.set()
        frame_ready_event.set() # Unblock waiters
        
        for p in processes:
            p.join(timeout=2.0)
            if p.is_alive():
                print(f"Force killing {p.name}")
                p.terminate()

        print("System shutdown complete.")

if __name__ == "__main__":
    # Ensure correct start method for SharedMemory on Linux
    multiprocessing.set_start_method('spawn', force=True) 
    main()
