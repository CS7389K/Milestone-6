#!/usr/bin/env python3
"""
Simple camera test script to identify which /dev/video device works.
Tests video0, video1, video2 and shows which one provides actual frames.
"""

import cv2
import sys

def test_camera(device_id):
    """Test a single video device."""
    print(f"\n{'='*60}")
    print(f"Testing /dev/video{device_id}...")
    print('='*60)
    
    cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print(f"❌ Failed to open /dev/video{device_id}")
        return False
    
    # Try to read a frame
    ret, frame = cap.read()
    
    if not ret or frame is None:
        print(f"❌ /dev/video{device_id} opened but cannot read frames")
        cap.release()
        return False
    
    # Get properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"✅ /dev/video{device_id} is WORKING!")
    print(f"   Resolution: {width}x{height}")
    print(f"   FPS: {fps}")
    print(f"   Frame shape: {frame.shape}")
    print(f"   Frame dtype: {frame.dtype}")
    
    # Check if frame looks valid (not all zeros or all same color)
    import numpy as np
    if np.all(frame == frame[0, 0]):
        print(f"⚠️  WARNING: Frame appears to be solid color (all pixels same)")
    
    # Show preview
    print(f"\nShowing 5 second preview from /dev/video{device_id}...")
    print("Press 'q' to skip to next device")
    
    start_time = cv2.getTickCount()
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Lost frame feed")
            break
        
        # Add info overlay
        info_text = f"/dev/video{device_id} - {width}x{height} @ {fps}fps"
        cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, (0, 255, 0), 2)
        
        cv2.imshow(f'Camera Test - video{device_id}', frame)
        
        # Exit after 5 seconds or on 'q' press
        elapsed = (cv2.getTickCount() - start_time) / cv2.getTickFrequency()
        if elapsed > 5:
            break
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    return True

def main():
    print("Camera Device Test Script")
    print("=" * 60)
    
    working_devices = []
    
    # Test all common video devices
    for device_id in [0, 1, 2]:
        if test_camera(device_id):
            working_devices.append(device_id)
    
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    if working_devices:
        print(f"✅ Working camera devices: {working_devices}")
        print(f"\nRECOMMENDATION: Use /dev/video{working_devices[0]} in your code")
    else:
        print("❌ No working camera devices found!")
        print("Check:")
        print("  - Is camera plugged in? (lsusb)")
        print("  - Do you have permissions? (ls -la /dev/video*)")
        print("  - Is another program using the camera?")
    
    print("=" * 60)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
