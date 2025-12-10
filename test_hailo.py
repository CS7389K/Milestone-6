#!/usr/bin/env python3
"""Test script to verify Hailo HEF model loading."""

import sys
import cv2
import numpy as np

try:
    from hailo_platform import (HEF, ConfigureParams, FormatType, HailoStreamInterface,
                                InferVStreams, InputVStreamParams, OutputVStreamParams,
                                HailoSchedulingAlgorithm, VDevice)
    print("✓ Hailo platform imported successfully")
except ImportError as e:
    print(f"✗ Failed to import hailo_platform: {e}")
    sys.exit(1)

# Test HEF model loading
hef_path = "models/yolov11n.hef"

try:
    print(f"\nLoading HEF model: {hef_path}")
    
    # Create VDevice
    params = VDevice.create_params()
    params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
    vdevice = VDevice(params)
    print("✓ VDevice created")
    
    # Load HEF
    hef = HEF(hef_path)
    print("✓ HEF model loaded")
    
    # Get input info
    input_info = hef.get_input_vstream_infos()[0]
    print(f"✓ Model input shape: {input_info.shape}")
    print(f"  Width: {input_info.shape[1]}, Height: {input_info.shape[0]}")
    
    # Configure network
    configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
    network_group = vdevice.configure(hef, configure_params)[0]
    print("✓ Network group configured")
    
    # Create params for inference
    network_group_params = network_group.create_params()
    input_vstreams_params = InputVStreamParams.make(network_group, quantized=False, format_type=FormatType.FLOAT32)
    output_vstreams_params = OutputVStreamParams.make(network_group, quantized=False, format_type=FormatType.FLOAT32)
    print("✓ VStream parameters created")
    
    # Test inference with dummy data
    model_height = input_info.shape[0]
    model_width = input_info.shape[1]
    dummy_frame = np.random.rand(model_height, model_width, 3).astype(np.float32)
    
    print(f"\nRunning test inference with {model_width}x{model_height} dummy frame...")
    
    # Get input layer name
    input_name = list(input_vstreams_params.keys())[0]
    print(f"  Input layer: {input_name}")
    
    with InferVStreams(network_group, input_vstreams_params, output_vstreams_params) as infer_pipeline:
        input_dict = {input_name: dummy_frame}
        
        with network_group.activate(network_group_params):
            import time
            t1 = time.time()
            results = infer_pipeline.infer(input_dict)
            t2 = time.time()
            
    print(f"✓ Inference completed in {(t2-t1)*1000:.1f}ms")
    print(f"✓ Output tensors: {list(results.keys())}")
    for name, tensor in results.items():
        print(f"  {name}: shape {tensor.shape}, dtype {tensor.dtype}")
    
    # Cleanup
    network_group.release()
    vdevice.release()
    print("\n✓ All tests passed! Hailo is ready to use.")
    
except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
