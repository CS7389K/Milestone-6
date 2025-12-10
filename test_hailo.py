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
    input_infos = hef.get_input_vstream_infos()
    output_infos = hef.get_output_vstream_infos()
    
    print(f"✓ Model has {len(input_infos)} input(s) and {len(output_infos)} output(s)")
    
    for i, info in enumerate(input_infos):
        print(f"  Input {i}: {info.name}")
        print(f"    Shape: {info.shape}")
    
    input_info = input_infos[0]
    
    # Parse shape - Hailo typically uses HWC format (no batch in shape)
    shape = input_info.shape
    if len(shape) == 4:
        batch_size, model_height, model_width, channels = shape
    elif len(shape) == 3:
        model_height, model_width, channels = shape
        batch_size = 1  # Will add batch dimension manually
    else:
        raise ValueError(f"Unexpected shape format: {shape}")
    
    print(f"\n  Parsed: h={model_height}, w={model_width}, c={channels} (will use batch=1)")
    
    # Configure network
    configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
    network_group = vdevice.configure(hef, configure_params)[0]
    print("✓ Network group configured")
    
    # Create params - quantized input (UINT8), non-quantized output (FLOAT32)
    network_group_params = network_group.create_params()
    input_vstreams_params = InputVStreamParams.make(network_group, quantized=True, format_type=FormatType.UINT8)
    output_vstreams_params = OutputVStreamParams.make(network_group, quantized=False, format_type=FormatType.FLOAT32)
    
    print("✓ VStream parameters created (input: UINT8, output: FLOAT32)")
    
    # Create dummy frame - UINT8 format (0-255)
    dummy_frame = np.random.randint(0, 255, (model_height, model_width, channels), dtype=np.uint8)
    print(f"  Created dummy data with shape: {dummy_frame.shape}, dtype: {dummy_frame.dtype}")
    
    print(f"\nRunning test inference with {model_width}x{model_height} dummy frame...")
    
    # Get input layer name
    input_name = list(input_vstreams_params.keys())[0]
    print(f"  Input layer: {input_name}")
    
    with InferVStreams(network_group, input_vstreams_params, output_vstreams_params) as infer_pipeline:
        import time
        t1 = time.time()
        results = infer_pipeline.infer({input_name: dummy_frame})
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
