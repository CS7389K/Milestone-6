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
        print(f"    Format: {info.format}")
        print(f"    Data bytes: {info.data_bytes}")
    
    input_info = input_infos[0]
    
    # Parse shape - Hailo typically uses NHWC format
    shape = input_info.shape
    if len(shape) == 4:
        batch_size, model_height, model_width, channels = shape
    else:
        model_height, model_width, channels = shape
        batch_size = 1
    
    print(f"\n  Parsed: batch={batch_size}, h={model_height}, w={model_width}, c={channels}")
    
    # Configure network
    configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
    network_group = vdevice.configure(hef, configure_params)[0]
    print("✓ Network group configured")
    
    # Create params for inference
    network_group_params = network_group.create_params()
    
    # Set batch size to 1 for single frame inference
    input_vstreams_params = InputVStreamParams.make(network_group, quantized=False, format_type=FormatType.FLOAT32)
    output_vstreams_params = OutputVStreamParams.make(network_group, quantized=False, format_type=FormatType.FLOAT32)
    
    # Override batch size if needed
    for param_name in input_vstreams_params:
        input_vstreams_params[param_name].user_buffer_format.shape[0] = 1  # Set batch to 1
    
    print("✓ VStream parameters created (batch size set to 1)")
    
    # Create dummy frame for single batch
    dummy_frame = np.random.rand(1, model_height, model_width, channels).astype(np.float32)
    print(f"  Created dummy data with shape: {dummy_frame.shape}")
    
    print(f"\nRunning test inference with {model_width}x{model_height} dummy frame...")
    
    # Get input layer name
    input_name = list(input_vstreams_params.keys())[0]
    print(f"  Input layer: {input_name}")
    
    with InferVStreams(network_group, input_vstreams_params, output_vstreams_params) as infer_pipeline:
        input_dict = {input_name: dummy_frame}
        
        # Don't use activate() with scheduler - it's deprecated
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
