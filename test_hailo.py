#!/usr/bin/env python3
"""Test script to verify Hailo HEF model loading."""

import sys
import numpy as np

try:
    from hailo_platform import HEF, VDevice, InferVStreams, InputVStreamParams, OutputVStreamParams, FormatType
    print("✓ Hailo platform imported successfully")
except ImportError as e:
    print(f"✗ Failed to import hailo_platform: {e}")
    sys.exit(1)

# Test HEF model loading
hef_path = "models/yolov11n.hef"

try:
    print(f"\nLoading HEF model: {hef_path}")
    
    # Load HEF
    hef = HEF(hef_path)
    print("✓ HEF model loaded")
    
    # Create VDevice (simple API)
    vdevice = VDevice()
    print("✓ VDevice created")
    
    # Configure and get network group
    network_groups = vdevice.configure(hef)
    network_group = network_groups[0]
    print("✓ Network group configured")
    
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
    
    print(f"\n  Parsed: h={model_height}, w={model_width}, c={channels}")
    
    # Create dummy frame - UINT8 format (0-255)
    dummy_frame = np.random.randint(0, 255, (model_height, model_width, channels), dtype=np.uint8)
    print(f"  Created dummy data: {dummy_frame.shape}, dtype: {dummy_frame.dtype}")
    
    print(f"\nRunning test inference...")
    
    # Run inference with activate context
    with network_group.activate():
        # Create params inside activate context
        input_vstreams_params = InputVStreamParams.make_from_network_group(
            network_group, quantized=True, format_type=FormatType.UINT8
        )
        output_vstreams_params = OutputVStreamParams.make_from_network_group(
            network_group, quantized=False, format_type=FormatType.FLOAT32
        )
        
        print(f"  Input layer: {list(input_vstreams_params.keys())[0]}")
        
        with InferVStreams(network_group, input_vstreams_params, output_vstreams_params) as infer_pipeline:
            import time
            t1 = time.time()
            input_name = list(input_vstreams_params.keys())[0]
            results = infer_pipeline.infer({input_name: dummy_frame})
            t2 = time.time()
            
    print(f"✓ Inference completed in {(t2-t1)*1000:.1f}ms")
    print(f"✓ Output tensors: {list(results.keys())}")
    for name, tensor in results.items():
        print(f"  {name}: shape {tensor.shape}, dtype {tensor.dtype}")
    
    # Cleanup
    vdevice.release()
    print("\n✓ All tests passed! Hailo is ready to use.")
    
except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
