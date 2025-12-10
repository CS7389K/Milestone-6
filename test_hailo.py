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
    
    # Get input/output info and check batch size
    input_infos = hef.get_input_vstream_infos()
    output_infos = hef.get_output_vstream_infos()
    
    print(f"✓ Model has {len(input_infos)} input(s) and {len(output_infos)} output(s)")
    
    input_info = input_infos[0]
    print(f"  Input: {input_info.name}")
    print(f"    Shape: {input_info.shape}")
    
    # Parse shape
    shape = input_info.shape
    if len(shape) == 4:
        batch_size, model_height, model_width, channels = shape
    elif len(shape) == 3:
        model_height, model_width, channels = shape
        batch_size = 1
    else:
        raise ValueError(f"Unexpected shape: {shape}")
    
    print(f"    Parsed: batch={batch_size}, h={model_height}, w={model_width}, c={channels}")
    
    # The error shows it wants 640 frames batched!
    # Calculate from error: 786432000 / 1228800 = 640
    # So we need to provide batch_size worth of frames
    single_frame_size = model_height * model_width * channels  # 1228800
    expected_batch = 786432000 // single_frame_size  # Should be 640
    
    print(f"\n  Model expects batch of {expected_batch} frames")
    print(f"  Creating batch of {expected_batch} dummy frames...")
    
    # Create batched input: [batch_size, H, W, C]
    dummy_batch = np.random.randint(0, 255, (expected_batch, model_height, model_width, channels), dtype=np.uint8)
    print(f"  Created data: {dummy_batch.shape}, dtype: {dummy_batch.dtype}")
    
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
            results = infer_pipeline.infer({input_name: dummy_batch})
            t2 = time.time()
            
    print(f"✓ Inference completed in {(t2-t1)*1000:.1f}ms")
    print(f"✓ Output tensors: {list(results.keys())}")
    for name, tensor in results.items():
        if isinstance(tensor, np.ndarray):
            print(f"  {name}: shape {tensor.shape}, dtype {tensor.dtype}")
        else:
            print(f"  {name}: type {type(tensor)}, value: {tensor}")
    
    print(f"\n✓ Inference time per frame: {(t2-t1)*1000/expected_batch:.1f}ms")
    print(f"✓ Theoretical FPS: {expected_batch/(t2-t1):.1f} (batched)")
    print(f"✓ Single frame FPS estimate: {1/((t2-t1)/expected_batch):.1f}")
    
    # Cleanup
    vdevice.release()
    
    print("\n" + "="*50)
    print("IMPORTANT: This HEF expects batch size of 640!")
    print("For real-time single-frame inference, you need to:")
    print("  1. Recompile HEF with batch_size=1")
    print("  2. Or send 640 frames at once (not practical)")
    print("="*50)
    print("\n✓ Hailo hardware works, but HEF needs recompilation.")
    
except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
