#!/bin/bash
# Check if Hailo AI accelerator is available

echo "========================================="
echo "Checking for Hailo AI Accelerator"
echo "========================================="

# Check for Hailo PCIe device
echo -e "\n1. PCIe Devices:"
lspci | grep -i hailo || echo "No Hailo PCIe device found"

# Check for Hailo kernel module
echo -e "\n2. Kernel Modules:"
lsmod | grep -i hailo || echo "No Hailo kernel module loaded"

# Check for hailort
echo -e "\n3. HailoRT Software:"
which hailortcli && hailortcli --version || echo "hailortcli not found"

# Check for .hef model
echo -e "\n4. HEF Model Files:"
ls -lh *.hef 2>/dev/null || ls -lh models/*.hef 2>/dev/null || echo "No .hef files found"

echo -e "\n========================================="
echo "SUMMARY:"
echo "========================================="
if lspci | grep -qi hailo; then
    echo "✓ Hailo hardware detected!"
    echo "  You can use .hef models for 10-50x speedup"
    echo "  Convert your model: hailortcli compile yolo11n.pt yolo11n.hef"
else
    echo "✗ No Hailo hardware found"
    echo "  Your options for speed:"
    echo "  1. Reduce resolution further (already at 320x192)"
    echo "  2. Increase frame skipping (currently every 3rd frame)"
    echo "  3. Use TensorRT (needs NVIDIA GPU support in PyTorch)"
fi
echo "========================================="
