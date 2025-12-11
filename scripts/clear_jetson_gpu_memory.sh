#!/bin/bash

# Script to clear GPU memory on Nvidia Jetson after LLaMA model crashes
# This helps recover from situations where CUDA processes don't properly release GPU memory

set -e

echo "=== Nvidia Jetson GPU Memory Cleanup ==="
echo ""

# Check if running on Jetson (has tegrastats)
if ! command -v tegrastats &> /dev/null; then
    echo "Warning: This script is designed for Nvidia Jetson devices"
    echo "Continuing anyway..."
fi

# Function to display GPU memory usage
show_gpu_memory() {
    echo "Current GPU Memory Usage:"
    if command -v tegrastats &> /dev/null; then
        # Use tegrastats for Jetson-specific info (run for 1 second)
        timeout 1s tegrastats 2>/dev/null | head -n 1 || true
    fi
    
    # Show memory from /proc if available
    if [ -f /proc/meminfo ]; then
        echo "System Memory:"
        grep -E "MemTotal|MemFree|MemAvailable" /proc/meminfo
    fi
    echo ""
}

echo "Before cleanup:"
show_gpu_memory

# Step 1: Find and kill processes using GPU
echo "Searching for processes using GPU..."
GPU_PIDS=$(sudo fuser -v /dev/nvidia* 2>&1 | grep -oP '\d+' | sort -u || true)

if [ -z "$GPU_PIDS" ]; then
    echo "No processes found using GPU devices"
else
    echo "Found processes using GPU: $GPU_PIDS"
    for PID in $GPU_PIDS; do
        PROCESS_NAME=$(ps -p $PID -o comm= 2>/dev/null || echo "unknown")
        echo "  PID $PID: $PROCESS_NAME"
        
        # Check if it's a Python process (likely LLaMA/ML process)
        if [[ "$PROCESS_NAME" == *"python"* ]]; then
            echo "    Killing Python process $PID..."
            sudo kill -9 $PID 2>/dev/null || echo "    Failed to kill PID $PID"
        fi
    done
fi

# Step 2: Clear any remaining CUDA contexts
echo ""
echo "Clearing CUDA contexts..."

# Kill any remaining python processes that might be holding GPU memory
pkill -9 -f "python.*llama" 2>/dev/null || true
pkill -9 -f "python.*model" 2>/dev/null || true

# Step 3: Reset GPU (requires root)
echo ""
echo "Attempting to reset GPU state..."

# For Jetson, clear nvmap (memory allocator) state
if [ -f /sys/kernel/debug/nvmap/iovmm/clients ]; then
    sudo bash -c 'echo 1 > /sys/kernel/debug/nvmap/iovmm/procrank' 2>/dev/null || true
fi

# Step 4: Clear system cache to free up memory
echo ""
echo "Clearing system caches..."
sudo sync
sudo bash -c 'echo 3 > /proc/sys/vm/drop_caches'

# Step 5: For Jetson-specific: Use jetson_clocks to reset
if command -v jetson_clocks &> /dev/null; then
    echo "Resetting Jetson clocks..."
    sudo jetson_clocks --restore 2>/dev/null || true
fi

# Wait a moment for cleanup to complete
sleep 2

echo ""
echo "=== Cleanup Complete ==="
echo ""
echo "After cleanup:"
show_gpu_memory

echo ""
echo "If GPU memory is still not freed, you may need to reboot the Jetson:"
echo "  sudo reboot"
echo ""
echo "To prevent this issue in the future:"
echo "  - Ensure proper cleanup in your LLaMA code (del model, torch.cuda.empty_cache())"
echo "  - Use context managers or try/finally blocks"
echo "  - Monitor GPU memory with: watch -n 1 tegrastats"
