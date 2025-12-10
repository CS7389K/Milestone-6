#!/bin/bash

echo "==========================================="
echo "YOLO Model Search and HEF Recompilation"
echo "==========================================="
echo ""

# Search for YOLO model files
echo "[1/4] Searching for YOLO model files..."
echo ""

echo "Searching in common locations:"
SEARCH_PATHS=(
    "$HOME"
    "/usr/local"
    "/opt"
    "/tmp"
    "$(pwd)"
)

FOUND_FILES=()

for path in "${SEARCH_PATHS[@]}"; do
    echo "  Checking $path..."
    while IFS= read -r file; do
        FOUND_FILES+=("$file")
        echo "    ✓ Found: $file"
    done < <(find "$path" -type f \( -name "*yolo*.onnx" -o -name "*yolo*.pt" -o -name "yolo11n.onnx" -o -name "yolov11n.onnx" \) 2>/dev/null)
done

echo ""
if [ ${#FOUND_FILES[@]} -eq 0 ]; then
    echo "❌ No YOLO ONNX or PyTorch files found!"
    echo ""
    echo "Options:"
    echo "  1. Download YOLOv11n ONNX from Ultralytics:"
    echo "     pip install ultralytics"
    echo "     python3 -c 'from ultralytics import YOLO; YOLO(\"yolo11n.pt\").export(format=\"onnx\")'"
    echo ""
    echo "  2. Convert existing PyTorch model:"
    echo "     If you have yolo11n.pt, run:"
    echo "     python3 -c 'from ultralytics import YOLO; YOLO(\"yolo11n.pt\").export(format=\"onnx\")'"
    echo ""
    exit 1
fi

echo "Found ${#FOUND_FILES[@]} model file(s)"
echo ""

# Check if Hailo tools are installed
echo "[2/4] Checking Hailo Dataflow Compiler..."
if command -v hailortcli &> /dev/null; then
    echo "  ✓ hailortcli found: $(which hailortcli)"
elif command -v hailo &> /dev/null; then
    echo "  ✓ hailo found: $(which hailo)"
else
    echo "  ❌ Hailo compiler not found!"
    echo ""
    echo "Install Hailo Dataflow Compiler:"
    echo "  1. Download from: https://hailo.ai/developer-zone/software-downloads/"
    echo "  2. Or check if it's in a virtual environment"
    echo ""
    exit 1
fi

# Check Python and hailo_sdk_client
echo ""
echo "[3/4] Checking Hailo Python SDK..."
if python3 -c "import hailo_sdk_client" 2>/dev/null; then
    echo "  ✓ hailo_sdk_client installed"
    SDK_VERSION=$(python3 -c "import hailo_sdk_client; print(hailo_sdk_client.__version__)" 2>/dev/null || echo "unknown")
    echo "    Version: $SDK_VERSION"
else
    echo "  ⚠️  hailo_sdk_client not found (optional for advanced compilation)"
fi

echo ""
echo "[4/4] Available model files:"
for i in "${!FOUND_FILES[@]}"; do
    echo "  [$i] ${FOUND_FILES[$i]}"
done

echo ""
echo "==========================================="
echo "Recompilation Commands"
echo "==========================================="
echo ""

# Generate recompilation commands for each file
for model_file in "${FOUND_FILES[@]}"; do
    filename=$(basename "$model_file")
    extension="${filename##*.}"
    basename_no_ext="${filename%.*}"
    
    if [ "$extension" = "onnx" ]; then
        output_hef="${basename_no_ext}_batch1.hef"
        
        echo "For: $model_file"
        echo "-------------------------------------------"
        echo "Option 1: Quick compile (may not be optimized):"
        echo "  hailortcli compile \"$model_file\" \"$output_hef\""
        echo ""
        echo "Option 2: With batch_size=1 explicitly:"
        echo "  hailortcli compile \"$model_file\" \"$output_hef\" --batch-size 1"
        echo ""
        echo "Option 3: Parse model first (recommended for YOLOv11):"
        echo "  hailortcli parse-hef \"$model_file\" --hw-arch hailo8"
        echo ""
        
    elif [ "$extension" = "pt" ]; then
        echo "For: $model_file (PyTorch - needs conversion)"
        echo "-------------------------------------------"
        echo "Step 1: Export to ONNX:"
        echo "  python3 -c 'from ultralytics import YOLO; YOLO(\"$model_file\").export(format=\"onnx\")'"
        echo ""
        echo "Step 2: Compile ONNX to HEF:"
        echo "  hailortcli compile \"${basename_no_ext}.onnx\" \"${basename_no_ext}_batch1.hef\" --batch-size 1"
        echo ""
    fi
    
    echo ""
done

echo "==========================================="
echo "Recommended Workflow for YOLOv11n"
echo "==========================================="
echo ""
echo "1. If you have yolo11n.pt or yolov11n.pt:"
echo "   python3 -c 'from ultralytics import YOLO; model = YOLO(\"yolo11n.pt\"); model.export(format=\"onnx\", dynamic=False, simplify=True)'"
echo ""
echo "2. Compile with Hailo (use one of these):"
echo "   # Method A: Simple compile"
echo "   hailortcli compile yolo11n.onnx yolo11n_batch1.hef"
echo ""
echo "   # Method B: With explicit batch size"
echo "   hailortcli compile yolo11n.onnx yolo11n_batch1.hef --batch-size 1"
echo ""
echo "   # Method C: Using Hailo Model Zoo (if available)"
echo "   hailo tutorial yolov8n  # Follow interactive tutorial"
echo ""
echo "3. Replace the HEF in your project:"
echo "   cp yolo11n_batch1.hef $(pwd)/models/yolov11n.hef"
echo ""
echo "4. Test the new HEF:"
echo "   cd $(pwd)"
echo "   source install/setup.bash"
echo "   ./run_part2.sh"
echo ""
echo "==========================================="
echo "Troubleshooting"
echo "==========================================="
echo ""
echo "If compilation fails:"
echo "  - Check Hailo hardware: lspci | grep Hailo"
echo "  - Verify driver: ls /dev/hailo*"
echo "  - Check tools version: hailortcli --version"
echo "  - Simplify ONNX: python3 -c 'import onnx, onnxsim; ...'"
echo ""
echo "If you don't have the ONNX file:"
echo "  - Download YOLOv11n weights and export:"
echo "    pip install ultralytics"
echo "    python3 << 'EOF'"
echo "from ultralytics import YOLO"
echo "model = YOLO('yolo11n.pt')  # Downloads automatically"
echo "model.export(format='onnx', dynamic=False, simplify=True, opset=11)"
echo "EOF"
echo ""
