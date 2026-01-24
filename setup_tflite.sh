#!/bin/bash

WORKSPACE_DIR=~/uav_agricultural_drone_project
RESOURCE_DIR=$WORKSPACE_DIR/src/yolov8_detection/resource

echo "🧠 Setting up TFLite Brain..."

mkdir -p "$RESOURCE_DIR"

# 1. Download MobileNetV2 Quantized (Fast INT8 model)
if [ ! -f "$RESOURCE_DIR/mobilenet_v2_1.0_224_quant.tflite" ]; then
    echo "   ⬇️  Downloading Model..."
    wget -q https://storage.googleapis.com/download.tensorflow.org/models/tflite/mobilenet_v2_1.0_224_quant_and_labels.zip -O /tmp/mobilenet.zip
    unzip -o /tmp/mobilenet.zip -d "$RESOURCE_DIR"
    rm /tmp/mobilenet.zip
else
    echo "   ✅ Model already exists."
fi

# 2. Install TFLite Runtime (Lightweight inference engine)
echo "   📦 Checking for TFLite Runtime..."
if ! python3 -c "import tflite_runtime" &> /dev/null; then
    echo "      -> Installing tflite-runtime..."
    pip3 install tflite-runtime --break-system-packages || pip3 install tensorflow --break-system-packages
else
    echo "   ✅ TFLite Runtime found."
fi

# 3. Generate Model Metadata (Versioning & Checksum)
echo "   📝 Generating Metadata (Version 1.0.0)..."
SHA=$(sha256sum "$RESOURCE_DIR/mobilenet_v2_1.0_224_quant.tflite" | awk '{print $1}')
cat > "$RESOURCE_DIR/model_info.json" <<EOL
{
    "version": "1.0.0",
    "model_name": "mobilenet_v2_1.0_224_quant",
    "sha256": "$SHA",
    "optimization": "INT8 Quantization"
}
EOL

echo "✨ AI Assets Ready in $RESOURCE_DIR"