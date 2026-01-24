import time
import numpy as np
import os

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    try:
        import tensorflow.lite.python.interpreter as tflite
    except ImportError:
        print("❌ TFLite not installed. Run ./setup_tflite.sh")
        exit(1)

print("🧪 Starting Latency Test...")

# Load Model
model_path = os.path.expanduser('~/uav_agricultural_drone_project/src/yolov8_detection/resource/mobilenet_v2_1.0_224_quant.tflite')
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Create Dummy Image (Random Noise)
input_shape = input_details[0]['shape']
dummy_image = np.random.randint(0, 255, input_shape, dtype=np.uint8)

print(f"   📸 Input Shape: {input_shape}")

# Measure Inference Time
start_time = time.time()
interpreter.set_tensor(input_details[0]['index'], dummy_image)
interpreter.invoke()
output_data = interpreter.get_tensor(output_details[0]['index'])
end_time = time.time()

latency_ms = (end_time - start_time) * 1000
print(f"✅ Inference Successful! Latency: {latency_ms:.2f} ms")