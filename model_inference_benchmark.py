import time
import numpy as np
import os
import argparse

from ament_index_python.packages import get_package_share_directory
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    try:
        import tensorflow.lite.python.interpreter as tflite
    except ImportError:
        print("❌ TFLite not installed.")
        exit(1)

def benchmark(model_path, num_threads=4, iterations=100):
    print(f"🔥 Loading Model: {os.path.basename(model_path)}")
    print(f"   Threads: {num_threads}")
    
    try:
        interpreter = tflite.Interpreter(model_path=model_path, num_threads=num_threads)
        interpreter.allocate_tensors()
    except Exception as e:
        print(f"❌ Failed to load model: {e}")
        return

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    input_shape = input_details[0]['shape']
    input_index = input_details[0]['index']
    output_index = output_details[0]['index']
    
    # Generate Random Input
    dtype = input_details[0]['dtype']
    if dtype == np.uint8:
        input_data = np.random.randint(0, 255, input_shape, dtype=np.uint8)
    else:
        input_data = np.random.random(input_shape).astype(dtype)
        
    print("🏃 Starting Warmup (10 iters)...")
    for _ in range(10):
        interpreter.set_tensor(input_index, input_data)
        interpreter.invoke()
        
    print(f"🚀 Running Benchmark ({iterations} iters)...")
    latencies = []
    
    start_global = time.time()
    for _ in range(iterations):
        t0 = time.time()
        interpreter.set_tensor(input_index, input_data)
        interpreter.invoke()
        interpreter.get_tensor(output_index)
        latencies.append((time.time() - t0) * 1000)
    end_global = time.time()
    
    avg_lat = np.mean(latencies)
    std_lat = np.std(latencies)
    fps = iterations / (end_global - start_global)
    
    print("\n📊 Results:")
    print(f"   Avg Latency: {avg_lat:.2f} ms ± {std_lat:.2f} ms")
    print(f"   Throughput:  {fps:.2f} FPS")
    print(f"   Status:      {'✅ EXCELLENT' if avg_lat < 50 else '✅ GOOD' if avg_lat < 200 else '⚠️ SLOW'}")

if __name__ == "__main__":
    # Use ament_index to find the package path dynamically
    # This makes the script portable and not tied to a specific workspace location
    yolov8_pkg_path = get_package_share_directory('yolov8_detection')
    default_path = os.path.join(yolov8_pkg_path, 'resource', 'mobilenet_v2_1.0_224_quant.tflite')
    
    benchmark(default_path)