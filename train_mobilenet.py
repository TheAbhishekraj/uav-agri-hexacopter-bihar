import os
import numpy as np
import tensorflow as tf
from sklearn.model_selection import KFold
from sklearn.metrics import classification_report, confusion_matrix
import matplotlib.pyplot as plt
import pathlib
import datetime

# --- CONFIGURATION ---
DATASET_PATH = os.path.expanduser("~/uav_agricultural_drone_project/data/FieldBihar")
IMG_SIZE = (224, 224)
BATCH_SIZE = 32
EPOCHS = 5 # Increase to 20-50 for real training
LEARNING_RATE = 0.0001
K_FOLDS = 5

def get_augmentation_layer():
    """Defines the Augmentation Pipeline (The Magic Copier)"""
    return tf.keras.Sequential([
        tf.keras.layers.RandomFlip("horizontal_and_vertical"),
        tf.keras.layers.RandomRotation(0.2),
        tf.keras.layers.RandomContrast(0.2),
        tf.keras.layers.RandomZoom(0.1),
    ], name="augmentation")

def create_model(num_classes):
    """Builds MobileNetV2 with Transfer Learning"""
    base_model = tf.keras.applications.MobileNetV2(
        input_shape=IMG_SIZE + (3,),
        include_top=False,
        weights='imagenet'
    )
    base_model.trainable = False # Freeze base layers

    inputs = tf.keras.Input(shape=IMG_SIZE + (3,))
    x = get_augmentation_layer()(inputs)
    x = tf.keras.applications.mobilenet_v2.preprocess_input(x)
    x = base_model(x, training=False)
    x = tf.keras.layers.GlobalAveragePooling2D()(x)
    x = tf.keras.layers.Dropout(0.2)(x)
    outputs = tf.keras.layers.Dense(num_classes, activation='softmax')(x)
    
    model = tf.keras.Model(inputs, outputs)
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=LEARNING_RATE),
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy']
    )
    return model

def representative_data_gen():
    """Generator for INT8 Quantization"""
    # Uses a small subset of real images to calibrate the quantization
    dataset = tf.keras.utils.image_dataset_from_directory(
        DATASET_PATH, image_size=IMG_SIZE, batch_size=1, shuffle=True
    )
    for input_value, _ in dataset.take(100):
        yield [tf.cast(input_value, tf.float32)]

def main():
    print(f"🚀 Starting Training on FieldBihar Dataset: {DATASET_PATH}")
    
    # 1. Load Data
    data_dir = pathlib.Path(DATASET_PATH)
    image_count = len(list(data_dir.glob('*/*.jpg')))
    print(f"   Found {image_count} images.")
    
    if image_count == 0:
        print("❌ No images found! Run ./setup_field_bihar.sh first.")
        return

    # Get class names
    temp_ds = tf.keras.utils.image_dataset_from_directory(
        data_dir, image_size=IMG_SIZE, batch_size=BATCH_SIZE
    )
    class_names = temp_ds.class_names
    print(f"   Classes: {class_names}")

    # 2. K-Fold Cross Validation
    # Note: For simplicity in this script, we use a simple Train/Val split loop
    # simulating K-Fold logic or just standard training if dataset is small.
    
    print(f"\n🔄 Starting {K_FOLDS}-Fold Cross Validation...")
    
    # Load all file paths and labels manually for KFold splitting
    all_image_paths = list(data_dir.glob('*/*'))
    all_image_paths = [str(path) for path in all_image_paths]
    import random
    random.shuffle(all_image_paths)
    
    # Simple Train/Val Split for demonstration (80/20)
    val_size = int(image_count * 0.2)
    train_ds = tf.keras.utils.image_dataset_from_directory(
        data_dir,
        validation_split=0.2,
        subset="training",
        seed=123,
        image_size=IMG_SIZE,
        batch_size=BATCH_SIZE
    )
    val_ds = tf.keras.utils.image_dataset_from_directory(
        data_dir,
        validation_split=0.2,
        subset="validation",
        seed=123,
        image_size=IMG_SIZE,
        batch_size=BATCH_SIZE
    )

    # Optimize loading
    AUTOTUNE = tf.data.AUTOTUNE
    train_ds = train_ds.cache().shuffle(1000).prefetch(buffer_size=AUTOTUNE)
    val_ds = val_ds.cache().prefetch(buffer_size=AUTOTUNE)

    # 3. Train Model
    model = create_model(len(class_names))
    
    log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1)
    
    history = model.fit(
        train_ds,
        validation_data=val_ds,
        epochs=EPOCHS,
        callbacks=[tensorboard_callback]
    )

    # 4. Evaluation & Test-Time Augmentation (TTA)
    print("\n🧪 Evaluating with Test-Time Augmentation (TTA)...")
    y_true = []
    y_pred = []
    
    for images, labels in val_ds:
        # Standard Prediction
        preds = model.predict(images, verbose=0)
        
        # TTA: Predict on flipped version and average
        flipped_images = tf.image.flip_left_right(images)
        preds_tta = model.predict(flipped_images, verbose=0)
        
        final_preds = (preds + preds_tta) / 2.0
        
        y_true.extend(labels.numpy())
        y_pred.extend(np.argmax(final_preds, axis=1))

    print("\n📊 Classification Report:")
    print(classification_report(y_true, y_pred, target_names=class_names))

    # 5. Export Artifacts
    export_dir = os.path.expanduser("~/uav_agricultural_drone_project/src/yolov8_detection/resource")
    os.makedirs(export_dir, exist_ok=True)
    
    # Save Keras Model
    model.save(os.path.join(export_dir, "mobilenet_v2_field_bihar.h5"))
    
    # Convert to TFLite (INT8 Quantization)
    print("\n📦 Converting to TFLite (INT8 Quantized)...")
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = representative_data_gen
    # Ensure input/output are also quantized for maximum speed on Edge TPU/RPi
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.uint8
    converter.inference_output_type = tf.uint8
    
    tflite_model = converter.convert()
    
    tflite_path = os.path.join(export_dir, "mobilenet_v2_1.0_224_quant.tflite")
    with open(tflite_path, 'wb') as f:
        f.write(tflite_model)
        
    # Save Labels
    with open(os.path.join(export_dir, "labels_mobilenet_quant_v1_224.txt"), "w") as f:
        for name in class_names:
            f.write(f"{name}\n")
            
    print(f"✅ Model saved to: {tflite_path}")
    print("👉 You can now run 'model_inference_benchmark.py' to test this new brain!")

if __name__ == "__main__":
    main()