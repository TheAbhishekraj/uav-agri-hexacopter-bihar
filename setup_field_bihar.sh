#!/bin/bash

WORKSPACE_DIR=~/uav_agricultural_drone_project
DATASET_DIR=$WORKSPACE_DIR/data/FieldBihar

echo "🌾 Setting up FieldBihar Dataset Structure..."

# 1. Create Directory Structure
mkdir -p "$DATASET_DIR/weed"
mkdir -p "$DATASET_DIR/crop"
mkdir -p "$DATASET_DIR/background"

echo "   📂 Created: $DATASET_DIR"

# 2. Generate Dummy Data for Pipeline Testing (Optional)
# We create 10 fake images in each class to ensure train_mobilenet.py runs out-of-the-box.
echo "   🎨 Generating dummy images for pipeline verification..."

python3 -c "
import cv2
import numpy as np
import os

base_path = '$DATASET_DIR'
classes = ['weed', 'crop', 'background']

for cls in classes:
    path = os.path.join(base_path, cls)
    for i in range(10):
        # Create a random colored image
        img = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
        # Add text so we can see it's fake
        cv2.putText(img, f'{cls}_{i}', (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imwrite(os.path.join(path, f'fake_{i}.jpg'), img)
"

echo "   ✅ Dummy data generated."
echo "   👉 Replace these files with REAL photos from Bihar later!"

# 3. Create Training Requirements File
cat > "$WORKSPACE_DIR/training_requirements.txt" <<EOL
tensorflow>=2.10.0
scikit-learn
matplotlib
albumentations
EOL

echo "   📝 Created training_requirements.txt"