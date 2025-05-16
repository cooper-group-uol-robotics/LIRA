from ultralytics import YOLO
from pathlib import Path
import os

# === CONFIG ===
model_path = "runs/classify/train/weights/best.pt"
test_dir = Path("/mnt/storage/yolov12/dataset/test")  # class subfolders here
supported_exts = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}

# === Load model ===
model = YOLO(model_path)

# === Gather all test images and ground truth labels ===
image_paths = list(test_dir.rglob("*"))
test_images = [p for p in image_paths if p.suffix.lower() in supported_exts]

correct = 0
total = 0
errors = []

for img_path in test_images:
    # Ground truth class is the parent folder name
    ground_truth = img_path.parent.name

    # Run prediction
    result = model.predict(source=img_path, verbose=False)[0]
    predicted_class_index = int(result.probs.top1)
    predicted_class_name = model.names[predicted_class_index]

    # Compare and count
    total += 1
    if predicted_class_name == ground_truth:
        correct += 1
    else:
        errors.append((img_path.name, ground_truth, predicted_class_name))

# === Report ===
accuracy = correct / total * 100 if total > 0 else 0
print(f"\n✅ Classification Accuracy: {accuracy:.2f}% ({correct}/{total})")

if errors:
    print("\n❌ Misclassified images:")
    for img_name, gt, pred in errors:
        print(f"  {img_name}: GT = {gt}, Pred = {pred}")
