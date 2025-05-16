from ultralytics import YOLO

model = YOLO("yolo11n-cls.pt") 

results = model.train(
    data='/mnt/storage/yolov12/dataset',  # points to folder with train/valid
    epochs=300,
    imgsz=384,      # 224 is more typical for classification; 640 is overkill
    batch=32,
    device=0
)

