from ultralytics import YOLO
from PIL import Image
import cv2

# Load a model
model = YOLO("../../model/yolov8m.pt")
video_path = "../../videos/video1.mp4"

# 打开视频文件
cap = cv2.VideoCapture(video_path)

# Run batched inference on a list of images
# results = model("../../images/杯子.png")
# print(results[0].names)
# print(results[0].boxes.cls)
# print(results[0].boxes.conf)
# results[0].show()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    for r in results:
        r.show()

cap.release()
cv2.destroyAllWindows()
