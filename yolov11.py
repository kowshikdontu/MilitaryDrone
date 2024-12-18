from ultralytics import YOLO
import cv2

model = YOLO('best.pt')

for i in range(24):
    print(i)
    img_path = f'black/image_{i}.jpg'
    image = cv2.imread(img_path)

    if image is None:
        print(f"Error: Unable to load image at path '{img_path}'.")
        continue

    results = model.predict(source=image)

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            confidence = box.conf[0]
            class_id = int(box.cls[0])
            print(f"Class: {class_id}, Confidence: {confidence:.2f}, Bounding box: ({x1}, {y1}), ({x2}, {y2})")
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"Class: {class_id}, Conf: {confidence:.2f}"
            cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("YOLOv8 Inference", image)

    key = cv2.waitKey(0)
    if key == ord('q'):
        break
    elif key == ord(' '):
        continue

cv2.destroyAllWindows()
