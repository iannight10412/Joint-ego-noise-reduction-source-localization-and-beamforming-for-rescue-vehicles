import cv2

cap = cv2.VideoCapture(0)  # 或改成你的 /dev/videoX 編號
if not cap.isOpened():
    print("無法開啟攝影機")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("無法擷取影像")
        break

    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
