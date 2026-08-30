import cv2

# 用 GStreamer pipeline 強制轉換格式
pipeline = (
    "v4l2src device=/dev/video0 ! "
    "video/x-bayer, width=640, height=480 ! "
    "bayer2rgb ! videoconvert ! appsink"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("❌ 無法開啟 GStreamer pipeline")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ 無法擷取影像")
        break
    cv2.imshow("Pi Camera (no libcamera)", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
