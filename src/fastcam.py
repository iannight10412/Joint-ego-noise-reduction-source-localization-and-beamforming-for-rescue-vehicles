import cv2
import time

# 開啟 /dev/video0
cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

# 設定低解析度（例如 640x480 或 320x240）
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("❌ 無法開啟相機，請確認 /dev/video0 是否存在")
    exit()

print("✅ 開始擷取影像中... (按 q 結束)")

# FPS 計算
fps_count = 0
t0 = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ 無法讀取影像")
        break

    fps_count += 1
    if fps_count % 30 == 0:
        elapsed = time.time() - t0
        fps = fps_count / elapsed
        print(f"📷 FPS: {fps:.1f}")

    cv2.imshow("Fast Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
