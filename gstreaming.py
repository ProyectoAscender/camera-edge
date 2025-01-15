import cv2

pipeline = (
    "udpsrc port=5000 multicast-group=239.255.12.42 auto-multicast=true caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! "
    "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: No se pudo abrir el flujo de video.")
else:
    ret, frame = cap.read()
    if ret:
        # Guardar el primer frame como imagen JPEG
        cv2.imwrite("captura.jpg", frame)
        print("Captura guardada como 'captura.jpg'")
    else:
        print("Error: No se pudo leer el frame.")

cap.release()
