#include <opencv2/opencv.hpp>
#include <stdio.h>

int main() {
    // Definir el pipeline de GStreamer
    const char* pipeline = 
        "udpsrc port=5000 multicast-group=239.255.12.42 auto-multicast=true caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";

    // Abrir el flujo de video con OpenCV
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    // Comprobar si se abrió correctamente
    if (!cap.isOpened()) {
        fprintf(stderr, "Error: No se pudo abrir el flujo de video.\n");
        return -1;
    }

    // Intentar capturar el primer frame
    cv::Mat frame;
    if (cap.read(frame)) {
        // Guardar el frame como imagen JPEG
        if (cv::imwrite("captura.jpg", frame)) {
            printf("Captura guardada como 'captura.jpg'\n");
        } else {
            fprintf(stderr, "Error: No se pudo guardar la imagen.\n");
        }
    } else {
        fprintf(stderr, "Error: No se pudo leer el frame.\n");
    }

    // Liberar el recurso de captura
    cap.release();
    return 0;
}
