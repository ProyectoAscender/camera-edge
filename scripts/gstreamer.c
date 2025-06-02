// #include <opencv2/opencv.hpp>
// #include <stdio.h>

// int main() {
//     // Definir el pipeline de GStreamer
//     const char* pipeline = 
//         "udpsrc port=5000 multicast-group=239.255.12.42 auto-multicast=true caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! "
//         "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";

//     // Abrir el flujo de video con OpenCV
//     cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

//     // Comprobar si se abrió correctamente
//     if (!cap.isOpened()) {
//         fprintf(stderr, "Error: No se pudo abrir el flujo de video.\n");
//         return -1;
//     }

//     // Intentar capturar el primer frame
//     cv::Mat frame;
//     if (cap.read(frame)) {
//         // Guardar el frame como imagen JPEG
//         if (cv::imwrite("captura.jpg", frame)) {
//             printf("Captura guardada como 'captura.jpg'\n");
//         } else {
//             fprintf(stderr, "Error: No se pudo guardar la imagen.\n");
//         }
//     } else {
//         fprintf(stderr, "Error: No se pudo leer el frame.\n");
//     }

//     // Liberar el recurso de captura
//     cap.release();
//     return 0;
// }


#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <iostream>

int main() {
    // Activar logs de identity
    setenv("GST_DEBUG", "identity:6", 1);
    setenv("GST_DEBUG_FILE", "/tmp/gst_debug.log", 1);

    // Imagen base (verde)
    cv::Mat dummy(720, 1280, CV_8UC3, cv::Scalar(0, 255, 0));

    // Pipeline con udpsink multicast
    std::string pipeline =
        "appsrc ! videoconvert ! "
        "video/x-raw,format=NV12 ! "
        "identity name=identity_out silent=false ! "
        "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
        "nvv4l2h264enc insert-sps-pps=true iframeinterval=1 idrinterval=1 control-rate=1 bitrate=32000000 ! "
        "h264parse ! rtph264pay config-interval=1 pt=96 ! "
        "udpsink host=239.255.12.41 port=5001 auto-multicast=true sync=0";

    cv::VideoWriter writer(pipeline, cv::CAP_GSTREAMER, 0, 30, dummy.size(), true);

    if (!writer.isOpened()) {
        std::cerr << "Error al abrir el pipeline" << std::endl;
        return 1;
    }

    std::cout << "Presiona Enter para enviar cada frame..." << std::endl;

    for (int i = 0; i < 10; ++i) {
        // Dibuja el número de iteración en la imagen
        cv::Mat frame = dummy.clone();
        std::string text = "Frame " + std::to_string(i);
        cv::putText(frame, text, cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 0, 255), 4);

        // Espera pulsación de tecla
        std::cin.get();

        writer.write(frame);
        std::cout << "Enviado frame " << i << std::endl;
    }

    return 0;
}
