#include "configuration.h"
#include "utils.h"


std::string executeCommandAndGetOutput(const char *command)
{
    FILE *fpipe;
    char c = 0;

    if (0 == (fpipe = (FILE *)popen(command, "r")))
        FatalError("popen() failed.");

    std::string output;
    while (fread(&c, sizeof c, 1, fpipe))
        output.push_back(c);
    // std::cout<<output<<std::endl;

    pclose(fpipe);
    return output;
}

std::string decryptString(std::string encrypted, const std::string &password)
{
    // when using OpenSSL 1.1.1 use the following
    // std::string command = "echo '"+ encrypted +"' | openssl enc -e -aes-256-cbc -a -d -salt -iter 100000 -pass pass:"+password;
    std::string command = "echo '" + encrypted + "' | openssl enc -e -aes-256-cbc -a -d -salt -pass pass:" + password;
    return executeCommandAndGetOutput(command.c_str());
}

std::string encryptString(std::string to_encrypt, const std::string &password)
{
    // when using OpenSSL 1.1.1 use the following
    // std::string command = "echo -n "+to_encrypt+" | openssl enc -e -aes-256-cbc -a -salt -iter 100000 -pass pass:"+password;
    std::string command = "echo -n " + to_encrypt + " | openssl enc -e -aes-256-cbc -a -salt -pass pass:" + password;
    return executeCommandAndGetOutput(command.c_str());
}

void readParamsFromYaml(const std::string &params_path, const std::vector<std::string> &cameras_ids, std::vector<edge::camera_params> &cameras_par, std::string &net, char &type, int &n_classes, std::string &tif_map_path)
{
    std::string password = "";
    YAML::Node config = YAML::LoadFile(params_path);
    net = config["net"].as<std::string>();
    type = config["type"].as<char>();
    n_classes = config["classes"].as<int>();
    tif_map_path = config["tif"].as<std::string>();
    if (config["password"])
        password = config["password"].as<std::string>();

    int stream_height, stream_width;
    stream_width = config["width"].as<int>();
    stream_height = config["height"].as<int>();

    if (stream_height < 0 || stream_width < 0)
        FatalError("The dimensions of the stream have to be greater than 0");

    int filter_type = 0; // EKF
    if (config["filter"])
        filter_type = config["filter"].as<int>();
    if (filter_type > 1)
        FatalError("The values allowed for filters are 0 (EKF) and 1 (UFK)");

    if (config["record"])
        record = config["record"].as<int>();
    if (config["recordBoxes"])
        recordBoxes = config["recordBoxes"].as<int>();
    if (config["stream"])
        stream = config["stream"].as<int>();

    YAML::Node cameras_yaml = config["cameras"];
    bool use_info;
    int n_cameras = 0;
    std::cout << "AAAAAAAAA" << std::endl;
    for (int i = 0; i < cameras_yaml.size(); i++)
    {
            std::cout << "BBBB" << std::endl;

        std::string camera_id = cameras_yaml[i]["id"].as<std::string>();
        
        // save infos only of the cameras whose ids where passed as args
        use_info = false;
        for (auto id : cameras_ids)
            if (camera_id == id)
                use_info = true;
        if (!use_info)
            continue;

        cameras_par.resize(++n_cameras);
        cameras_par[n_cameras - 1].id = camera_id;
        cameras_par[n_cameras - 1].framesToProcess = cameras_yaml[i]["framesToProcess"].as<int>();
        cameras_par[n_cameras - 1].portCommunicator = cameras_yaml[i]["portCommunicator"].as<int>();
                    std::cout << "BBBB2" << std::endl;

        if (cameras_yaml[i]["gstreamer"].as<bool>())
        {
            cameras_par[n_cameras - 1].gstreamer = true;
        }
        else
        {
            cameras_par[n_cameras - 1].gstreamer = false;
        }
                        std::cout << "BBBB3" << std::endl;

        if (cameras_par[n_cameras - 1].gstreamer)
        {
            // if it is a GStreamer stream it cannot be encrypted
            //  rtspsrc location=rtsp://admin:password@192.168.1.65:554 drop-on-latency=0 latency=10 !
            //  rtph264depay ! h264parse ! decodebin ! videoconvert! videoscale !
            //  video/x-raw, format=(string)BGR, width=(int){}, height=(int){},
            //  framerate=25/1 ! timeoverlay halignment=left valignment=bottom text ="Stream time:"
            //  font-desc="Sans, 20"! clockoverlay ! absolute s !
            //  videoconvert ! appsink emit-signals=true sync=true max-buffers=3 drop=false
            cameras_par[n_cameras - 1].input = "rtspsrc location=" + cameras_yaml[i]["input"].as<std::string>() +
                                               " latency=2000 ! queue ! rtph264depay ! h264parse ! omxh264dec !" +
                                               " nvvidconv ! video/x-raw,  width=(int)" + cameras_yaml[i]["gstreamer.width"].as<std::string>() +
                                               ", height=(int)" + cameras_yaml[i]["gstreamer.height"].as<std::string>() +
                                               " format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false";
            // cameras_par[n_cameras-1].input =  cameras_yaml[i]["input"].as<std::string>();
        }
        else
        {
            if (cameras_yaml[i]["encrypted"].as<int>())
            {
                if (password == "")
                {
                    std::cout << "Please insert the password to decript the cameras input" << std::endl;
                    std::cin >> password;
                }
                cameras_par[n_cameras - 1].input = decryptString(cameras_yaml[i]["input"].as<std::string>(), password);
            }
            else
                cameras_par[n_cameras - 1].input = cameras_yaml[i]["input"].as<std::string>();

            if (cameras_yaml[i]["resolution"])
            {
                cameras_par[n_cameras - 1].resolution = cameras_yaml[i]["resolution"].as<std::string>();
                // here we append some parameters to the rtsp string
                if (!cameras_par[n_cameras - 1].resolution.empty())
                    cameras_par[n_cameras - 1].input = cameras_par[n_cameras - 1].input + "?resolution=" + cameras_par[n_cameras - 1].resolution;
            }
        }
                    std::cout << "BBBB4" << std::endl;

        cameras_par[n_cameras - 1].pmatrixPath = cameras_yaml[i]["pmatrix"].as<std::string>();
        cameras_par[n_cameras - 1].streamWidth = stream_width;
        cameras_par[n_cameras - 1].streamHeight = stream_height;
        cameras_par[n_cameras - 1].filterType = filter_type;
        cameras_par[n_cameras - 1].show = true;

        if (cameras_yaml[i]["cameraCalib"])
            cameras_par[n_cameras - 1].cameraCalibPath = cameras_yaml[i]["cameraCalib"].as<std::string>();
        if (cameras_yaml[i]["maskFileOrient"])
            cameras_par[n_cameras - 1].maskFileOrientPath = cameras_yaml[i]["maskFileOrient"].as<std::string>();
        if (cameras_yaml[i]["maskfile"])
            cameras_par[n_cameras - 1].maskfilePath = cameras_yaml[i]["maskfile"].as<std::string>();
                                std::cout << "BBBB5" << std::endl;

        // simple check: if the input contains rtsp string and stream is set to false, then raise ad exception
        if (!stream && cameras_par[n_cameras - 1].input.find("rtsp") != std::string::npos)
        {
            FatalError("Error: it's a rtsp stream, so you have to change the field in the yaml file\n");
        }
                                        std::cout << "BBBB6" << std::endl;

    }
                                    std::cout << "BBBB¡7" << std::endl;

}

bool readParameters(int argc, char **argv, std::vector<edge::camera_params> &cameras_par, std::string &net, char &type, int &n_classes, std::string &tif_map_path)
{
    std::string help = "class-edge demo\nCommand:\n"
                       "-i\tparameters file\n"
                       "-n\tnetwork rt path\n"
                       "-t\ttype of network (only y|c|m admissed)\n"
                       "-c\tnumber of classes for the network\n"
                       "-m\tmap.tif path (to get GPS position)\n"
                       "-s\tshow (0=false, 1=true)\n"
                       "-v\tverbose (0=false, 1=true)\n"
                       "-u\tudp by bsc (0=false, 1=true)\n"
                       "\tlist of camera ids (n ids expected)\n\n";

    // default values
    net = "yolo4_berkeley_fp16.rt";
    tif_map_path = "../data/masa_map.tif";
    type = 'y';
    n_classes = 10;
    use_udp_socket = false;

    // read values
    std::string params_path = "";
    std::string read_net = "";
    std::string read_tif_map_path = "";
    char read_type = '\0';
    int read_n_classes = 0;

    // read arguments
    for (int opt; (opt = getopt(argc, argv, ":i:m:s:v:n:u:c:t:h")) != -1;)
    {
        switch (opt)
        {
        case 'h':
            std::cout << help << std::endl;
            exit(EXIT_SUCCESS);
        case 'i':
            params_path = optarg;
            break;
        case 'm':
            read_tif_map_path = optarg;
            break;
        case 'c':
            read_n_classes = atoi(optarg);
            break;
        case 's':
            show = atoi(optarg);
            break;
        case 'v':
            verbose = atoi(optarg);
            break;
        case 'n':
            read_net = optarg;
            break;
        case 'u':
            use_udp_socket = (bool)atoi(optarg);
            break;
        case 't':
            read_type = optarg[0];
            if (type != 'y' && type != 'm' && type != 'c')
                FatalError("Unknown type of network, only y|c|m admitted");
            break;
        case ':':
            FatalError("This option needs a value");
            break;
        case '?':
            printf("You have digited: -%c\t", optopt);
            FatalError("Unknown option");
            break;
        }
    }

    // look for extra arguments (the ids of the cameras)
    std::vector<std::string> cameras_ids;
    for (; optind < argc; optind++)
        cameras_ids.push_back(argv[optind]);

    std::cout << cameras_ids.size() << " camera id(s) given: ";
    for (size_t i = 0; i < cameras_ids.size(); ++i)
        std::cout << cameras_ids[i] << " ";
    std::cout << std::endl;

    // if no parameters file given, set all default values for 1 camera
    if (params_path == "")
    {
        cameras_par.resize(1);
        cameras_par[0].id = "20936";
        cameras_par[0].framesToProcess = -1;
        cameras_par[0].portCommunicator = 8888;
        cameras_par[0].input = "../data/20936.mp4";
        cameras_par[0].pmatrixPath = "../data/pmat_new/pmat_07-03-20936_20p.txt";
        cameras_par[0].maskfilePath = "";
        cameras_par[0].cameraCalibPath = "../data/calib_cameras/20936.params";
        cameras_par[0].maskFileOrientPath = "";
        cameras_par[0].streamWidth = 960;
        cameras_par[0].streamHeight = 540;
        cameras_par[0].filterType = 0;
        cameras_par[0].show = true;
        cameras_par[0].gstreamer = false;
    }
    else
        readParamsFromYaml(params_path, cameras_ids, cameras_par, net, type, n_classes, tif_map_path);
    
    std::cout << "BBBBQ1" << std::endl;

    // if specified from command line, override parameters read from file
    if (read_net != "")
        net = read_net;
    if (read_tif_map_path != "")
        tif_map_path = read_tif_map_path;
    if (read_type != '\0')
        type = read_type;
    if (read_n_classes != 0)
        n_classes = read_n_classes;

    std::cout << "BBBBQ1" << std::endl;

    std::cout << "Input parameters file in use:\t" << params_path << std::endl;
    std::cout << "Tif map in use:\t\t\t" << tif_map_path << std::endl;
    std::cout << "Network rt to use:\t\t" << net << std::endl;
    std::cout << "Type of network in use:\t\t" << type << std::endl;
    std::cout << "Number of classes specified:\t" << n_classes << std::endl
              << std::endl;
    return true;
}

// void initializeCamerasNetworks(std::vector<edge::camera> &cameras, const std::string &net, const char type, int &n_classes)
// {
//     // if the rt file does not esits, run the test to create it
//     if (!fileExist(net.c_str()))
//     {
//         std::string test_cmd = "tkDNN/test_" + net.substr(0, net.find("_fp"));
//         std::string precision = net.substr(net.find("_fp") + 3, 2);
//         if (std::stoi(precision) == 16)
//             setenv("TKDNN_MODE", "FP16", 1);

//         if (!fileExist(test_cmd.c_str()))
//             FatalError("Wrong network, the test does not exist for tkDNN");
//         system(test_cmd.c_str());
//     }

//     if (!fileExist(net.c_str()))
//         FatalError("Problem with rt creation");

//     // assign to each camera a detector
//     for (auto &c : cameras)
//     {
//         switch (type)
//         {
//         case 'y':
//             // c.detNN = new tk::dnn::Yolo3Detection();
//             break;
//         case 'c':
//             // c.detNN = new tk::dnn::CenternetDetection();
//             break;
//         case 'm':
//             // c.detNN = new tk::dnn::MobilenetDetection();
//             n_classes++;
//             break;
//         default:
//             FatalError("Network type not allowed\n");
//         }
//         // c.detNN->init(net, n_classes, 1, 0.5);
//     }
// }

void readProjectionMatrix(const std::string &path, cv::Mat &prj_mat)
{
    std::ifstream prj_mat_file;
    prj_mat_file.open(path);

    prj_mat = cv::Mat(cv::Size(3, 3), CV_64FC1);
    double *vals = (double *)prj_mat.data;

    double number = 0;
    int i;
    for (i = 0; prj_mat_file >> number && i < prj_mat.cols * prj_mat.rows; ++i)
        vals[i] = number;

    prj_mat_file.close();

    if (i != prj_mat.cols * prj_mat.rows)
        FatalError("Problem with projection matrix file");
}

void readCalibrationMatrix(const std::string &path, cv::Mat &calib_mat, cv::Mat &dist_coeff, int &image_width, int &image_height)
{
    YAML::Node config = YAML::LoadFile(path);

    // read calibration size
    image_width = config["image_width"].as<int>();
    image_height = config["image_height"].as<int>();

    // read camera matrix
    int rows = config["camera_matrix"]["rows"].as<int>();
    int cols = config["camera_matrix"]["cols"].as<int>();

    cv::Mat calib = cv::Mat(cv::Size(rows, cols), CV_64FC1);
    double *vals = (double *)calib.data;

    for (int i = 0; i < config["camera_matrix"]["data"].size(); ++i)
        vals[i] = config["camera_matrix"]["data"][i].as<double>();

    calib_mat = calib;

    // read distortion coefficents
    rows = config["distortion_coefficients"]["rows"].as<int>();
    cols = config["distortion_coefficients"]["cols"].as<int>();

    cv::Mat coeff = cv::Mat(cv::Size(rows, cols), CV_64FC1);
    vals = (double *)coeff.data;

    for (int i = 0; i < config["distortion_coefficients"]["data"].size(); ++i)
        vals[i] = config["distortion_coefficients"]["data"][i].as<double>();

    dist_coeff = coeff;
}


void readCaches(edge::camera &cam)
{
    std::string error_mat_data_path = "../data/" + cam.id + "/caches";
    if (cam.hasCalib)
        cam.precision = cv::Mat(cv::Size(cam.calibWidth, cam.calibHeight), CV_32F, 0.0);
    else
        cam.precision = cv::Mat(cv::Size(cam.streamWidth, cam.streamHeight), CV_32F, 0.0);

    std::ifstream error_mat;
    error_mat.open(error_mat_data_path.c_str());
    if (error_mat)
    {
        for (int y = 0; y < cam.calibHeight; y++)
        { // height (number of rows)
            for (int x = 0; x < cam.calibWidth; x++)
            { // width (number of columns)
                float tmp;
                // skip first 4 values, then the 5th is precision
                for (int z = 0; z < 4; z++)
                    error_mat.read(reinterpret_cast<char *>(&tmp), sizeof(float));

                error_mat.read(reinterpret_cast<char *>(&tmp), sizeof(float));
                cam.precision.at<float>(y, x) = tmp;
            }
        }
    }
}

std::vector<edge::camera> configure(int argc, char **argv)
{
    std::vector<edge::camera_params> cameras_par;
    std::string net, tif_map_path;
    char type;
    int n_classes;

    // read args from command line
    readParameters(argc, argv, cameras_par, net, type, n_classes, tif_map_path);
    std::cout << "Parameters read" << std::endl;
    // set dataset
    edge::Dataset_t dataset;
    switch (n_classes)
    {
    case 10:
        dataset = edge::Dataset_t::BDD;
        break;
    case 80:
        dataset = edge::Dataset_t::COCO;
        break;
    default:
        FatalError("Dataset type not supported yet, check number of classes in parameter file.");
    }

    if (verbose)
    {
        for (auto cp : cameras_par)
            std::cout << cp;
    }

    // read calibration matrixes for each camera
    std::vector<edge::camera> cameras(cameras_par.size());
    for (size_t i = 0; i < cameras.size(); ++i)
    {
            std::cout << "read 1" << cameras_par[i].cameraCalibPath << std::endl;

        if (cameras_par[i].cameraCalibPath != "")
        {
            cameras[i].hasCalib = true;
                        std::cout << "read 11" << cameras_par[i].cameraCalibPath << std::endl;

            readCalibrationMatrix(cameras_par[i].cameraCalibPath,
                                  cameras[i].calibMat,
                                  cameras[i].distCoeff,
                                  cameras[i].calibWidth,
                                  cameras[i].calibHeight);
            std::cout << "read 12" << cameras_par[i].cameraCalibPath << std::endl;

        }
                    std::cout << "read 2" << std::endl;
        readProjectionMatrix(cameras_par[i].pmatrixPath, cameras[i].prjMat);
                    std::cout << "read 3" << std::endl;

        cameras[i].id = cameras_par[i].id;
        cameras[i].input = cameras_par[i].input;
        cameras[i].framesToProcess = cameras_par[i].framesToProcess;
        cameras[i].portCommunicator = cameras_par[i].portCommunicator;
                    std::cout << "read 4" << std::endl;

        cameras[i].streamWidth = cameras_par[i].streamWidth;
        cameras[i].streamHeight = cameras_par[i].streamHeight;
        cameras[i].filterType = cameras_par[i].filterType;
        cameras[i].show = cameras_par[i].show;
                    std::cout << "read 5" << std::endl;

        cameras[i].gstreamer = cameras_par[i].gstreamer;
        cameras[i].invPrjMat = cameras[i].prjMat.inv();
        cameras[i].dataset = dataset;
                    std::cout << "read 6" << std::endl;

    }

    // initialize neural netwokr for each camera
    // initializeCamerasNetworks(cameras, net, type, n_classes);

    if (verbose)
    {
        for (auto c : cameras)
            std::cout << c;
    }

    // read tif image to get georeference parameters
    double *adfGeoTransform = (double *)malloc(6 * sizeof(double));
    if (verbose)
    {
        for (int i = 0; i < 6; i++)
            std::cout << adfGeoTransform[i] << " ;;";
        std::cout << std::endl;
    }

    // for (auto &c : cameras)
    // {
    //     readCaches(c);

    //     c.adfGeoTransform = (double *)malloc(6 * sizeof(double));
    //     memcpy(c.adfGeoTransform, adfGeoTransform, 6 * sizeof(double));

    //     c.adfGeoTransform[3] = 41.365663;
    //     c.adfGeoTransform[0] = 2.134057;
    //     // initialize the geodetic converter with a point in the MASA
    //     c.geoConv.initialiseReference(c.adfGeoTransform[3], c.adfGeoTransform[0], 0);
    //     std::cout << " -- -- CAM INIT REFERENCE " << c.adfGeoTransform[3] << " " << c.adfGeoTransform[0] << " " << c.adfGeoTransform[1] << " " << c.adfGeoTransform[2] << std::endl;
    // }
    // free(adfGeoTransform);

    return cameras;
}
