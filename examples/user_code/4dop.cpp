// ---------------------THIS WAS FORKED FROM: OpenPose C++ API Tutorial - Example 12 - Custom Input, Output, and Datum ---------------------
// Asynchronous mode: ideal for fast prototyping when performance is not an issue.
// In this function, the user can implement its own way to create frames (e.g., reading his own folder of images)
// and its own way to render/display them after being processed by OpenPose.

//4D association graph dependencies
#include "skel_updater.h"
#include "skel_painter.h"
#include "kruskal_associater.h"
#include "openpose.h"
#include "camera.cpp"
#include <Eigen/Eigen>
#include <conio.h>

#pragma comment(lib, "Ws2_32.lib")
#include <thread>
#include <chrono>

// Third-party dependencies
#include <opencv2/opencv.hpp>
// Command-line user interface
#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#define OPENPOSE_FLAGS_DISABLE_DISPLAY
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>
#include <openpose/pose/poseParameters.hpp>
#include "../../src/openpose/pose/poseParameters.cpp"

// Custom OpenPose flags
//Saving images
DEFINE_bool(save_images, false,
    "Enable to save the detecton, association and reprojection images created by the 4Dasscoc algorithm");
//Using webcam or demo videos
DEFINE_bool(use_webcams, false,
    "Wether to use the webcams for live demo or use the prerecorded sample videos");
//Undistort the input images
DEFINE_bool(undist, false,
    "Wether to undistort the input images. The distortion parameters have to be given");
//Use second computer
DEFINE_bool(sharedWork, false,
    "Wether to run the program on two separate PC-s. Run ClientCode on the other machine!");


// This worker will just read and return all the basic image file formats in a directory
class UserInputClass
{
public:
    UserInputClass() :
        mClosed{ false }
    {

    }

    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> createDatum(cv::Mat image)
    {
        auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
        datumsPtr->emplace_back();
        auto& datumPtr = datumsPtr->at(0);
        datumPtr = std::make_shared<op::Datum>();

        // Fill datum
        const cv::Mat cvInputData = image;
        datumPtr->cvInputData = OP_CV2OPCONSTMAT(cvInputData);

        // If empty frame -> return nullptr
        if (datumPtr->cvInputData.empty())
        {
            op::opLog("Program finished due to empty camera frame. Closing program.",
                op::Priority::High);
            mClosed = true;
            datumsPtr = nullptr;
        }

        return datumsPtr;
    }

    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> createDatum(op::Matrix image)
    {
        auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
        datumsPtr->emplace_back();
        auto& datumPtr = datumsPtr->at(0);
        datumPtr = std::make_shared<op::Datum>();

        // Fill datum
        datumPtr->cvInputData = image;

        // If empty frame -> return nullptr
        if (datumPtr->cvInputData.empty())
        {
            op::opLog("Program finished due to empty camera frame. Closing program.",
                op::Priority::High);
            mClosed = true;
            datumsPtr = nullptr;
        }

        return datumsPtr;
    }

    bool isFinished() const
    {
        return mClosed;
    }

    void exit()
    {
        mClosed = true;
    }

private:

    bool mClosed;
};

class Server
{
public:
    Server() {};
    Server(int numRemoteCams)
    {
        stop = false;
        op::Array<float> arrayToPush;
        for (int i = 0; i < numRemoteCams; i++)
        {
            remoteKeypoints.push_back(op::Array<float>({ 1, 25, 3 }));
        }
        client = initSocket();
    }
    
    SOCKET client;
    std::vector<op::Array<float>> remoteKeypoints;
    bool stop;

    SOCKET initSocket()
    {
        WSADATA wsa_data;
        SOCKADDR_IN server_addr, client_addr;

        WSAStartup(MAKEWORD(2, 2), &wsa_data);
        const auto server = socket(AF_INET, SOCK_STREAM, 0);

        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(5555);

        ::bind(server, reinterpret_cast<SOCKADDR*>(&server_addr), sizeof(server_addr));
        listen(server, 0);

        std::cout << "Listening for incoming connections..." << std::endl;

        int client_addr_size = sizeof(client_addr);

        SOCKET client;
        client = accept(server, reinterpret_cast<SOCKADDR*>(&client_addr), &client_addr_size);

        return client;
    }

    void on_client_connect(SOCKET client, std::vector<op::Array<float>> &keypointsvec)
    {
        std::cout << "Client connected!" << std::endl;
        char buffer[20480];
        while (true)
        {
            int numcam = 0;
            auto startt = std::chrono::high_resolution_clock::now();
            int numrec = recv(client, buffer, sizeof(buffer), 0);
            if (numrec == 0)
            {
                std::cerr << "Received 0 bytes!!" << std::endl;
                break;
            }

            numcam = buffer[0] - '0';
            std::string msg(buffer);
            msg.erase(0, 1);
            op::Array<float> keypoints = msg2Array(msg);

            switch (numcam)
            {
                case 2:
                    keypointsvec[0] = keypoints;
                    break;
                case 3:
                    keypointsvec[1] = keypoints;
                    break;
                default:
                    std::cerr << "Error: client defined the name of the camera wrong!" << std::endl;
                    break;
            }

            ZeroMemory(&buffer, sizeof(buffer));
            auto stopt = std::chrono::high_resolution_clock::now();
            std::cout << "Time to convert: " << std::chrono::duration_cast<std::chrono::milliseconds>(stopt - startt).count() << std::endl;
            if (stop) break;
        }
        closesocket(client);
        WSACleanup();
        std::cout << "Client disconnected." << std::endl;
    }
private:
    op::Array<float> msg2Array(std::string msg)
    {
        if (msg.size() < 50) return op::Array<float>({ 1, 25, 3 }, 0.0);
        std::string numPeopleS = msg.substr(0, msg.find(" "));
        int numPeopleI = std::stoi(numPeopleS);
        msg.erase(0, numPeopleS.size() + 1);
        std::stringstream stream(msg);
        std::string coord;
        op::Array<float> opkeypoints({ numPeopleI, 25, 3 });

        for(int i = 0; i < numPeopleI; i++)
        {
            for (int j = 0; j < 25; j++)
            {
                for (int k = 0; k < 3; k++)
                {
                    stream >> coord;
                    opkeypoints[{i, j, k}] = std::stof(coord);
                }
            }
        }
        return opkeypoints;
    }
};

void configureWrapper(op::WrapperT<op::Datum>& opWrapperT)
{
    try
    {
        // Configuring OpenPose

        // logging_level
        op::checkBool(
            0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
            __LINE__, __FUNCTION__, __FILE__);
        op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
        op::Profiler::setDefaultX(FLAGS_profile_speed);

        // Applying user defined configuration - GFlags to program variables
        // outputSize
        const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
        // netInputSize
        const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
        // faceNetInputSize
        const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
        // handNetInputSize
        const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
        // poseMode
        const auto poseMode = op::flagsToPoseMode(FLAGS_body);
        // poseModel
        const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
        // JSON saving
        if (!FLAGS_write_keypoint.empty())
            op::opLog(
                "Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json`"
                " instead.", op::Priority::Max);
        // keypointScaleMode
        const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
        // heatmaps to add
        const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
            FLAGS_heatmaps_add_PAFs);
        const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
        // >1 camera view?
        const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
        // Face and hand detectors
        const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
        const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
        // Enabling Google Logging
        const bool enableGoogleLogging = true;

        // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
        const op::WrapperStructPose wrapperStructPose{
            poseMode, netInputSize, FLAGS_net_resolution_dynamic, outputSize, keypointScaleMode, FLAGS_num_gpu,
            FLAGS_num_gpu_start, FLAGS_scale_number, (float)FLAGS_scale_gap,
            op::flagsToRenderMode(FLAGS_render_pose, multipleView), poseModel, !FLAGS_disable_blending,
            (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap, FLAGS_part_to_show, op::String(FLAGS_model_folder),
            heatMapTypes, heatMapScaleMode, FLAGS_part_candidates, (float)FLAGS_render_threshold,
            FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max, op::String(FLAGS_prototxt_path),
            op::String(FLAGS_caffemodel_path), (float)FLAGS_upsampling_ratio, enableGoogleLogging };
        opWrapperT.configure(wrapperStructPose);
        // Face configuration (use op::WrapperStructFace{} to disable it)
        const op::WrapperStructFace wrapperStructFace{
            FLAGS_face, faceDetector, faceNetInputSize,
            op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
            (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold };
        opWrapperT.configure(wrapperStructFace);
        // Hand configuration (use op::WrapperStructHand{} to disable it)
        const op::WrapperStructHand wrapperStructHand{
            FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
            op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
            (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold };
        opWrapperT.configure(wrapperStructHand);
        // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
        const op::WrapperStructExtra wrapperStructExtra{
            FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads };
        opWrapperT.configure(wrapperStructExtra);
        // Output (comment or use default argument to disable any output)
        const op::WrapperStructOutput wrapperStructOutput{
            FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
            op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
            FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
            op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
            op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
            op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
            op::String(FLAGS_udp_port) };
        opWrapperT.configure(wrapperStructOutput);
        // No GUI. Equivalent to: opWrapper.configure(op::WrapperStructGui{});
        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            opWrapperT.disableMultiThreading();
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

//Convert the OpenPose keypoint structure for the 4D association algorithm ([person, joint, xyscore] --> [joint, xyscore, person]) 
std::vector<Eigen::Matrix3Xf> convertOPtoEigenFullFrame(op::Array<float>& opOutput) {
    std::vector<Eigen::Matrix3Xf> eigenKeypointsVector(GetSkelDef(BODY25).jointSize);
    if (!opOutput.empty())
    {
        for (int jointID = 0; jointID < opOutput.getSize()[1]; jointID++)
        {
            Eigen::Matrix3Xf jointOfPeople(3, 0);

            for (int personID = 0; personID < opOutput.getSize()[0]; personID++)
            {
                if (opOutput[{personID, jointID, 0}] != 0)
                {
                    jointOfPeople.conservativeResize(Eigen::NoChange, jointOfPeople.cols() + 1);

                    for (int xyscore = 0; xyscore < opOutput.getSize()[2]; xyscore++)
                    {
                        jointOfPeople(xyscore, jointOfPeople.cols() - 1) = opOutput[{personID, jointID, xyscore}];
                    }
                }
            }
            eigenKeypointsVector[jointID] = jointOfPeople;
        }
    }
    return eigenKeypointsVector;
}

std::vector<Eigen::Matrix3Xf> convertOPCandidatesToEigenFullFrame(std::vector<std::vector<std::array<float, 3Ui64>>>& opOutput)
{
    std::vector<Eigen::Matrix3Xf> eigenKeypointsVector(GetSkelDef(BODY25).jointSize);
    if (!opOutput.empty())
    {
        for (int jID = 0; jID < opOutput.size(); jID++)
        {
            Eigen::Matrix3Xf jointOfPeople(3, 0);

            for (int personID = 0; personID < opOutput[jID].size(); personID++)
            {
                jointOfPeople.conservativeResize(Eigen::NoChange, jointOfPeople.cols() + 1);
                jointOfPeople.col(personID) = Eigen::Vector3f(opOutput[jID][personID].data());
            }
            eigenKeypointsVector[jID] = jointOfPeople;
        }
    }
    return eigenKeypointsVector;
}

//Returns a vector with the indexes of the people, whose given joint (jointID) was found
std::vector<int> validPersonOfJoint(op::Array<float>& opOutput, int jointID)
{
    std::vector<int> validJoints;
    for (int i = 0; i < opOutput.getSize(0); i++)
    {
        if (opOutput[{i, jointID, 0}] != (float)0.0)
        {
            validJoints.emplace_back(i);
        }
    }
    return validJoints;
}

//Joint matrix has rows (A joints) and jolumns (B joints). Each element represents a connection between the given A and B joints. The value is bigger, if the two given links have high chance, that they form a real link.
//This should be extracted from the OpenPose system before it assembles the keypoints and the heatmaps into people. Instead, we use the assembled people by OpenPose and generate "fake" matrixes (PAFs) for the 4D association algorithm.
std::vector<Eigen::MatrixXf> createFakePafsFullFrame(op::Array<float>& opOutput, SkelDef& def) {
    //Each element of the vector is a matrix that shows where that specific link is (for examle: 0 1 0 row tells us(this is the first row), that the link this matrix represents is given between the first found joint A and the second found joint B)
    std::vector<Eigen::MatrixXf> eigenPafVector(GetSkelDef(BODY25).pafSize);

    if (!opOutput.empty())
    {
        for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
            Eigen::MatrixXf eigenPafOfPair;
            const int jAIdx = def.pafDict(0, pafIdx);
            const int jBIdx = def.pafDict(1, pafIdx);
            std::vector<int> validPeopleOfJointA = validPersonOfJoint(opOutput, jAIdx);
            std::vector<int> validPeopleOfJointB = validPersonOfJoint(opOutput, jBIdx);

            eigenPafOfPair.resize(validPeopleOfJointA.size(), validPeopleOfJointB.size());
            for (int i = 0; i < eigenPafOfPair.rows(); i++) {
                for (int j = 0; j < eigenPafOfPair.cols(); j++) {
                    if (validPeopleOfJointA[i] == validPeopleOfJointB[j])
                        eigenPafOfPair(i, j) = (float)1.0;
                    else eigenPafOfPair(i, j) = (float)0.0;
                }
            }
            eigenPafVector[pafIdx] = eigenPafOfPair;
        }
    }

    return eigenPafVector;
}

std::vector<int> intArange(float start, float stop, float step = 1) {
    std::vector<int> values;
    if (start < stop)
        for (float value = start; value < stop; value += step)
            values.push_back((int)(value + 0.5));
    else
        for (float value = start; value > stop; value += step)
            values.push_back((int)(value + 0.5));
    return values;
}

float calcScoreForPafMatrix(float x1, float y1, float x2, float y2, cv::Mat pafMatX, cv::Mat pafMatY)
{
    int num_iter = 10;
    float dx = x2 - x1;
    float dy = y2 - y1;
    float normVec = std::sqrt(dx * dx + dy * dy);

    if (normVec < 1e-4)
        return 0.0;

    float vx = dx / normVec;
    float vy = dy / normVec;

    std::vector<int> xs;
    std::vector<int> ys;

    if ((int)x1 != (int)x2)
        xs = intArange(x1, x2, dx / (float)num_iter);
    else
        for (int i = 0; i < num_iter; i++)
            xs.push_back((int)(x1 + 0.5));

    if ((int)y1 != (int)y2)
        ys = intArange(y1, y2, dy / (float)num_iter);
    else
        for (int i = 0; i < num_iter; i++)
            ys.push_back((int)(y1 + 0.5));

    std::vector<float> pafXs(num_iter);
    std::vector<float> pafYs(num_iter);

    for (int i = 0; i < pafXs.size(); i++)
    {
        if ((ys[i] > pafMatX.rows) | (ys[i] > pafMatY.rows) | (xs[i] > pafMatX.cols) | (xs[i] > pafMatY.cols))
        {
            std::cout << "ERROR: index of heatmap array is out of bounds!!!!!!!!!!!" << std::endl;
            std::cout << "pafMatX.rows: " << pafMatX.rows << std::endl;
            std::cout << "pafMatX.cols: " << pafMatX.cols << std::endl;
            std::cout << "pafMatY.rows: " << pafMatY.rows << std::endl;
            std::cout << "pafMatY.cols: " << pafMatY.cols << std::endl;
            std::cout << "ys[i]: " << ys[i] << std::endl;
            std::cout << "xs[i]: " << xs[i] << std::endl;
        }
        pafXs[i] = pafMatX.at<float>(ys[i], xs[i]);
        pafYs[i] = pafMatY.at<float>(ys[i], xs[i]);
    }

    float localscoreX = 0;
    float localscoreY = 0;

    for (int i = 0; i < pafXs.size(); ++i)
        localscoreX += pafXs[i] * vx;
    for (int i = 0; i < pafYs.size(); ++i)
        localscoreY += pafYs[i] * vy;

    float finalscore = ((localscoreX + localscoreY) / num_iter > 0.1) ? ((localscoreX + localscoreY) / num_iter) : 0.0;

    return finalscore;
}

std::vector<Eigen::MatrixXf> createPafsFullFrame(std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datum, SkelDef& def, float scale) {
    //Each element of the vector is a matrix that shows where that specific link is (for examle: 0 1 0 row tells us(this is the first row), that the link this matrix represents is given between the first found joint A and the second found joint B)
    std::vector<Eigen::MatrixXf> eigenPafVector(GetSkelDef(BODY25).pafSize);
    auto& poseCandidates = datum->at(0)->poseCandidates;
    auto& pafHeatMaps = datum->at(0)->poseHeatMaps;
    const auto numberChannels = pafHeatMaps.getSize(0);
    const auto height = pafHeatMaps.getSize(1);
    const auto width = pafHeatMaps.getSize(2);

    cv::Mat desiredChannelHeatMapX;
    cv::Mat desiredChannelHeatMapY;

    if (!poseCandidates.empty())
    {
        for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
            Eigen::MatrixXf eigenPafOfPair;
            const int jAIdx = def.pafDict(0, pafIdx);
            const int jBIdx = def.pafDict(1, pafIdx);

            eigenPafOfPair.resize(poseCandidates[jAIdx].size(), poseCandidates[jBIdx].size());
            for (int i = 0; i < eigenPafOfPair.rows(); i++) {
                for (int j = 0; j < eigenPafOfPair.cols(); j++) {
                    auto& xHeatMap = cv::Mat(height, width, CV_32F, &pafHeatMaps.getPtr()[(2 * pafIdx) % numberChannels * height * width]);
                    auto& yHeatMap = cv::Mat(height, width, CV_32F, &pafHeatMaps.getPtr()[(2 * pafIdx + 1) % numberChannels * height * width]);

                    eigenPafOfPair(i, j) = calcScoreForPafMatrix(poseCandidates[jAIdx][i][0] / scale, poseCandidates[jAIdx][i][1] / scale, poseCandidates[jBIdx][j][0] / scale, poseCandidates[jBIdx][j][1] / scale, xHeatMap, yHeatMap);
                }
            }
            eigenPafVector[pafIdx] = eigenPafOfPair;
        }
    }

    return eigenPafVector;
}






int PoseDetection3D()
{
    try
    {


        op::opLog("Starting OpenPose demo...", op::Priority::High);
        const auto opTimer = op::getTimerInit();

        // Configuring OpenPose
        op::opLog("Configuring OpenPose...", op::Priority::High);
        op::WrapperT<op::Datum> opWrapperT{ op::ThreadManagerMode::Asynchronous };
        configureWrapper(opWrapperT);

        // Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
        op::opLog("Starting thread(s)...", op::Priority::High);
        opWrapperT.start();



        std::string dataset;     // data/*dataset* mappa neve 
        dataset = (!FLAGS_use_webcams) ? "mvmp" : "live";
        std::map<std::string, Camera> cameras = ParseCameras("C:/Users/admin/Documents/EasyMocap/data/" + dataset + "/calibration.json"); //dataset mappában a calibrációs json fájl
        Eigen::Matrix3Xf projs(3, cameras.size() * 4);
        std::vector<cv::Mat> rawImgs(cameras.size());
        std::vector<op::Matrix> rawImgsOpMat(cameras.size());
        std::vector<cv::VideoCapture> videos(cameras.size());  //annyi videót vár, ahány kamera van
        std::vector<op::IpCameraReader*> streams(cameras.size());
        std::vector<std::vector<OpenposeDetection>> seqDetections(cameras.size());
        const SkelDef& skelDef = GetSkelDef(SKEL19);
        std::vector<std::map<int, Eigen::Matrix4Xf>> skels; //A vektor minden eleme egy frame skeletonjait tartalmazza

        int numLocalCam = 2;
        Server server;
        std::thread t2;
        std::vector<std::string> ips{ "169.254.150.101","169.254.150.102","169.254.150.103","169.254.150.104" };
        std::vector<cv::Mat> map1s(cameras.size());
        std::vector<cv::Mat> map2s(cameras.size());
        if (FLAGS_undist)
        {
            for (auto const& cam : cameras)
            {
                std::cout << "Dist coeff of cam " << cam.first << ": " << cam.second.distCoeff << std::endl;
                std::cout << "Intri matrix of cam " << cam.first << ": " << cam.second.originK << std::endl;
                std::cout << "Img size of cam  " << cam.first << ": " << cameras.begin()->second.imgSize << std::endl;
                cv::initUndistortRectifyMap(cam.second.originK, cam.second.distCoeff, cv::Mat(), cam.second.originK, cameras.begin()->second.imgSize, CV_32FC1, map1s[std::stoi(cam.first)], map2s[std::stoi(cam.first)]);
            }
        }

#pragma omp parallel for
        for (int i = 0; i < cameras.size(); i++) {

            auto iter = std::next(cameras.begin(), i);

            if (!FLAGS_use_webcams)
            {
                videos[i] = cv::VideoCapture("C:/Users/admin/Documents/EasyMocap/data/" + dataset + "/video/" + iter->first + ".mp4"); //egy videó beolvasás
                videos[i].set(cv::CAP_PROP_POS_FRAMES, 0); //elsõ frame-el kezdjük
                cv::Size imgSize(int(videos[i].get(cv::CAP_PROP_FRAME_WIDTH)), int(videos[i].get(cv::CAP_PROP_FRAME_HEIGHT)));
                projs.middleCols(4 * i, 4) = iter->second.eiProj;
                rawImgs[i].create(imgSize, CV_8UC3);   //üres képkockák létrehozása a videó méretével
                if (!videos[i].isOpened())
                {
                    std::cout << "Camera " << i << " hasn't been opened." << std::endl;
                }
            }
            else
            {
                //videos[i] = cv::VideoCapture(i); //egy webcam beolvasás
                //streams[i] = new op::WebcamReader(0, op::Point<int>{1280, 720}); //egy webcam beolvasás
                streams[i] = new op::IpCameraReader("http://admin:admin123@" + ips[i] + "/cgi-bin/mjpg/video.cgi?channel=1&subtype=1", op::Point<int>{640, 480});
                //streams[i] = new op::IpCameraReader("rtsp://admin:admin123@192.168.1.108", op::Point<int>{1280, 720});
                //videos[i] = cv::VideoCapture("rtsp://admin:admin123@192.168.1.108"); //egy ipcam beolvasás
                cv::Size imgSize(int(streams[i]->get(cv::CAP_PROP_FRAME_WIDTH)), int(streams[i]->get(cv::CAP_PROP_FRAME_HEIGHT)));
                rawImgsOpMat[i] = op::Matrix(imgSize.width, imgSize.height, CV_8UC3);
                projs.middleCols(4 * i, 4) = iter->second.eiProj;
                rawImgs[i].create(imgSize, CV_8UC3);   //üres képkockák létrehozása a videó méretével

                //std::cout << "The stream image size is: " << int(videos[i].get(cv::CAP_PROP_FRAME_WIDTH)) << "×" << int(videos[i].get(cv::CAP_PROP_FRAME_HEIGHT)) << std::endl;
                std::cout << "The stream image size is: " << imgSize.width << "×" << imgSize.height << std::endl;
                if (!streams[i]->isOpened())
                {
                    std::cout << "Camera " << i << " hasn't been opened." << std::endl;
                }
            }
        }

        UserInputClass userInputClass;
        bool userWantsToExit = false;
        std::vector<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>> datums(cameras.size());
        SkelDef def = GetSkelDef(SkelType(BODY25));
        std::vector<OpenposeDetection> frameDetections(cameras.size(), BODY25);
        const float IOScale = (float)rawImgs[0].rows / std::stof(FLAGS_net_resolution.substr(FLAGS_net_resolution.find('x') + 1));

        //paraméterek beállítása
        KruskalAssociater associater(SKEL19, cameras);
        associater.SetMaxTempDist(0.3f);
        associater.SetMaxEpiDist(0.15f);
        associater.SetEpiWeight(1.f);
        associater.SetTempWeight(0.2f);
        associater.SetViewWeight(1.f);
        associater.SetPafWeight(2.f);
        associater.SetHierWeight(1.f);
        associater.SetViewCntWelsh(1.0);
        associater.SetMinCheckCnt(10);
        associater.SetNodeMultiplex(true);
        associater.SetNormalizeEdge(true);			// new feature

        /*associater.SetMaxTempDist(0.3f);
        associater.SetMaxEpiDist(0.2f);
        associater.SetEpiWeight(2.f);
        associater.SetTempWeight(2.f);
        associater.SetViewWeight(1.f);
        associater.SetPafWeight(1.f);
        associater.SetHierWeight(2.f);
        associater.SetViewCntWelsh(1.0);
        associater.SetMinCheckCnt(20);
        associater.SetNodeMultiplex(true);
        associater.SetNormalizeEdge(true);*/

        SkelPainter skelPainter(SKEL19);
        skelPainter.rate = 512.f / float(cameras.begin()->second.imgSize.width);
        SkelFittingUpdater skelUpdater(SKEL19, "C:/Users/admin/Documents/openpose/src4D/SKEL19_new");
        skelUpdater.SetTemporalTransTerm(1e-1f / std::powf(skelPainter.rate, 2));
        skelUpdater.SetTemporalPoseTerm(1e-1f / std::powf(skelPainter.rate, 2));


        if (FLAGS_sharedWork)
        {
            server = Server(cameras.size() - numLocalCam);
            if ((server.client != INVALID_SOCKET))
            {
                t2 = std::thread(&Server::on_client_connect, &server , server.client, std::ref(server.remoteKeypoints));
            }
        }

        auto startp = std::chrono::high_resolution_clock::now();
        int framecount = 0;

        while (!userInputClass.isFinished()) {
#pragma omp parallel for
            for (int i = 0; i < cameras.size(); i++)
            {
                if (!FLAGS_use_webcams)
                {
                    videos[i] >> rawImgs[i];
                    if (FLAGS_undist) cv::remap(rawImgs[i], rawImgs[i], map1s[i], map2s[i], cv::INTER_LINEAR);
                }
                else
                {
                    rawImgsOpMat[i] = streams[i]->getFrame();
                    if (FLAGS_undist) cv::remap(OP_OP2CVMAT(rawImgsOpMat[i]), OP_OP2CVMAT(rawImgsOpMat[i]), map1s[i], map2s[i], cv::INTER_LINEAR);
                    rawImgs[i] = OP_OP2CVMAT(rawImgsOpMat[i]);
                }
            }

            if(!FLAGS_sharedWork)
            {
                for (int i = 0; i < cameras.size(); i++)
                {
                    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumToProcess;
                    datumToProcess = (!FLAGS_use_webcams) ? userInputClass.createDatum(rawImgs[i]) : datumToProcess = userInputClass.createDatum(rawImgsOpMat[i]);

                    if (datumToProcess != nullptr)
                    {
                        auto successfullyEmplaced = opWrapperT.waitAndEmplace(datumToProcess);
                        std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumProcessed;
                        auto successfullyPopped = opWrapperT.waitAndPop(datumProcessed);
                        if (successfullyEmplaced && successfullyPopped)
                        {
                            frameDetections[i].joints = (FLAGS_part_candidates) ? convertOPCandidatesToEigenFullFrame(datumProcessed->at(0)->poseCandidates) : convertOPtoEigenFullFrame(datumProcessed->at(0)->poseKeypoints);
                            frameDetections[i].pafs = (FLAGS_part_candidates) ? createPafsFullFrame(datumProcessed, def, IOScale) : createFakePafsFullFrame(datumProcessed->at(0)->poseKeypoints, def);

                            cv::resize(rawImgs[i], rawImgs[i], cv::Size(), skelPainter.rate, skelPainter.rate);
                            auto& mappedDetection = frameDetections[i].Mapping(SKEL19);
                            associater.SetDetection(i, mappedDetection); //Associater osztály m_detections változójában tárolja a detectiont
                        }
                        else
                        {
                            op::opLog("Processed datum could not be emplaced.", op::Priority::High);
                        }
                    }
                }
            } //end if(!sharedWork)
            else
            {
                for (int i = 0; i < cameras.size(); i++)
                {
                    if (i < numLocalCam)
                    {
                        std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumToProcess;
                        datumToProcess = (!FLAGS_use_webcams) ? userInputClass.createDatum(rawImgs[i]) : datumToProcess = userInputClass.createDatum(rawImgsOpMat[i]);
                        if (datumToProcess != nullptr)
                        {
                            auto successfullyEmplaced = opWrapperT.waitAndEmplace(datumToProcess);
                            std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumProcessed;
                            auto successfullyPopped = opWrapperT.waitAndPop(datumProcessed);
                            if (successfullyEmplaced && successfullyPopped)
                            {
                                frameDetections[i].joints = (FLAGS_part_candidates) ? convertOPCandidatesToEigenFullFrame(datumProcessed->at(0)->poseCandidates) : convertOPtoEigenFullFrame(datumProcessed->at(0)->poseKeypoints);
                                frameDetections[i].pafs = (FLAGS_part_candidates) ? createPafsFullFrame(datumProcessed, def, IOScale) : createFakePafsFullFrame(datumProcessed->at(0)->poseKeypoints, def);
                            }
                            else
                            {
                                op::opLog("Processed datum could not be emplaced.", op::Priority::High);
                            }
                        }
                    }
                    else
                    {
                        frameDetections[i].joints = convertOPtoEigenFullFrame(server.remoteKeypoints[i-numLocalCam]);
                        frameDetections[i].pafs = createFakePafsFullFrame(server.remoteKeypoints[i-numLocalCam], def);
                    }
                    cv::resize(rawImgs[i], rawImgs[i], cv::Size(), skelPainter.rate, skelPainter.rate);
                    auto& mappedDetection = frameDetections[i].Mapping(SKEL19);
                    associater.SetDetection(i, mappedDetection);
                }
            }
            

            if (!userInputClass.isFinished() && !frameDetections.empty())
            {
                associater.SetSkels3dPrev(skelUpdater.GetSkel3d());   //egy framenek az összes kamera detectionbõl (SetDetection) elõállítjuk a 3D skeletont 
                associater.Associate(); //kiszámoljuk a 3D skeleton paramétereit
                skelUpdater.Update(associater.GetSkels2d(), projs);   //Updateljük az eddig meghatározott skeletonokat az újonnan meghatározottakkal.
                skels.emplace_back(skelUpdater.GetSkel3d()); //a 3D skeleton adatinak visszaírása mátrixba
            }

            //Saving images of detection, association and reprojection of the assembled 3D skeleton
            if (FLAGS_save_images && !userInputClass.isFinished()) {
                std::cout << "Saving images" << std::endl;
                cv::Mat detectImg, assocImg, reprojImg;
                const int layoutCols = 3; 
                std::vector<cv::Rect> rois = SkelPainter::MergeImgs(rawImgs, detectImg, layoutCols,
                    { rawImgs.begin()->cols, rawImgs.begin()->rows }); //az összes kamerakép összeillesztése egy gridbe (detectImg), erre rajzoljuk majd a végeredményt. Rois: A grid kis képeinek befoglaló téglalapjainak koordinátái (nagy kép almátrixa)
                detectImg.copyTo(assocImg);
                detectImg.copyTo(reprojImg);

#pragma omp parallel for
                for (int view = 0; view < cameras.size(); view++) {   //a vissza projektált skeletonok visszarajzolása az egyes képekre (még mindig frame-enként)
                    const OpenposeDetection detection = frameDetections[view].Mapping(SKEL19);
                    skelPainter.DrawDetect(detection.joints, detection.pafs, detectImg(rois[view]));
                    for (const auto& skel2d : associater.GetSkels2d())
                        skelPainter.DrawAssoc(skel2d.second.middleCols(view * skelDef.jointSize, skelDef.jointSize), assocImg(rois[view]), skel2d.first);

                    for (const auto& skel3d : skelUpdater.GetSkel3d())
                        skelPainter.DrawReproj(skel3d.second, projs.middleCols(4 * view, 4), reprojImg(rois[view]), skel3d.first);
                }

                //a frame kimentése képbe
                //cv::imwrite("output/detect93u/" + std::to_string(framecount) + ".jpg", detectImg);
                //cv::imwrite("output/assoc93u/" + std::to_string(framecount) + ".jpg", assocImg);
                //cv::imwrite("output/reproj95u_shared/" + std::to_string(framecount) + ".jpg", reprojImg);
                cv::imshow("reproj", reprojImg);
                cv::imshow("detect", detectImg);
                cv::waitKey(1);
            }

            std::cout << "------ End of processing frame " << std::to_string(framecount) << " ------" << std::endl;


            framecount++;

            if (kbhit() | framecount >= 300)
            {
                if (FLAGS_sharedWork)
                {
                    server.stop = true;
                    t2.join();
                }
                break;
            }

        }//end while
        auto stopp = std::chrono::high_resolution_clock::now();
        int duration = (int)std::chrono::duration_cast<std::chrono::seconds>(stopp - startp).count();
        std::cout << "Time to finish program: " << duration << "s" << std::endl;
        std::cout << "Average FPS: " << std::to_string((float)framecount / duration) << " frames/s" << std::endl;
        SerializeSkels(skels, "output/skel.txt"); //a 3D skeleton kiírása         

        op::opLog("Stopping thread(s)", op::Priority::High);
        opWrapperT.stop();

        // Measuring total time
        //op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);

        //Config:
        // System GPU: 1× NVidia GTX 1650
        // dataset: mvmp
        // net_resolution: -1×176
        // Assoc. algorithm: OP
        // 
        //Performance:
        // With image saving: ~1.8 fps
        // Without image saving: ~2.05 fps (300frame/149s) 

        //Config:
        // System GPU: 1× NVidia GTX 1650
        // dataset: mvmp
        // net_resolution: -1×176
        // Assoc. algorithm: 4D assoc
        // 
        //Performance:
        // With image saving: ~ fps
        // Without image saving: ~1.997 fps (300frame/151.7s)

        //Config:
        // System GPU: 1× NVidia GTX 1650
        // dataset: shelf
        // net_resolution: -1×288
        // Assoc. algorithm: OP
        // 
        //Performance:
        // Without image saving: ~1.35 fps (300frame/222s)

        //Config:
        // System GPU: 1× NVidia GTX 1650
        // dataset: shelf
        // net_resolution: -1×288
        // Assoc. algorithm: 4D assoc
        // 
        //Performance:
        // Without image saving: ~1.19 fps (300frame/ 252s)

        //Config:
        // System GPU: 1× NVidia RTX 2060
        // dataset: live
        // net_resolution: -1×176
        // Assoc. algorithm: OP
        // 
        //Performance:
        // Without image saving: ~3.2 fps (300frame/ 93s)

        //Config:
        // System GPU: 1× NVidia RTX 2060
        // dataset: live
        // net_resolution: -1×176
        // Assoc. algorithm: 4D assoc
        // 
        //Performance:
        // Without image saving: ~3.2 fps (300frame/ 94s)

        //Config:
        // System GPU: 1× NVidia RTX 2060
        // dataset: mvmp
        // net_resolution: -1×176
        // Assoc. algorithm: 4D assoc
        // 
        //Performance:
        // Without image saving: ~2.4 fps (300frame/ 124s)

        //Config:
        // System GPU: 2× NVidia RTX 2060 (shared work)
        // dataset: live
        // net_resolution: -1×176
        // Assoc. algorithm: OP
        // 
        //Performance:
        // Without image saving: ~6.52 fps (300frame/ 46s)

        return 0;
    }
    catch (const std::exception&)
    {
        return -1;
    }
}

int main(int argc, char* argv[])
{
    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_net_resolution = "-1x176";
    FLAGS_part_candidates = false; //if set to true -> FLAGS_heatmaps_add_PAFs needed

    FLAGS_part_to_show = 0;
    FLAGS_heatmaps_add_PAFs = false;
    FLAGS_heatmaps_add_bkg = false;
    FLAGS_heatmaps_add_parts = false;
    FLAGS_heatmaps_scale = 0; //0 --> in range [-1, 1] ; 1 --> in range [0, 1] ; 2(default) --> in range [0 , 255]

    FLAGS_save_images = true;
    FLAGS_use_webcams = true;
    FLAGS_undist = true;

    FLAGS_sharedWork = false;


    // Running PoseDetection3D
    return PoseDetection3D();
}
