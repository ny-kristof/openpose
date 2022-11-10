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
// Producer
//DEFINE_string(image_dir,                "examples/media2/",
//    "Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).");
// Display
DEFINE_bool(no_display, false,
    "To disable visualization.");
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
        //remoteKeypoints.resize(numRemoteCams);
        std::cout << "remotekeypoints size: " << remoteKeypoints.size() << std::endl;
        op::Array<float> arrayToPush;
        for (int i = 0; i < numRemoteCams; i++)
        {
            remoteKeypoints.push_back(op::Array<float>({ 1, 25, 3 }));
        }
        std::cout << "remotekeypoints size: " << remoteKeypoints.size() << std::endl;
        std::cout << "opMat Keypoints size 0: " << remoteKeypoints[0].getSize(0) << std::endl;
        std::cout << "opMat Keypoints size 1: " << remoteKeypoints[0].getSize(1) << std::endl;
        std::cout << "opMat Keypoints size 2: " << remoteKeypoints[0].getSize(2) << std::endl;
        client = initSocket();
        /*if ((client != INVALID_SOCKET))
        {
            t1 = std::thread(&on_client_connect, client, std::ref(remoteKeypoints));
        }*/
    }
    
    SOCKET client;
    std::vector<op::Array<float>> remoteKeypoints;
    bool stop;
    //std::thread t1;

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

//private:
    void on_client_connect(SOCKET client, std::vector<op::Array<float>> &keypointsvec)
    {
        std::cout << "Client connected!" << std::endl;
        char buffer[20480]; //ennyi karaktert olvas ki a szerverről
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
            std::cout << "Client says: " << buffer << std::endl;

            numcam = buffer[0] - '0';
            std::string msg(buffer);
            msg.erase(0, 1);
            std::cout << "start convert" << std::endl;
            //cv::Mat keypoints = msgToCVMat(msg);
            op::Array<float> keypoints = msgToCVMat2(msg);
            std::cout << "finished convert" << std::endl;

            /*std::cout << "cvMat Keypoints of cam" << numcam << ": " << std::endl;
            for (int i = 0; i < keypoints.getSize(0); i++)
            {
                for (int j = 0; j < keypoints.getSize(1); j++)
                {
                    for (int k = 0; k < keypoints.getSize(2); k++)
                    {
                        std::cout << keypoints[{i, j, k}] << " ";
                    }
                    std::cout << std::endl;
                }
                std::cout << std::endl;
            }*/

            /*std::cout << "cvMat Keypoints size 0: " << keypoints.size[0]  << std::endl;
            std::cout << "cvMat Keypoints size 1: " << keypoints.size[1]  << std::endl;
            std::cout << "cvMat Keypoints size 2: " << keypoints.size[2]  << std::endl;*/
            //std::cout << "opMat Keypoints size : " << keypointsvec[0].printSize() << std::endl;


            switch (numcam)
            {
                case 2:
                    //keypointsvec[0].setFrom(OP_CV2OPCONSTMAT(keypoints));
                    keypointsvec[0] = keypoints;
                    break;
                case 3:
                    //keypointsvec[1].setFrom(OP_CV2OPCONSTMAT(keypoints));
                    keypointsvec[1] = keypoints;
                    break;
                default:
                    std::cerr << "Error: client defined the name of the camera wrong!" << std::endl;
                    break;
            }

           

            std::cout << "opMat Keypoints of cam" << numcam << ": " << std::endl;
            for (int i = 0; i < keypointsvec[numcam-2].getSize(0); i++)
            {
                for (int j = 0; j < keypointsvec[numcam - 2].getSize(1); j++)
                {
                    for (int k = 0; k < keypointsvec[numcam - 2].getSize(2); k++)
                    {
                        std::cout << keypointsvec[numcam - 2].at({ i, j, k }) << " ";
                    }
                    std::cout << std::endl;
                }
                std::cout << std::endl;
            }

            //cout << "Last score of person 0: " << keypoints.at<float>(1, 0, 0) << endl;
            //readnum = atoi(buffer);
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
    op::Array<float> msgToCVMat2(std::string msg)
    {
        if (msg.size() < 50) return op::Array<float>({ 1, 25, 3 }, 0.0);
        //TODO: írd meg getlineal --> mégse, átbasszuk a streamet
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
                    /*std::cout << "numPeopleI: " << numPeopleI << std::endl;
                    std::cout << "numPeopleS: " << numPeopleS << std::endl;
                    std::cout << stream.str().length() << std::endl;*/
                    stream >> coord;
                    /*msg.erase(0, coord.size() + 1);
                    stream.str(msg);*/
                    opkeypoints[{i, j, k}] = std::stof(coord);
                }
            }
        }
        return opkeypoints;
    }
};
//bool Server::stop = false;


// This worker will just read and return all the jpg files in a directory
class UserOutputClass
{
public:
    bool display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
    {
        try
        {
            // User's displaying/saving/other processing here
                // datumPtr->cvOutputData: rendered frame with pose or heatmaps
                // datumPtr->poseKeypoints: Array<float> with the estimated pose
            if (datumsPtr != nullptr && !datumsPtr->empty())
            {
                // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
                const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
                if (!cvMat.empty())
                {
                    cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
                    cv::waitKey(0);
                }
                else
                    op::opLog("Empty cv::Mat as output.", op::Priority::High);
            }
            else
                op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
            if ((char)27 == (char)cv::waitKey(1))
                return true;
            return false;
        }
        catch (const std::exception& e)
        {
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return true;
        }
    }

    bool displayHeatmap(
        const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr, const int desiredChannel = 0)
    {
        try
        {
            if (datumsPtr != nullptr && !datumsPtr->empty())
            {
                // Note: Heatmaps are in net_resolution size, which does not necessarily match the final image size
                // Read heatmaps
                auto& poseHeatMaps = datumsPtr->at(0)->poseHeatMaps;
                // Read desired channel
                const auto numberChannels = poseHeatMaps.getSize(0);
                const auto height = poseHeatMaps.getSize(1);
                const auto width = poseHeatMaps.getSize(2);
                const cv::Mat desiredChannelHeatMap(
                    height, width, CV_32F, &poseHeatMaps.getPtr()[desiredChannel % numberChannels * height * width]);
                // Read image used from OpenPose body network (same resolution than heatmaps)
                auto& inputNetData = datumsPtr->at(0)->inputNetData[0];
                const cv::Mat inputNetDataB(
                    height, width, CV_32F, &inputNetData.getPtr()[0]);
                const cv::Mat inputNetDataG(
                    height, width, CV_32F, &inputNetData.getPtr()[height * width]);
                const cv::Mat inputNetDataR(
                    height, width, CV_32F, &inputNetData.getPtr()[2 * height * width]);
                cv::Mat netInputImage;
                cv::merge(std::vector<cv::Mat>{inputNetDataB, inputNetDataG, inputNetDataR}, netInputImage);
                netInputImage = (netInputImage + 0.5) * 255;
                // Turn into uint8 cv::Mat
                cv::Mat netInputImageUint8;
                netInputImage.convertTo(netInputImageUint8, CV_8UC1);
                cv::Mat desiredChannelHeatMapUint8;
                desiredChannelHeatMap.convertTo(desiredChannelHeatMapUint8, CV_8UC1);
                // Combining both images
                cv::Mat imageToRender;
                cv::applyColorMap(desiredChannelHeatMapUint8, desiredChannelHeatMapUint8, cv::COLORMAP_JET);
                cv::addWeighted(netInputImageUint8, 0.5, desiredChannelHeatMapUint8, 0.5, 0., imageToRender);
                SkelDef def = GetSkelDef(SkelType(BODY25));
                // Display image
                if (!imageToRender.empty())
                {
                    std::string xory = (desiredChannel % 2 == 0) ? "x" : "y";
                    //cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", imageToRender);
                    cv::imshow("PAF heatmap " + xory + " between joints " + std::to_string(def.pafDict(0, desiredChannel / 2)) + " and " + std::to_string(def.pafDict(1, desiredChannel / 2)), imageToRender);
                    cv::waitKey(0);
                }
                else
                    op::opLog("Empty cv::Mat as output.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
            }
            else
                op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
            const auto key = (char)cv::waitKey(1);
            return (key == 27);
        }
        catch (const std::exception& e)
        {
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return true;
        }
    }
    void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
    {
        // Example: How to use the pose keypoints
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            op::opLog("\nKeypoints:");
            // Accesing each element of the keypoints
            const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
            op::opLog("Person pose keypoints:");
            for (auto person = 0; person < poseKeypoints.getSize(0); person++)
            {
                op::opLog("Person " + std::to_string(person) + " (x, y, score):");
                for (auto bodyPart = 0; bodyPart < poseKeypoints.getSize(1); bodyPart++)
                {
                    std::string valueToPrint;
                    valueToPrint += std::to_string(bodyPart) + "\t";
                    for (auto xyscore = 0; xyscore < poseKeypoints.getSize(2); xyscore++)
                        valueToPrint += std::to_string(poseKeypoints[{person, bodyPart, xyscore}]) + " ";
                    op::opLog(valueToPrint);
                }
            }
            op::opLog(" ");
            // Alternative: just getting std::string equivalent
            op::opLog("Face keypoints: " + datumsPtr->at(0)->faceKeypoints.toString(), op::Priority::High);
            op::opLog("Left hand keypoints: " + datumsPtr->at(0)->handKeypoints[0].toString(), op::Priority::High);
            op::opLog("Right hand keypoints: " + datumsPtr->at(0)->handKeypoints[1].toString(), op::Priority::High);
            // Heatmaps
            const auto& poseHeatMaps = datumsPtr->at(0)->poseHeatMaps;
            if (!poseHeatMaps.empty())
            {
                op::opLog("Pose heatmaps size: [" + std::to_string(poseHeatMaps.getSize(0)) + ", "
                    + std::to_string(poseHeatMaps.getSize(1)) + ", "
                    + std::to_string(poseHeatMaps.getSize(2)) + "]");
                const auto& faceHeatMaps = datumsPtr->at(0)->faceHeatMaps;
                op::opLog("Face heatmaps size: [" + std::to_string(faceHeatMaps.getSize(0)) + ", "
                    + std::to_string(faceHeatMaps.getSize(1)) + ", "
                    + std::to_string(faceHeatMaps.getSize(2)) + ", "
                    + std::to_string(faceHeatMaps.getSize(3)) + "]");
                const auto& handHeatMaps = datumsPtr->at(0)->handHeatMaps;
                op::opLog("Left hand heatmaps size: [" + std::to_string(handHeatMaps[0].getSize(0)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(1)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(2)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(3)) + "]");
                op::opLog("Right hand heatmaps size: [" + std::to_string(handHeatMaps[1].getSize(0)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(1)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(2)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(3)) + "]");
            }
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
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

Eigen::Matrix3Xf convertOPtoEigen(op::Array<float>& opOutput, int pid) {
    int origSize = opOutput.getSize()[1];

    Eigen::Matrix3Xf eigen(opOutput.getSize()[2], opOutput.getSize()[1]);
    for (int r = 0; r < eigen.rows(); r++) {
        for (int c = 0; c < eigen.cols(); c++) {
            eigen(r, c) = opOutput[{pid, c, r}];
        }
    }

    return eigen;
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
        //pafXs[i] = pafMatX[{xs[i], ys[i]}];
        pafXs[i] = pafMatX.at<float>(ys[i], xs[i]);
        //pafYs[i] = pafMatY[{xs[i], ys[i]}];
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

//TODO: Azok a paf mátrixok hibásak, ahol a heatmapen egy pozitív és egy negatív vektorhalmazzal dolgozunk (egyik tag pl felkar pozitív, a másik pedig negatív irányba mutat) 
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
            //std::vector<int> validPeopleOfJointA = validPersonOfJoint(opOutput, jAIdx);
            //std::vector<int> validPeopleOfJointB = validPersonOfJoint(opOutput, jBIdx);

            eigenPafOfPair.resize(poseCandidates[jAIdx].size(), poseCandidates[jBIdx].size());
            for (int i = 0; i < eigenPafOfPair.rows(); i++) {
                for (int j = 0; j < eigenPafOfPair.cols(); j++) {
                    auto& xHeatMap = cv::Mat(height, width, CV_32F, &pafHeatMaps.getPtr()[(2 * pafIdx) % numberChannels * height * width]);
                    auto& yHeatMap = cv::Mat(height, width, CV_32F, &pafHeatMaps.getPtr()[(2 * pafIdx + 1) % numberChannels * height * width]);

                    //for (int i = 0; i < xHeatMap.cols; i++)
                    //    //std::cout << xHeatMap.at<float>(100, i) << "  ";
                    //    std::cout << i << ": " << pafHeatMaps[{0, 100, i}] << std::endl;
                    //cv::imshow("heatmapX", xHeatMap);
                    //cv::waitKey(0);
                    //cv::imshow("heatmapY", yHeatMap);
                    //cv::waitKey(0);
                    //continue;
                    //std::cout << "xHeatMap width and height: " << xHeatMap.cols << "x" << xHeatMap.rows << std::endl;
                    //std::cout << "yHeatMap width and height: " << yHeatMap.cols << "x" << yHeatMap.rows << std::endl;
                    //std::cout << "x2, y1, x2, y2 positions: " << poseCandidates[jAIdx][i][0] / scale << " , " << poseCandidates[jAIdx][i][1] / scale << " , " << poseCandidates[jBIdx][j][0] / scale << " , " << poseCandidates[jBIdx][j][1] / scale << std::endl;
                    eigenPafOfPair(i, j) = calcScoreForPafMatrix(poseCandidates[jAIdx][i][0] / scale, poseCandidates[jAIdx][i][1] / scale, poseCandidates[jBIdx][j][0] / scale, poseCandidates[jBIdx][j][1] / scale, xHeatMap, yHeatMap);
                }
            }
            //std::cout << "Pafs" << pafIdx << " , between joints " << jAIdx << " and " << jBIdx << ": " << std::endl << eigenPafOfPair << std::endl;
            eigenPafVector[pafIdx] = eigenPafOfPair;
        }
    }

    return eigenPafVector;
}






int tutorialApiCpp()
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
        //std::vector<op::Array<float>> remoteKeypoints(cameras.size() - numLocalCam);
        Server server;
        std::thread t2;
        std::vector<std::string> ips{ "169.254.150.101","169.254.150.102","169.254.150.103","169.254.150.104" };
        std::vector<cv::Mat> map1s(cameras.size());
        std::vector<cv::Mat> map2s(cameras.size());
        cv::Mat_<float> distCoeff = cv::Mat_<float>::zeros(5, 1);
        if (FLAGS_undist)
        {
            cv::Mat dist = (cv::Mat_<double>(5, 1) << -4.3733665180754527e-01, 2.1921356552711482e-01, 1.1597452449095796e-03,
                4.6874816837838441e-03, -5.9483271093969864e-02);
            for (auto const& cam : cameras)
            {
                std::cout << "Dist coeff of cam " << cam.first << ": " << cam.second.distCoeff << std::endl;
                std::cout << "Intri matrix of cam " << cam.first << ": " << cam.second.originK << std::endl;
                std::cout << "Img size of cam  " << cam.first << ": " << cameras.begin()->second.imgSize << std::endl;
                cv::initUndistortRectifyMap(cam.second.originK, cam.second.distCoeff, cv::Mat(), cam.second.originK, cameras.begin()->second.imgSize, CV_32FC1, map1s[std::stoi(cam.first)], map2s[std::stoi(cam.first)]);
                //cv::initUndistortRectifyMap(cam.second.originK, dist, cv::Mat(), cam.second.originK, cameras.begin()->second.imgSize, CV_32FC1, map1s[std::stoi(cam.first)], map2s[std::stoi(cam.first)]);
            }
            //TODO: distortion parameters 

            //cv::initUndistortRectifyMap(cameras.begin()->second.originK, dist, cv::Mat(), cameras.begin()->second.originK, cameras.begin()->second.imgSize, CV_32FC1, map1, map2);
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
                //streams.emplace_back("http://admin:admin123@192.168.1.108/cgi-bin/mjpg/video.cgi?channel=1&subtype=1", op::Point<int>{640, 480}); //ehelyett inkább pointereket használok, mert azt mondta a zinternet
                streams[i] = new op::IpCameraReader("http://admin:admin123@" + ips[i] + "/cgi-bin/mjpg/video.cgi?channel=1&subtype=1", op::Point<int>{640, 480});
                //streams[i] = new op::IpCameraReader("rtsp://admin:admin123@192.168.1.108", op::Point<int>{1280, 720});
                //videos[i] = cv::VideoCapture("rtsp://admin:admin123@192.168.1.108"); //egy ipcam beolvasás
                cv::Size imgSize(int(streams[i]->get(cv::CAP_PROP_FRAME_WIDTH)), int(streams[i]->get(cv::CAP_PROP_FRAME_HEIGHT)));
                rawImgsOpMat[i] = op::Matrix(imgSize.width, imgSize.height, CV_8UC3);
                projs.middleCols(4 * i, 4) = iter->second.eiProj;
                rawImgs[i].create(imgSize, CV_8UC3);   //üres képkockák létrehozása a videó méretével

                std::cout << "The stream image size is: " << int(videos[i].get(cv::CAP_PROP_FRAME_WIDTH)) << "×" << int(videos[i].get(cv::CAP_PROP_FRAME_HEIGHT)) << std::endl;
                if (!streams[i]->isOpened())
                {
                    std::cout << "Camera " << i << " hasn't been opened." << std::endl;
                }
            }
        }

        // User processing
        UserInputClass userInputClass;
        UserOutputClass userOutputClass;
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
        associater.SetTempWeight(0.5f);
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
            //server.on_client_connect(server.client, server.remoteKeypoints);
        }
        /*while (true)
        {
            std::cout << "Tick" << std::endl;
            Sleep(1000);
        }*/
        auto startp = std::chrono::high_resolution_clock::now();
        int framecount = 0;
        while (!userInputClass.isFinished()) {
#pragma omp parallel for
            for (int i = 0; i < cameras.size(); i++)
            {
                if (!FLAGS_use_webcams)
                {
                    videos[i] >> rawImgs[i];
                    //cv::imwrite("C:/Users/nykri/Documents/3Dhuman/4d_association-windows/data/shelf/video" + std::to_string(framecount) + ".jpg", rawImgs[i]);
                    if (FLAGS_undist) cv::remap(rawImgs[i], rawImgs[i], map1s[i], map2s[i], cv::INTER_LINEAR);
                }
                else
                {
                    ////TODO: make rawImgs type op::Matrix, make userInputClass.createDatumFromVideo function's input the same -->inkább ne, sokkal többet kell átírni máshol és OP_OP2CVCONSTMAT csak egy return, elvileg nem rontja a futási idõt -->mégis megcsináltam...
                    rawImgsOpMat[i] = streams[i]->getFrame();
                    if (FLAGS_undist) cv::remap(OP_OP2CVMAT(rawImgsOpMat[i]), OP_OP2CVMAT(rawImgsOpMat[i]), map1s[i], map2s[i], cv::INTER_LINEAR);
                    //cv::imshow("opmat" + 1, OP_OP2CVMAT(rawImgsOpMat[i]));
                    //cv::waitKey(0);
                    rawImgs[i] = OP_OP2CVMAT(rawImgsOpMat[i]);
                    //cv::imshow("cvmat" + 1, rawImgs[i]);
                    //cv::waitKey(0);
                    //if (FLAGS_undist) cv::remap(rawImgs[i], rawImgs[i], map1s[i], map2s[i], cv::INTER_LINEAR);
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
                        //std::cout << "successfullyEmplaced: " << successfullyEmplaced << std::endl;
                        auto successfullyPopped = opWrapperT.waitAndPop(datumProcessed);
                        //std::cout << "successfullyPopped: " << successfullyPopped << std::endl;
                        if (successfullyEmplaced && successfullyPopped)
                        {
                            //std::cout << "No Person detected: " << datumProcessed->at(0)->poseKeypoints.empty() << std::endl;
                            if (!FLAGS_no_display)
                            {
                                userWantsToExit = userOutputClass.display(datumProcessed);
                                if (userWantsToExit)
                                    userInputClass.exit();
                                //std::cout << "Display done!"  << std::endl;
                            }

                            //Print pose candidates
                            /*for (int j = 0; j < datumProcessed->at(0)->poseCandidates.size(); j++)
                            {

                                for (int k = 0; k < datumProcessed->at(0)->poseCandidates[j].size(); k++)
                                {
                                    std::cout << "Camera" << i <<" of frame " << framecount << "; Joints " << j << " from candidates: " << std::endl;
                                    for (int m = 0; m < datumProcessed->at(0)->poseCandidates[j][k].size(); m++)
                                    {
                                        std::cout  << datumProcessed->at(0)->poseCandidates[j][k][m] << std::endl;
                                    }

                                }
                            }*/

                            //Show each heatmap if added by flags
                            /*const auto numberChannels = datumProcessed->at(0)->poseHeatMaps.getSize(0);
                            for (auto desiredChannel = 0; desiredChannel < numberChannels; desiredChannel++)
                                userOutputClass.displayHeatmap(datumProcessed, desiredChannel);*/

                                ////TODO: itt "symbol file not loaded" exeptionnel kilép, ha "datumProcessed->at(0)->poseKeypoints" array üres (az openpose nem talált jointokat az adott framen)
                                ////TODO: If no person is detected, frameDetections[i] joints and pafs are vectors with the size 0, so Mapping() can't access the jID-th element --> joints need to be initialized as a vector with the correct size
                                //frameDetections[i].joints = convertOPtoEigenFullFrame(datumProcessed->at(0)->poseKeypoints);
                            frameDetections[i].joints = (FLAGS_part_candidates) ? convertOPCandidatesToEigenFullFrame(datumProcessed->at(0)->poseCandidates) : convertOPtoEigenFullFrame(datumProcessed->at(0)->poseKeypoints);
                            /*for (int j = 0; j < frameDetections[i].joints.size(); j++)
                            {
                                std::cout << "Joints" << j << ": " << std::endl << frameDetections[i].joints[j] << std::endl;
                            }*/
                            //frameDetections[i].pafs = createFakePafsFullFrame(datumProcessed->at(0)->poseKeypoints, def);
                            frameDetections[i].pafs = (FLAGS_part_candidates) ? createPafsFullFrame(datumProcessed, def, IOScale) : createFakePafsFullFrame(datumProcessed->at(0)->poseKeypoints, def);

                            //std::cout << "Conversion to Eigen/4Dassoc done!" << std::endl;

                            /*for (int j = 0; j < frameDetections[i].pafs.size(); j++)
                            {
                                std::cout << "Pafs" << j << " , between joints " << def.pafDict(0, j) << " and " << def.pafDict(1, j) << ": " << std::endl << frameDetections[i].pafs[j] << std::endl;
                            }*/
                            /* if (datumProcessed->at(0)->poseKeypoints.empty())
                             {
                                 associater.SetDetection(i, frameDetections[i]);
                                 continue;
                             }*/

                             //if (!FLAGS_use_webcams)
                            cv::resize(rawImgs[i], rawImgs[i], cv::Size(), skelPainter.rate, skelPainter.rate);
                            //else
                                //TODO: Ez így lehet nem rakja vissza a második paraméterbe a képet, mert közben konvertáljuk. Szóval simán lehet ezért szar...
                            //    cv::resize(OP_OP2CVCONSTMAT(rawImgsOpMat[i]), OP_OP2CVCONSTMAT(rawImgsOpMat[i]), cv::Size(), skelPainter.rate, skelPainter.rate);

                            //std::cout << "Resize for skelPainter done!" << std::endl;
                            auto& mappedDetection = frameDetections[i].Mapping(SKEL19);
                            ////TODO: Runtime error in Mapping() function, probably index out of bounds
                            //std::cout << "Mapping done!" << std::endl;
                            associater.SetDetection(i, mappedDetection); //Associater osztály m_detections változójában tárolja a detectiont
                            //std::cout << "One camera detection setting done!" << std::endl;
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
                //std::cout << "Skelsize: " << skels.size() << std::endl;
                //std::cout << "3D detection done!" << std::endl;
            }

            //Saving images of detection, association and reprojection of the assembled 3D skeleton
            if (FLAGS_save_images && !userInputClass.isFinished()) {
                std::cout << "Saving images" << std::endl;
                cv::Mat detectImg, assocImg, reprojImg;
                const int layoutCols = 3;
                ////TODO:Ez itt gond lesz, mert a rawImgsOpMat változókba mentjük az ipCameraReader-bõl érkezõ frameket. Így ez csak a videós megoldásra mûködik --> de, mert mostmár a getFramenél belerakjuk a rawImgs-be is a képet, hogy meglegyen cv::Mat-ban is 
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
                //cv::imwrite("output/reproj94u_shared/" + std::to_string(framecount) + ".jpg", reprojImg);
                cv::imshow("reproj", reprojImg);
                cv::imshow("detect", detectImg);
                cv::waitKey(1);
            }

            std::cout << "------ End of processing frame " << std::to_string(framecount) << " ------" << std::endl;


            framecount++;

            //This can decrease performance, but it hasn't been checked yet
            if (kbhit() | framecount >= 300)
            {
                if (FLAGS_sharedWork)
                {
                    server.stop = true;
                    //server.t1.join();
                    t2.join();
                }
                break;
            }

        }//end while
        auto stopp = std::chrono::high_resolution_clock::now();
        int duration = (int)std::chrono::duration_cast<std::chrono::seconds>(stopp - startp).count();
        std::cout << "Time to finish program: " << duration << "s" << std::endl;
        std::cout << "Average FPS: " << std::to_string((float)framecount / duration) << " frames/s";
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



        // Return
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
    FLAGS_part_candidates = false;

    FLAGS_part_to_show = 0;
    FLAGS_heatmaps_add_PAFs = false;
    FLAGS_heatmaps_add_bkg = false;
    FLAGS_heatmaps_add_parts = false;
    FLAGS_heatmaps_scale = 0; //0 --> in range [-1, 1] ; 1 --> in range [0, 1] ; 2(default) --> in range [0 , 255]

    FLAGS_no_display = true;
    FLAGS_save_images = true;
    FLAGS_use_webcams = true;
    //FLAGS_write_images = "C:\\Users\\nykri\\Documents\\3Dhuman\\openpose\\examples\\media2\\output";
    FLAGS_undist = true;

    FLAGS_sharedWork = true;


    // Running tutorialApiCpp
    return tutorialApiCpp();
}
