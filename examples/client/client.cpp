// --------------------- OpenPose C++ API Tutorial - Example 12 - Custom Input, Output, and Datum ---------------------
// Asynchronous mode: ideal for fast prototyping when performance is not an issue.
// In this function, the user can implement its own way to create frames (e.g., reading his own folder of images)
// and its own way to render/display them after being processed by OpenPose.

#pragma comment(lib, "Ws2_32.lib")
#include <ws2tcpip.h>

#include <winsock2.h>

// Third-party dependencies
#include <opencv2/opencv.hpp>
// Command-line user interface
#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#define OPENPOSE_FLAGS_DISABLE_DISPLAY
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>
#include <conio.h>

#include "camera.cpp"

//#include <winsock2.h>

#include <tchar.h>

// Custom OpenPose flags
// Display
DEFINE_bool(no_display, false,
    "Enable to disable the visual display.");
//Undistort images
DEFINE_bool(undist, false,
    "Wether to undistort the input images. The distortion parameters have to be given");


// If the user needs his own variables, he can inherit the op::Datum struct and add them in there.
// UserDatum can be directly used by the OpenPose wrapper because it inherits from op::Datum, just define
// WrapperT<std::vector<std::shared_ptr<UserDatum>>> instead of Wrapper
// (or equivalently WrapperT<std::vector<std::shared_ptr<UserDatum>>>)
struct UserDatum : public op::Datum
{
    bool boolThatUserNeedsForSomeReason;

    UserDatum(const bool boolThatUserNeedsForSomeReason_ = false) :
        boolThatUserNeedsForSomeReason{ boolThatUserNeedsForSomeReason_ }
    {}
};

// This worker will just read and return all the basic image file formats in a directory
class UserInputClass
{
public:
    UserInputClass() :
        // mImageFiles{op::getFilesOnDirectory(directoryPath, std::vector<std::string>{"jpg", "png"})},
        mCounter{ 0 },
        mClosed{ false },
        server{ initSocket() }
    {
        //stream = new op::IpCameraReader("http://admin:admin123@169.254.150.104/cgi-bin/mjpg/video.cgi?channel=1&subtype=1", op::Point<int>{640, 480});
    }

    std::shared_ptr<std::vector<std::shared_ptr<UserDatum>>> createDatum(op::Matrix image)
    {
        auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<UserDatum>>>();
        datumsPtr->emplace_back();
        auto& datumPtr = datumsPtr->at(0);
        datumPtr = std::make_shared<UserDatum>();

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

    unsigned long long getCounter()
    {
        return mCounter;
    }
    
    SOCKET getServer()
    {
        return server;
    }

    void terminateServer()
    {
        closesocket(server);
        WSACleanup();
    }

private:
    unsigned long long mCounter;
    bool mClosed;
    op::IpCameraReader* stream;
    
    SOCKET server;

    const SOCKET initSocket()
    {
        WSADATA wsa_data;
        SOCKADDR_IN addr;

        WSAStartup(MAKEWORD(2, 0), &wsa_data);
        const auto server = socket(AF_INET, SOCK_STREAM, 0);

        //InetPton(AF_INET, _T("127.0.0.1"), &addr.sin_addr.s_addr);
        InetPton(AF_INET, _T("169.254.154.231"), &addr.sin_addr.s_addr);

        addr.sin_family = AF_INET;
        addr.sin_port = htons(5555);

        connect(server, reinterpret_cast<SOCKADDR*>(&addr), sizeof(addr));
        std::cout << "Connected to server!" << std::endl;

        return server;
    }
};

// This worker will just read and return all the jpg files in a directory
class UserOutputClass
{
public:
    bool display(const std::shared_ptr<std::vector<std::shared_ptr<UserDatum>>>& datumsPtr)
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
                    cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
                else
                    op::opLog("Empty cv::Mat as output.", op::Priority::High);
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
    void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<UserDatum>>>& datumsPtr)
    {
        // Example: How to use the pose keypoints
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            op::opLog("\nKeypoints:");
            // Accessing each element of the keypoints
            const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
            //op::opLog("Face keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);
            std::cout << "Person keypoints: " << datumsPtr->at(0)->poseKeypoints.toString() << std::endl;
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

void configureWrapper(op::WrapperT<UserDatum>& opWrapperT)
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

int tutorialApiCpp()
{
    try
    {
        op::opLog("Starting OpenPose demo...", op::Priority::High);
        const auto opTimer = op::getTimerInit();

        // Configuring OpenPose
        op::opLog("Configuring OpenPose...", op::Priority::High);
        op::WrapperT<UserDatum> opWrapperT{ op::ThreadManagerMode::Asynchronous };
        configureWrapper(opWrapperT);

        // Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
        op::opLog("Starting thread(s)...", op::Priority::High);
        opWrapperT.start();

        // User processing
        Sleep(4000);

        UserInputClass userInputClass;
        std::cout << "init socket kesz" << std::endl;
        UserOutputClass userOutputClass;
        bool userWantsToExit = false;
        std::string msg;
        std::string sKeypoints;

        std::vector<std::string> ips{ "169.254.150.103" , "169.254.150.104" };
        std::string dataset = "live";
        //TODO: csak a 2-3 indexű kamera szerepeljen a kalibrációs fileban
        std::map<std::string, Camera> cameras = ParseCameras("D:/openpose/calibration_live2cam.json"); //dataset mappában a calibrációs json fájl
        std::cout << cameras.size() << std::endl;
        std::cout << ips.size() << std::endl;
        std::vector<op::Matrix> rawImgsOpMat(ips.size());
        std::vector<op::IpCameraReader*> streams(ips.size());
        op::WebcamReader* video = new op::WebcamReader(0);
        std::vector<cv::Mat> map1s(ips.size());
        std::vector<cv::Mat> map2s(ips.size());
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

            //cv::initUndistortRectifyMap(cameras.begin()->second.originK, dist, cv::Mat(), cameras.begin()->second.originK, cameras.begin()->second.imgSize, CV_32FC1, map1, map2);
        }
        std::cout << "init distortion kesz" << std::endl;
        for (int i = 0; i < cameras.size(); i++)
        {
            streams[i] = new op::IpCameraReader("http://admin:admin123@" + ips[i] + "/cgi-bin/mjpg/video.cgi?channel=1&subtype=1", op::Point<int>{640, 480});
            cv::Size imgSize(int(streams[i]->get(cv::CAP_PROP_FRAME_WIDTH)), int(streams[i]->get(cv::CAP_PROP_FRAME_HEIGHT)));
            rawImgsOpMat[i] = op::Matrix(imgSize.width, imgSize.height, CV_8UC3);
            
        }
        std::cout << "init streams kesz" << std::endl;
        int framecount = 0;
        while (!userWantsToExit && !userInputClass.isFinished())
        {
            Sleep(10);
#pragma omp parallel for
            for (int i = 0; i < cameras.size(); i++)
            {
                rawImgsOpMat[i] = streams[i]->getFrame();
                if (FLAGS_undist) cv::remap(OP_OP2CVMAT(rawImgsOpMat[i]), OP_OP2CVMAT(rawImgsOpMat[i]), map1s[i], map2s[i], cv::INTER_LINEAR);
            }

            for (int i = 0; i < cameras.size(); i++)
            {
                // Push frame
                auto datumToProcess = userInputClass.createDatum(rawImgsOpMat[i]);
                if (datumToProcess != nullptr)
                {
                    auto successfullyEmplaced = opWrapperT.waitAndEmplace(datumToProcess);
                    // Pop frame
                    std::shared_ptr<std::vector<std::shared_ptr<UserDatum>>> datumProcessed;
                    if (successfullyEmplaced && opWrapperT.waitAndPop(datumProcessed))
                    {
                        if (!FLAGS_no_display)
                            userWantsToExit = userOutputClass.display(datumProcessed);
                        std::cout << "cam: " << i << std::endl;
                        userOutputClass.printKeypoints(datumProcessed);
                        sKeypoints = datumProcessed->at(0)->poseKeypoints.toStringOneLine();
                        if (!sKeypoints.empty())
                        {
                            msg = std::to_string(i + 2) + sKeypoints;
                            send(userInputClass.getServer(), msg.c_str(), msg.size(), 0);
                        }
                    }
                    else
                        op::opLog("Processed datum could not be emplaced.", op::Priority::High);
                }

            }
            if (framecount >= 1000 | kbhit())
                break;
            std::cout << "finished frame " << framecount << std::endl;
            framecount++;
        }

        op::opLog("Stopping thread(s)", op::Priority::High);
        opWrapperT.stop();
        userInputClass.terminateServer();

        // Measuring total time
        op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);

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
    FLAGS_no_display = true;

    FLAGS_part_to_show = 0;
    FLAGS_heatmaps_add_PAFs = false;
    FLAGS_heatmaps_add_bkg = false;
    FLAGS_heatmaps_add_parts = false;
    FLAGS_heatmaps_scale = 0; //0 --> in range [-1, 1] ; 1 --> in range [0, 1] ; 2(default) --> in range [0 , 255]

    FLAGS_undist = true;

    // Running tutorialApiCpp
    return tutorialApiCpp();
}
