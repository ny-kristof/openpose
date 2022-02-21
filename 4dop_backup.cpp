// --------------------- OpenPose C++ API Tutorial - Example 12 - Custom Input, Output, and Datum ---------------------
// Asynchronous mode: ideal for fast prototyping when performance is not an issue.
// In this function, the user can implement its own way to create frames (e.g., reading his own folder of images)
// and its own way to render/display them after being processed by OpenPose.

#include "kruskal_associater.h"
#include "skel_updater.h"
#include "skel_painter.h"
#include "openpose.h"
#include "openpose.cpp"
#include "camera.cpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

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
DEFINE_string(image_dir,                "examples/media2/",
    "Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).");
// Display
DEFINE_bool(no_display,                 false,
    "Enable to disable the visual display.");

// If the user needs his own variables, he can inherit the op::Datum struct and add them in there.
// UserDatum can be directly used by the OpenPose wrapper because it inherits from op::Datum, just define
// WrapperT<std::vector<std::shared_ptr<UserDatum>>> instead of Wrapper
// (or equivalently WrapperT<std::vector<std::shared_ptr<UserDatum>>>)
struct UserDatum : public op::Datum
{
    bool boolThatUserNeedsForSomeReason;

    UserDatum(const bool boolThatUserNeedsForSomeReason_ = false) :
        boolThatUserNeedsForSomeReason{boolThatUserNeedsForSomeReason_}
    {}
};

// This worker will just read and return all the basic image file formats in a directory
class UserInputClass
{
public:
    UserInputClass(const std::string& directoryPath) :
        mImageFiles{op::getFilesOnDirectory(directoryPath, op::Extensions::Images)}, // For all basic image formats
        // If we want only e.g., "jpg" + "png" images
        // mImageFiles{op::getFilesOnDirectory(directoryPath, std::vector<std::string>{"jpg", "png"})},
        mCounter{0},
        mClosed{false}
    {
        if (mImageFiles.empty())
            op::error("No images found on: " + directoryPath, __LINE__, __FUNCTION__, __FILE__);
    }

    std::shared_ptr<std::vector<std::shared_ptr<UserDatum>>> createDatum()
    {
        // Close program when empty frame
        if (mClosed || mImageFiles.size() <= mCounter)
        {
            op::opLog("Last frame read and added to queue. Closing program after it is processed.", op::Priority::High);
            // This funtion stops this worker, which will eventually stop the whole thread system once all the frames
            // have been processed
            mClosed = true;
            return nullptr;
        }
        else // if (!mClosed)
        {
            // Create new datum
            auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<UserDatum>>>();
            datumsPtr->emplace_back();
            auto& datumPtr = datumsPtr->at(0);
            datumPtr = std::make_shared<UserDatum>();

            // Fill datum
            const cv::Mat cvInputData = cv::imread(mImageFiles.at(mCounter++));
            datumPtr->cvInputData = OP_CV2OPCONSTMAT(cvInputData);

            // If empty frame -> return nullptr
            if (datumPtr->cvInputData.empty())
            {
                op::opLog("Empty frame detected on path: " + mImageFiles.at(mCounter-1) + ". Closing program.",
                        op::Priority::High);
                mClosed = true;
                datumsPtr = nullptr;
            }

            return datumsPtr;
        }
    }

    std::shared_ptr<std::vector<std::shared_ptr<UserDatum>>> createDatumFromVideo(cv::Mat image)
    {
        auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<UserDatum>>>();
        datumsPtr->emplace_back();
        auto& datumPtr = datumsPtr->at(0);
        datumPtr = std::make_shared<UserDatum>();

        // Fill datum
        const cv::Mat cvInputData = image;
        datumPtr->cvInputData = OP_CV2OPCONSTMAT(cvInputData);

        // If empty frame -> return nullptr
        if (datumPtr->cvInputData.empty())
        {
            op::opLog("Empty frame detected on path: " + mImageFiles.at(mCounter - 1) + ". Closing program.",
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

private:
    const std::vector<std::string> mImageFiles;
    unsigned long long mCounter;
    bool mClosed;
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
            const auto key = (char)cv::waitKey(0);
            return (key == 27);
        }
        catch (const std::exception& e)
        {
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return true;
        }
    }

    bool displayHeatmap(
        const std::shared_ptr<std::vector<std::shared_ptr<UserDatum>>>& datumsPtr, const int desiredChannel = 0)
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
                // Display image
                if (!imageToRender.empty())
                {
                    cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", imageToRender);
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
    void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<UserDatum>>>& datumsPtr)
    {
        // Example: How to use the pose keypoints
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            op::opLog("\nKeypoints:");
            // Accesing each element of the keypoints
            const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
            op::opLog("Person pose keypoints:");
            for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
            {
                op::opLog("Person " + std::to_string(person) + " (x, y, score):");
                for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
                {
                    std::string valueToPrint;
                    valueToPrint += std::to_string(bodyPart) + "\t";
                    for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                        valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
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
            op::String(FLAGS_caffemodel_path), (float)FLAGS_upsampling_ratio, enableGoogleLogging};
        opWrapperT.configure(wrapperStructPose);
        // Face configuration (use op::WrapperStructFace{} to disable it)
        const op::WrapperStructFace wrapperStructFace{
            FLAGS_face, faceDetector, faceNetInputSize,
            op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
            (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold};
        opWrapperT.configure(wrapperStructFace);
        // Hand configuration (use op::WrapperStructHand{} to disable it)
        const op::WrapperStructHand wrapperStructHand{
            FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
            op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
            (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold};
        opWrapperT.configure(wrapperStructHand);
        // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
        const op::WrapperStructExtra wrapperStructExtra{
            FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads};
        opWrapperT.configure(wrapperStructExtra);
        // Output (comment or use default argument to disable any output)
        const op::WrapperStructOutput wrapperStructOutput{
            FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
            op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
            FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
            op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
            op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
            op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
            op::String(FLAGS_udp_port)};
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

//Eigen::Matrix3Xf convertOPtoEigen(op::Array<float>& opOutput, int pid) {
Eigen::Matrix3Xf convertOPtoEigen(op::Array<float>&opOutput, int pid) {
    //if (pid > opOutput.getSize()[0]) throw error
        int origSize = opOutput.getSize()[1];


        //for (auto size : opOutput.getSize()) {
        //    std::cout << "Sizes: " << size << std::endl;

        //}

        //std::cout << "Sizes: " << opOutput.getSize()[0] << "\t" << opOutput.getSize()[1] << "\t" << opOutput.getSize()[2] << std::endl;

    Eigen::Matrix3Xf eigen(opOutput.getSize()[2], opOutput.getSize()[1]);
    for (int r = 0; r < eigen.rows(); r++) {
        for (int c = 0; c < eigen.cols(); c++) {
            //std::cout << "Eigen rowcols, pid: " << eigen.rows() << " , " << eigen.cols()  << " , "  << pid << std::endl;
            //eigen(r, c) = opOutput[pid * eigen.rows() * eigen.cols() + eigen.cols() * r + c];
            eigen(r, c) = opOutput[{pid, c, r}];
        }
    }

    return eigen;
}

std::vector<Eigen::Matrix3Xf> convertOPtoEigenFullFrame(op::Array<float>&opOutput) {
    std::vector<Eigen::Matrix3Xf> eigenKeypointsVector;
    //KI KELLENE SZÛRNI A 0 OSZLOPOKAT, HOGY LE LEHESSEN GYÁRTANI A FAKE PAF MÁTRIXOKAT
    //NEM JÓ, MERT ÍGY ELCSÚSZNAK A PAF-OK 1ESEI
    for (int  jointID = 0; jointID < opOutput.getSize()[1]; jointID++)
    {
        //Eigen::Matrix3Xf jointOfPeople(opOutput.getSize()[2], opOutput.getSize()[0]);
        Eigen::Matrix3Xf jointOfPeople(3,0);

        for (int personID = 0; personID < opOutput.getSize()[0]; personID++)
        {
            //Eigen::Vector3f toAppend;
            if (opOutput[{personID, jointID, 0}] != 0) 
            {
                jointOfPeople.conservativeResize(Eigen::NoChange, jointOfPeople.cols() + 1);
                for (int xyscore = 0; xyscore < opOutput.getSize()[2]; xyscore++)
                {
                    //std::cout << "JointID, xyscore and PersonID: "<< jointID << " , " << xyscore  << " , " << personID << std::endl;
                    //jointOfPeople(xyscore, personID) = opOutput[{personID, jointID , xyscore}];
                    //toAppend[xyscore] = opOutput[{personID, jointID, xyscore}];
                    jointOfPeople(xyscore, jointOfPeople.cols() - 1) = opOutput[{personID, jointID, xyscore}];
                }

            }
        }

        eigenKeypointsVector.emplace_back(jointOfPeople);
    }
    return eigenKeypointsVector;
}

std::vector<Eigen::MatrixXf> createFakePafsFullFrame(std::vector<Eigen::Matrix3Xf>& keypoints, SkelDef& def) {
    std::vector<Eigen::MatrixXf> eigenPafVector;
    
    for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
        Eigen::MatrixXf eigenPafOfPair;
        const int jAIdx = def.pafDict(0, pafIdx);
        const int jBIdx = def.pafDict(1, pafIdx);
        eigenPafOfPair.resize(keypoints[jAIdx].cols(), keypoints[jBIdx].cols());
        for (int i = 0; i < eigenPafOfPair.rows(); i++) {
            for (int j = 0; j < eigenPafOfPair.cols(); j++) {
                if (i == j) eigenPafOfPair(i, j) = (float)1.0;
                else       eigenPafOfPair(i, j) = (float)0.0;
            }
        }
        eigenPafVector.emplace_back(eigenPafOfPair);
    }

    return eigenPafVector;
}

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

std::vector<Eigen::MatrixXf> createFakePafsFullFrame2(op::Array<float>& opOutput, SkelDef& def) {
    std::vector<Eigen::MatrixXf> eigenPafVector;

    for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
        Eigen::MatrixXf eigenPafOfPair;
        const int jAIdx = def.pafDict(0, pafIdx);
        const int jBIdx = def.pafDict(1, pafIdx);
        std::vector<int> validPeopleOfJointA = validPersonOfJoint(opOutput, jAIdx);
        std::vector<int> validPeopleOfJointB = validPersonOfJoint(opOutput, jBIdx);
        
        eigenPafOfPair.resize(validPeopleOfJointA.size(), validPeopleOfJointB.size());
        for (int i = 0; i < eigenPafOfPair.rows(); i++) {
            for (int j = 0; j < eigenPafOfPair.cols(); j++) {
                //if (i == j) {
                    //if (opOutput[{i, jAIdx, 0}] != (float)0.0 && opOutput[{j, jBIdx, 0}] != (float)0.0) 
                    if(validPeopleOfJointA[i] == validPeopleOfJointB[j])
                        eigenPafOfPair(i, j) = (float)1.0;
                //}
                else eigenPafOfPair(i, j) = (float)0.0;
            }
        }
        eigenPafVector.emplace_back(eigenPafOfPair);
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
        op::WrapperT<UserDatum> opWrapperT{op::ThreadManagerMode::Asynchronous};
        configureWrapper(opWrapperT);

        // Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
        op::opLog("Starting thread(s)...", op::Priority::High);
        opWrapperT.start();



        const std::string dataset = "shelf";    // data/*dataset* mappa neve 
        std::map<std::string, Camera> cameras = ParseCameras("C:/Users/nykri/Documents/3Dhuman/4d_association-windows/data/" + dataset + "/calibration.json"); //dataset mappában a calibrációs json fájl
        Eigen::Matrix3Xf projs(3, cameras.size() * 4);
        std::vector<cv::Mat> rawImgs(cameras.size());
        std::vector<cv::VideoCapture> videos(cameras.size());  //annyi videót vár, ahány kamera van
        std::vector<std::vector<OpenposeDetection>> seqDetections(cameras.size());
        const SkelDef& skelDef = GetSkelDef(SKEL19);
        std::vector<std::map<int, Eigen::Matrix4Xf>> skels;

#pragma omp parallel for
        for (int i = 0; i < cameras.size(); i++) {
            int framec = 0;
            auto iter = std::next(cameras.begin(), i);
            //std::cout << "Iter: " << i << iter->second.eiProj << std::endl;
            videos[i] = cv::VideoCapture("C:/Users/nykri/Documents/3Dhuman/4d_association-windows/data/" + dataset + "/video/" + iter->first + ".mp4"); //egy videó beolvasás
            videos[i].set(cv::CAP_PROP_POS_FRAMES, 0); //elsõ frame-el kezdjük
            cv::Size imgSize(int(videos[i].get(cv::CAP_PROP_FRAME_WIDTH)), int(videos[i].get(cv::CAP_PROP_FRAME_HEIGHT)));
            projs.middleCols(4 * i, 4) = iter->second.eiProj;

            //seqDetections[i] = ParseDetections("C:/Users/nykri/Documents/3Dhuman/4d_association-windows/data/" + dataset + "/detection/" + iter->first + ".txt");   //detektált jointok kiolvasása fájlból
            //
            //for (auto&& detection : seqDetections[i]) {
            //    int jointc = 0;
            //    for (auto&& joints : detection.joints) {
            //        joints.row(0) *= (imgSize.width - 1);
            //        joints.row(1) *= (imgSize.height - 1);
            //        jointc++;
            //    }
            //    framec++;
            //}
            rawImgs[i].create(imgSize, CV_8UC3);   //üres képkockák létrehozása a videó méretével
        }

        // User processing
        UserInputClass userInputClass(FLAGS_image_dir);
        UserOutputClass userOutputClass;
        bool userWantsToExit = false;
        std::vector<std::shared_ptr<std::vector<std::shared_ptr<UserDatum>>>> datums(cameras.size());
        SkelDef def = GetSkelDef(SkelType(BODY25));
        std::vector<OpenposeDetection> frameDetections(cameras.size());

        //paraméterek beállítása
	KruskalAssociater associater(SKEL19, cameras);
	associater.SetMaxTempDist(0.3f);
	associater.SetMaxEpiDist(0.15f);
	associater.SetEpiWeight(1.f);
	associater.SetTempWeight(2.f);
	associater.SetViewWeight(1.f);
	associater.SetPafWeight(2.f);
	associater.SetHierWeight(1.f);
	associater.SetViewCntWelsh(1.0);
	associater.SetMinCheckCnt(10);
	associater.SetNodeMultiplex(true);
	associater.SetNormalizeEdge(true);			// new feature

	SkelPainter skelPainter(SKEL19);
	skelPainter.rate = 512.f / float(cameras.begin()->second.imgSize.width);
	SkelFittingUpdater skelUpdater(SKEL19, "../data/skel/SKEL19_new");
	skelUpdater.SetTemporalTransTerm(1e-1f / std::powf(skelPainter.rate, 2));
	skelUpdater.SetTemporalPoseTerm(1e-1f / std::powf(skelPainter.rate, 2));

        while (!userInputClass.isFinished()) {
            for (int i = 0; i < cameras.size(); i++)
            {
                videos[i] >> rawImgs[i];
                auto datumToProcess = userInputClass.createDatumFromVideo(rawImgs[i]);
                if (datumToProcess != nullptr)
                {
                    auto successfullyEmplaced = opWrapperT.waitAndEmplace(datumToProcess);
                    std::shared_ptr<std::vector<std::shared_ptr<UserDatum>>> datumProcessed;
                    if (successfullyEmplaced && opWrapperT.waitAndPop(datumProcessed))
                    {
                        if (!FLAGS_no_display)
                            userWantsToExit = userOutputClass.display(datumProcessed);
                        /*for (int i = 0; i < datumProcessed->at(0)->poseHeatMaps.getSize(0); i++)
                        {
                            userOutputClass.displayHeatmap(datumProcessed, i);
                        }*/
                        //userOutputClass.printKeypoints(datumProcessed);
                        
                        frameDetections[i].joints = convertOPtoEigenFullFrame(datumProcessed->at(0)->poseKeypoints);
                        frameDetections[i].pafs = createFakePafsFullFrame2(datumProcessed->at(0)->poseKeypoints, def);

                        /*for (int j = 0; j < frameDetections[i].joints.size(); j++)
                        {
                            std::cout << "Joints" << j << ": " << std::endl << frameDetections[i].joints[j] << std::endl;
                        }
                        for (int j = 0; j < frameDetections[i].pafs.size(); j++)
                        {
                            std::cout << "FakePafs" << j << " , between joints " << def.pafDict(0, j) << " and " << def.pafDict(1, j) << ": " << std::endl << frameDetections[i].pafs[j] << std::endl;
                        }*/



                        /*for (int i = 0; i < datumProcessed->at(0)->poseScores.getSize(0); i++) {
                            std::cout << "OPoutput type: " << typeid(datumProcessed->at(0)->poseScores).name() << std::endl;
                        }*/
                    }
                    else
                    {
                        op::opLog("Processed datum could not be emplaced.", op::Priority::High);
                    }
                }
            }





        }

        return 0;
        
        while (!userWantsToExit && !userInputClass.isFinished())
        {
            // Push frame
            auto datumToProcess = userInputClass.createDatum();
            if (datumToProcess != nullptr)
            {
                auto successfullyEmplaced = opWrapperT.waitAndEmplace(datumToProcess);
                // Pop frame
                std::shared_ptr<std::vector<std::shared_ptr<UserDatum>>> datumProcessed;
                if (successfullyEmplaced && opWrapperT.waitAndPop(datumProcessed))
                {
                    if (!FLAGS_no_display)
                        userWantsToExit = userOutputClass.display(datumProcessed);
                    /*for (int i = 0; i < datumProcessed->at(0)->poseHeatMaps.getSize(0); i++)
                    {
                        userOutputClass.displayHeatmap(datumProcessed, i);
                    }*/
                    userOutputClass.printKeypoints(datumProcessed);

                    for (int i = 0; i < datumProcessed->at(0)->poseScores.getSize(0); i++) {
                        std::cout << "OPoutput type: " << typeid(datumProcessed->at(0)->poseScores).name() << std::endl;
                    }
                }
                else
                {
                    op::opLog("Processed datum could not be emplaced.", op::Priority::High);
                }

                std::vector<OpenposeDetection> seqDetections;
                OpenposeDetection frameDetection = OpenposeDetection(SkelType(BODY25));

                const auto& poseKeypoints = datumProcessed->at(0)->poseKeypoints;
                //SkelDef def = GetSkelDef(SkelType(BODY25));
                std::vector<Eigen::Matrix3Xf> eigenKeypoints;
                std::vector<Eigen::MatrixXf> eigenPafs;
                /*for (int  i = 0; i < poseKeypoints.getSize(0); i++)
                {
                    eigenKeypoints.emplace_back(convertOPtoEigen(datumProcessed->at(0)->poseKeypoints, i));
                }*/
                eigenKeypoints = convertOPtoEigenFullFrame(datumProcessed->at(0)->poseKeypoints);
                eigenPafs = createFakePafsFullFrame2(datumProcessed->at(0)->poseKeypoints, def);

                for (int i = 0; i < eigenKeypoints.size(); i++)
                {
                    std::cout << "Joints" << i << ": " << std::endl << eigenKeypoints[i] << std::endl;
                }
                for (int i = 0; i < eigenPafs.size(); i++)
                {
                    std::cout << "FakePafs" << i << " , between joints " << def.pafDict(0,i) << " and " << def.pafDict(1, i) << ": " << std::endl << eigenPafs[i] << std::endl;
                }
                frameDetection.joints = eigenKeypoints;
                frameDetection.pafs = eigenPafs;
                seqDetections.emplace_back(frameDetection);
                SerializeDetections(seqDetections, "testdetection");

                

                //PRINT JOINT CANDIDATES

                /*const auto& candidates = datumProcessed->at(0)->poseCandidates;
                for (int joint = 0; joint < candidates.size(); joint++)
                {
                    for (int person = 0; person < candidates[joint].size(); person++)
                    {
                        std::cout << "Candidate of joint " << joint << " ,of personcand " << person << ": " << std::endl;
                        for (int i = 0; i < 3; i++)
                        {
                            std::cout << candidates[joint][person][i] << std::endl;
                        }
                    }
                }*/
            }
        }

        op::opLog("Stopping thread(s)", op::Priority::High);
        opWrapperT.stop();

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

int main(int argc, char *argv[])
{
    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_net_resolution = "320x176";
    FLAGS_part_to_show = 0;
    //FLAGS_write_images = "C:\\Users\\nykri\\Documents\\3Dhuman\\openpose\\examples\\media2\\output";
    //FLAGS_heatmaps_add_PAFs = true;
    //FLAGS_part_candidates = true;
    //FLAGS_heatmaps_add_bkg = true;


    // Running tutorialApiCpp
    return tutorialApiCpp();
}
