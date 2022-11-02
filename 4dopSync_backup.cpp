// --- OpenPose C++ API Tutorial - Example 17 - Custom Input, Pre-processing, Post-processing, Output, and Datum ---
// Synchronous mode: ideal for production integration. It provides the fastest results with respect to runtime
// performance.
// In this function, the user can implement its own way to read frames, implement its own post-processing (i.e., his
// function will be called after OpenPose has processed the frames but before saving), visualizing any result
// render/display/storage the results, and use their custom Datum structure

//4D association graph dependencies
#include "skel_updater.h"
#include "skel_painter.h"
#include "kruskal_associater.h"
#include "openpose.h"
#include "camera.cpp"
#include <Eigen/Eigen>
#include <conio.h>

// Third-party dependencies
#include <opencv2/opencv.hpp>
// Command-line user interface
#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#define OPENPOSE_FLAGS_DISABLE_DISPLAY
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>

// Custom OpenPose flags
// Producer
DEFINE_string(image_dir, "examples/media/",
    "Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).");
// Display
DEFINE_bool(no_display, false,
    "Enable to disable the visual display.");
DEFINE_bool(save_images, false,
    "Enable to save the detecton, association and reprojection images created by the 4Dasscoc algorithm");
//Using webcam or demo videos
DEFINE_bool(use_webcams, false,
    "Wether to use the webcams for live demo or use the prerecorded sample videos");
//Undistort the input images
DEFINE_bool(undist, false,
    "Wether to undistort the input images. The distortion parameters have to be given");



// This worker will just read and return all the basic image file formats in a directory
class WUserInput : public op::WorkerProducer<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
{
public:
    WUserInput() :
        mCounter{ 0 },
        cameras{ ParseCameras("C:/Users/admin/Documents/EasyMocap/data/live/calibration.json") },
        numCamera{ 0 },
        mClosed{ false },
        ips{ "169.254.150.101","169.254.150.102","169.254.150.103","169.254.150.104" }
    {
        if (ips.size() != cameras.size())
        {
            std::cout << "Ipsize: " << ips.size() << std::endl;
            std::cout << "Camsize: " << cameras.size() << std::endl;
            op::error("Number of IPs is wrong!");
        }
        streams.resize(cameras.size());

        for (int i = 0; i < cameras.size(); i++)
        {
            //TODO: stremek init... fú remélem ez sokkal rövidebb lesz, mint az első verzió
            streams[i] = new op::IpCameraReader("http://admin:admin123@" + ips[i] + "/cgi-bin/mjpg/video.cgi?channel=1&subtype=1", op::Point<int>{640, 480});
            if (!streams[i]->isOpened())
            {
                std::cout << "Camera " << i << " hasn't been opened." << std::endl;
            }
        }
        //stream = new op::IpCameraReader("http://admin:admin123@169.254.150.104/cgi-bin/mjpg/video.cgi?channel=1&subtype=1", op::Point<int>{640, 480});
        rawImgOpMat = op::Matrix(cameras.begin()->second.imgSize.width, cameras.begin()->second.imgSize.height, CV_8UC3);

        //prepare undistort
        map1s.resize(cameras.size());
        map2s.resize(cameras.size());
        if (FLAGS_undist)
        {
            for (auto const& cam : cameras)
            {
                std::cout << "Dist coeff of cam " << cam.first << ": " << cam.second.distCoeff << std::endl;
                std::cout << "Intri matrix of cam " << cam.first << ": " << cam.second.originK << std::endl;
                std::cout << "Img size of cam  " << cam.first << ": " << cameras.begin()->second.imgSize << std::endl;
                cv::initUndistortRectifyMap(cam.second.originK, cam.second.distCoeff, cv::Mat(), cam.second.originK, cameras.begin()->second.imgSize, CV_32FC1, map1s[std::stoi(cam.first)], map2s[std::stoi(cam.first)]);
            }
            //TODO: distortion parameters 
            //cv::Mat dist = (cv::Mat_<double>(5, 1) << -4.3733665180754527e-01, 2.1921356552711482e-01, 1.1597452449095796e-03,
            //    4.6874816837838441e-03, -5.9483271093969864e-02);
            //cv::initUndistortRectifyMap(cameras.begin()->second.originK, dist, cv::Mat(), cameras.begin()->second.originK, cameras.begin()->second.imgSize, CV_32FC1, map1, map2);
        }
    }

    void initializationOnThread() {}

    unsigned long long getCounter()
    {
        return mCounter;
    }

    std::map<std::string, Camera> getCameras()
    {
        return cameras;
    }

    bool getClosed()
    {
        return mClosed;
    }

    int getNumCam()
    {
        return numCamera;
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

    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> workProducer()
    {
        try
        {
            // Close program when empty frame
            if (1200 <= mCounter && kbhit())
            {
                op::opLog(
                    "Last frame read and added to queue. Closing program after it is processed.", op::Priority::High);
                // This funtion stops this worker, which will eventually stop the whole thread system once all the
                // frames have been processed
                mClosed = true;
                this->stop();
                return nullptr;
            }
            else
            {
                numCamera = mCounter % cameras.size();
                // Create new datum
                auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
                datumsPtr->emplace_back();
                auto& datumPtr = datumsPtr->at(0);
                datumPtr = std::make_shared<op::Datum>();

                // Fill datum
                /*const cv::Mat cvInputData = cv::imread(mImageFiles.at(mCounter++));
                datumPtr->cvInputData = OP_CV2OPCONSTMAT(cvInputData);*/
                rawImgOpMat = streams[numCamera]->getFrame();
                if (FLAGS_undist) cv::remap(OP_OP2CVMAT(rawImgOpMat), OP_OP2CVMAT(rawImgOpMat), map1s[mCounter % cameras.size()], map2s[mCounter % cameras.size()], cv::INTER_LINEAR);
                datumPtr->cvInputData = rawImgOpMat;
                std::cout << mCounter++ << std::endl;

                // If empty frame -> return nullptr
                if (datumPtr->cvInputData.empty())
                {
                    op::opLog(
                        "Empty frame detected at frame: " + mCounter,
                        op::Priority::High);
                    this->stop();
                    datumsPtr = nullptr;
                }

                return datumsPtr;
            }
        }
        catch (const std::exception& e)
        {
            this->stop();
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return nullptr;
        }
    }

private:
    unsigned long long mCounter;
    std::map<std::string, Camera> cameras;
    int numCamera;
    std::vector<op::IpCameraReader*> streams;
    std::vector<std::string> ips;
    bool mClosed;
    op::Matrix rawImgOpMat;
    std::vector<cv::Mat> map1s;
    std::vector<cv::Mat> map2s;
};

// This worker will just invert the image
class WUserPostProcessing : public op::Worker<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
{
public:
    WUserPostProcessing(std::shared_ptr<WUserInput> uinput) :
        def{ GetSkelDef(SkelType(BODY25)) },
        skelDef{ GetSkelDef(SKEL19) },
        skelPainter{ SkelPainter(SKEL19) },
        skelUpdater{ SkelFittingUpdater(SKEL19, "C:/Users/admin/Documents/openpose/src4D/SKEL19_new") }
    {
        uInput = uinput;
        projs = Eigen::Matrix3Xf(3, uInput->getCameras().size() * 4);
        rawImgs.resize(uInput->getCameras().size());

        for (int i = 0; i < uInput->getCameras().size(); i++)
        {
            auto iter = std::next(uInput->getCameras().begin(), i);
            projs.middleCols(4 * i, 4) = iter->second.eiProj;
            rawImgs[i].create(uInput->getCameras().begin()->second.imgSize, CV_8UC3);
        }

        frameDetection = OpenposeDetection(BODY25);
        IOScale = (float)rawImgs[0].rows / std::stof(FLAGS_net_resolution.substr(FLAGS_net_resolution.find('x') + 1));

        //paraméterek beállítása
        associater = KruskalAssociater(SKEL19, uInput->getCameras());
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
        associater.SetNormalizeEdge(true);

        skelPainter.rate = 512.f / float(uInput->getCameras().begin()->second.imgSize.width);
        skelUpdater.SetTemporalTransTerm(1e-1f / std::powf(skelPainter.rate, 2));
        skelUpdater.SetTemporalPoseTerm(1e-1f / std::powf(skelPainter.rate, 2));
    }

    void initializationOnThread() {}

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

    void work(std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
    {
        try
        {
            // User's displaying/saving/other processing here
                // datumPtr->cvOutputData: rendered frame with pose or heatmaps
                // datumPtr->poseKeypoints: Array<float> with the estimated pose
            if (datumsPtr != nullptr && !datumsPtr->empty())
            {
                const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
                // Display results (if enabled)
                if (!FLAGS_no_display)
                {
                    // Display rendered output image
                    if (!cvMat.empty())
                    {
                        cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
                        // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
                        const char key = (char)cv::waitKey(1);
                        if (key == 27)
                            this->stop();
                    }
                    else
                        op::opLog("Empty cv::Mat as output.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
                }

                frameDetection.joints = (FLAGS_part_candidates) ? convertOPCandidatesToEigenFullFrame(datumsPtr->at(0)->poseCandidates) : convertOPtoEigenFullFrame(datumsPtr->at(0)->poseKeypoints);
                frameDetection.pafs = (FLAGS_part_candidates) ? createPafsFullFrame(datumsPtr, def, IOScale) : createFakePafsFullFrame(datumsPtr->at(0)->poseKeypoints, def);
                auto& mappedDetection = frameDetection.Mapping(SKEL19);
				//TODO: getNumCam hibás, mert nem független a preprocessintől. UserDatum-mal kell dolgozni és annak egy változója mondja meg, hogy melyik kameránál járunk.
                associater.SetDetection(uInput->getNumCam(), mappedDetection);
                rawImgs[uInput->getNumCam()] = cvMat;
                cv::resize(rawImgs[uInput->getNumCam()], rawImgs[uInput->getNumCam()], cv::Size(), skelPainter.rate, skelPainter.rate);

            }

            if (uInput->getNumCam() == uInput->getCameras().size() - 1)
            {
                associater.SetSkels3dPrev(skelUpdater.GetSkel3d());   //egy framenek az összes kamera detectionbõl (SetDetection) elõállítjuk a 3D skeletont 
                associater.Associate(); //kiszámoljuk a 3D skeleton paramétereit
                skelUpdater.Update(associater.GetSkels2d(), projs);   //Updateljük az eddig meghatározott skeletonokat az újonnan meghatározottakkal.
                skels.emplace_back(skelUpdater.GetSkel3d()); //a 3D skeleton adatinak visszaírása mátrixba

                //Saving images of detection, association and reprojection of the assembled 3D skeleton
                if (FLAGS_save_images) {
                    std::cout << "Saving images" << std::endl;
                    cv::Mat detectImg, assocImg, reprojImg;
                    const int layoutCols = 3;
                    ////TODO:Ez itt gond lesz, mert a rawImgsOpMat változókba mentjük az ipCameraReader-bõl érkezõ frameket. Így ez csak a videós megoldásra mûködik --> de, mert mostmár a getFramenél belerakjuk a rawImgs-be is a képet, hogy meglegyen cv::Mat-ban is 
                    std::vector<cv::Rect> rois = SkelPainter::MergeImgs(rawImgs, detectImg, layoutCols,
                        { rawImgs.begin()->cols, rawImgs.begin()->rows }); //az összes kamerakép összeillesztése egy gridbe (detectImg), erre rajzoljuk majd a végeredményt. Rois: A grid kis képeinek befoglaló téglalapjainak koordinátái (nagy kép almátrixa)
                    detectImg.copyTo(assocImg);
                    detectImg.copyTo(reprojImg);

#pragma omp parallel for
                    for (int view = 0; view < uInput->getCameras().size(); view++) {   //a vissza projektált skeletonok visszarajzolása az egyes képekre (még mindig frame-enként)
                        const OpenposeDetection detection = associater.GetDetections()[view];
                        skelPainter.DrawDetect(detection.joints, detection.pafs, detectImg(rois[view]));
                        for (const auto& skel2d : associater.GetSkels2d())
                            skelPainter.DrawAssoc(skel2d.second.middleCols(view * skelDef.jointSize, skelDef.jointSize), assocImg(rois[view]), skel2d.first);

                        for (const auto& skel3d : skelUpdater.GetSkel3d())
                            skelPainter.DrawReproj(skel3d.second, projs.middleCols(4 * view, 4), reprojImg(rois[view]), skel3d.first);
                    }

                    //a frame kimentése képbe
                    //cv::imwrite("output/detect93u/" + std::to_string(framecount) + ".jpg", detectImg);
                    //cv::imwrite("output/assoc93u/" + std::to_string(framecount) + ".jpg", assocImg);
                    //cv::imwrite("output/reproj93u/" + std::to_string(framecount) + ".jpg", reprojImg);
                    cv::imshow("reproj", reprojImg);
                    cv::imshow("detect", detectImg);
                    cv::waitKey(1);
                }
            }

            std::cout << "------ End of processing frame " << std::to_string(uInput->getCounter()) << " ------" << std::endl;

            if (uInput->getClosed())
            {
                //TODO: postprocess after program closed
                SerializeSkels(skels, "output/skel.txt"); //a 3D skeleton kiírása  
            }
        }
        catch (const std::exception& e)
        {
            this->stop();
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }
private:
    Eigen::Matrix3Xf projs;
    std::shared_ptr<WUserInput> uInput;
    std::vector<cv::Mat> rawImgs;
    SkelDef def;
    const SkelDef& skelDef;
    OpenposeDetection frameDetection;
    float IOScale;
    KruskalAssociater associater;
    SkelPainter skelPainter;
    SkelFittingUpdater skelUpdater;
    std::vector<std::map<int, Eigen::Matrix4Xf>> skels;
};

// This worker will just read and return all the jpg files in a directory
//class WUserOutput : public op::WorkerConsumer<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
//{
//public:
//    WUserOutput(std::shared_ptr<WUserInput> uinput) :
//        def{ GetSkelDef(SkelType(BODY25)) },
//        skelDef{ GetSkelDef(SKEL19) },
//        skelPainter{ SkelPainter(SKEL19) },
//        skelUpdater{ SkelFittingUpdater(SKEL19, "C:/Users/admin/Documents/openpose/src4D/SKEL19_new") }
//    {
//        uInput = uinput;
//        projs = Eigen::Matrix3Xf(3, uInput->getCameras().size() * 4);
//        rawImgs.resize(uInput->getCameras().size());
//
//        for (int i = 0; i < uInput->getCameras().size(); i++)
//        {
//            auto iter = std::next(uInput->getCameras().begin(), i);
//            projs.middleCols(4 * i, 4) = iter->second.eiProj;
//            rawImgs[i].create(uInput->getCameras().begin()->second.imgSize, CV_8UC3);
//        }
//
//        frameDetection = OpenposeDetection(BODY25);
//        IOScale = (float)rawImgs[0].rows / std::stof(FLAGS_net_resolution.substr(FLAGS_net_resolution.find('x') + 1));
//
//        //paraméterek beállítása
//        associater = KruskalAssociater(SKEL19, uInput->getCameras());
//        associater.SetMaxTempDist(0.3f);
//        associater.SetMaxEpiDist(0.15f);
//        associater.SetEpiWeight(1.f);
//        associater.SetTempWeight(0.5f);
//        associater.SetViewWeight(1.f);
//        associater.SetPafWeight(2.f);
//        associater.SetHierWeight(1.f);
//        associater.SetViewCntWelsh(1.0);
//        associater.SetMinCheckCnt(10);
//        associater.SetNodeMultiplex(true);
//        associater.SetNormalizeEdge(true);
//
//        skelPainter.rate = 512.f / float(uInput->getCameras().begin()->second.imgSize.width);
//        skelUpdater.SetTemporalTransTerm(1e-1f / std::powf(skelPainter.rate, 2));
//        skelUpdater.SetTemporalPoseTerm(1e-1f / std::powf(skelPainter.rate, 2));
//
//    }
//    void initializationOnThread() {}
//
//    //Convert the OpenPose keypoint structure for the 4D association algorithm ([person, joint, xyscore] --> [joint, xyscore, person]) 
//    std::vector<Eigen::Matrix3Xf> convertOPtoEigenFullFrame(op::Array<float>& opOutput) {
//        std::vector<Eigen::Matrix3Xf> eigenKeypointsVector(GetSkelDef(BODY25).jointSize);
//        if (!opOutput.empty())
//        {
//            for (int jointID = 0; jointID < opOutput.getSize()[1]; jointID++)
//            {
//                Eigen::Matrix3Xf jointOfPeople(3, 0);
//
//                for (int personID = 0; personID < opOutput.getSize()[0]; personID++)
//                {
//                    if (opOutput[{personID, jointID, 0}] != 0)
//                    {
//                        jointOfPeople.conservativeResize(Eigen::NoChange, jointOfPeople.cols() + 1);
//
//                        for (int xyscore = 0; xyscore < opOutput.getSize()[2]; xyscore++)
//                        {
//                            jointOfPeople(xyscore, jointOfPeople.cols() - 1) = opOutput[{personID, jointID, xyscore}];
//                        }
//                    }
//                }
//                eigenKeypointsVector[jointID] = jointOfPeople;
//            }
//        }
//        return eigenKeypointsVector;
//    }
//
//    std::vector<Eigen::Matrix3Xf> convertOPCandidatesToEigenFullFrame(std::vector<std::vector<std::array<float, 3Ui64>>>& opOutput)
//    {
//        std::vector<Eigen::Matrix3Xf> eigenKeypointsVector(GetSkelDef(BODY25).jointSize);
//        if (!opOutput.empty())
//        {
//            for (int jID = 0; jID < opOutput.size(); jID++)
//            {
//                Eigen::Matrix3Xf jointOfPeople(3, 0);
//
//                for (int personID = 0; personID < opOutput[jID].size(); personID++)
//                {
//                    jointOfPeople.conservativeResize(Eigen::NoChange, jointOfPeople.cols() + 1);
//                    jointOfPeople.col(personID) = Eigen::Vector3f(opOutput[jID][personID].data());
//                }
//                eigenKeypointsVector[jID] = jointOfPeople;
//            }
//        }
//        return eigenKeypointsVector;
//    }
//
//    //Returns a vector with the indexes of the people, whose given joint (jointID) was found
//    std::vector<int> validPersonOfJoint(op::Array<float>& opOutput, int jointID)
//    {
//        std::vector<int> validJoints;
//        for (int i = 0; i < opOutput.getSize(0); i++)
//        {
//            if (opOutput[{i, jointID, 0}] != (float)0.0)
//            {
//                validJoints.emplace_back(i);
//            }
//        }
//        return validJoints;
//    }
//
//    //Joint matrix has rows (A joints) and jolumns (B joints). Each element represents a connection between the given A and B joints. The value is bigger, if the two given links have high chance, that they form a real link.
//    //This should be extracted from the OpenPose system before it assembles the keypoints and the heatmaps into people. Instead, we use the assembled people by OpenPose and generate "fake" matrixes (PAFs) for the 4D association algorithm.
//    std::vector<Eigen::MatrixXf> createFakePafsFullFrame(op::Array<float>& opOutput, SkelDef& def) {
//        //Each element of the vector is a matrix that shows where that specific link is (for examle: 0 1 0 row tells us(this is the first row), that the link this matrix represents is given between the first found joint A and the second found joint B)
//        std::vector<Eigen::MatrixXf> eigenPafVector(GetSkelDef(BODY25).pafSize);
//
//        if (!opOutput.empty())
//        {
//            for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
//                Eigen::MatrixXf eigenPafOfPair;
//                const int jAIdx = def.pafDict(0, pafIdx);
//                const int jBIdx = def.pafDict(1, pafIdx);
//                std::vector<int> validPeopleOfJointA = validPersonOfJoint(opOutput, jAIdx);
//                std::vector<int> validPeopleOfJointB = validPersonOfJoint(opOutput, jBIdx);
//
//                eigenPafOfPair.resize(validPeopleOfJointA.size(), validPeopleOfJointB.size());
//                for (int i = 0; i < eigenPafOfPair.rows(); i++) {
//                    for (int j = 0; j < eigenPafOfPair.cols(); j++) {
//                        if (validPeopleOfJointA[i] == validPeopleOfJointB[j])
//                            eigenPafOfPair(i, j) = (float)1.0;
//                        else eigenPafOfPair(i, j) = (float)0.0;
//                    }
//                }
//                eigenPafVector[pafIdx] = eigenPafOfPair;
//            }
//        }
//
//        return eigenPafVector;
//    }
//
//    std::vector<int> intArange(float start, float stop, float step = 1) {
//        std::vector<int> values;
//        if (start < stop)
//            for (float value = start; value < stop; value += step)
//                values.push_back((int)(value + 0.5));
//        else
//            for (float value = start; value > stop; value += step)
//                values.push_back((int)(value + 0.5));
//        return values;
//    }
//
//    float calcScoreForPafMatrix(float x1, float y1, float x2, float y2, cv::Mat pafMatX, cv::Mat pafMatY)
//    {
//        int num_iter = 10;
//        float dx = x2 - x1;
//        float dy = y2 - y1;
//        float normVec = std::sqrt(dx * dx + dy * dy);
//
//        if (normVec < 1e-4)
//            return 0.0;
//
//        float vx = dx / normVec;
//        float vy = dy / normVec;
//
//        std::vector<int> xs;
//        std::vector<int> ys;
//
//        if ((int)x1 != (int)x2)
//            xs = intArange(x1, x2, dx / (float)num_iter);
//        else
//            for (int i = 0; i < num_iter; i++)
//                xs.push_back((int)(x1 + 0.5));
//
//        if ((int)y1 != (int)y2)
//            ys = intArange(y1, y2, dy / (float)num_iter);
//        else
//            for (int i = 0; i < num_iter; i++)
//                ys.push_back((int)(y1 + 0.5));
//
//        std::vector<float> pafXs(num_iter);
//        std::vector<float> pafYs(num_iter);
//
//        for (int i = 0; i < pafXs.size(); i++)
//        {
//            if ((ys[i] > pafMatX.rows) | (ys[i] > pafMatY.rows) | (xs[i] > pafMatX.cols) | (xs[i] > pafMatY.cols))
//            {
//                std::cout << "ERROR: index of heatmap array is out of bounds!!!!!!!!!!!" << std::endl;
//                std::cout << "pafMatX.rows: " << pafMatX.rows << std::endl;
//                std::cout << "pafMatX.cols: " << pafMatX.cols << std::endl;
//                std::cout << "pafMatY.rows: " << pafMatY.rows << std::endl;
//                std::cout << "pafMatY.cols: " << pafMatY.cols << std::endl;
//                std::cout << "ys[i]: " << ys[i] << std::endl;
//                std::cout << "xs[i]: " << xs[i] << std::endl;
//            }
//            //pafXs[i] = pafMatX[{xs[i], ys[i]}];
//            pafXs[i] = pafMatX.at<float>(ys[i], xs[i]);
//            //pafYs[i] = pafMatY[{xs[i], ys[i]}];
//            pafYs[i] = pafMatY.at<float>(ys[i], xs[i]);
//        }
//
//        float localscoreX = 0;
//        float localscoreY = 0;
//
//        for (int i = 0; i < pafXs.size(); ++i)
//            localscoreX += pafXs[i] * vx;
//        for (int i = 0; i < pafYs.size(); ++i)
//            localscoreY += pafYs[i] * vy;
//
//        float finalscore = ((localscoreX + localscoreY) / num_iter > 0.1) ? ((localscoreX + localscoreY) / num_iter) : 0.0;
//
//        return finalscore;
//    }
//
//    //TODO: Azok a paf mátrixok hibásak, ahol a heatmapen egy pozitív és egy negatív vektorhalmazzal dolgozunk (egyik tag pl felkar pozitív, a másik pedig negatív irányba mutat) 
//    std::vector<Eigen::MatrixXf> createPafsFullFrame(std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datum, SkelDef& def, float scale) {
//        //Each element of the vector is a matrix that shows where that specific link is (for examle: 0 1 0 row tells us(this is the first row), that the link this matrix represents is given between the first found joint A and the second found joint B)
//        std::vector<Eigen::MatrixXf> eigenPafVector(GetSkelDef(BODY25).pafSize);
//        auto& poseCandidates = datum->at(0)->poseCandidates;
//        auto& pafHeatMaps = datum->at(0)->poseHeatMaps;
//        const auto numberChannels = pafHeatMaps.getSize(0);
//        const auto height = pafHeatMaps.getSize(1);
//        const auto width = pafHeatMaps.getSize(2);
//
//        cv::Mat desiredChannelHeatMapX;
//        cv::Mat desiredChannelHeatMapY;
//
//        if (!poseCandidates.empty())
//        {
//            for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
//                Eigen::MatrixXf eigenPafOfPair;
//                const int jAIdx = def.pafDict(0, pafIdx);
//                const int jBIdx = def.pafDict(1, pafIdx);
//                //std::vector<int> validPeopleOfJointA = validPersonOfJoint(opOutput, jAIdx);
//                //std::vector<int> validPeopleOfJointB = validPersonOfJoint(opOutput, jBIdx);
//
//                eigenPafOfPair.resize(poseCandidates[jAIdx].size(), poseCandidates[jBIdx].size());
//                for (int i = 0; i < eigenPafOfPair.rows(); i++) {
//                    for (int j = 0; j < eigenPafOfPair.cols(); j++) {
//                        auto& xHeatMap = cv::Mat(height, width, CV_32F, &pafHeatMaps.getPtr()[(2 * pafIdx) % numberChannels * height * width]);
//                        auto& yHeatMap = cv::Mat(height, width, CV_32F, &pafHeatMaps.getPtr()[(2 * pafIdx + 1) % numberChannels * height * width]);
//
//                        //for (int i = 0; i < xHeatMap.cols; i++)
//                        //    //std::cout << xHeatMap.at<float>(100, i) << "  ";
//                        //    std::cout << i << ": " << pafHeatMaps[{0, 100, i}] << std::endl;
//                        //cv::imshow("heatmapX", xHeatMap);
//                        //cv::waitKey(0);
//                        //cv::imshow("heatmapY", yHeatMap);
//                        //cv::waitKey(0);
//                        //continue;
//                        //std::cout << "xHeatMap width and height: " << xHeatMap.cols << "x" << xHeatMap.rows << std::endl;
//                        //std::cout << "yHeatMap width and height: " << yHeatMap.cols << "x" << yHeatMap.rows << std::endl;
//                        //std::cout << "x2, y1, x2, y2 positions: " << poseCandidates[jAIdx][i][0] / scale << " , " << poseCandidates[jAIdx][i][1] / scale << " , " << poseCandidates[jBIdx][j][0] / scale << " , " << poseCandidates[jBIdx][j][1] / scale << std::endl;
//                        eigenPafOfPair(i, j) = calcScoreForPafMatrix(poseCandidates[jAIdx][i][0] / scale, poseCandidates[jAIdx][i][1] / scale, poseCandidates[jBIdx][j][0] / scale, poseCandidates[jBIdx][j][1] / scale, xHeatMap, yHeatMap);
//                    }
//                }
//                //std::cout << "Pafs" << pafIdx << " , between joints " << jAIdx << " and " << jBIdx << ": " << std::endl << eigenPafOfPair << std::endl;
//                eigenPafVector[pafIdx] = eigenPafOfPair;
//            }
//        }
//
//        return eigenPafVector;
//    }
//
//    void workConsumer(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
//    {
//        try
//        {
//            // User's displaying/saving/other processing here
//                // datumPtr->cvOutputData: rendered frame with pose or heatmaps
//                // datumPtr->poseKeypoints: Array<float> with the estimated pose
//            if (datumsPtr != nullptr && !datumsPtr->empty())
//            {
//                const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
//                // Display results (if enabled)
//                if (!FLAGS_no_display)
//                {
//                    // Display rendered output image
//                    if (!cvMat.empty())
//                    {
//                        cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
//                        // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
//                        const char key = (char)cv::waitKey(1);
//                        if (key == 27)
//                            this->stop();
//                    }
//                    else
//                        op::opLog("Empty cv::Mat as output.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
//                }
//
//                frameDetection.joints = (FLAGS_part_candidates) ? convertOPCandidatesToEigenFullFrame(datumsPtr->at(0)->poseCandidates) : convertOPtoEigenFullFrame(datumsPtr->at(0)->poseKeypoints);
//                frameDetection.pafs = (FLAGS_part_candidates) ? createPafsFullFrame(datumsPtr, def, IOScale) : createFakePafsFullFrame(datumsPtr->at(0)->poseKeypoints, def);
//                auto& mappedDetection = frameDetection.Mapping(SKEL19);
//                associater.SetDetection(uInput->getNumCam(), mappedDetection);
//                rawImgs[uInput->getNumCam()] = cvMat;
//                cv::resize(rawImgs[uInput->getNumCam()], rawImgs[uInput->getNumCam()], cv::Size(), skelPainter.rate, skelPainter.rate);
//
//            }
//
//            if (uInput->getNumCam() == uInput->getCameras().size() - 1)
//            {
//                associater.SetSkels3dPrev(skelUpdater.GetSkel3d());   //egy framenek az összes kamera detectionbõl (SetDetection) elõállítjuk a 3D skeletont 
//                associater.Associate(); //kiszámoljuk a 3D skeleton paramétereit
//                skelUpdater.Update(associater.GetSkels2d(), projs);   //Updateljük az eddig meghatározott skeletonokat az újonnan meghatározottakkal.
//                skels.emplace_back(skelUpdater.GetSkel3d()); //a 3D skeleton adatinak visszaírása mátrixba
//
//                //Saving images of detection, association and reprojection of the assembled 3D skeleton
//                if (FLAGS_save_images) {
//                    std::cout << "Saving images" << std::endl;
//                    cv::Mat detectImg, assocImg, reprojImg;
//                    const int layoutCols = 3;
//                    ////TODO:Ez itt gond lesz, mert a rawImgsOpMat változókba mentjük az ipCameraReader-bõl érkezõ frameket. Így ez csak a videós megoldásra mûködik --> de, mert mostmár a getFramenél belerakjuk a rawImgs-be is a képet, hogy meglegyen cv::Mat-ban is 
//                    std::vector<cv::Rect> rois = SkelPainter::MergeImgs(rawImgs, detectImg, layoutCols,
//                        { rawImgs.begin()->cols, rawImgs.begin()->rows }); //az összes kamerakép összeillesztése egy gridbe (detectImg), erre rajzoljuk majd a végeredményt. Rois: A grid kis képeinek befoglaló téglalapjainak koordinátái (nagy kép almátrixa)
//                    detectImg.copyTo(assocImg);
//                    detectImg.copyTo(reprojImg);
//
//#pragma omp parallel for
//                    for (int view = 0; view < uInput->getCameras().size(); view++) {   //a vissza projektált skeletonok visszarajzolása az egyes képekre (még mindig frame-enként)
//                        const OpenposeDetection detection = associater.GetDetections()[view];
//                        skelPainter.DrawDetect(detection.joints, detection.pafs, detectImg(rois[view]));
//                        for (const auto& skel2d : associater.GetSkels2d())
//                            skelPainter.DrawAssoc(skel2d.second.middleCols(view * skelDef.jointSize, skelDef.jointSize), assocImg(rois[view]), skel2d.first);
//
//                        for (const auto& skel3d : skelUpdater.GetSkel3d())
//                            skelPainter.DrawReproj(skel3d.second, projs.middleCols(4 * view, 4), reprojImg(rois[view]), skel3d.first);
//                    }
//
//                    //a frame kimentése képbe
//                    //cv::imwrite("output/detect93u/" + std::to_string(framecount) + ".jpg", detectImg);
//                    //cv::imwrite("output/assoc93u/" + std::to_string(framecount) + ".jpg", assocImg);
//                    //cv::imwrite("output/reproj93u/" + std::to_string(framecount) + ".jpg", reprojImg);
//                    cv::imshow("reproj", reprojImg);
//                    cv::imshow("detect", detectImg);
//                    cv::waitKey(1);
//                }
//            }
//
//            std::cout << "------ End of processing frame " << std::to_string(uInput->getCounter()) << " ------" << std::endl;
//
//            if (uInput->getClosed())
//            {
//                //TODO: postprocess after program closed
//                SerializeSkels(skels, "output/skel.txt"); //a 3D skeleton kiírása  
//            }
//        }
//        catch (const std::exception& e)
//        {
//            this->stop();
//            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
//        }
//    }
//private:
//    Eigen::Matrix3Xf projs;
//    std::shared_ptr<WUserInput> uInput;
//    std::vector<cv::Mat> rawImgs;
//    SkelDef def;
//    const SkelDef& skelDef;
//    OpenposeDetection frameDetection;
//    float IOScale;
//    KruskalAssociater associater;
//    SkelPainter skelPainter;
//    SkelFittingUpdater skelUpdater;
//    std::vector<std::map<int, Eigen::Matrix4Xf>> skels;
//};

class WUserOutput : public op::WorkerConsumer<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
{
public:
    void initializationOnThread() {}

    void workConsumer(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
    {
        try
        {
            // User's displaying/saving/other processing here
                // datumPtr->cvOutputData: rendered frame with pose or heatmaps
                // datumPtr->poseKeypoints: Array<float> with the estimated pose
            if (datumsPtr != nullptr && !datumsPtr->empty())
            {
                // Show in command line the resulting pose keypoints for body, face and hands
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
                        for (auto xyscore = 0; xyscore < poseKeypoints.getSize(2); xyscore++)
                        {
                            valueToPrint += std::to_string(poseKeypoints[{person, bodyPart, xyscore}]) + " ";
                        }
                        op::opLog(valueToPrint);
                    }
                }
                op::opLog(" ");
                // Alternative: just getting std::string equivalent
                op::opLog("Face keypoints: " + datumsPtr->at(0)->faceKeypoints.toString());
                op::opLog("Left hand keypoints: " + datumsPtr->at(0)->handKeypoints[0].toString());
                op::opLog("Right hand keypoints: " + datumsPtr->at(0)->handKeypoints[1].toString());
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

                // Display results (if enabled)
                if (!FLAGS_no_display)
                {
                    // Display rendered output image
                    const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
                    if (!cvMat.empty())
                    {
                        cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
                        // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
                        const char key = (char)cv::waitKey(1);
                        if (key == 27)
                            this->stop();
                    }
                    else
                        op::opLog("Empty cv::Mat as output.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
                }
            }
        }
        catch (const std::exception& e)
        {
            this->stop();
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
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

        // Initializing the user custom classes
        // Frames producer (e.g., video, webcam, ...)
        auto wUserInput = std::make_shared<WUserInput>();
        // Processing
        auto wUserPostProcessing = std::make_shared<WUserPostProcessing>(wUserInput);
        // GUI (Display)
        auto wUserOutput = std::make_shared<WUserOutput>();

        // Add custom input
        const auto workerInputOnNewThread = false;
        opWrapperT.setWorker(op::WorkerType::Input, wUserInput, workerInputOnNewThread);
        // Add custom processing
        const auto workerProcessingOnNewThread = false;
        opWrapperT.setWorker(op::WorkerType::PostProcessing, wUserPostProcessing, workerProcessingOnNewThread);
        // Add custom output
        const auto workerOutputOnNewThread = true;
        opWrapperT.setWorker(op::WorkerType::Output, wUserOutput, workerOutputOnNewThread);

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

        // OpenPose wrapper
        op::opLog("Configuring OpenPose...", op::Priority::High);
        op::WrapperT<op::Datum> opWrapperT;
        configureWrapper(opWrapperT);

        // Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
        op::opLog("Starting thread(s)...", op::Priority::High);
        opWrapperT.exec();

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

    FLAGS_part_to_show = 0;
    FLAGS_heatmaps_add_PAFs = false;
    FLAGS_heatmaps_add_bkg = false;
    FLAGS_heatmaps_add_parts = false;
    FLAGS_heatmaps_scale = 0; //0 --> in range [-1, 1] ; 1 --> in range [0, 1] ; 2(default) --> in range [0 , 255]
    //FLAGS_render_pose = 0;

    FLAGS_no_display = true;
    FLAGS_save_images = true;
    FLAGS_use_webcams = true;
    //FLAGS_write_images = "C:\\Users\\nykri\\Documents\\3Dhuman\\openpose\\examples\\media2\\output";
    FLAGS_undist = true;

    // Running tutorialApiCpp
    return tutorialApiCpp();
}
