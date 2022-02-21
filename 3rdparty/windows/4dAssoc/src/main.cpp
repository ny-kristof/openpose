#include "kruskal_associater.h"
#include "skel_updater.h"
#include "skel_painter.h"
#include "openpose.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <json/json.h>

// Command-line user interface
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>


void configureWrapper(op::Wrapper& opWrapper)
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
        // producerType
        op::ProducerType producerType;
        op::String producerString;
        std::tie(producerType, producerString) = op::flagsToProducer(
            op::String(FLAGS_image_dir), op::String(FLAGS_video), op::String(FLAGS_ip_camera), FLAGS_camera,
            FLAGS_flir_camera, FLAGS_flir_camera_index);
        // cameraSize
        const auto cameraSize = op::flagsToPoint(op::String(FLAGS_camera_resolution), "-1x-1");
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
        const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
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
        opWrapper.configure(wrapperStructPose);
        // Face configuration (use op::WrapperStructFace{} to disable it)
        const op::WrapperStructFace wrapperStructFace{
            FLAGS_face, faceDetector, faceNetInputSize,
            op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
            (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold };
        opWrapper.configure(wrapperStructFace);
        // Hand configuration (use op::WrapperStructHand{} to disable it)
        const op::WrapperStructHand wrapperStructHand{
            FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
            op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
            (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold };
        opWrapper.configure(wrapperStructHand);
        // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
        const op::WrapperStructExtra wrapperStructExtra{
            FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads };
        opWrapper.configure(wrapperStructExtra);
        // Producer (use default to disable any input)
        const op::WrapperStructInput wrapperStructInput{
            producerType, producerString, FLAGS_frame_first, FLAGS_frame_step, FLAGS_frame_last,
            FLAGS_process_real_time, FLAGS_frame_flip, FLAGS_frame_rotate, FLAGS_frames_repeat,
            cameraSize, op::String(FLAGS_camera_parameter_path), FLAGS_frame_undistort, FLAGS_3d_views };
        opWrapper.configure(wrapperStructInput);
        // Output (comment or use default argument to disable any output)
        const op::WrapperStructOutput wrapperStructOutput{
            FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
            op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
            FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
            op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
            op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
            op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
            op::String(FLAGS_udp_port) };
        opWrapper.configure(wrapperStructOutput);
        // GUI (comment or use default argument to disable any visual output)
        const op::WrapperStructGui wrapperStructGui{
            op::flagsToDisplayMode(FLAGS_display, FLAGS_3d), !FLAGS_no_gui_verbose, FLAGS_fullscreen };
        opWrapper.configure(wrapperStructGui);
        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            opWrapper.disableMultiThreading();
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

int openPoseDemo()
{
    try
    {
        op::opLog("Starting OpenPose demo...", op::Priority::High);
        const auto opTimer = op::getTimerInit();

        // Configure OpenPose
        op::opLog("Configuring OpenPose...", op::Priority::High);
        op::Wrapper opWrapper;
        configureWrapper(opWrapper);

        // Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
        op::opLog("Starting thread(s)...", op::Priority::High);
        opWrapper.exec();

        // Measuring total time
        op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);

        // Return successful message
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

    FLAGS_net_resolution = "320x176";

    // Running openPoseDemo
    return openPoseDemo();

	const std::string dataset = "shelf";    // data/*dataset* mappa neve 
	std::map<std::string, Camera> cameras = ParseCameras("../data/" + dataset + "/calibration.json"); //dataset mappában a calibrációs json fájl
	Eigen::Matrix3Xf projs(3, cameras.size() * 4);
	std::vector<cv::Mat> rawImgs(cameras.size());
	std::vector<cv::VideoCapture> videos(cameras.size());  //annyi videót vár, ahány kamera van
	std::vector<std::vector<OpenposeDetection>> seqDetections(cameras.size());
	const SkelDef& skelDef = GetSkelDef(SKEL19);
	std::vector<std::map<int, Eigen::Matrix4Xf>> skels;

#pragma omp parallel for
	for (int i = 0; i < cameras.size(); i++) {
		auto iter = std::next(cameras.begin(), i);   
		std::cout << "Iter: " << i << iter->second.eiProj << std::endl;
		videos[i] = cv::VideoCapture("../data/" + dataset + "/video/" + iter->first + ".mp4"); //egy videó beolvasás
		videos[i].set(cv::CAP_PROP_POS_FRAMES, 0); //elsõ frame-el kezdjük

		projs.middleCols(4 * i, 4) = iter->second.eiProj;
		seqDetections[i] = ParseDetections("../data/" + dataset + "/detection/" + iter->first + ".txt");   //detektált jointok kiolvasása fájlból
		cv::Size imgSize(int(videos[i].get(cv::CAP_PROP_FRAME_WIDTH)), int(videos[i].get(cv::CAP_PROP_FRAME_HEIGHT)));
		for (auto&& detection : seqDetections[i]) {
			for (auto&& joints : detection.joints) {
				joints.row(0) *= (imgSize.width - 1);
				joints.row(1) *= (imgSize.height - 1);
			}
		}
		rawImgs[i].create(imgSize, CV_8UC3);   //üres képkockák létrehozása a videó méretével
	}

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
	cv::Mat detectImg, assocImg, reprojImg;
	cv::Mat resizeImg;
	for (int frameIdx = 0; ; frameIdx++) { //minden képkockán
		bool flag = true;
		for (int view = 0; view < cameras.size(); view++) { //minden kamerán
			videos[view] >> rawImgs[view];    //következõ képkocka beolvasása és eltárolása
			if (rawImgs[view].empty()) {
				flag = false;
				break;
			}
			cv::resize(rawImgs[view], rawImgs[view], cv::Size(), skelPainter.rate, skelPainter.rate);
			associater.SetDetection(view, seqDetections[view][frameIdx].Mapping(SKEL19));   //egy kamera egy képére illesztjük a szöveges adat detectiont
		}
		if (!flag)
			break;

		associater.SetSkels3dPrev(skelUpdater.GetSkel3d());   //egy framenek az összes kamera detectionbõl (SetDetection) elõállítjuk a 3D skeletont 
		associater.Associate(); //kiszámoljuk a 3D skeleton paramétereit
		skelUpdater.Update(associater.GetSkels2d(), projs);   //??? vissza projektájuk az egyes képekre 

		
		// save
		const int layoutCols = 3;
		std::vector<cv::Rect> rois = SkelPainter::MergeImgs(rawImgs, detectImg, layoutCols,
			{ rawImgs.begin()->cols, rawImgs.begin()->rows}); //detectImg elõállítása az egyes kamera képekbõl
		detectImg.copyTo(assocImg);
		detectImg.copyTo(reprojImg);

#pragma omp parallel for
		for (int view = 0; view < cameras.size(); view++) {   //a vissza projektált skeletonok visszarajzolása az egyes képekre (még mindig frame-enként)
			const OpenposeDetection detection = seqDetections[view][frameIdx].Mapping(SKEL19);
			skelPainter.DrawDetect(detection.joints, detection.pafs, detectImg(rois[view]));
			for (const auto& skel2d : associater.GetSkels2d())
				skelPainter.DrawAssoc(skel2d.second.middleCols(view * skelDef.jointSize, skelDef.jointSize), assocImg(rois[view]), skel2d.first);

			for(const auto& skel3d : skelUpdater.GetSkel3d())
				skelPainter.DrawReproj(skel3d.second, projs.middleCols(4 * view, 4), reprojImg(rois[view]), skel3d.first);
		}
		
		skels.emplace_back(skelUpdater.GetSkel3d()); //a 3D skeleton adatinak visszaírása mátrixba
		//a frame kimentése képbe
		cv::imwrite("../output/detect/" + std::to_string(frameIdx) + ".jpg", detectImg);
		cv::imwrite("../output/assoc/" + std::to_string(frameIdx) + ".jpg", assocImg);
		cv::imwrite("../output/reproj/" + std::to_string(frameIdx) + ".jpg", reprojImg);
		std::cout << std::to_string(frameIdx) << std::endl;
	}

	SerializeSkels(skels, "../output/skel.txt"); //a 3D skeleton kiírása fájlba
	return 0;
}
