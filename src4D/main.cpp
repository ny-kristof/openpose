#include "kruskal_associater.h"
#include "skel_updater.h"
#include "skel_painter.h"
#include "openpose.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <json/json.h>


int main()
{
	const std::string dataset = "shelf";    // data/*dataset* mappa neve 
	std::map<std::string, Camera> cameras = ParseCameras("../data/" + dataset + "/calibration.json"); //dataset mapp�ban a calibr�ci�s json f�jl
	Eigen::Matrix3Xf projs(3, cameras.size() * 4);
	std::vector<cv::Mat> rawImgs(cameras.size());
	std::vector<cv::VideoCapture> videos(cameras.size());  //annyi vide�t v�r, ah�ny kamera van
	std::vector<std::vector<OpenposeDetection>> seqDetections(cameras.size());
	const SkelDef& skelDef = GetSkelDef(SKEL19);
	std::vector<std::map<int, Eigen::Matrix4Xf>> skels;

//#pragma omp parallel for
	for (int i = 0; i < cameras.size(); i++) {
		int framec = 0;
		auto iter = std::next(cameras.begin(), i);   
		std::cout << "Iter: " << i << iter->second.eiProj << std::endl;
		videos[i] = cv::VideoCapture("../data/" + dataset + "/video/" + iter->first + ".mp4"); //egy vide� beolvas�s
		videos[i].set(cv::CAP_PROP_POS_FRAMES, 0); //els� frame-el kezdj�k

		projs.middleCols(4 * i, 4) = iter->second.eiProj;
		seqDetections[i] = ParseDetections("../data/" + dataset + "/detection/" + iter->first + ".txt");   //detekt�lt jointok kiolvas�sa f�jlb�l
		cv::Size imgSize(int(videos[i].get(cv::CAP_PROP_FRAME_WIDTH)), int(videos[i].get(cv::CAP_PROP_FRAME_HEIGHT)));
		for (auto&& detection : seqDetections[i]) {
			int jointc = 0;
			for (auto&& joints : detection.joints) {
				joints.row(0) *= (imgSize.width - 1);
				joints.row(1) *= (imgSize.height - 1);
				std::cout << "Joints of camera" << i << ", frame " << framec << ", joint" << jointc << ": " << std::endl << joints << std::endl;
				jointc++;
			}
			framec++;
		}
		rawImgs[i].create(imgSize, CV_8UC3);   //�res k�pkock�k l�trehoz�sa a vide� m�ret�vel
	}
	return 0;
	//param�terek be�ll�t�sa
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
	for (int frameIdx = 0; ; frameIdx++) { //minden k�pkock�n
		bool flag = true;
		for (int view = 0; view < cameras.size(); view++) { //minden kamer�n
			videos[view] >> rawImgs[view];    //k�vetkez� k�pkocka beolvas�sa �s elt�rol�sa
			if (rawImgs[view].empty()) {
				flag = false;
				break;
			}
			cv::resize(rawImgs[view], rawImgs[view], cv::Size(), skelPainter.rate, skelPainter.rate);
			associater.SetDetection(view, seqDetections[view][frameIdx].Mapping(SKEL19));   //egy kamera egy k�p�re illesztj�k a sz�veges adat detectiont
		}
		if (!flag)
			break;

		associater.SetSkels3dPrev(skelUpdater.GetSkel3d());   //egy framenek az �sszes kamera detectionb�l (SetDetection) el��ll�tjuk a 3D skeletont 
		associater.Associate(); //kisz�moljuk a 3D skeleton param�tereit
		skelUpdater.Update(associater.GetSkels2d(), projs);   //updatelj�k a megl�v� skeletonokat a framek k�z�tt

		
		// save
		const int layoutCols = 3;
		std::vector<cv::Rect> rois = SkelPainter::MergeImgs(rawImgs, detectImg, layoutCols,
			{ rawImgs.begin()->cols, rawImgs.begin()->rows}); //detectImg el��ll�t�sa az egyes kamera k�pekb�l
		detectImg.copyTo(assocImg);
		detectImg.copyTo(reprojImg);

#pragma omp parallel for
		for (int view = 0; view < cameras.size(); view++) {   //a vissza projekt�lt skeletonok visszarajzol�sa az egyes k�pekre (m�g mindig frame-enk�nt)
			const OpenposeDetection detection = seqDetections[view][frameIdx].Mapping(SKEL19);
			skelPainter.DrawDetect(detection.joints, detection.pafs, detectImg(rois[view]));
			for (const auto& skel2d : associater.GetSkels2d())
				skelPainter.DrawAssoc(skel2d.second.middleCols(view * skelDef.jointSize, skelDef.jointSize), assocImg(rois[view]), skel2d.first);

			for(const auto& skel3d : skelUpdater.GetSkel3d())
				skelPainter.DrawReproj(skel3d.second, projs.middleCols(4 * view, 4), reprojImg(rois[view]), skel3d.first);
		}
		
		skels.emplace_back(skelUpdater.GetSkel3d()); //a 3D skeleton adatinak vissza�r�sa m�trixba
		//a frame kiment�se k�pbe
		cv::imwrite("../output/detect/" + std::to_string(frameIdx) + ".jpg", detectImg);
		cv::imwrite("../output/assoc/" + std::to_string(frameIdx) + ".jpg", assocImg);
		cv::imwrite("../output/reproj/" + std::to_string(frameIdx) + ".jpg", reprojImg);
		std::cout << std::to_string(frameIdx) << std::endl;
	}

	SerializeSkels(skels, "../output/skel.txt"); //a 3D skeleton ki�r�sa f�jlba
	return 0;
}
