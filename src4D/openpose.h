#pragma once
#include <Eigen/Core>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include "skel.h"

//Egy kamera egy frame detection-je
struct OpenposeDetection
{
	OpenposeDetection() { type = SkelType::SKEL_TYPE_NONE; }
	OpenposeDetection(const SkelType& _type);
	OpenposeDetection Mapping(const SkelType& tarType);
	std::vector<Eigen::Matrix3Xf> Associate(const int& jcntThresh = 5);

	SkelType type;
	//joints size: jointSize *( 3 * detectedPeople)
	std::vector<Eigen::Matrix3Xf> joints;
	std::vector<Eigen::MatrixXf> pafs;
};

std::vector<OpenposeDetection> ParseDetections(const std::string& filename);
void SerializeDetections(const std::vector<OpenposeDetection>& detections, const std::string& filename);

