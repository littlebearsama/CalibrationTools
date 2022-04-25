#pragma once
#include"commonFunctions.h"

class StereoCalibration
{
public:
	StereoCalibration();
	StereoCalibration(std::string stereoModelFileName, std::string leftCameraModel, std::string rightCameraModel); //内参文件名称
	~StereoCalibration();

	bool readStereoModel(std::string stereoModelFileName, std::string leftCameraModel, std::string rightCameraModel);
	bool saveStereoModel(std::string filename);
	bool compute();
	bool rectifyImage(const cv::Mat& left, const cv::Mat& right, cv::Mat& rectifyImageL, cv::Mat& rectifyImageR, bool showRecticfyFlag = false);


	CameraIntrinsic m_leftCamera;
	CameraIntrinsic m_rightCamera;
	cv::Mat m_R, m_T, m_E, m_F, m_RFirst, m_RSec, m_PFirst, m_PSec, m_Q;

private:
	bool m_isLoadStereoModel;
	std::vector<std::vector<cv::Point2f>> m_imagePointsFirst, m_imagePointsSec;
	std::vector<std::vector<cv::Point3f>> m_ObjectPoints;
	cv::Rect m_validRoi[2];
	cv::Size m_imageSize;
	double m_rms = 0;

	bool m_RectifyMapFlag;//是否已经生成矫正map
	cv::Mat m_rmapFirst[2];
	cv::Mat m_rmapSec[2];

};

