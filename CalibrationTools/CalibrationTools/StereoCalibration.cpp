#include "StereoCalibration.h"



StereoCalibration::StereoCalibration()
{
	m_RectifyMapFlag = false;
	m_isLoadStereoModel = false;
}

StereoCalibration::StereoCalibration(std::string stereoModelFileName, std::string leftCameraModel, std::string rightCameraModel)
{
	m_RectifyMapFlag = false;
	m_isLoadStereoModel = readStereoModel(stereoModelFileName, leftCameraModel, rightCameraModel);

}


StereoCalibration::~StereoCalibration()
{

}

bool StereoCalibration::readStereoModel(std::string stereoModelFileName, std::string leftCameraModel, std::string rightCameraModel)
{
	//读取文件
	int size = stereoModelFileName.size();
	if (stereoModelFileName.substr(size - 4, size) != ".yml")
	{
		std::cout << "输入文件名字不是.yml文件名" << std::endl;
		return false;
	}
	cv::FileStorage fs(stereoModelFileName, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "打不开文件.yml: " << stereoModelFileName << std::endl;
		return false;
	}
	fs["m_imageSize"] >> m_imageSize;
	fs["m_R"] >> m_R;
	fs["m_T"] >> m_T;
	fs["m_E"] >> m_E;
	fs["m_F"] >> m_F;
	fs["m_RFirst"] >> m_RFirst;
	fs["m_RSec"] >> m_RSec;
	fs["m_PFirst"] >> m_PFirst;
	fs["m_PSec"] >> m_PSec;
	fs["m_Q"] >> m_Q;
	fs["m_validRoileft"] >> m_validRoi[0];
	fs["m_validRoiright"] >> m_validRoi[1];

	fs.release();
	std::cout << "加载内参成功，文件名为：" << stereoModelFileName << std::endl;
	m_isLoadStereoModel = true;
	m_leftCamera = CameraIntrinsic(leftCameraModel);
	bool leftCamLoaded = m_leftCamera.isCalibrated();
	m_rightCamera = CameraIntrinsic (rightCameraModel);
	bool rightCamLoaded = m_rightCamera.isCalibrated();

	return m_isLoadStereoModel&&leftCamLoaded&&rightCamLoaded;
}

bool StereoCalibration::saveStereoModel(std::string filename)
{
	//保存文件
	int size = filename.size();
	if (filename.substr(size - 4, size) != ".yml")
	{
		std::cout << "输入文件名字不是.yml文件名" << std::endl;
		return false;
	}
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	//保存
	fs << "m_imageSize" << m_imageSize 
		<< "m_R" << m_R
		<< "m_T" << m_T
		<< "m_E" << m_E
		<< "m_F" << m_F
		<< "m_RFirst" << m_RFirst
		<< "m_RSec" << m_RSec
		<< "m_PFirst" << m_PFirst
		<< "m_PSec" << m_PSec
		<< "m_Q" << m_Q
		<< "m_validRoileft" << m_validRoi[0]
		<< "m_validRoiright" << m_validRoi[1];

	return true;
}

bool StereoCalibration::compute()
{
	if (m_leftCamera.m_detectResults.size()!= m_rightCamera.m_detectResults.size())
	{
		return false;
	}
	int picsCount = m_leftCamera.m_detectResults.size();

	m_imageSize = m_leftCamera.m_imageSize;
	std::cout << "stereo calibrate..." << std::endl;
	//并不是所有图像都能同时检测出角点，故找出所有在同一帧的左右角点
	std::vector<std::vector<cv::Point2f>> calibImagePointLeftFull;//棋盘格角点左
	std::vector<std::vector<cv::Point2f>> calibImagePointRightFull;//棋盘格角点右
	int leftCount = 0;
	int rightCount = 0;
	for (int i = 0; i < picsCount; i++)
	{
		if (m_leftCamera.m_detectResults[i])
		{
			calibImagePointLeftFull.push_back(m_leftCamera.m_calibImagePoint[leftCount]);
			leftCount++;
		}
		else
		{
			std::vector<cv::Point2f> emptyVector;
			calibImagePointLeftFull.push_back(emptyVector);
		}
	}
	for (int i = 0; i < picsCount; i++)
	{
		if (m_rightCamera.m_detectResults[i])
		{
			calibImagePointRightFull.push_back(m_rightCamera.m_calibImagePoint[rightCount]);
			rightCount++;
		}
		else
		{
			std::vector<cv::Point2f> emptyVector;
			calibImagePointRightFull.push_back(emptyVector);
		}
	}
	//
	std::vector<std::vector<cv::Point2f>> calibImagePointLeft;//棋盘格角点左
	std::vector<std::vector<cv::Point2f>> calibImagePointRight;//棋盘格角点右

	for (int i = 0; i < picsCount; i++)
	{
		if (m_leftCamera.m_detectResults[i]&& m_rightCamera.m_detectResults[i])
		{
			calibImagePointLeft.push_back(calibImagePointLeftFull[i]);
			calibImagePointRight.push_back(calibImagePointRightFull[i]);
		}
	}

	int pairsCount = calibImagePointLeft.size();
	std::cout << "一共有" << pairsCount << "数量的左右棋盘格角点点对。" << std::endl;
	std::vector<std::vector<cv::Point3f>> calibWorldPoint;
	generateWorldPoints_Multi(pairsCount, m_leftCamera.getPatternSize(), m_leftCamera.getPatternLength(), calibWorldPoint);

	float rms = stereoCalibrate(calibWorldPoint, calibImagePointLeft, calibImagePointRight,
		m_leftCamera.m_cameraMatrix, m_leftCamera.m_distCoeffs, m_rightCamera.m_cameraMatrix, m_rightCamera.m_distCoeffs,
		m_imageSize, m_R, m_T, m_E, m_F, cv::CALIB_USE_INTRINSIC_GUESS,//CV_CALIB_FIX_INTRINSIC, CALIB_USE_INTRINSIC_GUESS
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));          //计算重投影误差
	std::cout << "done with RMS error=" << rms << std::endl;
	//stereo rectify
	std::cout << "stereo rectify..." << std::endl;
	stereoRectify(m_leftCamera.m_cameraMatrix, m_leftCamera.m_distCoeffs, m_rightCamera.m_cameraMatrix, m_rightCamera.m_distCoeffs, m_imageSize,
		m_R, m_T, m_RFirst, m_RSec, m_PFirst, m_PSec, m_Q, cv::CALIB_ZERO_DISPARITY, -1, m_imageSize, &m_validRoi[0], &m_validRoi[1]);
	//
	std::cout << "Left roi:" << m_validRoi[0] << ",Right roi" << m_validRoi[1] << std::endl;
	return false;
}

bool StereoCalibration::rectifyImage(const cv::Mat& left, const cv::Mat& right, cv::Mat& rectifyImageL, cv::Mat& rectifyImageR, bool showRectifyFlag)
{

	if (!m_RectifyMapFlag)
	{
		std::cout << "m_leftCamera.m_cameraMatrix:" << m_leftCamera.m_cameraMatrix << std::endl;
		std::cout << "m_leftCamera.m_distCoeffs:" << m_leftCamera.m_distCoeffs << std::endl;
		std::cout << "m_RFirst:" << m_RFirst << std::endl;
		std::cout << "m_PFirst:" << m_PFirst << std::endl;
		std::cout << "m_imageSize:" << m_imageSize << std::endl;

		std::cout << "m_rightCamera.m_cameraMatrix:" << m_rightCamera.m_cameraMatrix << std::endl;
		std::cout << "m_rightCamera.m_distCoeffs:" << m_rightCamera.m_distCoeffs << std::endl;
		std::cout << "m_RSec:" << m_RSec << std::endl;
		std::cout << "m_PSec:" << m_PSec << std::endl;
		std::cout << "m_imageSize:" << m_imageSize << std::endl;

		initUndistortRectifyMap(m_leftCamera.m_cameraMatrix, m_leftCamera.m_distCoeffs, m_RFirst, m_PFirst,
			m_imageSize, CV_16SC2, m_rmapFirst[0], m_rmapFirst[1]);//CV_16SC2

		initUndistortRectifyMap(m_rightCamera.m_cameraMatrix, m_rightCamera.m_distCoeffs, m_RSec, m_PSec,//CV_16SC2
			m_imageSize, CV_16SC2, m_rmapSec[0], m_rmapSec[1]);
		m_RectifyMapFlag = true;
	}

	cv::remap(left, rectifyImageL, m_rmapFirst[0], m_rmapFirst[1], cv::INTER_LINEAR);
	cv::remap(right, rectifyImageR, m_rmapSec[0], m_rmapSec[1], cv::INTER_LINEAR);

	//显示极线矫正对齐后的左右图像
	if (showRectifyFlag)
	{
		cv::Mat canvas(left.rows, left.cols * 2, CV_8UC3);
		cv::Mat canLeft = canvas(cv::Rect(0, 0, left.cols, left.rows));
		cv::Mat canRight = canvas(cv::Rect(left.cols, 0, left.cols, left.rows));
		cv::namedWindow("RectifyLR",1);
		rectifyImageL.copyTo(canLeft);
		rectifyImageR.copyTo(canRight);
		for (int j = 0; j <= canvas.rows; j += 16)  //画绿线
			line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
		imshow("RectifyLR", canvas);
		cv::waitKey();
	}
	return true;
}
