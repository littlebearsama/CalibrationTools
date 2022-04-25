#pragma once
#include"commonFunctions.h"

class RGBtofCalibration
{
public:
	RGBtofCalibration();
	RGBtofCalibration(std::string fileName);
	~RGBtofCalibration();

	//设置计算参数
	void setCheckBoardParam(cv::Size patternSize, cv::Size2f patternLength, bool isCricle = false);
	void setParametersRGB(const std::vector<std::string>& RGBpics, 
		cv::Size imageSize, float maxReprojectError, bool isfisheye = false, bool complexCameraModel = false);
	void setParameterstof(const std::vector<std::string>& TOFpics, 
		cv::Size imageSize, float maxReprojectError, bool isfisheye = false, bool complexCameraModel = false);
	bool calibrateCams(bool isvisible = true);
	bool compute();
	void getPointCloudWithColor(const cv::Mat& rgb, const cv::Mat& depth, pcl::PointCloud<pcl::PointXYZRGB>& ouputcloud,std::string savepath);
	bool saveParameters(std::string saveName);
	bool loadParameters(std::string fileName);

public:
	cv::Size m_imageSizeRGB;//图像大小
	cv::Size m_imageSizetof;//图像大小
	std::vector<std::vector<cv::Point3f>> m_calibWorldPoint;//物理坐标点
	std::vector<std::vector<cv::Point2f>> m_calibImagePointRGB;//棋盘格角点
	std::vector<std::vector<cv::Point2f>> m_calibImagePointtof;//棋盘格角点
	std::vector<Eigen::Vector3f> m_PointSetRGB;
	std::vector<Eigen::Vector3f> m_PointSettof;

	cv::Mat m_cameraMatrixRGB;//      内参
	cv::Mat m_cameraMatrixtof;//      内参
	cv::Mat m_distCoeffs_fisheyeRGB;//鱼眼相机畸变
	cv::Mat m_distCoeffs_fisheyetof;//鱼眼相机畸变
	cv::Mat m_distCoeffsRGB;//        相机畸变
	cv::Mat m_distCoeffstof;//        相机畸变
	std::vector<bool> m_detectResultsRGB;//角点检测是否成功
	std::vector<bool> m_detectResultstof;//角点检测是否成功


private:
	//复杂相机模型参数
	bool m_complexCameraModelRGB;
	bool m_complexCameraModeltof;
	//标定过程参数
	//bool m_isLoadCameraIntrinsicRGB;
	//bool m_isLoadCameraIntrinsictof;
	std::vector<std::string> m_filenamesRGB;
	std::vector<std::string> m_filenamestof;

	cv::Size m_patternSize;//角点布局
	cv::Size2f m_patternLength;//两个角点之间的物理距离(15, 15)
	bool m_iscircle;//使用圆点标定板进行角点检测

	cv::Size m_cornerROISizeRGB;//
	cv::Size m_cornerROISizetof;//
	bool m_isfisheyeRGB;//使用的是鱼眼参数模型
	bool m_isfisheyetof;//使用的是鱼眼参数模型

	float m_MaxReporjectionErrorRGB;//预先估算的最大重投影误差
	float m_MaxReporjectionErrortof;//预先估算的最大重投影误差
	float m_reprojectionErrorRGB;//重投影误差
	float m_reprojectionErrortof;//重投影误差

	//点集
	std::vector<Eigen::Vector3f> m_RGBPointSet;
	std::vector<Eigen::Vector3f> m_tofPointSet;
	//结果
	Eigen::Matrix4f m_transformation_matrix;
	float m_AverageErrorDistance;
};

