#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <Eigen/dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

/*
brief:相机内参标定类：
初始化:
1. 所有用于标定的图片名称路径
2 .标定chart的大小以及物理间隔
3. 所使用棋盘格的类型（圆点还是棋盘格）
4. 相机类型(鱼眼还是普通相机)
*/
class CameraIntrinsic
{
public:
	CameraIntrinsic(); //默认构造函数
	CameraIntrinsic(std::string filename); //内参文件名称
	~CameraIntrinsic();


	bool readCameraIntrinsic(std::string filename);
	bool saveCameraIntrinsic(std::string filename);
	//设置计算参数
	void setMaxReproectionError(float error) { m_MaxReporjectionError = error; }
	void setParameters(std::vector<std::string> filenames, 
		cv::Size imageSize, cv::Size patternSize, cv::Size2f patternLength, bool isCricle = false, 
		bool isfisheye = false, bool complexCameraModel = false);
	void printAllParameter() const;

	//求解一个棋盘格的外参
	bool getCheckBoardPose(const cv::Mat&inputImage, Eigen::Matrix4d& matrixCheck2Cam);
	bool compute(bool isvisible = true);//是否显示计算的角点
	bool isCalibrated() const{ return m_isLoadCameraIntrinsic; }//是否已经加载了标定文件
	
	//根据相机模型对图像进行矫正
	void getUndistoredImage(const cv::Mat& input, cv::Mat& output) const;
	//根据相机模型对像素点进行矫正
	void getUndistoredPixels(const std::vector<cv::Point2f>& input, std::vector<cv::Point2f>& output) const;

	//返回计算参数
	cv::Size getPatternSize() const {
		return m_patternSize;
	}
	cv::Size getCornerROISize() const {
		return m_cornerROISize;
	}
	cv::Size2f getPatternLength() const
	{
		return m_patternLength;
	}


private:
	//计算当前图像的角点并且将结果（m_detectResults）和角点（m_calibImagePoint）信息存储起来
	bool getCheckBoardCorners(const cv::Mat&inputImage, cv::Mat& grayImage, bool isvisible);

public:
	cv::Size m_imageSize;//图像大小
	std::vector<std::vector<cv::Point3f>> m_calibWorldPoint;//物理坐标点
	std::vector<std::vector<cv::Point2f>> m_calibImagePoint;//棋盘格角点
	cv::Mat m_cameraMatrix;//      内参
	cv::Mat m_distCoeffs_fisheye;//鱼眼相机畸变
	cv::Mat m_distCoeffs;//        相机畸变
	std::vector<bool> m_detectResults;//角点检测是否成功
	bool m_isfisheye;//使用的是鱼眼参数模型

private:
	//复杂相机模型参数
	bool m_complexCameraModel;
	//标定过程参数
	bool m_isLoadCameraIntrinsic;
	std::vector<std::string> m_filenames;
	cv::Size m_patternSize;//角点布局
	cv::Size m_cornerROISize;//
	cv::Size2f m_patternLength;//两个角点之间的物理距离(15, 15)
	bool m_iscircle;//使用圆点标定板进行角点检测
	
	


	//预先估算的最大重投影误差
	float m_MaxReporjectionError;
	float m_reprojectionError;//重投影误差

};

