//将pcl的头文件放在ifndef前面
#include <Eigen/Dense>
#include "RGBtofCalibration.h"

#include <opencv2/core/eigen.hpp>
#include <algorithm>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/core/core.hpp"

RGBtofCalibration::RGBtofCalibration()
{
	m_complexCameraModelRGB = false;
	m_MaxReporjectionErrorRGB = 0.04;
	m_MaxReporjectionErrortof = 0.08;
	m_cornerROISizeRGB = cv::Size(11, 11);
	m_cornerROISizetof = cv::Size(5, 5);
}

RGBtofCalibration::RGBtofCalibration(std::string fileName)
{
	loadParameters(fileName);
}


RGBtofCalibration::~RGBtofCalibration()
{

}

void RGBtofCalibration::setCheckBoardParam(cv::Size patternSize, cv::Size2f patternLength, bool isCricle)
{
	m_patternSize = patternSize;
	m_patternLength = patternLength;
	m_iscircle = isCricle;
}

void RGBtofCalibration::setParametersRGB(const std::vector<std::string>& RGBpics, cv::Size imageSize, float maxReprojectError, bool isfisheye, bool complexCameraModel)
{
	m_calibWorldPoint.clear();
	m_calibImagePointRGB.clear();
	m_detectResultsRGB.clear();
	m_PointSetRGB.clear();
	m_filenamesRGB.clear();

	m_filenamesRGB = RGBpics;
	m_imageSizeRGB = imageSize;
	m_MaxReporjectionErrorRGB = maxReprojectError;
	m_isfisheyeRGB = isfisheye;
	m_complexCameraModelRGB = complexCameraModel;
}

void RGBtofCalibration::setParameterstof(const std::vector<std::string>& TOFpics, cv::Size imageSize, float maxReprojectError, bool isfisheye, bool complexCameraModel)
{
	m_calibWorldPoint.clear();
	m_calibImagePointtof.clear();
	m_detectResultstof.clear();
	m_PointSettof.clear();
	m_filenamestof.clear();

	m_filenamestof = TOFpics;
	m_imageSizetof = imageSize;
	m_MaxReporjectionErrortof = maxReprojectError;
	m_isfisheyetof = isfisheye;
	m_complexCameraModeltof = complexCameraModel;
}



bool RGBtofCalibration::calibrateCams(bool isvisible)
{
	m_calibWorldPoint.clear();

	m_calibImagePointtof.clear();
	m_detectResultstof.clear();
	m_PointSettof.clear();

	m_calibImagePointRGB.clear();
	m_detectResultsRGB.clear();
	m_PointSetRGB.clear();

	if (m_imageSizeRGB.width >= 1280)
	{
		m_cornerROISizeRGB = cv::Size(11, 11);
	}
	else if (m_imageSizeRGB.width <= 640)
	{
		m_cornerROISizeRGB = cv::Size(5, 5);
	}
	else
	{
		m_cornerROISizeRGB = cv::Size(7, 7);
	}

	if (m_imageSizetof.width >= 1280)
	{
		m_cornerROISizetof = cv::Size(11, 11);
	}
	else if (m_imageSizetof.width <= 640)
	{
		m_cornerROISizetof = cv::Size(5, 5);
	}
	else
	{
		m_cornerROISizetof = cv::Size(7, 7);
	}

	int sizeOfRGB = m_filenamesRGB.size();
	int sizeOftof = m_filenamestof.size();
	if (sizeOfRGB != sizeOftof)
	{
		return false;
	}

	//标定RGB和TOF相机内参，找到所有共同的角点
	for (int i = 0; i < sizeOfRGB; i++)
	{
		std::string nameRGB = m_filenamesRGB[i];
		std::string nametof = m_filenamestof[i];
		cv::Mat picRGB = cv::imread(nameRGB);
		cv::Mat pictof = cv::imread(nametof);
		cv::Mat grayImageRGB;
		cv::Mat grayImagetof;

		bool result = false;
		std::vector<cv::Point2f> targetPointRGB;
		std::vector<cv::Point2f> targetPointtof;

		//检测角点
		if (m_iscircle)
		{
			if (0 == findCirclesGrid(picRGB, m_patternSize, targetPointRGB))
			{
				result = false;
				m_detectResultsRGB.push_back(result);
				m_detectResultstof.push_back(result);
				std::cout << "RGB false-calib" << std::endl;
				continue;
			}
			else
			{
				if (0 == findCirclesGrid(pictof, m_patternSize, targetPointtof))
				{
					result = false;
					m_detectResultsRGB.push_back(result);
					m_detectResultstof.push_back(result);
					std::cout << "tof false-calib" << std::endl;
					continue;
				}
				else
				{
					drawChessboardCorners(picRGB, m_patternSize, targetPointRGB, true);
					result = true;
					m_detectResultsRGB.push_back(result);
					m_detectResultstof.push_back(result);
					m_calibImagePointRGB.push_back(targetPointRGB);
					m_calibImagePointtof.push_back(targetPointtof);
					std::cout << "RGB tof true-calib" << std::endl;
				}

			}
		}
		else
		{
			if (0 == findChessboardCorners(picRGB, m_patternSize, targetPointRGB,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE))
			{
				result = false;
				m_detectResultsRGB.push_back(result);
				m_detectResultstof.push_back(result);
				std::cout << "RGB false-calib" << std::endl;
				continue;
			}
			else
			{
				if (0 == findChessboardCorners(pictof, m_patternSize, targetPointtof))
					//if (0 == findChessboardCorners(pictof, m_patternSize, targetPointtof,
					//	CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE))
				{
					result = false;
					m_detectResultsRGB.push_back(result);
					m_detectResultstof.push_back(result);
					std::cout << "tof false-calib" << std::endl;
					continue;
				}
				else
				{
					result = true;
					cv::cvtColor(picRGB, grayImageRGB, cv::COLOR_RGB2GRAY);
					cv::cornerSubPix(grayImageRGB, targetPointRGB, m_cornerROISizeRGB,
						cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
					m_calibImagePointRGB.push_back(targetPointRGB);
					drawChessboardCorners(grayImageRGB, m_patternSize, targetPointRGB, true);

					cv::cvtColor(pictof, grayImagetof, cv::COLOR_RGB2GRAY);
					cv::cornerSubPix(grayImagetof, targetPointtof, m_cornerROISizetof,
						cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
					m_calibImagePointtof.push_back(targetPointtof);
					drawChessboardCorners(grayImagetof, m_patternSize, targetPointtof, true);

					m_detectResultsRGB.push_back(result);
					m_detectResultstof.push_back(result);
					m_calibImagePointRGB.push_back(targetPointRGB);
					m_calibImagePointtof.push_back(targetPointtof);

					std::cout << "RGB tof true-calib" << std::endl;
				}
			}
		}
		if (result&&isvisible)
		{
			bitwise_not(grayImageRGB, grayImageRGB);
			imshow("grayImageRGB", grayImageRGB);
			bitwise_not(grayImagetof, grayImagetof);
			imshow("grayImagetof", grayImagetof);
			cv::waitKey(50);
		}
	}
	//生成世界坐标系的点
	int picCount = m_calibImagePointRGB.size();//根据角点副数
	generateWorldPoints_Multi(picCount, m_patternSize, m_patternLength, m_calibWorldPoint);
	//
	std::vector<float> anglesRGB;
	getAngleOfChecks(m_calibImagePointRGB, m_patternSize, anglesRGB);
	std::vector<float> anglestof;
	getAngleOfChecks(m_calibImagePointtof, m_patternSize, anglestof);


	//计算内参时候得到的棋盘格角点的外参			
	std::vector<cv::Mat> R_vecRGB;
	std::vector<cv::Mat> T_vecRGB;
	std::vector<cv::Mat> R_vectof;
	std::vector<cv::Mat> T_vectof;

	//根据相机模型进行标定
	cv::Mat mapxRGB, mapyRGB;
	cv::Mat mapxtof, mapytof;
	if (m_isfisheyeRGB)
	{
		int flags = 0;
		flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
		flags |= cv::fisheye::CALIB_CHECK_COND;
		flags |= cv::fisheye::CALIB_FIX_SKEW;

		double error = cv::fisheye::calibrate(m_calibWorldPoint, m_calibImagePointRGB, m_imageSizeRGB, 
			m_cameraMatrixRGB, m_distCoeffs_fisheyeRGB, R_vecRGB, T_vecRGB, flags, cv::TermCriteria(3, 20, 1e-6));
		cv::fisheye::initUndistortRectifyMap(m_cameraMatrixRGB, m_distCoeffs_fisheyeRGB, cv::Matx33d::eye(), 
			m_cameraMatrixRGB, m_imageSizeRGB, CV_16SC2, mapxRGB, mapyRGB);//得到矫正矩阵mapx，mapy
	}
	else
	{
		cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
		int flags = 0;
		if (m_complexCameraModelRGB)
		{
			//根据自己的相机模型进行设置
			flags |= cv::CALIB_RATIONAL_MODEL;//理想模型，启用k4，k5,k6三个畸变参数
			flags |= cv::CALIB_TILTED_MODEL;//倾斜模型
			flags |= cv::CALIB_THIN_PRISM_MODEL;//薄棱镜畸变模型
		}

		//flags |= CALIB_USE_INTRINSIC_GUESS;//使用固有的内参模型，然后使用solvePnP.CALIB_FIX_K1
		//flags |= CALIB_FIX_K1;//固定K1
		//flags |= CALIB_FIX_K2;//固定K2
		//flags |= CALIB_FIX_K3;//固定K3
		//flags |= CALIB_FIX_FOCAL_LENGTH;//固定焦距
		//flags |= CALIB_ZERO_TANGENT_DIST;//切向系数p1,p2设为0
		double rms = calibrateCamera(m_calibWorldPoint, m_calibImagePointRGB, m_imageSizeRGB,
			m_cameraMatrixRGB, m_distCoeffsRGB, R_vecRGB, T_vecRGB, flags, criteria);
		std::cout << "Total Reprojection error:" << rms << std::endl;
	}

	//计算重投影误差
	double errRGB = 0.0;
	double mean_errRGB = 0.0;
	double total_errRGB = 0.0;
	std::vector<cv::Point2f> reprojectionPointRGB;
	for (int i = 0; i < m_calibWorldPoint.size(); i++)
	{
		std::vector<cv::Point3f> tempPointSet = m_calibWorldPoint[i];
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		if (m_isfisheyeRGB)
		{
			cv::fisheye::projectPoints(tempPointSet, reprojectionPointRGB, R_vecRGB[i], T_vecRGB[i], m_cameraMatrixRGB, m_distCoeffs_fisheyeRGB);
		}
		else
		{
			projectPoints(tempPointSet, R_vecRGB[i], T_vecRGB[i], m_cameraMatrixRGB, m_distCoeffsRGB, reprojectionPointRGB);
		}
		/* 计算新的投影点和旧的投影点之间的误差*/
		std::vector<cv::Point2f> tempImagePoint = m_calibImagePointRGB[i];
		cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
		cv::Mat image_points2Mat = cv::Mat(1, reprojectionPointRGB.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(reprojectionPointRGB[j].x, reprojectionPointRGB[j].y);
			tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		errRGB = norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);

		mean_errRGB = errRGB / (m_patternSize.width*m_patternSize.height);
		total_errRGB += mean_errRGB;
	}
	m_reprojectionErrorRGB = total_errRGB / m_calibWorldPoint.size();
	std::cout << "RGB畸变校正后，Reprojection error:" << m_reprojectionErrorRGB << std::endl;
	//继续标定tof相机
	//根据相机模型进行标定
	if (m_isfisheyetof)
	{
		int flags = 0;
		flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
		flags |= cv::fisheye::CALIB_CHECK_COND;
		flags |= cv::fisheye::CALIB_FIX_SKEW;


		double error = cv::fisheye::calibrate(m_calibWorldPoint, m_calibImagePointtof, m_imageSizetof, m_cameraMatrixtof,
			m_distCoeffs_fisheyetof, R_vectof, T_vectof, flags, cv::TermCriteria(3, 20, 1e-6));
		cv::fisheye::initUndistortRectifyMap(m_cameraMatrixtof, m_distCoeffs_fisheyetof, cv::Matx33d::eye(), m_cameraMatrixtof,
			m_imageSizetof, CV_16SC2, mapxtof, mapytof);//得到矫正矩阵mapx，mapy
	}
	else
	{
		cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
		int flags = 0;
		if (m_complexCameraModeltof)
		{
			//根据自己的相机模型进行设置
			flags |= cv::CALIB_RATIONAL_MODEL;//理想模型，启用k4，k5,k6三个畸变参数
			flags |= cv::CALIB_TILTED_MODEL;//倾斜模型
			flags |= cv::CALIB_THIN_PRISM_MODEL;//薄棱镜畸变模型
		}

		//flags |= CALIB_USE_INTRINSIC_GUESS;//使用固有的内参模型，然后使用solvePnP.CALIB_FIX_K1
		//flags |= CALIB_FIX_K1;//固定K1
		//flags |= CALIB_FIX_K2;//固定K2
		//flags |= CALIB_FIX_K3;//固定K3
		//flags |= CALIB_FIX_FOCAL_LENGTH;//固定焦距
		//flags |= CALIB_ZERO_TANGENT_DIST;//切向系数p1,p2设为0
		double rms = calibrateCamera(m_calibWorldPoint, m_calibImagePointtof, m_imageSizetof, m_cameraMatrixtof,
			m_distCoeffstof, R_vectof, T_vectof, flags, criteria);
		std::cout << "Total Reprojection error:" << rms << std::endl;
	}

	//计算重投影误差
	double errtof = 0.0;
	double mean_errtof = 0.0;
	double total_errtof = 0.0;
	std::vector<cv::Point2f> reprojectionPointtof;
	for (int i = 0; i < m_calibWorldPoint.size(); i++)
	{
		std::vector<cv::Point3f> tempPointSet = m_calibWorldPoint[i];
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		if (m_isfisheyetof)
		{
			cv::fisheye::projectPoints(tempPointSet, reprojectionPointtof, R_vectof[i], T_vectof[i], m_cameraMatrixtof, m_distCoeffs_fisheyetof);
		}
		else
		{
			projectPoints(tempPointSet, R_vectof[i], T_vectof[i], m_cameraMatrixtof, m_distCoeffstof, reprojectionPointtof);
		}
		/* 计算新的投影点和旧的投影点之间的误差*/
		std::vector<cv::Point2f> tempImagePoint = m_calibImagePointtof[i];
		cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
		cv::Mat image_points2Mat = cv::Mat(1, reprojectionPointtof.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(reprojectionPointtof[j].x, reprojectionPointtof[j].y);
			tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		errtof = norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);

		mean_errtof = errtof / (m_patternSize.width*m_patternSize.height);
		total_errtof += mean_errtof;
	}
	m_reprojectionErrortof = total_errtof / m_calibWorldPoint.size();
	std::cout << "tof畸变校正后，Reprojection error:" << m_reprojectionErrortof << std::endl;


	int numOfChecks = m_calibWorldPoint.size();
	//求解点集
	std::vector<Eigen::Vector3f> RGBPointSet;
	std::vector<Eigen::Vector3f> tofPointSet;
	RGBPointSet.reserve(numOfChecks*m_patternSize.height*m_patternSize.height);
	tofPointSet.reserve(numOfChecks*m_patternSize.height*m_patternSize.height);

	for (int i = 0; i < numOfChecks; i++)
	{
		//当前相机外参
		Eigen::Matrix3f RGBR;
		Eigen::Vector3f RGBT;
		cv::Mat RGBRotationMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
		Rodrigues(R_vecRGB[i], RGBRotationMatrix);
		cv::cv2eigen(RGBRotationMatrix, RGBR); // cv::eigen2cv();
		cv::cv2eigen(T_vecRGB[i], RGBT); // cv::eigen2cv();

		Eigen::Matrix3f tofR;
		Eigen::Vector3f tofT;
		cv::Mat RightRotationMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
		cv::Rodrigues(R_vectof[i], RightRotationMatrix);
		cv::cv2eigen(RightRotationMatrix, tofR); // cv::eigen2cv();
		cv::cv2eigen(T_vectof[i], tofT); // cv::eigen2cv();

		int currtObjNUM = m_calibWorldPoint[i].size();
		for (int j = 0; j < currtObjNUM; j++)
		{
			//pointcloud[i] = transformation_matrix.topLeftCorner<3, 3>()*pointcloud[i] + transformation_matrix.topRightCorner<3, 1>();
			Eigen::Vector3f currObjPoint = Eigen::Vector3f(m_calibWorldPoint[i][j].x, m_calibWorldPoint[i][j].y, m_calibWorldPoint[i][j].z);
			Eigen::Vector3f currRGBPoint = RGBR*currObjPoint + RGBT;
			Eigen::Vector3f currtofPoint = tofR*currObjPoint + tofT;
			RGBPointSet.emplace_back(currRGBPoint);
			tofPointSet.emplace_back(currtofPoint);
		}
		//saveData(std::string("left.txt"), LeftPointSet);
		//saveData(std::string("right.txt"), RightPointSet);
	}
	RGBPointSet.shrink_to_fit();
	tofPointSet.shrink_to_fit();
	m_RGBPointSet = RGBPointSet;
	m_tofPointSet = tofPointSet;



}

bool RGBtofCalibration::compute()
{
	//SVD求解左右相机外参
	Correspondences correspondences;
	Eigen::Matrix4f transformation_matrix;
	float AeragePointDis;
	float RMSE;
	int inlierNUM;

	int numOfCorrespondece = m_RGBPointSet.size();
	correspondences.resize(numOfCorrespondece);
	for (int i = 0; i < numOfCorrespondece; i++)
	{
		correspondences[i] = Correspondence(i, i, 0);
	}

	//使用SVD求解外参（两个相机之间的旋转变换矩阵）
	//右相机到左相机的变换矩阵
	TransformationEstimationSVD(m_RGBPointSet, m_tofPointSet, correspondences,
		transformation_matrix, AeragePointDis, RMSE, inlierNUM);
	std::cout << "右相机到左相机的变换矩阵为：" << std::endl;
	std::cout << transformation_matrix << std::endl << std::endl;

	//求在工作空间范围内的误差
	float AverageErrorDistance = 0;
	for (int i = 0; i < m_RGBPointSet.size(); i++)
	{
		Eigen::Vector3f currTranformatedPoint = transformation_matrix.topLeftCorner<3, 3>()*m_tofPointSet[i] + transformation_matrix.topRightCorner<3, 1>();
		AverageErrorDistance += (m_RGBPointSet[i] - currTranformatedPoint).norm();
	}
	AverageErrorDistance /= m_RGBPointSet.size();
	std::cout << "左右相机对齐后的点对误差为：" << AverageErrorDistance <<"mm" << std::endl << std::endl;
	
	m_AverageErrorDistance = AverageErrorDistance;
	m_transformation_matrix = transformation_matrix;

	//saveData(std::string("left.txt"), LeftPointSet);
	//saveData(std::string("right.txt"), RightPointSet);
	return true;
}

void RGBtofCalibration::getPointCloudWithColor(const cv::Mat& rgb, 
	const cv::Mat& depth, 
	pcl::PointCloud<pcl::PointXYZRGB>& ouputcloud,
	std::string saveName)
{
	bool debugPrint = false;
	std::vector<Eigen::Vector3f> pointCloud;
	pointCloud.reserve(depth.rows*depth.cols);
	Eigen::Matrix4f T_tof_RGB;
	float scale = 1.0f;
	float fx = m_cameraMatrixtof.ptr<double>(0)[0];
	float fy = m_cameraMatrixtof.ptr<double>(1)[1];
	float cx = m_cameraMatrixtof.ptr<double>(0)[2];
	float cy = m_cameraMatrixtof.ptr<double>(1)[2];
	if (debugPrint)
	{
		std::cout << "m_cameraMatrixtof:" << m_cameraMatrixtof << std::endl;
	}


	for (int m = 0; m < depth.rows; m++)
	{
		for (int n = 0; n < depth.cols; n++)
		{
			// 获取深度图中(m,n)处的值
			float d = depth.ptr<ushort>(m)[n];//深度
											  // d 可能没有值，若如此，跳过此点
			if (d == 0)
				continue;
			// d 存在值，则向点云增加一个点
			Eigen::Vector3f  p;
			// 计算这个点的空间坐标
			p.z() = d / scale;
			p.x() = (float(n) - cx) * p.z() / fx;
			p.y() = (float(m) - cy) * p.z() / fy;
			//保留小数点后三位
			pointCloud.emplace_back(p);
		}
	}
	pointCloud.shrink_to_fit();

	Eigen::Matrix4f transformation_matrix2 = m_transformation_matrix.inverse();
	if (debugPrint)
	{
		std::cout << "transformation_matrix2:" << transformation_matrix2 << std::endl;
	}
	TransformPointCloud(pointCloud, transformation_matrix2);

	std::vector<cv::Point3f> pointsInRGB;//RGB坐标系下的物理点集
	pointsInRGB.resize(pointCloud.size());

#pragma parallel for
	for (int i = 0; i < pointCloud.size(); i++)
	{
		pointsInRGB[i] = cv::Point3f(pointCloud[i].x(), pointCloud[i].y(), pointCloud[i].z());
	}

	std::vector<cv::Point2f> imageProjectedInRGB; /* 保存重新计算得到的投影点 */
										 /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
	cv::Mat R_rgb = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));//外参
	cv::Mat t_rgb = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));//内参

	//讲三维点投影到图像坐标系
	projectPoints(pointsInRGB, R_rgb, t_rgb, m_cameraMatrixRGB, m_distCoeffsRGB, imageProjectedInRGB);
	//给新点云赋值彩色
	ouputcloud.clear();
	ouputcloud.resize(pointsInRGB.size());
	int rightCols = rgb.cols;
	int rightRows = rgb.rows;
	for (int i = 0; i < ouputcloud.size(); i++)
	{
		int x = imageProjectedInRGB[i].x;
		int y = imageProjectedInRGB[i].y;
		if (x<0 || x>rightCols || y<0 || y>rightRows)
		{
			ouputcloud.points[i].x = pointCloud[i].x();
			ouputcloud.points[i].y = pointCloud[i].y();
			ouputcloud.points[i].z = pointCloud[i].z();
			ouputcloud.points[i].r = 0;
			ouputcloud.points[i].g = 0;
			ouputcloud.points[i].b = 0;
		}
		else
		{
			ouputcloud.points[i].x = pointCloud[i].x();
			ouputcloud.points[i].y = pointCloud[i].y();
			ouputcloud.points[i].z = pointCloud[i].z();
			int r = rgb.ptr<uchar>(y)[x * 3];
			int g = rgb.ptr<uchar>(y)[x * 3 + 1];
			int b = rgb.ptr<uchar>(y)[x * 3 + 2];
			ouputcloud.points[i].r = r;
			ouputcloud.points[i].g = g;
			ouputcloud.points[i].b = b;
		}

	}

	pcl::PCDWriter writer;
	writer.write(saveName, ouputcloud);
}

bool RGBtofCalibration::saveParameters(std::string savePath)
{
	//保存文件
	int size = savePath.size();
	if (savePath.substr(size - 4, size) != ".yml")
	{
		std::cout << "输入文件名字不是.yml文件名" << std::endl;
		return false;
	}
	cv::FileStorage fs(savePath, cv::FileStorage::WRITE);
	//保存
	cv::Mat transformationMat;
	cv::eigen2cv(m_transformation_matrix, transformationMat);
	fs << "transformationMat" << transformationMat
		<< "m_AverageErrorDistance" << m_AverageErrorDistance;
	fs << "m_cameraMatrixRGB" << m_cameraMatrixRGB;
	if (m_isfisheyeRGB)
	{
		fs << "m_isfisheyeRGB" << m_isfisheyeRGB;
		fs << "m_distCoeffs_fisheyeRGB" << m_distCoeffs_fisheyeRGB;
	}
	else
	{
		fs << "m_distCoeffsRGB" << m_distCoeffsRGB;
	}

	fs << "m_cameraMatrixtof" << m_cameraMatrixtof;
	if (m_isfisheyetof)
	{
		fs << "m_isfisheyetof" << m_isfisheyetof;
		fs << "m_distCoeffs_fisheyetof" << m_distCoeffs_fisheyetof;
	}
	else
	{
		fs << "m_distCoeffstof" << m_distCoeffstof;
	}
	fs.release();
	return true;
}

bool RGBtofCalibration::loadParameters(std::string filename)
{
	//读取文件
	int size = filename.size();
	if (filename.substr(size - 4, size) != ".yml")
	{
		std::cout << "输入文件名字不是.yml文件名" << std::endl;
		return false;
	}
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "打不开文件.yml: " << filename << std::endl;
		return false;
	}
	cv::Mat transformationMat;


	fs["transformationMat"] >> transformationMat;
	cv::cv2eigen(transformationMat,m_transformation_matrix);
	fs["m_AverageErrorDistance"] >> m_AverageErrorDistance;
	fs["m_cameraMatrixRGB"] >> m_cameraMatrixRGB;
	fs["m_isfisheyeRGB"] >> m_isfisheyeRGB;
	fs["m_isfisheyetof"] >> m_isfisheyetof;
	//m_isfisheye = (int)fs["m_isfisheye"];
	if (m_isfisheyeRGB)
	{
		fs["m_distCoeffs_fisheyeRGB"] >> m_distCoeffs_fisheyeRGB;
	}
	else
	{
		fs["m_distCoeffsRGB"] >> m_distCoeffsRGB;
	}

	fs["m_cameraMatrixtof"] >> m_cameraMatrixtof;
	if (m_isfisheyetof)
	{
		fs["m_distCoeffs_fisheyetof"] >> m_distCoeffs_fisheyetof;
	}
	else
	{
		fs["m_distCoeffstof"] >> m_distCoeffstof;
	}
	fs.release();
	return true;
}
