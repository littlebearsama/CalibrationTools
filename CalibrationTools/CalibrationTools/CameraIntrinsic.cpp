#include "CameraIntrinsic.h"
#include "commonFunctions.h"
#include <algorithm>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/calib3d/calib3d.hpp>

CameraIntrinsic::CameraIntrinsic()
{
	m_complexCameraModel = false;
	m_MaxReporjectionError = 0.04;
	m_isLoadCameraIntrinsic = false;
	m_cornerROISize = cv::Size(5, 5);

}
CameraIntrinsic::CameraIntrinsic(std::string filename)
{
	m_cornerROISize = cv::Size(5, 5);
	m_complexCameraModel = false;
	m_MaxReporjectionError = 0.04;
	m_isLoadCameraIntrinsic = readCameraIntrinsic(filename);

}


CameraIntrinsic::~CameraIntrinsic()
{

}

bool CameraIntrinsic::readCameraIntrinsic(std::string filename)
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
		std::cout << "打不开文件.yml: "<< filename << std::endl;
		return false;
	}
	//m_complexCameraModel = (int)fs["m_complexCameraModel"];
	fs["m_complexCameraModel"] >> m_complexCameraModel;
	fs["m_imageSize"] >> m_imageSize;
	fs["m_cameraMatrix"] >> m_cameraMatrix;
	fs["m_isfisheye"] >> m_isfisheye;
	//m_isfisheye = (int)fs["m_isfisheye"];
	if (m_isfisheye)
	{
		fs["m_distCoeffs_fisheye"] >> m_distCoeffs_fisheye;
	}
	else
	{
		fs["m_distCoeffs"] >> m_distCoeffs;
	}

	fs["m_patternSize"] >> m_patternSize;
	fs["m_patternLength"] >> m_patternLength;
	fs["m_iscircle"] >> m_iscircle;
	fs["m_reprojectionError"] >> m_reprojectionError;
	fs.release();
	std::cout << "加载内参成功，文件名为：" << filename << std::endl;
	m_isLoadCameraIntrinsic = true;
	return true;
}

bool CameraIntrinsic::saveCameraIntrinsic(std::string filename)
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
	fs << "m_complexCameraModel" << m_complexCameraModel << "m_imageSize" << m_imageSize << "m_cameraMatrix" << m_cameraMatrix << "m_isfisheye" << m_isfisheye;

	if (m_isfisheye)
	{
		fs << "m_distCoeffs_fisheye" << m_distCoeffs_fisheye;
	}
	else
	{
		fs << "m_distCoeffs" << m_distCoeffs;
	}
	fs << "m_patternSize" << m_patternSize << "m_patternLength" << m_patternLength << "m_iscircle" << m_iscircle << "m_reprojectionError" << m_reprojectionError;
	fs.release();

	return true;
}

void CameraIntrinsic::setParameters(std::vector<std::string> filenames, cv::Size imageSize, cv::Size patternSize, cv::Size2f patternLength, bool isCricle, bool isfisheye, bool complexCameraModel)
{
	m_complexCameraModel = complexCameraModel;
	m_filenames.clear();
	m_detectResults.clear();
	m_calibImagePoint.clear();
	m_calibWorldPoint.clear();
	m_filenames = filenames;
	m_imageSize = imageSize;
	m_patternSize = patternSize;
	m_patternLength = patternLength;
	m_iscircle = isCricle;
	m_isfisheye = isfisheye;
}

void CameraIntrinsic::printAllParameter() const
{
	std::cout << std::endl;
	std::cout << "打印参数......" << std::endl;
	std::cout << "m_imageSize " << m_imageSize << std::endl;
	std::cout << "m_cameraMatrix " << m_cameraMatrix << std::endl;
	std::cout << "m_distCoeffs_fisheye " << m_distCoeffs_fisheye << std::endl;
	std::cout << "m_distCoeffs " << m_distCoeffs << std::endl;
	std::cout << "m_complexCameraModel " << m_complexCameraModel << std::endl;
	std::cout << "m_isLoadCameraIntrinsic " << m_isLoadCameraIntrinsic << std::endl;
	std::cout << "m_patternSize " << m_patternSize << std::endl;
	std::cout << "m_patternLength " << m_patternLength << std::endl;
	std::cout << "m_iscircle " << m_iscircle << std::endl;
	std::cout << "m_isfisheye " << m_isfisheye << std::endl;
	std::cout << "m_MaxReporjectionError " << m_MaxReporjectionError << std::endl;
	std::cout << "m_reprojectionError " << m_reprojectionError << std::endl;
	std::cout << std::endl;
}

bool CameraIntrinsic::getCheckBoardCorners(const cv::Mat&inputImage, cv::Mat& grayImage, bool isvisible)
{
	bool result = false;
	std::vector<cv::Point2f> targetPoint;

	//检测角点
	if (m_iscircle)
	{
		if (0 == findCirclesGrid(inputImage, m_patternSize, targetPoint))
		{
			result = false;
			m_detectResults.push_back(result);
			std::cout << "false-calib" << std::endl;
			return false;
		}
		else
		{
			//resetCorners(targetPoint, m_patternSize);//不知道找圆是否需要角点重排
			drawChessboardCorners(inputImage, m_patternSize, targetPoint, true);
			result = true;
			m_detectResults.push_back(result);
			m_calibImagePoint.push_back(targetPoint);
			std::cout << "true-calib" << std::endl;
			return true;
		}
	}
	else
	{
		if (0 == findChessboardCorners(inputImage, m_patternSize, targetPoint,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE))
			//if (0 == findChessboardCorners(pic, m_patternSize, targetPoint,
			//	CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE))
		{
			result = false;
			m_detectResults.push_back(result);
			std::cout << "false-calib" << std::endl;
			return false;
		}
		else
		{
			result = true;
			//resetCorners(targetPoint, m_patternSize);

			cv::cvtColor(inputImage, grayImage, cv::COLOR_RGB2GRAY);
			m_detectResults.push_back(result);
			//find4QuadCornerSubpix(grayImage, targetPoint, cv::Size(5, 5));
			cv::cornerSubPix(grayImage, targetPoint, m_cornerROISize,
				cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 500, 0.005));
			m_calibImagePoint.push_back(targetPoint);
			drawChessboardCorners(grayImage, m_patternSize, targetPoint, true);
			std::cout << "true-calib" << std::endl;
			return true;

		}
	}

	if (result&&isvisible)
	{
		bitwise_not(grayImage, grayImage);
		imshow("grayImage", grayImage);
		cv::waitKey(50);
	}
}

bool CameraIntrinsic::getCheckBoardPose(const cv::Mat&inputImage, Eigen::Matrix4d& matrixCheck2Cam)
{
	cv::Mat grayImage;
	bool successflag = getCheckBoardCorners(inputImage, grayImage, false);
	if (!successflag)
	{
		//移除检测结果的最后一个元素
		m_detectResults.pop_back();
		m_detectResults.shrink_to_fit();
		return false;
	}
	//删除掉刚刚添加到vector的检测结果
	m_detectResults.pop_back();
	m_detectResults.shrink_to_fit();

	std::vector<cv::Point2f> cornerPoints = m_calibImagePoint[m_calibImagePoint.size() - 1];
	std::vector<cv::Point2f> cornerPointsUndistored;
	m_calibImagePoint.pop_back();
	m_calibImagePoint.shrink_to_fit();
	
	//生成世界坐标系的点
	std::vector<cv::Point3f> worldPoints;
	generateWorldPoints(m_patternSize, m_patternLength, worldPoints);

	//PNP求解位姿
	//使用PNP求解外参
	cv::Mat rvec;
	cv::Mat tvec;
	if (m_isfisheye)
	{
		//畸变矫正
		cv::fisheye::undistortPoints(cornerPoints, cornerPointsUndistored, m_cameraMatrix, m_distCoeffs_fisheye, cv::Mat(), m_cameraMatrix);//矫正角点
		//畸变校正后该函数不需要畸变，将畸变系数设为0
		cv::Mat NodistCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
		//rvec tvec是从模型坐标系到相机坐标系的
		solvePnPRansac(worldPoints, cornerPointsUndistored, m_cameraMatrix, NodistCoeffs, rvec, tvec);//求解外参
	}
	else
	{
		//先求解再矫正
		solvePnPRansac(worldPoints, cornerPoints, m_cameraMatrix, m_distCoeffs, rvec, tvec);//求解外参																						   //矫正
		//undistortPoints(targetPoint, targetPoint, m_cameraMatrix, m_distCoeffs, cv::Mat(), m_cameraMatrix);//矫正角点
	}
	//计算误差
	//对角点（世界坐标系）进行重投影计算，与提取的角点坐标系比较
	std::vector<cv::Point2f> boardPointsReprojected;
	if (!m_isfisheye)
		cv::projectPoints(worldPoints, rvec, tvec, m_cameraMatrix, m_distCoeffs, boardPointsReprojected);
	else 
		cv::fisheye::projectPoints(worldPoints, boardPointsReprojected, rvec, tvec, m_cameraMatrix, m_distCoeffs_fisheye);

	double err = cv::norm(boardPointsReprojected, cornerPoints, cv::NORM_L2);    ///是否支持该格式
	err /= (m_patternSize.width * m_patternSize.height);     //角点平均误差
	printf("外参的重投影误差为: %lf\n", err);

	//将Mat转为Eigen::Matrix4d
	//旋转向量转旋转矩阵
	cv::Mat rotation_matrix; //(row, col) = (3, 3)
	cv::Rodrigues(rvec, rotation_matrix);
	//输出
	matrixCheck2Cam << rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2), tvec.at<double>(0, 0),
		rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2), tvec.at<double>(1, 0),
		rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2), tvec.at<double>(2, 0),
		0, 0, 0, 1;

	return true;
}

bool CameraIntrinsic::compute(bool isvisible)
{
	m_detectResults.clear();
	m_calibImagePoint.clear();
	m_calibWorldPoint.clear();

	if (m_imageSize.width >= 1280) 
	{
		m_cornerROISize = cv::Size(11,11);
	}
	else if (m_imageSize.width <= 640)
	{
		m_cornerROISize = cv::Size(5, 5);
	}
	else
	{
		m_cornerROISize = cv::Size(7, 7);
	}

	int picNum = m_filenames.size();
	for (int i = 0; i < picNum; i++)
	{
		std::string name = m_filenames[i];
		cv::Mat pic = cv::imread(name);
		cv::Mat grayImage;
		bool result = getCheckBoardCorners(pic, grayImage, isvisible);
	}


	//生成世界坐标系的点
	int picCount = m_calibImagePoint.size();//根据角点副数
	generateWorldPoints_Multi(picCount, m_patternSize, m_patternLength, m_calibWorldPoint);

	std::vector<float> angles;
	getAngleOfChecks(m_calibImagePoint, m_patternSize, angles);

	bool calibrationSucessFlag = false;

	while (m_calibImagePoint.size() > 5 && !calibrationSucessFlag) 
	{

		//计算内参时候得到的棋盘格角点的外参			
		std::vector<cv::Mat> R_vec;
		std::vector<cv::Mat> T_vec;
		//根据相机模型进行标定
		cv::Mat mapx, mapy;
		if (m_isfisheye)
		{
			int flags = 0;
			flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
			flags |= cv::fisheye::CALIB_CHECK_COND;
			flags |= cv::fisheye::CALIB_FIX_SKEW;


			double error = cv::fisheye::calibrate(m_calibWorldPoint, m_calibImagePoint, m_imageSize, m_cameraMatrix, m_distCoeffs_fisheye, R_vec, T_vec, flags, cv::TermCriteria(3, 20, 1e-6));
			cv::fisheye::initUndistortRectifyMap(m_cameraMatrix, m_distCoeffs_fisheye, cv::Matx33d::eye(), m_cameraMatrix, m_imageSize, CV_16SC2, mapx, mapy);//得到矫正矩阵mapx，mapy
		}
		else
		{
			cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
			int flags = 0;
			if (m_complexCameraModel)
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
			double rms = calibrateCamera(m_calibWorldPoint, m_calibImagePoint, m_imageSize, m_cameraMatrix, m_distCoeffs, R_vec, T_vec, flags, criteria);
			std::cout << "Total Reprojection error:" << rms << std::endl;
		}

		//计算重投影误差
		double err = 0.0;
		double mean_err = 0.0;
		double total_err = 0.0;
		std::vector<cv::Point2f> reprojectionPoint;
		std::vector<float> errors;
		for (int i = 0; i < m_calibWorldPoint.size(); i++)
		{
			std::vector<cv::Point3f> tempPointSet = m_calibWorldPoint[i];
			/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
			if (m_isfisheye)
			{
				cv::fisheye::projectPoints(tempPointSet, reprojectionPoint, R_vec[i], T_vec[i], m_cameraMatrix, m_distCoeffs_fisheye);
			}
			else
			{
				projectPoints(tempPointSet, R_vec[i], T_vec[i], m_cameraMatrix, m_distCoeffs, reprojectionPoint);
			}
			/* 计算新的投影点和旧的投影点之间的误差*/
			std::vector<cv::Point2f> tempImagePoint = m_calibImagePoint[i];
			cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
			cv::Mat image_points2Mat = cv::Mat(1, reprojectionPoint.size(), CV_32FC2);
			for (int j = 0; j < tempImagePoint.size(); j++)
			{
				image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(reprojectionPoint[j].x, reprojectionPoint[j].y);
				tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
			}
			err = norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
			errors.push_back(err);//保存用于剔除误差过大的棋盘格图
			mean_err = err / (m_patternSize.width*m_patternSize.height);
			total_err += mean_err;
		}
		m_reprojectionError = total_err / m_calibWorldPoint.size();
		std::cout << "畸变校正后，Reprojection error:" << m_reprojectionError << std::endl;
		if (m_reprojectionError <= m_MaxReporjectionError)
			calibrationSucessFlag = true;
		else
		{
			bool isUsedReprojectError = true;
			int maxpos = -1;
			if (isUsedReprojectError)
			{
				std::vector<float>::iterator maxposIt = std::max_element(errors.begin(), errors.end());
				maxpos = maxposIt - errors.begin();
				//剔除(不用下面也可以 因为每次计算都会刷新)
				std::vector<float>::iterator it = errors.begin() + maxpos;
				errors.erase(it);
			}
			else
			{
				std::vector<float>::iterator maxposIt = std::max_element(angles.begin(), angles.end());
				maxpos = maxposIt - angles.begin();
				//剔除
				std::vector<float>::iterator it = angles.begin() + maxpos;
				angles.erase(it);
			}
			//筛选根据棋盘格角度（在棋盘格角点比较密集、相机畸变较大、棋盘格比较倾斜的情况起作用）
			//筛选根绝重投影误差

			std::vector<std::vector<cv::Point3f>>::iterator it2 = m_calibWorldPoint.begin() + maxpos;
			m_calibWorldPoint.erase(it2);
			std::vector<std::vector<cv::Point2f>>::iterator it3 = m_calibImagePoint.begin() + maxpos;
			m_calibImagePoint.erase(it3);
			//将m_detectResults对应位置置为false
			int posCount = -1;
			for (int resultPos = 0; resultPos < m_detectResults.size(); resultPos++)
			{
				if (m_detectResults[resultPos])
				{
					posCount++;
				}
				else
				{
					continue;
				}

				if (posCount == maxpos) 
				{
					m_detectResults[resultPos] = false;
					break;
				}
			}
		}
	}
	std::cout << "used " << m_calibWorldPoint.size() << " pictures of checkerborad" << std::endl;
	return true;
}

void CameraIntrinsic::getUndistoredImage(const cv::Mat& input, cv::Mat& output) const
{
	if (m_isfisheye)
	{
		cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
		cv::Size image_size = m_imageSize;
		cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
		cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
		cv::fisheye::initUndistortRectifyMap(m_cameraMatrix, m_distCoeffs_fisheye, R, m_cameraMatrix, image_size, CV_32FC1, mapx, mapy);
		//
		cv::remap(input, output, mapx, mapy, cv::INTER_LINEAR);
	}
	else
	{
		undistort(input, output, m_cameraMatrix, m_distCoeffs, m_cameraMatrix);
	}
}

void CameraIntrinsic::getUndistoredPixels(const std::vector<cv::Point2f>& input, std::vector<cv::Point2f>& output) const
{
	if (m_isfisheye)
	{
		//畸变矫正
		cv::fisheye::undistortPoints(input, output, m_cameraMatrix, m_distCoeffs_fisheye, cv::Mat(), m_cameraMatrix);//矫正角点
	}
	else
	{
		undistortPoints(input, output, m_cameraMatrix, m_distCoeffs, cv::Mat(), m_cameraMatrix);//矫正角点
	}
}
