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
	//��ȡ�ļ�
	int size = filename.size();
	if (filename.substr(size - 4, size) != ".yml")
	{
		std::cout << "�����ļ����ֲ���.yml�ļ���" << std::endl;
		return false;
	}
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "�򲻿��ļ�.yml: "<< filename << std::endl;
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
	std::cout << "�����ڲγɹ����ļ���Ϊ��" << filename << std::endl;
	m_isLoadCameraIntrinsic = true;
	return true;
}

bool CameraIntrinsic::saveCameraIntrinsic(std::string filename)
{
	//�����ļ�
	int size = filename.size();
	if (filename.substr(size - 4, size) != ".yml") 
	{
		std::cout << "�����ļ����ֲ���.yml�ļ���" << std::endl;
		return false;
	}
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	//����
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
	std::cout << "��ӡ����......" << std::endl;
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

	//���ǵ�
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
			//resetCorners(targetPoint, m_patternSize);//��֪����Բ�Ƿ���Ҫ�ǵ�����
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
		//�Ƴ�����������һ��Ԫ��
		m_detectResults.pop_back();
		m_detectResults.shrink_to_fit();
		return false;
	}
	//ɾ�����ո���ӵ�vector�ļ����
	m_detectResults.pop_back();
	m_detectResults.shrink_to_fit();

	std::vector<cv::Point2f> cornerPoints = m_calibImagePoint[m_calibImagePoint.size() - 1];
	std::vector<cv::Point2f> cornerPointsUndistored;
	m_calibImagePoint.pop_back();
	m_calibImagePoint.shrink_to_fit();
	
	//������������ϵ�ĵ�
	std::vector<cv::Point3f> worldPoints;
	generateWorldPoints(m_patternSize, m_patternLength, worldPoints);

	//PNP���λ��
	//ʹ��PNP������
	cv::Mat rvec;
	cv::Mat tvec;
	if (m_isfisheye)
	{
		//�������
		cv::fisheye::undistortPoints(cornerPoints, cornerPointsUndistored, m_cameraMatrix, m_distCoeffs_fisheye, cv::Mat(), m_cameraMatrix);//�����ǵ�
		//����У����ú�������Ҫ���䣬������ϵ����Ϊ0
		cv::Mat NodistCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
		//rvec tvec�Ǵ�ģ������ϵ���������ϵ��
		solvePnPRansac(worldPoints, cornerPointsUndistored, m_cameraMatrix, NodistCoeffs, rvec, tvec);//������
	}
	else
	{
		//������ٽ���
		solvePnPRansac(worldPoints, cornerPoints, m_cameraMatrix, m_distCoeffs, rvec, tvec);//������																						   //����
		//undistortPoints(targetPoint, targetPoint, m_cameraMatrix, m_distCoeffs, cv::Mat(), m_cameraMatrix);//�����ǵ�
	}
	//�������
	//�Խǵ㣨��������ϵ��������ͶӰ���㣬����ȡ�Ľǵ�����ϵ�Ƚ�
	std::vector<cv::Point2f> boardPointsReprojected;
	if (!m_isfisheye)
		cv::projectPoints(worldPoints, rvec, tvec, m_cameraMatrix, m_distCoeffs, boardPointsReprojected);
	else 
		cv::fisheye::projectPoints(worldPoints, boardPointsReprojected, rvec, tvec, m_cameraMatrix, m_distCoeffs_fisheye);

	double err = cv::norm(boardPointsReprojected, cornerPoints, cv::NORM_L2);    ///�Ƿ�֧�ָø�ʽ
	err /= (m_patternSize.width * m_patternSize.height);     //�ǵ�ƽ�����
	printf("��ε���ͶӰ���Ϊ: %lf\n", err);

	//��MatתΪEigen::Matrix4d
	//��ת����ת��ת����
	cv::Mat rotation_matrix; //(row, col) = (3, 3)
	cv::Rodrigues(rvec, rotation_matrix);
	//���
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


	//������������ϵ�ĵ�
	int picCount = m_calibImagePoint.size();//���ݽǵ㸱��
	generateWorldPoints_Multi(picCount, m_patternSize, m_patternLength, m_calibWorldPoint);

	std::vector<float> angles;
	getAngleOfChecks(m_calibImagePoint, m_patternSize, angles);

	bool calibrationSucessFlag = false;

	while (m_calibImagePoint.size() > 5 && !calibrationSucessFlag) 
	{

		//�����ڲ�ʱ��õ������̸�ǵ�����			
		std::vector<cv::Mat> R_vec;
		std::vector<cv::Mat> T_vec;
		//�������ģ�ͽ��б궨
		cv::Mat mapx, mapy;
		if (m_isfisheye)
		{
			int flags = 0;
			flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
			flags |= cv::fisheye::CALIB_CHECK_COND;
			flags |= cv::fisheye::CALIB_FIX_SKEW;


			double error = cv::fisheye::calibrate(m_calibWorldPoint, m_calibImagePoint, m_imageSize, m_cameraMatrix, m_distCoeffs_fisheye, R_vec, T_vec, flags, cv::TermCriteria(3, 20, 1e-6));
			cv::fisheye::initUndistortRectifyMap(m_cameraMatrix, m_distCoeffs_fisheye, cv::Matx33d::eye(), m_cameraMatrix, m_imageSize, CV_16SC2, mapx, mapy);//�õ���������mapx��mapy
		}
		else
		{
			cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
			int flags = 0;
			if (m_complexCameraModel)
			{
				//�����Լ������ģ�ͽ�������
				flags |= cv::CALIB_RATIONAL_MODEL;//����ģ�ͣ�����k4��k5,k6�����������
				flags |= cv::CALIB_TILTED_MODEL;//��бģ��
				flags |= cv::CALIB_THIN_PRISM_MODEL;//���⾵����ģ��
			}

			//flags |= CALIB_USE_INTRINSIC_GUESS;//ʹ�ù��е��ڲ�ģ�ͣ�Ȼ��ʹ��solvePnP.CALIB_FIX_K1
			//flags |= CALIB_FIX_K1;//�̶�K1
			//flags |= CALIB_FIX_K2;//�̶�K2
			//flags |= CALIB_FIX_K3;//�̶�K3
			//flags |= CALIB_FIX_FOCAL_LENGTH;//�̶�����
			//flags |= CALIB_ZERO_TANGENT_DIST;//����ϵ��p1,p2��Ϊ0
			double rms = calibrateCamera(m_calibWorldPoint, m_calibImagePoint, m_imageSize, m_cameraMatrix, m_distCoeffs, R_vec, T_vec, flags, criteria);
			std::cout << "Total Reprojection error:" << rms << std::endl;
		}

		//������ͶӰ���
		double err = 0.0;
		double mean_err = 0.0;
		double total_err = 0.0;
		std::vector<cv::Point2f> reprojectionPoint;
		std::vector<float> errors;
		for (int i = 0; i < m_calibWorldPoint.size(); i++)
		{
			std::vector<cv::Point3f> tempPointSet = m_calibWorldPoint[i];
			/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
			if (m_isfisheye)
			{
				cv::fisheye::projectPoints(tempPointSet, reprojectionPoint, R_vec[i], T_vec[i], m_cameraMatrix, m_distCoeffs_fisheye);
			}
			else
			{
				projectPoints(tempPointSet, R_vec[i], T_vec[i], m_cameraMatrix, m_distCoeffs, reprojectionPoint);
			}
			/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
			std::vector<cv::Point2f> tempImagePoint = m_calibImagePoint[i];
			cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
			cv::Mat image_points2Mat = cv::Mat(1, reprojectionPoint.size(), CV_32FC2);
			for (int j = 0; j < tempImagePoint.size(); j++)
			{
				image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(reprojectionPoint[j].x, reprojectionPoint[j].y);
				tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
			}
			err = norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
			errors.push_back(err);//���������޳�����������̸�ͼ
			mean_err = err / (m_patternSize.width*m_patternSize.height);
			total_err += mean_err;
		}
		m_reprojectionError = total_err / m_calibWorldPoint.size();
		std::cout << "����У����Reprojection error:" << m_reprojectionError << std::endl;
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
				//�޳�(��������Ҳ���� ��Ϊÿ�μ��㶼��ˢ��)
				std::vector<float>::iterator it = errors.begin() + maxpos;
				errors.erase(it);
			}
			else
			{
				std::vector<float>::iterator maxposIt = std::max_element(angles.begin(), angles.end());
				maxpos = maxposIt - angles.begin();
				//�޳�
				std::vector<float>::iterator it = angles.begin() + maxpos;
				angles.erase(it);
			}
			//ɸѡ�������̸�Ƕȣ������̸�ǵ�Ƚ��ܼ����������ϴ����̸�Ƚ���б����������ã�
			//ɸѡ������ͶӰ���

			std::vector<std::vector<cv::Point3f>>::iterator it2 = m_calibWorldPoint.begin() + maxpos;
			m_calibWorldPoint.erase(it2);
			std::vector<std::vector<cv::Point2f>>::iterator it3 = m_calibImagePoint.begin() + maxpos;
			m_calibImagePoint.erase(it3);
			//��m_detectResults��Ӧλ����Ϊfalse
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
		//�������
		cv::fisheye::undistortPoints(input, output, m_cameraMatrix, m_distCoeffs_fisheye, cv::Mat(), m_cameraMatrix);//�����ǵ�
	}
	else
	{
		undistortPoints(input, output, m_cameraMatrix, m_distCoeffs, cv::Mat(), m_cameraMatrix);//�����ǵ�
	}
}
