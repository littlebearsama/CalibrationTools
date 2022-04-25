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
brief:����ڲα궨�ࣺ
��ʼ��:
1. �������ڱ궨��ͼƬ����·��
2 .�궨chart�Ĵ�С�Լ�������
3. ��ʹ�����̸�����ͣ�Բ�㻹�����̸�
4. �������(���ۻ�����ͨ���)
*/
class CameraIntrinsic
{
public:
	CameraIntrinsic(); //Ĭ�Ϲ��캯��
	CameraIntrinsic(std::string filename); //�ڲ��ļ�����
	~CameraIntrinsic();


	bool readCameraIntrinsic(std::string filename);
	bool saveCameraIntrinsic(std::string filename);
	//���ü������
	void setMaxReproectionError(float error) { m_MaxReporjectionError = error; }
	void setParameters(std::vector<std::string> filenames, 
		cv::Size imageSize, cv::Size patternSize, cv::Size2f patternLength, bool isCricle = false, 
		bool isfisheye = false, bool complexCameraModel = false);
	void printAllParameter() const;

	//���һ�����̸�����
	bool getCheckBoardPose(const cv::Mat&inputImage, Eigen::Matrix4d& matrixCheck2Cam);
	bool compute(bool isvisible = true);//�Ƿ���ʾ����Ľǵ�
	bool isCalibrated() const{ return m_isLoadCameraIntrinsic; }//�Ƿ��Ѿ������˱궨�ļ�
	
	//�������ģ�Ͷ�ͼ����н���
	void getUndistoredImage(const cv::Mat& input, cv::Mat& output) const;
	//�������ģ�Ͷ����ص���н���
	void getUndistoredPixels(const std::vector<cv::Point2f>& input, std::vector<cv::Point2f>& output) const;

	//���ؼ������
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
	//���㵱ǰͼ��Ľǵ㲢�ҽ������m_detectResults���ͽǵ㣨m_calibImagePoint����Ϣ�洢����
	bool getCheckBoardCorners(const cv::Mat&inputImage, cv::Mat& grayImage, bool isvisible);

public:
	cv::Size m_imageSize;//ͼ���С
	std::vector<std::vector<cv::Point3f>> m_calibWorldPoint;//���������
	std::vector<std::vector<cv::Point2f>> m_calibImagePoint;//���̸�ǵ�
	cv::Mat m_cameraMatrix;//      �ڲ�
	cv::Mat m_distCoeffs_fisheye;//�����������
	cv::Mat m_distCoeffs;//        �������
	std::vector<bool> m_detectResults;//�ǵ����Ƿ�ɹ�
	bool m_isfisheye;//ʹ�õ������۲���ģ��

private:
	//�������ģ�Ͳ���
	bool m_complexCameraModel;
	//�궨���̲���
	bool m_isLoadCameraIntrinsic;
	std::vector<std::string> m_filenames;
	cv::Size m_patternSize;//�ǵ㲼��
	cv::Size m_cornerROISize;//
	cv::Size2f m_patternLength;//�����ǵ�֮����������(15, 15)
	bool m_iscircle;//ʹ��Բ��궨����нǵ���
	
	


	//Ԥ�ȹ���������ͶӰ���
	float m_MaxReporjectionError;
	float m_reprojectionError;//��ͶӰ���

};

