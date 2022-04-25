#pragma once
#include"commonFunctions.h"

class RGBtofCalibration
{
public:
	RGBtofCalibration();
	RGBtofCalibration(std::string fileName);
	~RGBtofCalibration();

	//���ü������
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
	cv::Size m_imageSizeRGB;//ͼ���С
	cv::Size m_imageSizetof;//ͼ���С
	std::vector<std::vector<cv::Point3f>> m_calibWorldPoint;//���������
	std::vector<std::vector<cv::Point2f>> m_calibImagePointRGB;//���̸�ǵ�
	std::vector<std::vector<cv::Point2f>> m_calibImagePointtof;//���̸�ǵ�
	std::vector<Eigen::Vector3f> m_PointSetRGB;
	std::vector<Eigen::Vector3f> m_PointSettof;

	cv::Mat m_cameraMatrixRGB;//      �ڲ�
	cv::Mat m_cameraMatrixtof;//      �ڲ�
	cv::Mat m_distCoeffs_fisheyeRGB;//�����������
	cv::Mat m_distCoeffs_fisheyetof;//�����������
	cv::Mat m_distCoeffsRGB;//        �������
	cv::Mat m_distCoeffstof;//        �������
	std::vector<bool> m_detectResultsRGB;//�ǵ����Ƿ�ɹ�
	std::vector<bool> m_detectResultstof;//�ǵ����Ƿ�ɹ�


private:
	//�������ģ�Ͳ���
	bool m_complexCameraModelRGB;
	bool m_complexCameraModeltof;
	//�궨���̲���
	//bool m_isLoadCameraIntrinsicRGB;
	//bool m_isLoadCameraIntrinsictof;
	std::vector<std::string> m_filenamesRGB;
	std::vector<std::string> m_filenamestof;

	cv::Size m_patternSize;//�ǵ㲼��
	cv::Size2f m_patternLength;//�����ǵ�֮����������(15, 15)
	bool m_iscircle;//ʹ��Բ��궨����нǵ���

	cv::Size m_cornerROISizeRGB;//
	cv::Size m_cornerROISizetof;//
	bool m_isfisheyeRGB;//ʹ�õ������۲���ģ��
	bool m_isfisheyetof;//ʹ�õ������۲���ģ��

	float m_MaxReporjectionErrorRGB;//Ԥ�ȹ���������ͶӰ���
	float m_MaxReporjectionErrortof;//Ԥ�ȹ���������ͶӰ���
	float m_reprojectionErrorRGB;//��ͶӰ���
	float m_reprojectionErrortof;//��ͶӰ���

	//�㼯
	std::vector<Eigen::Vector3f> m_RGBPointSet;
	std::vector<Eigen::Vector3f> m_tofPointSet;
	//���
	Eigen::Matrix4f m_transformation_matrix;
	float m_AverageErrorDistance;
};

