#pragma once
#include <vector>
#include <string>
#include <direct.h>
#include <opencv2/opencv.hpp>
#include "CameraIntrinsic.h"
#include "StereoCalibration.h"
#include "Functions.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

//�����ļ���
void createFolder(std::string path);
//��ȡ�ļ������������ļ�
void getFiles(const std::string & path, std::vector<std::string> & files);
void getAllFiles(const std::string & path, std::vector<std::string> & files);
void saveData(std::string filename, const std::vector<Eigen::Vector3f> &points);

//ɸѡ�ļ�
void getFileWithEndStr(const std::vector<std::string> & filesin, std::vector<std::string> & filesout, std::string substr);//�ҳ���׺Ϊĳ���ַ����ļ���
void getFileWithHeadStr(const std::vector<std::string> & filesin, std::vector<std::string> & filesout, std::string substr);//�ҳ���ͷΪĳ���ַ����ļ���
void getNameWithpath(std::string filein, std::string& name);//�õ��ļ�����ȥ����׺��


//�ǵ�������
void resetCorners(std::vector<cv::Point2f>& board_points, const cv::Size& board_count);
//�õ��ǵ�����̸�Ƕ�
void getAngleOfChecks(const std::vector<std::vector<cv::Point2f>>& board_points, const cv::Size checkSize, std::vector<float>& angles);
//������������ϵ�ĵ�
void generateWorldPoints(const cv::Size& patternSize, const cv::Size2f& patternLength, std::vector<cv::Point3f>& calibBoardPoint);
//������������ϵ�ĵ�
void generateWorldPoints_Multi(int imageCount, const cv::Size& patternSize, const cv::Size2f& patternLength, std::vector<std::vector<cv::Point3f>>& calibBoardPoint);

//��ȡpcd�ļ�
void readPCDfile(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud);


//ͨ����ֵ��ȡ���������ĵ�
void extratLightLinebyThreshold(
	const cv::Mat & inputImage, 
	int Low, int height, 
	std::vector<cv::Point2f>& lightImagePoint, 
	std::vector<cv::Point2f>& subPixelImagePoint);

//ͨ��stegerline��ȡ���������ĵ�
void extratLightLinebyStegerLine(
	const cv::Mat & inputImage,
	const  cv::Mat & inputlight,
	const cv::Size patternSize,
	const std::vector<cv::Point2f>& targetPoint,
	std::vector<cv::Point2f>& lightImagePoint,
	std::vector<cv::Point2f>& subPixelImagePoint);

//�Ҷ����ķ�
void extratLightLinebyGrayCenter(
	const cv::Mat & inputImage,
	int Low,
	std::vector<cv::Point2f>& lightImagePoint,
	std::vector<cv::Point2f>& subPixelImagePoint,
	const std::vector<cv::Point2f>&ROICorners,
	bool lldirection = true);//�߼����Ƿ�Ϊ��ֱ����

//�����߷������
void fitParabola(const std::vector<cv::Point2f> &vecPoints, double &a, double &b, double &c);