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

//创建文件夹
void createFolder(std::string path);
//获取文件夹下面所有文件
void getFiles(const std::string & path, std::vector<std::string> & files);
void getAllFiles(const std::string & path, std::vector<std::string> & files);
void saveData(std::string filename, const std::vector<Eigen::Vector3f> &points);

//筛选文件
void getFileWithEndStr(const std::vector<std::string> & filesin, std::vector<std::string> & filesout, std::string substr);//找出后缀为某段字符的文件名
void getFileWithHeadStr(const std::vector<std::string> & filesin, std::vector<std::string> & filesout, std::string substr);//找出开头为某段字符的文件名
void getNameWithpath(std::string filein, std::string& name);//得到文件名（去除后缀）


//角点重排列
void resetCorners(std::vector<cv::Point2f>& board_points, const cv::Size& board_count);
//得到角点的棋盘格角度
void getAngleOfChecks(const std::vector<std::vector<cv::Point2f>>& board_points, const cv::Size checkSize, std::vector<float>& angles);
//生成世界坐标系的点
void generateWorldPoints(const cv::Size& patternSize, const cv::Size2f& patternLength, std::vector<cv::Point3f>& calibBoardPoint);
//生成世界坐标系的点
void generateWorldPoints_Multi(int imageCount, const cv::Size& patternSize, const cv::Size2f& patternLength, std::vector<std::vector<cv::Point3f>>& calibBoardPoint);

//读取pcd文件
void readPCDfile(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud);


//通过阈值提取激光线中心点
void extratLightLinebyThreshold(
	const cv::Mat & inputImage, 
	int Low, int height, 
	std::vector<cv::Point2f>& lightImagePoint, 
	std::vector<cv::Point2f>& subPixelImagePoint);

//通过stegerline提取激光线中心点
void extratLightLinebyStegerLine(
	const cv::Mat & inputImage,
	const  cv::Mat & inputlight,
	const cv::Size patternSize,
	const std::vector<cv::Point2f>& targetPoint,
	std::vector<cv::Point2f>& lightImagePoint,
	std::vector<cv::Point2f>& subPixelImagePoint);

//灰度质心法
void extratLightLinebyGrayCenter(
	const cv::Mat & inputImage,
	int Low,
	std::vector<cv::Point2f>& lightImagePoint,
	std::vector<cv::Point2f>& subPixelImagePoint,
	const std::vector<cv::Point2f>&ROICorners,
	bool lldirection = true);//线激光是否为竖直方向

//抛物线方程拟合
void fitParabola(const std::vector<cv::Point2f> &vecPoints, double &a, double &b, double &c);