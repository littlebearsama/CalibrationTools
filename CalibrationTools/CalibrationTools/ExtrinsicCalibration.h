#pragma once
#include "CameraIntrinsic.h"
#include "StereoCalibration.h"
#include "RGBtofCalibration.h"

//基于点云标定
//分割出地面数据后，得到当前点云回到地面到Z=0处的变换矩阵（地面是Z值方向）
bool getBackToGroundTransformation(
	const pcl::PointCloud<pcl::PointXYZ>& cloudin,
	float minH, float maxH, float minD, float maxD, float maxOutliersDis,
	Eigen::Matrix4f& T);


//基于图像标定
/*
计算棋盘格到机器人的变换矩阵需要的参数
roll,pitch,yaw: 单位：角度deg
x,y,z: 单位：米m
*/
struct StructureParam
{
	double roll;
	double pitch;
	double yaw;
	double x;
	double y;
	double z;
};

//格式转换
void getTransformationFormRPYNXYZ(StructureParam param, Eigen::Matrix4d& transformationMatrix);

//通过计算相机到棋盘格的变换矩阵，以及已知的棋盘格到机器人（结构）变换矩阵(需要手动测量)，得到相机到机器人的变换矩阵
bool CalibCam2Robot(CameraIntrinsic& cam, const cv::Mat& inputImage, StructureParam check2RobotParam, Eigen::Matrix4d& matrixCam2Robot);

