#pragma once
#include "CameraIntrinsic.h"
#include "StereoCalibration.h"
#include "RGBtofCalibration.h"

//���ڵ��Ʊ궨
//�ָ���������ݺ󣬵õ���ǰ���ƻص����浽Z=0���ı任���󣨵�����Zֵ����
bool getBackToGroundTransformation(
	const pcl::PointCloud<pcl::PointXYZ>& cloudin,
	float minH, float maxH, float minD, float maxD, float maxOutliersDis,
	Eigen::Matrix4f& T);


//����ͼ��궨
/*
�������̸񵽻����˵ı任������Ҫ�Ĳ���
roll,pitch,yaw: ��λ���Ƕ�deg
x,y,z: ��λ����m
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

//��ʽת��
void getTransformationFormRPYNXYZ(StructureParam param, Eigen::Matrix4d& transformationMatrix);

//ͨ��������������̸�ı任�����Լ���֪�����̸񵽻����ˣ��ṹ���任����(��Ҫ�ֶ�����)���õ�����������˵ı任����
bool CalibCam2Robot(CameraIntrinsic& cam, const cv::Mat& inputImage, StructureParam check2RobotParam, Eigen::Matrix4d& matrixCam2Robot);

