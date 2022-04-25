#include "ExtrinsicCalibration.h"
#include <pcl/segmentation/sac_segmentation.h>
#include "Functions.h"

bool getBackToGroundTransformation(const pcl::PointCloud<pcl::PointXYZ>& cloudin, float minH, float maxH, float minD, float maxD, float maxOutliersDis, Eigen::Matrix4f& T)
{
	T = Eigen::Matrix4f::Identity();
	//currentSaveNUM++;
	const int MINcountOfGroundpoints = 20;//一个地面上最少有20个点。

										  //对两个方向上地点进行直通滤波（距离方向X，高度方向Z）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < cloudin.size(); i++)
	{
		if (cloudin.points[i].x >= minD&&cloudin.points[i].x <= maxD&&cloudin.points[i].z >= minH&&cloudin.points[i].z <= maxH)
		{
			cloud_filtered->points.push_back(cloudin.points[i]);
		}
	}

	if (cloud_filtered->size() <= MINcountOfGroundpoints)
	{
		std::cout << "Error1: the counts of point is not enough to fit groung plane" << std::endl;
		return false;
	}
	//std::string savefilename = "Recitify2Ground1//" + std::to_string(currentSaveNUM) + "filtered.txt";
	//writerPC(savefilename, *cloud_filtered);
	//拟合平面
	float A, B, C, D;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(1000);

	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return false;
	}
	A = coefficients->values[0];
	B = coefficients->values[1];
	C = coefficients->values[2];
	D = coefficients->values[3];

	//remove outliers
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < cloud_filtered->points.size(); i++)
	{
		float currentDis = A*cloud_filtered->points[i].x + B*cloud_filtered->points[i].y + C*cloud_filtered->points[i].z + D;
		if (currentDis <= maxOutliersDis)
		{
			cloud_filtered2->points.push_back(cloud_filtered->points[i]);
		}
	}
	//std::string savefilename2 = "Recitify2Ground1//" + std::to_string(currentSaveNUM) + "filtered2.txt";
	//writerPC(savefilename2, *cloud_filtered2);

	if (cloud_filtered2->size() <= MINcountOfGroundpoints)
	{
		std::cout << "Error2: the counts of point is not enough to fit groung plane" << std::endl;
		return false;
	}
	//再拟合一遍去噪点的平面
	float a, b, c, d;
	pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg2;
	seg2.setOptimizeCoefficients(true);
	seg2.setModelType(pcl::SACMODEL_PLANE);
	seg2.setMethodType(pcl::SAC_RANSAC);
	//seg.setDistanceThreshold(0.01);
	seg2.setDistanceThreshold(1000);
	seg2.setInputCloud(cloud_filtered2);
	seg2.segment(*inliers2, *coefficients2);
	a = coefficients2->values[0];
	b = coefficients2->values[1];
	c = coefficients2->values[2];
	d = coefficients2->values[3];
	//std::cout << "a:" << a << " b:" << b << " c:" << c << " d:" << d << std::endl;
	//求旋转矩阵
	Eigen::Vector3f before = Eigen::Vector3f(a, b, c);
	Eigen::Vector3f after = Eigen::Vector3f(0, 0, 1);
	Eigen::Matrix4f R = CreateRotateMatrix(before, after);
	T = R;


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud_filtered2, *cloud_filtered3, T);
	//std::cout << "R:" << std::endl;
	//std::cout << T << std::endl;
	Eigen::Vector4f centroidOriginal;
	Eigen::Vector4f centroidMoved;
	pcl::compute3DCentroid(*cloud_filtered2, centroidOriginal);
	pcl::compute3DCentroid(*cloud_filtered3, centroidMoved);
	Eigen::Vector4f tanslation = centroidMoved - centroidOriginal;
	T(0, 3) = tanslation.x();//平移回原来的重心位置
	T(1, 3) = tanslation.y();//平移回原来的重心位置
	T(2, 3) = -centroidMoved.z();//平移到地面
	return true;
}


double deg2radian(float angleRadian)
{
	return angleRadian * M_PI / 180;
}
void getTransformationFormRPYNXYZ(StructureParam param, Eigen::Matrix4d& transformationMatrix)
{
	//旋转矩阵,T坐标系在R坐标系下的描述
	double roll_rad = deg2radian(param.roll);
	Eigen::Matrix3d R_roll;
	R_roll << 1, 0, 0,
		0, cos(roll_rad), -sin(roll_rad),
		0, sin(roll_rad), cos(roll_rad);

	double pitch_rad = deg2radian(param.pitch);
	Eigen::Matrix3d R_pitch;
	R_pitch << cos(pitch_rad), 0, sin(pitch_rad),
		0, 1, 0,
		-sin(pitch_rad), 0, cos(pitch_rad);

	double yaw_rad = deg2radian(param.yaw);
	Eigen::Matrix3d R_yaw;
	R_yaw << cos(yaw_rad), -sin(yaw_rad), 0,
		sin(yaw_rad), cos(yaw_rad), 0,
		0, 0, 1;

	//内旋（非定轴）顺序：z->y->x（右乘）， 外旋（定轴）顺序：x->y->z（左乘）
	//此处使用内旋，方便手动测量
	Eigen::Matrix3d R = R_yaw * R_pitch * R_roll;


	//输出
	Eigen::Matrix4d matrix_t2r;
	matrix_t2r << R(0, 0), R(0, 1), R(0, 2), param.x,
		R(1, 0), R(1, 1), R(1, 2), param.y,
		R(2, 0), R(2, 1), R(2, 2), param.z,
		0, 0, 0, 1;
}

bool CalibCam2Robot(CameraIntrinsic& cam, const cv::Mat& inputImage, StructureParam check2RobotParam, Eigen::Matrix4d& matrixCam2Robot)
{
	//畸变矫正
	Eigen::Matrix4d matrixCheck2Cam;
	bool flag = cam.getCheckBoardPose(inputImage, matrixCheck2Cam);
	
	//得到棋盘格位姿失败就返回
	if (!flag)
	{
		return false;
	}
	Eigen::Matrix4d matrixCheck2RobotStructure;
	getTransformationFormRPYNXYZ(check2RobotParam, matrixCheck2RobotStructure);
	matrixCam2Robot = matrixCheck2RobotStructure*matrixCheck2Cam.inverse();
	return true;
}
