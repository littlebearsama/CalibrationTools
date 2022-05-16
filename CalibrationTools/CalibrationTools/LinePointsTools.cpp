#include "LinePointsTools.h"
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "commonFunctions.h"
#include "ExtrinsicCalibration.h"

LinePointsTools::LinePointsTools()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointer(new pcl::PointCloud<pcl::PointXYZ>);
	m_origincloud = *cloudPointer;
	cloudPointer = m_origincloud.makeShared();
}


LinePointsTools::~LinePointsTools()
{

}

void LinePointsTools::setPointCloud(std::string filename)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	readPCDfile(filename, cloud);
	pcl::copyPointCloud(cloud, m_origincloud);
}

void LinePointsTools::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>&cloud)
{
	pcl::copyPointCloud(cloud, m_origincloud);
}

void LinePointsTools::getAccuracy(float minZ, float maxZ, float standardDistance, float D_Tolerance, float& meanDistance, float& maxD, float&minD, int &numOfPoints, int& numOfInliners)
{
	pcl::PassThrough<pcl::PointXYZ> passfilter;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(m_origincloud, *cloud);

	passfilter.setInputCloud(cloud);
	passfilter.setFilterFieldName("z");
	passfilter.setFilterLimits(minZ, maxZ);
	passfilter.filter(*cloud_filtered);

	passfilter.setInputCloud(cloud_filtered);
	passfilter.setFilterFieldName("x");
	passfilter.setFilterLimits(1, 10000);//距离值必须大于0
	passfilter.filter(*cloud_filtered);

	numOfPoints = cloud_filtered->points.size();
	int inlierscount = 0;
	double totalDistance = 0;
	float maxDistance = -FLT_MAX;
	float minDistance = FLT_MAX;
	for (int i = 0; i < numOfPoints; i++)
	{
		pcl::PointXYZ currentPoint = cloud_filtered->points[i];
		maxDistance = maxDistance > currentPoint.x ? maxDistance : currentPoint.x;
		minDistance = minDistance < currentPoint.x ? minDistance : currentPoint.x;
		if (fabs(currentPoint.x- standardDistance)>D_Tolerance)
		{
			continue;
		}
		totalDistance += currentPoint.x;
		inlierscount++;
	}

	if (inlierscount)
	{
		meanDistance = totalDistance / inlierscount;
	}
	else
	{
		meanDistance = NAN;
	}
	maxD = maxDistance;
	minD = minDistance;
	numOfInliners = inlierscount;
}

void LinePointsTools::getBack2GroundT(float minH, float maxH, float minD, float maxD, float maxOutliersDis, Eigen::Matrix4f& T) const
{
	getBackToGroundTransformation(m_origincloud, minH, maxH, minD, maxD, maxOutliersDis, T);
}

void getWallPitch(const pcl::PointCloud<pcl::PointXYZ>& cloudin, Eigen::Matrix4f & T, float & pitch)
{
	T = Eigen::Matrix4f::Identity();
	//currentSaveNUM++;
	const int MINcountOfGroundpoints = 20;//一个墙面上最少有20个点。
	//去除零点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < cloudin.size(); i++)
	{
		pcl::PointXYZ currentPoint = cloudin.points[i];
		if (abs(currentPoint.x) < 0.01&&abs(currentPoint.y) < 0.01&&abs(currentPoint.z) < 0.01)
			continue;
		cloud_filtered->points.push_back(currentPoint);
	}
	
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
		return;
	}
	A = coefficients->values[0];
	B = coefficients->values[1];
	C = coefficients->values[2];
	D = coefficients->values[3];

	Eigen::Vector3f before(0, 0, C);
	Eigen::Vector3f after(0, 0, 1);
	T = CreateRotateMatrix(before, after);
	pitch = acos(C);
}

void getLineAngle(const pcl::PointCloud<pcl::PointXYZ>& cloudin, float distance, float delta, float maxZ, float minZ, float&angle)
{
	pcl::PassThrough<pcl::PointXYZ> passfilter;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloudin, *cloud);

	passfilter.setInputCloud(cloud);
	passfilter.setFilterFieldName("x");
	passfilter.setFilterLimits(distance-delta, distance+delta);
	passfilter.filter(*cloud_filtered);

	passfilter.setInputCloud(cloud_filtered);
	passfilter.setFilterFieldName("z");
	passfilter.setFilterLimits(minZ, maxZ);//距离值必须大于0
	passfilter.filter(*cloud_filtered);
	//计算线激光角度
	int sizeOfPoints = cloud_filtered->points.size();
	if (sizeOfPoints < 3)
		return;

	//在ZOY平面上拟合直线
	cv::Vec4f lightline_para;//vx, vy, x0, y0
	std::vector<cv::Point2f> lightLinePoint;//取出第i张标定图像 的亚像素光条中心点
	for (int i = 0; i < sizeOfPoints; i++)
	{
		lightLinePoint.push_back(cv::Point2f(cloud_filtered->points[i].y, cloud_filtered->points[i].z));
	}

	fitLine(lightLinePoint, lightline_para, cv::DIST_L2, 0, 1e-2, 1e-2);//将第i张图的 光条中心点进行直线拟合

	std::cout << "line_para = " << lightline_para << std::endl;

	//获取点斜式的点和斜率
	cv::Point point0;
	point0.x = lightline_para[2];
	point0.y = lightline_para[3];
	//
	if (abs(lightline_para[1])<FLT_MIN)
	{
		angle = 0;
		return;
	}
	//double k = lightline_para[0] / lightline_para[1];
	//(y = k(x - x0) + y0)
	//y = (p0/p1)*(x-p2)+p3
	float currentangle = atan2f(lightline_para[1], lightline_para[0]);
	currentangle = 180 * currentangle / M_PI;
	angle = (currentangle>0) ? (90 - currentangle) : -(90 + currentangle);
}
