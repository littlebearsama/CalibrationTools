#pragma once
//线激光点云工具
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

class LinePointsTools
{
public:
	LinePointsTools();
	~LinePointsTools();

	void setPointCloud(const pcl::PointCloud<pcl::PointXYZ>&cloud);
	void setPointCloud(std::string filename);
	
	//得到机器人坐标系下的线激光精度(内点索引)
	//输入：高度直通滤波，距离基准，最大容忍距离
	//输出：内点的平均距离
	//输出：最大的距离，最小距离
	//输出：点数，内点点数
	void getAccuracy(float minZ, float maxZ, float standardDistance, float D_Tolerance,
		float& meanDistance, float& maxD, float&minD, int &numOfPoints,int& numOfInliners);
	void getBack2GroundT(
		float minH, float maxH, float minD, float maxD, float maxOutliersDis,
		Eigen::Matrix4f& T)const;


private:
	pcl::PointCloud<pcl::PointXYZ> m_origincloud;

};

//通过点云得到线激光角度鱼垂直平面的角度
void getLineAngle(const pcl::PointCloud<pcl::PointXYZ>& cloudin,float distance, float delta, float maxZ, float minZ, float&angle);