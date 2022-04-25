#pragma once
//�߼�����ƹ���
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
	
	//�õ�����������ϵ�µ��߼��⾫��(�ڵ�����)
	//���룺�߶�ֱͨ�˲��������׼��������̾���
	//������ڵ��ƽ������
	//��������ľ��룬��С����
	//������������ڵ����
	void getAccuracy(float minZ, float maxZ, float standardDistance, float D_Tolerance,
		float& meanDistance, float& maxD, float&minD, int &numOfPoints,int& numOfInliners);
	void getBack2GroundT(
		float minH, float maxH, float minD, float maxD, float maxOutliersDis,
		Eigen::Matrix4f& T)const;


private:
	pcl::PointCloud<pcl::PointXYZ> m_origincloud;

};

//ͨ�����Ƶõ��߼���Ƕ��㴹ֱƽ��ĽǶ�
void getLineAngle(const pcl::PointCloud<pcl::PointXYZ>& cloudin,float distance, float delta, float maxZ, float minZ, float&angle);