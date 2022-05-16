#pragma once
#include"commonFunctions.h"
#include <Eigen/Dense>
#include <io.h> 
#include <iostream>
#include <fstream>
#include <sstream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

struct Correspondence
{
	/** \brief Index of the query (source) point. */
	int index_query;
	/** \brief Index of the matching (target) point. Set to -1 if no correspondence found. */
	int index_match;
	/** \brief Distance between the corresponding points, or the weight denoting the confidence in correspondence estimation */
	union
	{
		float distance;
		float weight;
	};

	inline Correspondence() : index_query(0), index_match(-1),
		distance(std::numeric_limits<float>::max())
	{}

	/** \brief Constructor. */
	inline Correspondence(int _index_query, int _index_match, float _distance) :
		index_query(_index_query), index_match(_index_match), distance(_distance)
	{}

	/** \brief Empty destructor. */
	virtual ~Correspondence() {}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** \brief overloaded << operator */
std::ostream& operator << (std::ostream& os, const Correspondence& c);

typedef std::vector<Correspondence, Eigen::aligned_allocator<Correspondence> > Correspondences;

void TransformationEstimationSVD(const std::vector<Eigen::Vector3f>& targetcloud,
	const std::vector<Eigen::Vector3f>& inputcloud, 
	const Correspondences& correspondences, 
	Eigen::Matrix4f& transformation_matrix, 
	float& AeragePointDis,
	float& RMSE,
	int& inlierNUM);

void TransformPointCloud(std::vector<Eigen::Vector3f>& pointcloud, const Eigen::Matrix4f& transformation_matrix);

//ͨ�����߼�����ת����
//֮ǰ�ķ��ߣ�֮��ķ���
Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f before, Eigen::Vector3f after);

void getPlaneFlatness(pcl::PointCloud<pcl::PointXYZ>& cloudin, float outlierPercent, float& mazDistance, float&minDistance);