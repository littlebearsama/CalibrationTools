#include"Functions.h"

void TransformPointCloud(std::vector<Eigen::Vector3f>& pointcloud, const Eigen::Matrix4f& transformation_matrix)
{
	if (transformation_matrix == Eigen::Matrix4f::Identity())
	{
		return;
	}
#pragma parallel for
	for (int i = 0; i < pointcloud.size(); i++)
	{
		pointcloud[i] = transformation_matrix.topLeftCorner<3, 3>()*pointcloud[i] + transformation_matrix.topRightCorner<3, 1>();
	}
}

//去重心坐标点集计算变换矩阵
void TransformationEstimationSVD(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& cloud_src_demean,
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& cloud_tgt_demean,
	const Eigen::Matrix<float, 4, 1>& centroid_src,
	const Eigen::Matrix<float, 4, 1>& centroid_tgt,
	Eigen::Matrix4f& transformation_matrix)
{
	transformation_matrix.setIdentity();
	// Assemble the correlation matrix H = source * target'
	Eigen::Matrix<float, 3, 3> H = (cloud_src_demean * cloud_tgt_demean.transpose()).topLeftCorner(3, 3);
	// Compute the Singular Value Decomposition
	Eigen::JacobiSVD<Eigen::Matrix<float, 3, 3>> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix<float, 3, 3> u = svd.matrixU();
	Eigen::Matrix<float, 3, 3> v = svd.matrixV();
	// Compute R = V * U'
	if (u.determinant() * v.determinant() < 0) {
		for (int x = 0; x < 3; ++x)
			v(x, 2) *= -1;
	}
	Eigen::Matrix<float, 3, 3> R = v * u.transpose();
	// Return the correct transformation
	transformation_matrix.topLeftCorner(3, 3) = R;
	const Eigen::Matrix<float, 3, 1> Rc(R * centroid_src.head(3));
	transformation_matrix.block(0, 3, 3, 1) = centroid_tgt.head(3) - Rc;
}



void TransformationEstimationSVD(const std::vector<Eigen::Vector3f>& targetcloud,
	const std::vector<Eigen::Vector3f>& inputcloud, 
	const Correspondences& correspondences, 
	Eigen::Matrix4f& transformation_matrix, 
	float& AeragePointDis, 
	float& RMSE, 
	int& inlierNUM)
{
	int num = correspondences.size();
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cloud_src = Eigen::Matrix<float, 4, Eigen::Dynamic>::Zero(4, num);
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cloud_tgt = Eigen::Matrix<float, 4, Eigen::Dynamic>::Zero(4, num);
	Eigen::Matrix<float, 4, 1> centroid_src, centroid_tgt;
	centroid_src.setZero();
	centroid_tgt.setZero();
	unsigned cp = 0;
	float squareDistance = 0;
	float pointdis = 0;
	for (int i = 0; i < num; i++)
	{
		int q_index = correspondences[i].index_query;
		int m_index = correspondences[i].index_match;
		if (q_index == -1)
			continue;
		cloud_src(0, cp) = inputcloud[q_index].x();
		cloud_src(1, cp) = inputcloud[q_index].y();
		cloud_src(2, cp) = inputcloud[q_index].z();
		centroid_src[0] += inputcloud[q_index].x();
		centroid_src[1] += inputcloud[q_index].y();
		centroid_src[2] += inputcloud[q_index].z();

		cloud_tgt(0, cp) = targetcloud[m_index].x();
		cloud_tgt(1, cp) = targetcloud[m_index].y();
		cloud_tgt(2, cp) = targetcloud[m_index].z();
		centroid_tgt[0] += targetcloud[m_index].x();
		centroid_tgt[1] += targetcloud[m_index].y();
		centroid_tgt[2] += targetcloud[m_index].z();
		++cp;
		squareDistance += correspondences[i].distance;
		pointdis += sqrt(correspondences[i].distance);
	}
	inlierNUM = cp;
	if (cp == 0)
		return;

	RMSE = sqrt(squareDistance / static_cast<float>(cp));
	AeragePointDis = pointdis / cp;

	centroid_src /= static_cast<float>(cp);
	centroid_src[3] = 1;
	centroid_tgt /= static_cast<float>(cp);
	centroid_tgt[3] = 1;
	//Eigen::Matrix<float, 4, 1> centroid_src = cloud_src.rowwise().mean();
	//Eigen::Matrix<float, 4, 1> centroid_tgt = cloud_tgt.rowwise().mean();
	//cloud_src_demean = cloud_tgt - centroid_src;
	//cloud_tgt_demean = cloud_tgt - centroid_tgt;
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cloud_src_demean = Eigen::Matrix<float, 4, Eigen::Dynamic>::Zero(4, cp);
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cloud_tgt_demean = Eigen::Matrix<float, 4, Eigen::Dynamic>::Zero(4, cp);

#pragma omp parallel for
	for (int i = 0; i < cp; i++)
	{
		cloud_src_demean(0, i) = cloud_src(0, i) - centroid_src[0];
		cloud_src_demean(1, i) = cloud_src(1, i) - centroid_src[1];
		cloud_src_demean(2, i) = cloud_src(2, i) - centroid_src[2];
		cloud_tgt_demean(0, i) = cloud_tgt(0, i) - centroid_tgt[0];
		cloud_tgt_demean(1, i) = cloud_tgt(1, i) - centroid_tgt[1];
		cloud_tgt_demean(2, i) = cloud_tgt(2, i) - centroid_tgt[2];
	}
	//
	TransformationEstimationSVD(cloud_src_demean, cloud_tgt_demean, centroid_src, centroid_tgt, transformation_matrix);
}

//struct CameraInstrinsics
//{
//	float scale;
//	float camera_cx;
//	float camera_cy;
//	float camera_fx;
//	float camera_fy;
//
//
//};
//
//void GetPointCloudFormDepthNInstrinsics(cv::Mat& depth, CameraInstrinsics& instrinsics, std::vector<Eigen::Vector3f>& pointcloud)
//{
//	pointcloud.clear();
//	float scale = instrinsics.scale;
//	float cx = instrinsics.camera_cx;
//	float cy = instrinsics.camera_cy;
//	float fx = instrinsics.camera_fx;
//	float fy = instrinsics.camera_fy;
//	for (int m = 0; m < depth.rows; m++)
//		for (int n = 0; n < depth.cols; n++)
//		{
//			// 获取深度图中(m,n)处的值
//			float d = depth.ptr<ushort>(m)[n];//深度
//											  // d 可能没有值，若如此，跳过此点
//			if (d == 0)
//				continue;
//			// d 存在值，则向点云增加一个点
//			Eigen::Vector3f  p;
//			// 计算这个点的空间坐标
//			p.z() = d / scale;
//			p.x() = (float(n) - cx) * p.z() / fx;
//			p.y() = (float(m) - cy) * p.z() / fy;
//			//保留小数点后三位
//
//
//			//// 从rgb图像中获取它的颜色
//			//// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
//			//p.b = color_.ptr<uchar>(m)[n * 3];
//			//p.g = color_.ptr<uchar>(m)[n * 3 + 1];
//			//p.r = color_.ptr<uchar>(m)[n * 3 + 2];
//			//// 把p加入到点云中
//			pointcloud.push_back(p);
//		}
//}
Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f before, Eigen::Vector3f after)
{
	before.normalize();
	after.normalize();

	float angle = acos(before.dot(after));
	Eigen::Vector3f p_rotate = before.cross(after);
	p_rotate.normalize();

	Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
	rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
	rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle));//这里跟公式比多了一个括号，但是看实验结果它是对的。
	rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));


	rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
	rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
	rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));


	rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
	rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
	rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));

	return rotationMatrix;
}
