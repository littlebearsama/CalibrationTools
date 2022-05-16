#include "commonFunctions.h"
#include <vector>
#include <sstream>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <io.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // !M_PI

std::string currrentDataToString()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	//strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
	strftime(buffer, sizeof(buffer), "%Y_%m%d_%H%M_%S", timeinfo);
	std::string str(buffer);

	return str;
}

void planeFitting(Eigen::Matrix<float, Eigen::Dynamic, 3> const & points, Eigen::Vector3f & center, Eigen::Vector3f & norm)
{
	// -----------------------------------------------------
	// Plane Fitting using Singular Value Decomposition (SVD)
	// -----------------------------------------------------

	const auto n_points = points.rows();
	if (n_points == 0)
	{
		return;
	}

	//find the center by averaging the points positions
	center = points.colwise().mean().transpose();

	//copy points - average (center)
	const Eigen::Matrix<float, Eigen::Dynamic, 3> A = points.rowwise() - center.transpose();

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);
	norm = svd.matrixV().col(2);
}

void planeFitting(const pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::Vector3f & center, Eigen::Vector3f & norm)
{
	int numOfPoints = cloud.points.size();
	Eigen::Matrix<float, Eigen::Dynamic, 3> points(numOfPoints, 3);
	//points = cloud.getMatrixXfMap();
	for (int i = 0; i < numOfPoints; i++)
	{
		Eigen::Vector3f currentPt = cloud.points[i].getVector3fMap();
		//points << currentPt;
		points(i, 0) = currentPt.x();
		points(i, 1) = currentPt.y();
		points(i, 2) = currentPt.z();
	}
	planeFitting(points, center, norm);
}

std::vector<std::string> split(std::string str, char del) {
	std::stringstream ss(str);
	std::string temp;
	std::vector<std::string> ret;
	while (getline(ss, temp, del)) {
		ret.push_back(temp);
	}
	return ret;
}

bool readTXTfile(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	FILE *fp = fopen(filename.c_str(), "r");
	if (fp == nullptr)
		return false;
	cloud.points.clear();
	cloud.points.reserve(3000000);
	while (!feof(fp))
	{
		float temp[3];
		fscanf(fp, "%f %f %f\n", &temp[0], &temp[1], &temp[2]);
		cloud.points.emplace_back(temp[0], temp[1], temp[2]);
	}
	cloud.points.shrink_to_fit();
	cloud.width = 1;
	cloud.height = cloud.points.size();
	fclose(fp);
	return true;
}

void readPCDfile(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	ifstream infile;
	infile.open(filename, ios::in);

	if (!infile.is_open())
	{
		cout << "读取文件失败" << endl;
		return;
	}
	//第二种读取方法
	char buf[1024];
	//跳过前九行信息
	for (int i = 0; i < 10; i++)
	{
		infile.getline(buf, sizeof(buf));
	}

	std::vector<std::string> tempStrs = split(buf, ' ');
	//cout << tempStrs[1] << endl;
	infile.getline(buf, sizeof(buf));
	//数值
	cloud.points.clear();
	cloud.points.resize(atoi(tempStrs[1].c_str()));
	int i = 0;
	while (infile.getline(buf, sizeof(buf)))
	{
		tempStrs = split(buf, ' ');
		cloud.points[i].x = atof(tempStrs[0].c_str());
		cloud.points[i].y = atof(tempStrs[1].c_str());
		cloud.points[i].z = atof(tempStrs[2].c_str());
		//std::cout << atof(tempStrs[0].c_str()) << " " << atof(tempStrs[1].c_str()) << " " << atof(tempStrs[2].c_str()) << std::endl;
		i++;
	}

	infile.close();
	return;
}


void saveData(std::string filename, const std::vector<Eigen::Vector3f> &points)
{
	FILE * fp;

	fp = fopen(filename.c_str(), "w");
	if (fp == nullptr)
		return;
	int num = points.size();
	for (size_t i = 0; i < num; i++)
	{
		fprintf(fp, "%f %f %f\n", points[i].x(), points[i].y(), points[i].z());
	}
	fclose(fp);
}

void createFolder(std::string path)
{
	if (0 != access(path.c_str(), 0))
	{
		mkdir(path.c_str());//返回0标识创建成功；-1表示失败
	}
}

void getFiles(const std::string & path, std::vector<std::string> & files)
{
	files.clear();
	//文件句柄  
	long long hFile = 0;
	//文件信息，_finddata_t需要io.h头文件  
	struct _finddata_t fileinfo;
	std::string p;
	int i = 0;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之  
			//如果不是,加入列表  
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				//if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				//getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

void getAllFiles(const std::string & path, std::vector<std::string>& files)
{
	//文件句柄  
	long long hFile = 0;
	//文件信息，_finddata_t需要io.h头文件  
	struct _finddata_t fileinfo;
	std::string p;
	int i = 0;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之  
			//如果不是,加入列表  
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

void getFileWithEndStr(const std::vector<std::string>& filesin, std::vector<std::string>& filesout, std::string substr)
{
	filesout.clear();
	int num = filesin.size();
	int suffixSize = substr.size();
	for (int i = 0; i < num; i++) 
	{
		std::string name = filesin[i];
		std::string suffix = name.substr(name.length() - suffixSize, name.length());//末尾
		if (suffix != substr)
		{
			continue;
		}
		filesout.push_back(name);
	}


}

void getFileWithHeadStr(const std::vector<std::string>& filesin, std::vector<std::string>& filesout, std::string substr)
{
	filesout.clear();
	int num = filesin.size();
	for (int i = 0; i < num; i++)
	{
		std::string name = filesin[i];
		char drive[_MAX_DRIVE] = { 0 };
		char dir[_MAX_DIR] = { 0 };
		char fname[_MAX_FNAME] = { 0 };
		char ext[_MAX_EXT] = { 0 };
		_splitpath(name.c_str(), drive, dir, fname, ext);
		std::string currentName = std::string(fname);//文件名
		int headLen = substr.size();
		std::string headerName = currentName.substr(0, headLen);
		if (headerName != substr)
		{
			continue;
		}
		filesout.push_back(name);

	}
}

void getNameWithpath(std::string filein, std::string & name)
{
	char drive[_MAX_DRIVE] = { 0 };
	char dir[_MAX_DIR] = { 0 };
	char fname[_MAX_FNAME] = { 0 };
	char ext[_MAX_EXT] = { 0 };
	_splitpath(filein.c_str(), drive, dir, fname, ext);
	std::string currentName = std::string(drive) + std::string(dir) + std::string(fname);//文件名+路径
	std::string extName = std::string(ext);
	int Len = extName.size();
	name = currentName.substr(0, currentName.size()- Len);
}

void resetCorners(std::vector<cv::Point2f>& board_points, const cv::Size& board_count)
{
	//确保角点从上往下，从左往右排列
	if (board_points[0].x > board_points.back().x)
	{
		//交换列
		for (int i = 0; i < (int)board_count.height; i++)  //行
			for (int j = 0; j < (int)board_count.width / 2; j++) //列
				std::swap(board_points[i*board_count.width + j], board_points[(i + 1)*board_count.width - j - 1]);
	}
	if (board_points[0].y > board_points.back().y)
	{
		//交换行
		for (int i = 0; i < (int)board_count.width; i++) //列
			for (int j = 0; j < (int)board_count.height / 2; j++) //行
				std::swap(board_points[j*board_count.width + i], board_points[(board_count.height - j - 1)*board_count.width + i]);
	}
}

void getAngleOfChecks(const std::vector<std::vector<cv::Point2f>>& board_points, const cv::Size checkSize, std::vector<float>& angles)
{
	angles.clear();
	angles.resize(board_points.size());

	for (int i = 0; i < board_points.size(); i++)
	{
		std::vector<cv::Point2f> currentCheck = board_points[i];
		cv::Point2f firstPoint = currentCheck[0];
		cv::Point2f rightPoint = currentCheck[checkSize.width - 1];
		float deltaX = abs(firstPoint.x - rightPoint.x);
		float deltaY = abs(firstPoint.y - rightPoint.y);
		float angle = atan2(deltaY, deltaX)*180/ M_PI;
		angles[i] = angle;
	}
}

void generateWorldPoints(const cv::Size& patternSize, const cv::Size2f& patternLength, std::vector<cv::Point3f>& calibBoardPoint)
{
	calibBoardPoint.clear();
	for (int j = 0; j < patternSize.height; j++)
	{
		for (int k = 0; k < patternSize.width; k++)
		{
			cv::Point3f temp;
			temp.x = k*patternLength.width;
			temp.y = j*patternLength.height;
			temp.z = 0;
			calibBoardPoint.push_back(temp);
		}
	}
}

void generateWorldPoints_Multi(int imageCount, const cv::Size& patternSize, const cv::Size2f& patternLength, std::vector<std::vector<cv::Point3f>>& calibBoardPoint)
{
	calibBoardPoint.clear();
	for (int i = 0; i <imageCount; i++)
	{
		std::vector<cv::Point3f> tempPoint;
		for (int j = 0; j < patternSize.height; j++)
		{
			for (int k = 0; k < patternSize.width; k++)
			{
				cv::Point3f temp;
				temp.x = k*patternLength.width;
				temp.y = j*patternLength.height;
				temp.z = 0;
				tempPoint.push_back(temp);
			}
		}
		calibBoardPoint.push_back(tempPoint);
		tempPoint.clear();
	}
}

void extratLightLinebyThreshold(
	const cv::Mat & inputImage, 
	int Low, int height, 
	std::vector<cv::Point2f>& lightImagePoint, 
	std::vector<cv::Point2f>& subPixelImagePoint)
{
	int HEIGHT_UP = 0;
	int HEIGHT_DOWN = 0;
	int WIDTH_LEFT = 0;
	int WIDTH_RIGHT = 0;
	lightImagePoint.clear();
	subPixelImagePoint.clear();

	cv::Mat outputImage;
	threshold(inputImage, outputImage, Low, height, cv::THRESH_BINARY);
	for (size_t i = HEIGHT_UP; i < outputImage.rows + HEIGHT_DOWN; i++)//激光打到地面
	{
		float sum = 0; int num = 0;
		for (size_t j = WIDTH_LEFT; j < outputImage.cols + WIDTH_RIGHT; j++)//两边有噪点
		{
			if (outputImage.at<uchar>(i, j) == 255)
			{
				lightImagePoint.push_back(cv::Point2f(j, i));
				sum += j;
				num++;
			}
		}
		if (num == 0)
			continue;

		subPixelImagePoint.push_back(cv::Point2f(1.0*sum / num, i));
	}


}

void extratLightLinebyStegerLine(
	const cv::Mat & inputImage, 
	const  cv::Mat & inputlight, 
	const cv::Size patternSize, 
	const std::vector<cv::Point2f>& targetPoint, 
	std::vector<cv::Point2f>& lightImagePoint, 
	std::vector<cv::Point2f>& subPixelImagePoint)
{
	lightImagePoint.clear();
	subPixelImagePoint.clear();

	cv::Mat outputImage;

	cv::Mat diff;
	absdiff(inputImage, inputlight, diff);
	imwrite("save/diff.jpg", diff);

	cv::Mat element1 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	morphologyEx(diff, diff, cv::MORPH_OPEN, element1);
	imwrite("save/open.jpg", diff);

	threshold(diff, diff, 30, 255, cv::THRESH_BINARY);
	imwrite("save/threshold.jpg", diff);

	cv::Mat element2 = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	erode(diff, diff, element2);
	imwrite("save/erode.jpg", diff);
	std::vector<std::vector<cv::Point>>  allmaskPoint;
	allmaskPoint.push_back(std::vector<cv::Point>());
	cv::Mat mask = cv::Mat::zeros(inputImage.size(), CV_8UC1);
	cv::Point2f tempPoint;
	cv::Point maskPoint;
	std::vector<cv::Point2f> tempPointSet = targetPoint;
	std::vector<cv::Point2f> maskPointSet;

	tempPoint = tempPointSet[0];
	maskPoint.x = cvRound(tempPoint.x);
	maskPoint.y = cvRound(tempPoint.y);
	allmaskPoint[0].push_back(maskPoint);

	tempPoint = tempPointSet[patternSize.width - 1];
	maskPoint.x = cvRound(tempPoint.x);
	maskPoint.y = cvRound(tempPoint.y);
	allmaskPoint[0].push_back(maskPoint);

	tempPoint = tempPointSet[patternSize.width*patternSize.height - 1];
	maskPoint.x = cvRound(tempPoint.x);
	maskPoint.y = cvRound(tempPoint.y);
	allmaskPoint[0].push_back(maskPoint);

	tempPoint = tempPointSet[patternSize.width*(patternSize.height - 1)];
	maskPoint.x = cvRound(tempPoint.x);
	maskPoint.y = cvRound(tempPoint.y);
	allmaskPoint[0].push_back(maskPoint);

	drawContours(mask, allmaskPoint, 0, cv::Scalar(255), cv::FILLED, 8);
	diff.copyTo(outputImage, mask);
	imwrite("save/mask.jpg", mask);
	imwrite("save/ROI.jpg", outputImage);


	//一阶偏导数
	cv::Mat m1, m2;
	m1 = (cv::Mat_<float>(1, 2) << 1, -1);
	m2 = (cv::Mat_<float>(2, 1) << 1, -1);

	cv::Mat dx, dy;
	filter2D(outputImage, dx, CV_32FC1, m1);
	filter2D(outputImage, dy, CV_32FC1, m2);

	//二阶偏导数
	cv::Mat m3, m4, m5;
	m3 = (cv::Mat_<float>(1, 3) << 1, -2, 1);     //二阶x偏导
	m4 = (cv::Mat_<float>(3, 1) << 1, -2, 1);     //二阶y偏导
	m5 = (cv::Mat_<float>(2, 2) << 1, -1, -1, 1); //二阶xy偏导

	cv::Mat dxx, dyy, dxy;
	filter2D(outputImage, dxx, CV_32FC1, m3);
	filter2D(outputImage, dyy, CV_32FC1, m4);
	filter2D(outputImage, dxy, CV_32FC1, m5);

	for (int i = 0; i < outputImage.cols; i++)
	{
		for (int j = 0; j < outputImage.rows; j++)
		{
			//通过灰度值确定光条像素

			if (outputImage.at<uchar>(j, i) != 0)
			{
				cv::Mat hessian(2, 2, CV_32FC1);
				hessian.at<float>(0, 0) = dxx.at<float>(j, i);
				hessian.at<float>(0, 1) = dxy.at<float>(j, i);
				hessian.at<float>(1, 0) = dxy.at<float>(j, i);
				hessian.at<float>(1, 1) = dyy.at<float>(j, i);

				cv::Mat eValue, eVectors;
				eigen(hessian, eValue, eVectors);

				double nx, ny;
				double fmaxD = 0;
				if (fabs(eValue.at<float>(0, 0)) >= fabs(eValue.at<float>(1, 0)))
				{
					nx = eVectors.at<float>(0, 0);
					ny = eVectors.at<float>(0, 1);
					fmaxD = eValue.at<float>(0, 0);
				}
				else
				{
					nx = eVectors.at<float>(1, 0);
					ny = eVectors.at<float>(1, 1);
					fmaxD = eValue.at<float>(1, 0);
				}

				double t = -(nx*dx.at<float>(j, i) + ny*dy.at<float>(j, i)) / (nx*nx*dxx.at<float>(j, i) + 2 * nx*ny*dyy.at<float>(j, i) + ny*ny*dyy.at<float>(j, i));

				if ((fabs(t*nx) <= 0.5) && (fabs(t*ny) <= 0.5))
				{
					cv::Point2i oriPoint;
					oriPoint.x = i;
					oriPoint.y = j;
					lightImagePoint.push_back(oriPoint);

					cv::Point2f subpixelPoint;
					subpixelPoint.x = i + t*nx;
					subpixelPoint.y = j + t*ny;
					subPixelImagePoint.push_back(subpixelPoint);//亚像素的光条中心点
				}
			}
		}
	}
}

//灰度质心法
void extratLightLinebyGrayCenter(
	const cv::Mat & inputImage, 
	int Low, 
	std::vector<cv::Point2f>& lightImagePoint, 
	std::vector<cv::Point2f>& subPixelImagePoint, 
	const std::vector<cv::Point2f>&ROICorners, 
	bool lldirection)
{
	cv::Point2f lefttop, rightdown;
	if (ROICorners.size() == 0)
	{
		lefttop = cv::Point2f(0, 0);
		rightdown = cv::Point2f(inputImage.cols, inputImage.rows);
	}
	else
	{
		lefttop = ROICorners[0];
		rightdown = ROICorners[1];
	}

	if (lldirection)
	{
		//行
		for (size_t i = 0; i < inputImage.rows; i++)//激光打到地面
		{
			//排除ROI外的点
			if (i<lefttop.y || i>rightdown.y)
				continue;

			float sumOfGray = 0;;
			float sumOfValue = 0;
			int numOfGoodPoint = 0;
			//列
			for (size_t j = 0; j < inputImage.cols; j++)//两边有噪点
			{
				//排除ROI外的点
				if (j<lefttop.x || j>rightdown.x)
					continue;

				int current_gray = inputImage.at<uchar>(i, j);
				if (current_gray > Low)
				{
					sumOfValue += current_gray*j;
					sumOfGray += current_gray;
					numOfGoodPoint++;
				}
			}
			if (numOfGoodPoint == 0)
				continue;

			lightImagePoint.push_back(cv::Point2f(sumOfValue / sumOfGray, i));
			subPixelImagePoint.push_back(cv::Point2f(sumOfValue / sumOfGray, i));
		}
	}
	else
	{
		//列
		for (size_t i = 0; i < inputImage.cols; i++)//激光打到地面
		{
			//排除ROI外的点
			if (i<lefttop.x || i>rightdown.x)
				continue;

			float sumOfGray = 0;;
			float sumOfValue = 0;
			int numOfGoodPoint = 0;
			//行
			for (size_t j = 0; j < inputImage.rows; j++)//两边有噪点
			{
				if (i<lefttop.y || i>rightdown.y)
					continue;

				int current_gray = inputImage.at<uchar>(j, i);
				if (current_gray > Low)
				{
					sumOfValue += current_gray*j;
					sumOfGray += current_gray;
					numOfGoodPoint++;
				}
			}
			if (numOfGoodPoint == 0)
				continue;

			lightImagePoint.push_back(cv::Point2f(sumOfValue / sumOfGray, i));
			subPixelImagePoint.push_back(cv::Point2f(sumOfValue / sumOfGray, i));
		}
	}
	//把质心画在原图上
	if (0)
	{
		cv::Mat drawCenterLinePic = inputImage.clone();
		cv::cvtColor(drawCenterLinePic, drawCenterLinePic, CV_GRAY2RGB);
		for (int i = 0; i < subPixelImagePoint.size(); i++)
		{
			drawCenterLinePic.at<cv::Vec3b>(subPixelImagePoint[i].y, subPixelImagePoint[i].x) = { 255,0,0 };
		}
		cv::imwrite("centerLine.bmp", drawCenterLinePic);
	}

}


	void fitParabola(const std::vector<cv::Point2f> &vecPoints, double &a, double &b, double &c) {
		// 代表拟合的是二次方程，取 3 则为三次曲线样条
		int pow_number = 2;
		// 离散点个数
		int N = vecPoints.size();
		// 构造 X
		cv::Mat X = cv::Mat::zeros(pow_number + 1, pow_number + 1, CV_64FC1);
		for (int i = 0; i < pow_number + 1; i++) {
			for (int j = 0; j < pow_number + 1; j++) {
				for (int k = 0; k < N; k++) {
					X.at<double>(i, j) = X.at<double>(i, j) + std::pow(vecPoints[k].y, i + j);
				}
			}
		}
		// 构造 Y
		cv::Mat Y = cv::Mat::zeros(pow_number + 1, 1, CV_64FC1);
		for (int i = 0; i < pow_number + 1; i++) {
			for (int k = 0; k < N; k++) {
				Y.at<double>(i, 0) = Y.at<double>(i, 0) + std::pow(vecPoints[k].y, i) * vecPoints[k].x;
			}
		}

		// 求解 A
		cv::Mat A = cv::Mat::zeros(pow_number + 1, 1, CV_64FC1);
		cv::solve(X, Y, A, 0);

		// 输出结果，如果是三次表达式，则继续倒着往回取值
		a = A.at<double>(2, 0);
		b = A.at<double>(1, 0);
		c = A.at<double>(0, 0);
	}