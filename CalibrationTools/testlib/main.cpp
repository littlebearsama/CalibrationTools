#include "commonFunctions.h"
#include "CameraIntrinsic.h"
#include "StereoCalibration.h"
#include "RGBtofCalibration.h"
#include "LinePointsTools.h"
#include "Functions.h"

#include <io.h> 
#include <iostream>
#include <fstream>
#include <sstream>

#include <pcl/filters/passthrough.h>


//测试相机内参标定
void testCameraIntrinsic1();
//测试立体标定
void testStereoCalibaration1();
//测试立体矫正
void testStereoRectify();
//测试RGB相机与TOF深度数据融合
void testRGBtofCalibration();
//测试RealSense立体标定
void testStereoCalibaration_realsense();
//测试RealSense立体校正
void testStereoRectify_realsense();

//标定相机内参
void calcamintrinsic();

void rectifyPC2floor();

//计算激光线角度
void getLineAngleByPointCloud();

//测试计算点云平面度
void testPlaneFlatness();


int main()
{
	//testPlaneFlatness();
	//getLineAngleByPointCloud();
	//calcamintrinsic();
	//testRGBtofCalibration();
	//testStereoCalibaration1();
	testStereoRectify();
	//testCameraIntrinsic1();

	//testStereoCalibaration_realsense();
	//testStereoRectify_realsense();
	return 0;
}


void calcamintrinsic()
{
	std::string saveName1 = "../../Data/CalibrationData/sameCam/2022_03_28_17_30_03_119/intinsic.yml";
	//std::string saveName1 = "../../Data/CalibrationData/sameCam/all/intinsic.yml";
	std::string saveName2 = "../../Data/CalibrationData/sameCam/2022_03_28_19_34_26_137/intinsic.yml";
	std::string picsFolder1 = "../../Data/CalibrationData/sameCam/2022_03_28_17_30_03_119/";
	//std::string picsFolder1 = "../../Data/CalibrationData/sameCam/all/";
	std::string picsFolder2 = "../../Data/CalibrationData/sameCam/2022_03_28_19_34_26_137/";
	//std::string saveName = "../../Data/CalibrationData/2022_03_09_17_34_05_025/intinsic.yml";
	//std::string picsFolder = "../../Data/CalibrationData/2022_03_09_17_34_05_025/2022_03_09_17_34_05_025/";
	std::vector<std::string> filesNames1;
	std::vector<std::string> filesNames2;
	getFiles(picsFolder1, filesNames1);
	getFiles(picsFolder2, filesNames2);
	std::vector<std::string> filesout1;
	std::vector<std::string> filesout2;
	std::string substr = ".bmp";
	getFileWithEndStr(filesNames1, filesout1, substr);
	getFileWithEndStr(filesNames2, filesout2, substr);

	CameraIntrinsic intrinsic1;
	CameraIntrinsic intrinsic2;
	cv::Size imageSize(640, 480);
	cv::Size patternSize(23, 17);
	cv::Size2f patternLength(5, 5);
	bool isCricle = false;
	bool isfisheye = false;
	bool complexCameraModel = false;
	intrinsic1.setParameters(filesout1, imageSize, patternSize, patternLength, isCricle, isfisheye, complexCameraModel);
	intrinsic1.compute();
	intrinsic1.saveCameraIntrinsic(saveName1);

	intrinsic2.setParameters(filesout2, imageSize, patternSize, patternLength, isCricle, isfisheye, complexCameraModel);
	intrinsic2.compute();
	intrinsic2.saveCameraIntrinsic(saveName2);
}
//测试内参
void testCameraIntrinsic1()
{
	std::string saveName = "../../Data/CalibrationData/2022_03_09_20_17_49_094/intinsic.yml";
	std::string picsFolder = "../../Data/CalibrationData/2022_03_09_20_17_49_094/2022_03_09_20_17_49_094/";
	//std::string saveName = "../../Data/CalibrationData/2022_03_09_17_34_05_025/intinsic.yml";
	//std::string picsFolder = "../../Data/CalibrationData/2022_03_09_17_34_05_025/2022_03_09_17_34_05_025/";
	std::vector<std::string> filesNames;
	getFiles(picsFolder, filesNames);
	CameraIntrinsic intrinsic_025;
	cv::Size imageSize(640, 480);
	cv::Size patternSize(23, 17);
	cv::Size2f patternLength(5, 5);
	bool isCricle = false;
	bool isfisheye = false;
	bool complexCameraModel = false;
	//if (!intrinsic_025.readCameraIntrinsic(saveName)) 
	//{

	//}
	intrinsic_025.setParameters(filesNames, imageSize, patternSize, patternLength, isCricle, isfisheye, complexCameraModel);
	intrinsic_025.compute();
	intrinsic_025.saveCameraIntrinsic(saveName);
}
//测试立体标定
void testStereoCalibaration1()
{
	StereoCalibration stereoCalib;
	CameraIntrinsic camLeft, camRight;
	std::string intrinsicSaveNameLeft = "D://codes//Reconstruction//Calibrations//Data//StereoData//testStereoAlgorithmData2//intinsicLeft.yml";
	std::string intrinsicSaveNameRight = "D://codes//Reconstruction//Calibrations//Data//StereoData//testStereoAlgorithmData2//intinsicRight.yml";
	std::string intrinsicSaveNameStereo = "D://codes//Reconstruction//Calibrations//Data//StereoData//testStereoAlgorithmData2//stereoModel.yml";
	std::string calibrationPicsFolder = "D://codes//Reconstruction//Calibrations//Data//StereoData//testStereoAlgorithmData2//CalibrationPics//";
	std::vector<std::string> filesNamesLeft;
	std::vector<std::string> filesNamesRight;
	std::vector<std::string> filesNamesAll;
	getFiles(calibrationPicsFolder, filesNamesAll);
	std::string leftHead = "Left";
	std::string rightHead = "Right";
	bool isCricle = false;
	bool isfisheye = false;
	bool complexCameraModel = false;
	getFileWithHeadStr(filesNamesAll, filesNamesLeft, leftHead);
	getFileWithHeadStr(filesNamesAll, filesNamesRight, rightHead);
	cv::Size imageSize(1280, 960);
	cv::Size patternSize(11, 8);
	cv::Size2f patternLength(30, 30);
	
	//为了获取角点信息必须重新标定左相机
	//为了获取角点信息必须重新标定右相机
	camLeft.setMaxReproectionError(0.01);
	camLeft.setParameters(filesNamesLeft, imageSize, patternSize, patternLength, isCricle, isfisheye, complexCameraModel);
	camLeft.compute();
	camLeft.saveCameraIntrinsic(intrinsicSaveNameLeft);

	camRight.setMaxReproectionError(0.01);
	camRight.setParameters(filesNamesRight, imageSize, patternSize, patternLength, isCricle, isfisheye, complexCameraModel);
	camRight.compute();
	camRight.saveCameraIntrinsic(intrinsicSaveNameRight);

	//
	stereoCalib.m_leftCamera = camLeft;
	stereoCalib.m_rightCamera = camRight;
	stereoCalib.m_leftCamera.printAllParameter();
	stereoCalib.m_rightCamera.printAllParameter();
	stereoCalib.compute();
	stereoCalib.saveStereoModel(intrinsicSaveNameStereo);
}
//测试立体标定
void testStereoCalibaration_realsense()
{
	StereoCalibration stereoCalib;
	CameraIntrinsic camLeft, camRight;
	std::string intrinsicSaveNameLeft = "../../Data/StereoData/realsense/intinsicLeft.yml";
	std::string intrinsicSaveNameRight = "../../Data/StereoData/realsense/intinsicRight.yml";
	std::string intrinsicSaveNameStereo = "../../Data/StereoData/realsense/stereoModel.yml";
	std::string cam1PicsFolder = "../../Data/StereoData/realsense/realsense/";
	std::vector<std::string> filesNamesLeft;
	std::vector<std::string> filesNamesRight;
	std::vector<std::string> filesNamesAll;
	getFiles(cam1PicsFolder, filesNamesAll);
	std::string leftHead = "ir_left";
	std::string rightHead = "ir_right";
	bool isCricle = false;
	bool isfisheye = false;
	bool complexCameraModel = false;
	getFileWithHeadStr(filesNamesAll, filesNamesLeft, leftHead);
	getFileWithHeadStr(filesNamesAll, filesNamesRight, rightHead);
	cv::Size imageSize(1280, 720);
	cv::Size patternSize(11, 8);
	cv::Size2f patternLength(30, 30);

	//为了获取角点信息必须重新标定左相机
	//为了获取角点信息必须重新标定右相机
	camLeft.setMaxReproectionError(1.0);//必须设大，不然会剔除一些质量不是很好的角点使得左右相机的角点对应不上
	camLeft.setParameters(filesNamesLeft, imageSize, patternSize, patternLength, isCricle, isfisheye, complexCameraModel);
	camLeft.compute();
	camLeft.saveCameraIntrinsic(intrinsicSaveNameLeft);

	camRight.setMaxReproectionError(1.0);
	camRight.setParameters(filesNamesRight, imageSize, patternSize, patternLength, isCricle, isfisheye, complexCameraModel);
	camRight.compute();
	camRight.saveCameraIntrinsic(intrinsicSaveNameRight);

	//
	stereoCalib.m_leftCamera = camLeft;
	stereoCalib.m_rightCamera = camRight;
	stereoCalib.m_leftCamera.printAllParameter();
	stereoCalib.m_rightCamera.printAllParameter();
	stereoCalib.compute();
	stereoCalib.saveStereoModel(intrinsicSaveNameStereo);
}

//测试立体矫正
void testStereoRectify_realsense()
{
	std::string intrinsicSaveNameLeft = "../../Data/StereoData/realsense/intinsicLeft.yml";
	std::string intrinsicSaveNameRight = "../../Data/StereoData/realsense/intinsicRight.yml";
	std::string intrinsicSaveNameStereo = "../../Data/StereoData/realsense/stereoModel.yml";

	//std::string  testImageFolder = "../../Data/StereoData/120mm_baseline/cameraData";
	//std::string  testImageUndistoredFolder = "../../Data/StereoData/120mm_baseline/cameraData_undistord";
	std::string  testImageFolder = "../../Data/StereoData/realsense/realsense_test";
	std::string  testImageUndistoredFolder = "../../Data/StereoData/realsense/realsense_test_undistord";

	std::vector<std::string> files;
	getFiles(testImageFolder, files);
	std::vector<std::string> filesNamesLeft;
	std::vector<std::string> filesNamesRight;
	std::string leftHead = "ir_left";
	std::string rightHead = "ir_right";
	getFileWithHeadStr(files, filesNamesLeft, leftHead);
	getFileWithHeadStr(files, filesNamesRight, rightHead);
	StereoCalibration stereoCalib(intrinsicSaveNameStereo, intrinsicSaveNameLeft, intrinsicSaveNameRight);
	for (int i = 0; i < filesNamesLeft.size(); i++)
	{
		cv::Mat rectifyImageL, rectifyImageR;
		cv::Mat ImageL, ImageR;
		ImageL = cv::imread(filesNamesLeft[i]);
		ImageR = cv::imread(filesNamesRight[i]);
		stereoCalib.rectifyImage(ImageL, ImageR, rectifyImageL, rectifyImageR, true);

		char drive[_MAX_DRIVE] = { 0 };
		char dir[_MAX_DIR] = { 0 };
		char fname[_MAX_FNAME] = { 0 };
		char ext[_MAX_EXT] = { 0 };
		_splitpath(filesNamesLeft[i].c_str(), drive, dir, fname, ext);
		std::string leftUndistordedName = testImageUndistoredFolder + "/" + std::string(fname) + "_Un.png";
		_splitpath(filesNamesRight[i].c_str(), drive, dir, fname, ext);
		std::string rightUndistordedName = testImageUndistoredFolder + "/" + std::string(fname) + "_Un.png";

		cv::imwrite(leftUndistordedName, rectifyImageL);
		cv::imwrite(rightUndistordedName, rectifyImageR);
	}
}
//测试立体矫正
void testStereoRectify()
{
	std::string intrinsicSaveNameLeft = "D://codes//Reconstruction//Calibrations//Data//StereoData//testStereoAlgorithmData2//intinsicLeft.yml";
	std::string intrinsicSaveNameRight = "D://codes//Reconstruction//Calibrations//Data//StereoData//testStereoAlgorithmData2//intinsicRight.yml";
	std::string intrinsicSaveNameStereo = "D://codes//Reconstruction//Calibrations//Data//StereoData//testStereoAlgorithmData2//stereoModel.yml";
	std::string  testImageFolder = "D://codes//Reconstruction//Calibrations//Data//StereoData//testStereoAlgorithmData2//YLWlight//";
	std::string  testImageUndistoredFolder = "D://codes//Reconstruction//Calibrations//Data//StereoData//testStereoAlgorithmData2//YLWlightUndistored//";

	createFolder(testImageUndistoredFolder);

	std::vector<std::string> files;
	getFiles(testImageFolder, files);
	std::vector<std::string> filesNamesLeft;
	std::vector<std::string> filesNamesRight;
	std::string leftHead = "Left";
	std::string rightHead = "Right";
	getFileWithHeadStr(files, filesNamesLeft, leftHead);
	getFileWithHeadStr(files, filesNamesRight, rightHead);
	StereoCalibration stereoCalib(intrinsicSaveNameStereo, intrinsicSaveNameLeft, intrinsicSaveNameRight);
	for (int i = 0; i < filesNamesLeft.size(); i++)
	{
		cv::Mat rectifyImageL, rectifyImageR;
		cv::Mat ImageL, ImageR;
		ImageL = cv::imread(filesNamesLeft[i]);
		ImageR = cv::imread(filesNamesRight[i]);
		stereoCalib.rectifyImage(ImageL, ImageR, rectifyImageL, rectifyImageR, false);

		char drive[_MAX_DRIVE] = { 0 };
		char dir[_MAX_DIR] = { 0 };
		char fname[_MAX_FNAME] = { 0 };
		char ext[_MAX_EXT] = { 0 };
		_splitpath(filesNamesLeft[i].c_str(), drive, dir, fname, ext);
		std::string leftUndistordedName = testImageUndistoredFolder + "/" + std::string(fname) + "_Un.png";
		_splitpath(filesNamesRight[i].c_str(), drive, dir, fname, ext);
		std::string rightUndistordedName = testImageUndistoredFolder + "/" + std::string(fname) + "_Un.png";

		cv::imwrite(leftUndistordedName, rectifyImageL);
		cv::imwrite(rightUndistordedName, rectifyImageR);
	}
}

//测试
void testRGBtofCalibration()
{
	//获取所有图像
	std::vector<std::string>  IR_FileNames;
	std::vector<std::string>  RGB_FileNames;
	std::vector<std::string>  PCD_FileNames;
	std::vector<std::string>  Depth_FileNames;
	for (int i = 1; i <= 26; i++)
	{
		std::stringstream ss;
		ss << "D://codes//Reconstruction//Calibrations//Data//RGBandTof//trainSet//" << i;
		std::vector<std::string>  filesInCurrFolder;
		//cout << "文件夹当前读取路径" << ss.str() << endl;
		getFiles(ss.str(), filesInCurrFolder);
		//cout << "文件夹下文件格数为：" << filesInCurrFolder.size() << endl;
		bool IR_ImageFlag = true;
		bool RGB_ImageFlag = true;
		bool PCD_ImageFlag = true;
		bool Depth_ImageFlag = true;

		std::string curr_IR_FileName;
		std::string curr_RGB_FileName;
		std::string curr_PCD_FileName;
		std::string curr_Depth_FileName;
		for (int i = 0; i < filesInCurrFolder.size(); i++)
		{

			if ((!IR_ImageFlag) && (!RGB_ImageFlag) && (!PCD_ImageFlag) && (!Depth_ImageFlag))
				break;
			if (IR_ImageFlag)
			{
				if (std::string::npos != filesInCurrFolder[i].find("_ir.png"))
				{
					curr_IR_FileName = filesInCurrFolder[i];
					IR_ImageFlag = false;
					continue;
				}

			}
			if (RGB_ImageFlag)
			{
				if (std::string::npos != filesInCurrFolder[i].find("_rgb.jpg"))
				{
					curr_RGB_FileName = filesInCurrFolder[i];
					RGB_ImageFlag = false;
					continue;
				}

			}
			if (RGB_ImageFlag)
			{
				if (std::string::npos != filesInCurrFolder[i].find("_rgb.jpg"))
				{
					curr_RGB_FileName = filesInCurrFolder[i];
					RGB_ImageFlag = false;
					continue;
				}

			}
			if (PCD_ImageFlag)
			{
				if (std::string::npos != filesInCurrFolder[i].find("_points.pcd"))
				{
					curr_PCD_FileName = filesInCurrFolder[i];
					PCD_ImageFlag = false;
					continue;
				}

			}
			if (Depth_ImageFlag)
			{
				if (std::string::npos != filesInCurrFolder[i].find("_depth.png"))
				{
					curr_Depth_FileName = filesInCurrFolder[i];
					Depth_ImageFlag = false;
					continue;
				}

			}


		}

		if ((!IR_ImageFlag) && (!RGB_ImageFlag) && (!PCD_ImageFlag) && (!Depth_ImageFlag))
		{
			IR_FileNames.push_back(curr_IR_FileName);
			RGB_FileNames.push_back(curr_RGB_FileName);
			PCD_FileNames.push_back(curr_PCD_FileName);
			Depth_FileNames.push_back(curr_Depth_FileName);
		}

	}

	std::cout << "IR_FileNames:" << IR_FileNames.size()
		<< " RGB_FileNames:" << RGB_FileNames.size()
		<< " Depth_FileNames:" << Depth_FileNames.size() << std::endl;

	//开始标定
	RGBtofCalibration calibrationOject;
	std::string ymlname = "D://codes//Reconstruction//Calibrations//Data//RGBandTof//calibration.yml";
	if (!calibrationOject.loadParameters(ymlname))
	{
		cv::Size patternSize(11, 8);
		cv::Size2f patternLength(30.0f, 30.0f);
		calibrationOject.setCheckBoardParam(patternSize, patternLength);
		cv::Size imageSizeRGB(642, 362);
		calibrationOject.setParametersRGB(RGB_FileNames, imageSizeRGB, 0.08);
		cv::Size imageSizetof(224, 172);
		calibrationOject.setParameterstof(IR_FileNames, imageSizetof, 0.08);
		calibrationOject.calibrateCams(true);
		calibrationOject.compute();
		calibrationOject.saveParameters(ymlname);
	}

	
	std::string testFolderName = "D://codes//Reconstruction//Calibrations//Data//RGBandTof//testSet//plane3";
	std::vector<std::string>  filesInCurrFolder;
	getFiles(testFolderName, filesInCurrFolder);
	bool IR_ImageFlag = true;
	bool RGB_ImageFlag = true;
	bool PCD_ImageFlag = true;
	bool Depth_ImageFlag = true;

	std::string curr_IR_FileName;
	std::string curr_RGB_FileName;
	std::string curr_PCD_FileName;
	std::string curr_Depth_FileName;
	for (int i = 0; i < filesInCurrFolder.size(); i++)
	{

		if ((!IR_ImageFlag) && (!RGB_ImageFlag) && (!PCD_ImageFlag) && (!Depth_ImageFlag))
			break;
		if (IR_ImageFlag)
		{
			if (std::string::npos != filesInCurrFolder[i].find("_ir.png"))
			{
				curr_IR_FileName = filesInCurrFolder[i];
				IR_ImageFlag = false;
				continue;
			}

		}
		if (RGB_ImageFlag)
		{
			if (std::string::npos != filesInCurrFolder[i].find("_rgb.jpg"))
			{
				curr_RGB_FileName = filesInCurrFolder[i];
				RGB_ImageFlag = false;
				continue;
			}

		}
		if (RGB_ImageFlag)
		{
			if (std::string::npos != filesInCurrFolder[i].find("_rgb.jpg"))
			{
				curr_RGB_FileName = filesInCurrFolder[i];
				RGB_ImageFlag = false;
				continue;
			}

		}
		if (PCD_ImageFlag)
		{
			if (std::string::npos != filesInCurrFolder[i].find("_points.pcd"))
			{
				curr_PCD_FileName = filesInCurrFolder[i];
				PCD_ImageFlag = false;
				continue;
			}

		}
		if (Depth_ImageFlag)
		{
			if (std::string::npos != filesInCurrFolder[i].find("_depth.png"))
			{
				curr_Depth_FileName = filesInCurrFolder[i];
				Depth_ImageFlag = false;
				continue;
			}

		}


	}

	if ((!IR_ImageFlag) && (!RGB_ImageFlag) && (!PCD_ImageFlag) && (!Depth_ImageFlag))
	{
		IR_FileNames.push_back(curr_IR_FileName);
		RGB_FileNames.push_back(curr_RGB_FileName);
		PCD_FileNames.push_back(curr_PCD_FileName);
		Depth_FileNames.push_back(curr_Depth_FileName);
	}
	cv::Mat rgb = cv::imread(curr_RGB_FileName);
	cv::Mat depth = cv::imread(curr_Depth_FileName, cv::IMREAD_UNCHANGED);
	std::string savename = testFolderName + "//RGBcloud.pcd";
	pcl::PointCloud<pcl::PointXYZRGB> ouputcloud;
	calibrationOject.getPointCloudWithColor(rgb, depth, ouputcloud, savename);


}


//将点云标定到地面
void rectifyPC2floor()
{
	std::string pcName = "D://codes//Reconstruction//Calibrations//Data//rectifyPC2floor//50320220421110157260.pcd";
	LinePointsTools tools;
	tools.setPointCloud(pcName);
	Eigen::Matrix4f T;
	tools.getBack2GroundT(-10, 10, 220, 300, 1, T);
	std::cout << "T:" << std::endl;
	std::cout << T << std::endl;

}

//测试通过点云计算激光线夹角
void getLineAngleByPointCloud()
{
	std::string pointcloudname = "201.pcd";
	pcl::PointCloud<pcl::PointXYZ> cloudall;
	readPCDfile(pointcloudname, cloudall);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudallPtr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloudall, *cloudallPtr);

	
	pcl::PassThrough<pcl::PointXYZ> passfilter;
	passfilter.setInputCloud(cloudallPtr);
	passfilter.setFilterFieldName("y");
	passfilter.setFilterLimits(-80, 0);
	passfilter.filter(*cloud_filtered);


	float angle1;
	getLineAngle(*cloud_filtered, 200, 10, FLT_MAX, -FLT_MAX, angle1);
	std::cout << "夹角为：" << angle1;

	passfilter.setFilterFieldName("y");
	passfilter.setFilterLimits(100, FLT_MAX);
	passfilter.filter(*cloud_filtered);
	getLineAngle(*cloud_filtered, 200, 10, FLT_MAX, -FLT_MAX, angle1);
	std::cout << "夹角为：" << angle1;

}

//测试平面点云平面度
void testPlaneFlatness()
{
	std::string pointcloudname = "1.txt";
	pcl::PointCloud<pcl::PointXYZ> cloud;
	readTXTfile(pointcloudname, cloud);
	float outlierPercent = 0.001;
	float mazDistance;
	float minDistance;
	getPlaneFlatness(cloud, outlierPercent, mazDistance, minDistance);

}