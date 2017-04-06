#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H
/*
height	rows Y j
width	cols X i
*/
#include "Head.h"

class CameraCalibrator
{
public:
	bool checkChessBoard(Mat temp);

	CameraCalibrator() {};
	~CameraCalibrator() {};
	void InitCameraCalibrator(int i);
	cv::Mat getIntrinsicMatrix() const {return intrinsicMatrix;}
	cv::Mat getDistortionCoeffs() const {return distortionCoeffs;}
	std::vector<std::vector<cv::Point2f>> getImagePoint() const {return imagePoints;};
	int addChessboardPoints(const std::vector<std::string> &fileList,cv::Size &boardSize);	//检测角点
	double calibrate();	//相机标定
	void CalcUndistort();//计算畸变参数矩阵
	cv::Mat undistorted(const cv::Mat &image);	//去畸变
	void SaveIntrinsicDistortion();
	void ReadIntrinsicDistortion();

private:
	std::vector<std::vector<cv::Point3f>> objectPoints;	//世界坐标
	std::vector<std::vector<cv::Point2f>> imagePoints;	//图像坐标
	cv::Mat intrinsicMatrix;	//内参数矩阵
	cv::Mat distortionCoeffs;	//畸变系数
	cv::Mat mapX,mapY;			//去畸变参数矩阵
	void addPoint(const std::vector<cv::Point2f> &imageCorners, const std::vector<cv::Point3f> &objectCorners)
	{
		imagePoints.push_back(imageCorners);
		objectPoints.push_back(objectCorners);
	}
	int device;
};
void CameraCalibrator::InitCameraCalibrator(int i)
{
	device = i;
}

bool CameraCalibrator::checkChessBoard(Mat temp)
{
	bool flag = false;
	std::vector<cv::Point2f> imageCorners;
	flag = cv::findChessboardCorners(temp,boardSize,imageCorners);	//8bit灰度or彩色图
	if(flag && imageCorners.size() == boardSize.area())
	{
		flag = true;
	}
	return flag;
}

int CameraCalibrator::addChessboardPoints(const std::vector<std::string> &filelist,cv::Size &boardSize)
{
	std::vector<cv::Point2f> imageCorners;
	std::vector<cv::Point3f> objectCorners;
	for(int i = 0;i < boardSize.height;i++)
	{
		for(int j = 0;j < boardSize.width;j++)
		{
			objectCorners.push_back(cv::Point3f(i,j,0.0f));
		}
	}
	cv::Mat temp;
	int success = 0;
	for(int i = 0;i < filelist.size();i++)
	{
		temp = cv::imread(filelist[i],0);
		if(temp.empty())
		{
			return -1;
		}
		bool found = cv::findChessboardCorners(temp,boardSize,imageCorners);	//8bit灰度or彩色图
		//搜索窗口(5*2+1)*(5*2+1),zeroZone搜索死区(-1,-1)表示无,角点精准终止条件迭代30次or eps<0.01
		cv::cornerSubPix(temp,imageCorners,cv::Size(5,5),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30,0.1));	
		if(found && imageCorners.size() == boardSize.area())
		{
			addPoint(imageCorners,objectCorners);
			success++;
			std::cout<<success<<std::endl;
			cv::drawChessboardCorners(temp,boardSize,imageCorners,found);
			cv::imshow("Corners on Chessboard",temp);
			cv::waitKey(100);
		}
		else
		{
			std::cout<<"Can't find "<<i <<"-th chessImage corner."<<std::endl;
		}
	}
	cv::destroyWindow("Corners on Chessboard");
	return success;
}

double CameraCalibrator::calibrate()
{
	std::vector<cv::Mat> Rvecs,Tvecs;
	return cv::calibrateCamera(objectPoints,imagePoints,RESOLUTION,intrinsicMatrix,distortionCoeffs,Rvecs,Tvecs);
}

void CameraCalibrator::CalcUndistort()
{
	cv::initUndistortRectifyMap(intrinsicMatrix,distortionCoeffs,cv::Mat(),cv::Mat(),RESOLUTION,CV_32FC1,mapX,mapY);
}

cv::Mat CameraCalibrator::undistorted(const cv::Mat &image)
{
	cv::Mat undistorted;
	//应用映射矩阵（双线性插值）
	cv::remap(image,undistorted,mapX,mapY,cv::INTER_LINEAR);
	return undistorted;
}

void CameraCalibrator::SaveIntrinsicDistortion()
{
	std::stringstream filename;
	filename<<"cap"<<device<<"\\"<<"Matrix.xml";
	cv::FileStorage temp(filename.str(),cv::FileStorage::WRITE);  
	temp << "Intrinsic" << intrinsicMatrix;
	temp << "Distortion" << distortionCoeffs;
	temp.release();
}

void CameraCalibrator::ReadIntrinsicDistortion()
{
	std::stringstream filename;
	filename<<"cap"<<device<<"\\"<<"Matrix.xml";
	cv::FileStorage temp(filename.str(),cv::FileStorage::READ);  
	if(temp.isOpened())
	{
		temp["Intrinsic"] >> intrinsicMatrix;
		temp["Distortion"] >> distortionCoeffs;
	}
	else
	{
		std::cout<<"Can't read "<<device<<"-th intrinsic and distortion matrix."<<std::endl;
	}
	temp.release();
}

#endif