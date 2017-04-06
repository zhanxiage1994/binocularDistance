#ifndef CAMERACAPTURE_H
#define CAMERACAPUTRE_H
#include "Head.h"
#include<direct.h>

class CameraCapture
{
public:
	CameraCapture() {};
	void CameraCapture::InitCameraCapture(int i);
	~CameraCapture(){capture1.release();};
	void startCapture(std::vector<std::string> *filelist);
	void setCameraEXPOSURE(int exposure);
	void freeCamera() {capture1.release();};

	cv::VideoCapture capture1;

private:
	bool stop;
	int device;
};


void CameraCapture::InitCameraCapture(int i)
{
	device = i;
	stop = false;
	capture1.open(device);
	if(!capture1.isOpened())
	{
		std::cout<<"Failed open "<<device<<"-th camera"<<std::endl;
	}
	capture1.set(CV_CAP_PROP_FRAME_WIDTH,RESOLUTION.width);
	capture1.set(CV_CAP_PROP_FRAME_HEIGHT,RESOLUTION.height);
	capture1.set(CV_CAP_PROP_BRIGHTNESS,camBRIGHTNESS);	
	capture1.set(CV_CAP_PROP_EXPOSURE,camEXPOSURE1);		
}


void CameraCapture::startCapture(std::vector<std::string> *filelist)
{
	_mkdir("cap");
	cv::Mat temp;
	cv::namedWindow("cap",0);
	int capNum = 1;
	while (!stop)
	{
		capture1 >> temp;
		cv::imshow("cap",temp);
		if(cv::waitKey(30) == int('s'))
		{
			std::stringstream filename;
			filename<<"cap\\"<<device;
			_mkdir(filename.str().c_str());
			filename<<"\\"<<capNum<<".jpg";
			filelist->push_back(filename.str());
			cv::imwrite(filename.str(),temp);
			std::cout<<capNum++<<std::endl;
		}
		else if(cv::waitKey(10) == 27)
		{
			std::cout<<"capture images: "<<--capNum<<std::endl;
			stop = true;
		}
	}
	cv::destroyWindow("cap");
}
void CameraCapture::setCameraEXPOSURE(const int exposure)
{
	capture1.set(CV_CAP_PROP_EXPOSURE,exposure);
}
#endif