#include"CameraCapture.h"
#include"CameraCalibrator.h"
#include "rectify.h"
#include "stereoDepth.h"
#include<time.h>
#include"stereoCalib.h"

int flag = true;
Mat distanceSGM;//distanceGC,distancePRJopencv;
void onMouse(int event, int x, int y, int flags, void* param)  
{  
	
    Mat *im = reinterpret_cast<Mat*>(param);  
    switch (event)  
    {  
        case CV_EVENT_LBUTTONDOWN:     //鼠标左键按下响应：返回坐标和灰度  
            std::cout<<"at("<<x<<","<<y<<")value is: SGM: "  
                <<static_cast<int>(distanceSGM.at<int>(cv::Point(x,y)))
				//<<"   GC: "<<static_cast<int>(distanceGC.at<int>(cv::Point(x,y)))
				//<<"   PRJopencv"<<static_cast<int>(distancePRJopencv.at<int>(cv::Point(x,y)))
				<<std::endl;  
            break;  
        case CV_EVENT_RBUTTONDOWN:    //鼠标右键按下响应：输入坐标并返回该坐标的灰度  
            //std::cout<<"input(x,y)"<<endl;  
            //std::cout<<"x ="<<endl;  
            //cin>>x;  
            //std::cout<<"y ="<<endl;  
            //cin>>y;  
            //std::cout<<"at("<<x<<","<<y<<")value is:"  
            //    <<static_cast<int>(im->at<uchar>(cv::Point(x,y)))<<std::endl;  
			flag = false;
            break;            
    }  
} 

int main()
{
	CameraCapture camCapture[CAMNUM];
	CameraCalibrator camCalibrate;
	Mat leftCam,rightCam;
//#define capture
#ifdef capture
	camCapture[0].InitCameraCapture(0);
	camCapture[1].InitCameraCapture(1);
#define CAMERACALIBRATION
#ifdef CAMERACALIBRATION

	namedWindow("capture",1);
	_mkdir("capture");
	_mkdir("capture\\left");
	_mkdir("capture\\right");
	int numChessboard = 20;
	vector<string> imagelist;
	stringstream road;
	while (numChessboard)
	{
		camCapture[0].capture1 >> leftCam;
		camCapture[1].capture1 >> rightCam;
		if(leftCam.empty() || leftCam.empty())
		{
			continue;
		}
		imshow("capture",leftCam);
		if(waitKey(30) == 's')
		{
			cout<<"Putchar 's'!"<<endl;
			camCapture[0].capture1 >> leftCam;
			camCapture[1].capture1 >> rightCam;
			if(leftCam.empty() || leftCam.empty())
			{
				continue;
			}
			if(camCalibrate.checkChessBoard(leftCam))
			{
				if(camCalibrate.checkChessBoard(rightCam))
				{
					cout<<"Find pair  "<<numChessboard<<endl;
					road.clear();road.str("");
					road<<"capture\\left\\"<<numChessboard<<".jpg";
					imwrite(road.str(),leftCam);
					imagelist.push_back(road.str());
					road.clear();road.str("");
					road<<"capture\\right\\"<<numChessboard<<".jpg";
					imwrite(road.str(),rightCam);
					imagelist.push_back(road.str());
					numChessboard--;
				}
			}
		}
	}
	StereoCalibration stereoCalib;
	stereoCalib.run(&imagelist);
#endif
	while (true)
	{
		camCapture[0].capture1 >> leftCam;
		camCapture[1].capture1 >> rightCam;
		if(leftCam.empty() || rightCam.empty())
		{
			continue;
		}
		imshow("capture",leftCam);
		if(waitKey(30) == 'c')
		{
			cout<<"Putchar 'c'!"<<endl;
			camCapture[0].capture1 >> leftCam;
			camCapture[1].capture1 >> rightCam;
			if(leftCam.empty() || rightCam.empty())
			{
				continue;
			}
			cv::imwrite("left.jpg",leftCam);
			cv::imwrite("right.jpg",rightCam);
			destroyAllWindows();
			camCapture[0].freeCamera();
			camCapture[1].freeCamera();
			break;
		}
	}
#else
	leftCam = imread("left.jpg");
	rightCam = imread("right.jpg");
#endif
	Mat alineImgL,alineImgR;
	ImageRectify imgRectify;
	imgRectify.run(&leftCam,&rightCam,&alineImgL,&alineImgR);

	cout<<"Start disparity!"<<endl;
	StereoDepth calcDepth;
	calcDepth.depthSGM(alineImgL,alineImgR);
	//cout<<" SGBM Code Time: "<<(double)(clock()-start)/CLOCKS_PER_SEC<<"s！"<<endl;
	//calcDepth.depthGC(alineImgL,alineImgR);
	//cout<<" GC Code Time: "<<(double)(clock()-start)/CLOCKS_PER_SEC<<"s！"<<endl;

	Mat dispSGM;//dispGC,dispPRJopencv;
	cv::FileStorage temp("disparity\\SGMdisp.xml",cv::FileStorage::READ);  
	temp["disp"] >> dispSGM;
	temp.release();
	//temp.open("disparity\\GCdisp.xml",cv::FileStorage::READ);  
	//temp["disp"] >> dispGC;
	//temp.release();
	//cv::FileStorage temp3("disparity\\prjSGBMdisp.xml",cv::FileStorage::READ);  
	//temp3["disp"] >> dispPRJopencv;
	//temp3.release();

	Mat showSGMDisp;
	normalize(dispSGM,showSGMDisp,0,255,CV_MINMAX);
	showSGMDisp.convertTo(showSGMDisp,CV_8U);
	imshow("hahaprj",showSGMDisp);
	waitKey();
	cv::imwrite("disparity\\showSGMDisp.jpg",showSGMDisp);
	//normalize(dispGC,dispGC,0,255,CV_MINMAX);
	//dispGC.convertTo(dispGC,CV_8U);
	//imshow("heheprj",dispGC);
	//waitKey();

	Mat reprojectQ;
	temp.open("rectifyData.xml",cv::FileStorage::READ);  
	temp["Q"] >> reprojectQ;
	temp.release();
	Mat tempSGM;//tempGC,tempPRJopencv;
	reprojectImageTo3D(dispSGM,tempSGM,reprojectQ,true,CV_32SC3);
	vector<Mat> v;
	split(tempSGM,v);
	distanceSGM = v[2];
	//reprojectImageTo3D(dispGC,tempGC,reprojectQ,true,CV_32SC3);
	//vector<Mat> v1;
	//split(tempGC,v1);
	//distanceGC = v1[2];
	//reprojectImageTo3D(dispPRJopencv,tempPRJopencv,reprojectQ,true,CV_32SC3);
	//vector<Mat> v2;
	//split(tempPRJopencv,v2);
	//distancePRJopencv = v2[2];

	namedWindow("Origin",0);
	//while(flag)
	{
		cv::setMouseCallback("Origin",onMouse,reinterpret_cast<void*> (&alineImgL));
	}
	imshow("Origin",alineImgL);
	while(waitKey(33) != 27);
	destroyAllWindows();

	system("pause");
	return 0;
}