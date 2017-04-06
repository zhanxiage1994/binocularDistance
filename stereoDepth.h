#ifndef STEREODEPTH_H
#define STEREODEPTH_H

#include"Head.h"
#include<legacy\legacy.hpp>

class StereoDepth
{
public:
	StereoDepth();
	~StereoDepth();
	void depthSGM(Mat _imgL,Mat _imgR);
	void depthGC(Mat _imgL,Mat _imgR);

private:
	StereoSGBM sgm;
	int cn;

	void SGMinit();
};

void StereoDepth::depthGC(Mat _imgL,Mat _imgR)
{
	Mat tempImgL,tempImgR;
	if(_imgL.channels() != 1)
	{
		cvtColor(_imgL,tempImgL,CV_BGR2GRAY);
		cvtColor(_imgR,tempImgR,CV_BGR2GRAY);
	}

	CvMat imgL = tempImgL;
	CvMat imgR = tempImgR;

	CvStereoGCState* state = cvCreateStereoGCState( 16 * 17, 4 );  
	state->minDisparity = 16*12;//64;

	CvMat* dispL  = cvCreateMat( imgL.height,imgL.width, CV_32F );  
	CvMat* dispR = cvCreateMat( imgR.height,imgR.width,CV_32F );  
	cvFindStereoCorrespondenceGC( &imgL, &imgR, dispL, dispR, state, 0 );  
	cvReleaseStereoGCState( &state );  
	cvConvertScale( dispL, dispL, -1);
	cv::FileStorage temp("disparity\\GCdisp.xml",cv::FileStorage::WRITE);  
	temp<<"disp"<<dispL;
	temp.release();

	//imwrite("disparityGC.pgm",(Mat)dispL);
	//cvSave("disparityGC.pgm",dispL);
	cout<<"Save disparityGC!"<<endl;
	
	CvMat* disp8 = cvCreateMat(imgR.height,imgR.width,CV_8U);
	//double minVal,maxVal;
	//minMaxLoc((Mat)dispL,&minVal,&maxVal);
	//cvConvertScale( dispL, disp8, 255/(maxVal - minVal));
	cvNormalize(dispL,disp8,0,255,CV_MINMAX);

    namedWindow("GC_disparity",0);
    cvShowImage("GC_disparity",disp8);
	while (waitKey() != 27);
	destroyWindow("GC_disparity");

    cvReleaseMat(&dispL);
    cvReleaseMat(&dispR);
    cvReleaseMat(&disp8);
}

void StereoDepth::SGMinit()
{
	
	sgm.SADWindowSize = 9;
	//视差搜索范围（16的整数倍）
	sgm.numberOfDisparities = numberOfDisparities;
	//预处理滤波器的截断值
	sgm.preFilterCap = 63;
	sgm.minDisparity = minDisparity;
	//视差唯一性百分比
	sgm.uniquenessRatio = 10;
	//检查视差连通区域变化度的窗口大小
	sgm.speckleWindowSize = 100;
	//当窗口内视差变化大于阈值则清零窗口内的视差
	sgm.speckleRange = 32;
	//左视差图和右视差图之间的最大容许差异
	sgm.disp12MaxDiff = 1;
	sgm.fullDP = false;
	//DP 控制视差变化平滑性的参数
	sgm.P1 = 8 * cn*sgm.SADWindowSize*sgm.SADWindowSize;
	sgm.P2 = 32 * cn*sgm.SADWindowSize*sgm.SADWindowSize;
	
}

void StereoDepth::depthSGM(Mat imgL,Mat imgR)
{
	cn = imgL.channels();
	SGMinit();

	Mat gL,gR;
	Mat disp,disp8;
	//cvtColor(imgL,gL,CV_BGR2GRAY);
	//cvtColor(imgR,gR,CV_BGR2GRAY);
	sgm(imgL,imgR,disp);
	///////Output disparity map. It is a (指定类型不可改)16-bit signed single-channel image of the same size as the input image. 
	//////It contains disparity values scaled by 16. 
	//////So, to get the floating-point disparity map, you need to divide each disp element by 16.
	//输出视差被扩大16倍，需要视差图除16
	disp.convertTo(disp,CV_16S,1/16.0);

	cv::FileStorage temp("disparity\\SGMdisp.xml",cv::FileStorage::WRITE);  
	temp<<"disp"<<disp;
	temp.release();

	//imwrite("disparitySGBM.pgm",disp);
	cout<<"Save disparitySGBM !"<<endl;
	
	//double minVal, maxVal;
	//minMaxLoc(disp,&minVal,&maxVal);
	//disp.convertTo(disp8,CV_8U,255/(maxVal - minVal));


	//normalize(disp,disp8,0,255,CV_MINMAX,CV_8U);
	//namedWindow("SGBM_disparity",0);
	//imshow("SGBM_disparity",disp8);
	//while (waitKey() != 27);
	//destroyWindow("SGBM_disparity");
}


StereoDepth::StereoDepth()
{
}

StereoDepth::~StereoDepth()
{
}

#endif