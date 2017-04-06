#ifndef RECTIFY_H
#define RECTIFY_H

#include"Head.h"

class ImageRectify
{
public:
	ImageRectify() {};
	~ImageRectify() {};
	void run(Mat* _imgL,Mat* _imgR,Mat* recImgL,Mat* recImgR);
	Mat getQ() {return Q;};

private:
	//bool loadXML(string str,Mat* dst);
	void initParameter();
	Size testResolution();
	//void leftRTright();
	void saveParaXml();
	void mapping();
	void displayResult();

	Mat* imgL;Mat* imgR;
	Mat REimgL,REimgR;
	Size imageSize;
	Mat R,T;
	//Mat rotL,rotR;
	Mat transL,transR;
	Mat intrinsicL,intrinsicR;
	Mat distortionL,distortionR;
	Mat R1, R2, P1, P2, Q;	//旋转矩阵和投影矩阵
	Rect validRoi[2];
};

void ImageRectify::run(Mat* _imgL,Mat* _imgR,Mat* recImgL,Mat* recImgR)
{
	imgL = _imgL;
	imgR = _imgR;
	initParameter();
	imageSize = testResolution();
	//leftRTright();
	stereoRectify(intrinsicL,distortionL,intrinsicR,distortionR,imageSize,R,T,R1,R2,P1,P2,Q,CV_CALIB_ZERO_DISPARITY  ,-1,imageSize,&validRoi[0], &validRoi[1]);
	saveParaXml();
	mapping();
	displayResult();
	REimgL.copyTo(* recImgL);
	REimgR.copyTo(* recImgR);
}

//bool ImageRectify::loadXML(string str,Mat* dst)
//{
//	string strTemp = str + ".xml";
//	cv::FileStorage temp(strTemp,cv::FileStorage::READ);  
//	if(temp.isOpened())
//	{
//		temp[str] >> *dst;
//	}
//	else
//	{
//		cout<<"Can't read xml file!"<<endl;
//		return false;
//	}
//	temp.release();
//	return true;
//}

void ImageRectify::initParameter()
{
	cv::FileStorage temp1("intrinsics.xml",cv::FileStorage::READ);  
	if(temp1.isOpened())
	{
		temp1["M1"] >> intrinsicL;
		temp1["D1"] >> distortionL;
		temp1["M2"] >> intrinsicR;
		temp1["D2"] >> distortionR;
	}
	else
	{
		std::cout<<"Can't open intrinsics.xml."<<std::endl;
		while (true);
	}
	temp1.release();
	cv::FileStorage temp2("extrinsics.xml",cv::FileStorage::READ);  
	if(temp2.isOpened())
	{
		temp2["R"] >> R;
		temp2["T"] >> T;
	}
	else
	{
		std::cout<<"Can't open extrinsics.xml."<<std::endl;
		while (true);
	}
	temp2.release();
}

Size ImageRectify::testResolution()
{
	if(imgL->size() != imgR->size())
	{
		std::cout<<"Images size don't equal";
		while (1);
	}
	return imgL->size();
}

//void ImageRectify::leftRTright()
//{
//	//R = Rr*inv(Rl); T = Tr - R*Tl;
//	invert(rotL,rotL);
//	R = rotR.clone();
//	gemm(R,rotL,1,1,0,R);
//
//	gemm(R,transL,1,1,0,T);
//	T = transR - T;
//}
	
void ImageRectify::saveParaXml()
{
	cv::FileStorage temp("rectifyData.xml",cv::FileStorage::WRITE);  
	temp<<"R"<<R;
	temp<<"T"<<T;
	temp<<"R1"<<R1;
	temp<<"R2"<<R2;
	temp<<"P1"<<P1;
	temp<<"P2"<<P2;
	temp<<"Q"<<Q;
	temp.release();
}

void ImageRectify::mapping()
{
	//计算映射
	Mat rmap[2][2];
    initUndistortRectifyMap(intrinsicL,distortionL, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(intrinsicR,distortionR, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
	remap(*imgL, REimgL, rmap[0][0], rmap[0][1], CV_INTER_AREA);//左校正
    remap(*imgR, REimgR, rmap[1][0], rmap[1][1], CV_INTER_AREA);//右校正
	imwrite("REimgL.bmp",REimgL);
	imwrite("REimgR.bmp",REimgR);
}

void ImageRectify::displayResult()
{
	Mat showImage(2*imageSize.height,2*imageSize.width,CV_8UC3);
	Rect originL(0,0,imageSize.width,imageSize.height);
	Rect originR(imageSize.width,0,imageSize.width,imageSize.height);
	Rect mapL(0,imageSize.height,imageSize.width,imageSize.height);
	Rect mapR(imageSize.width,imageSize.height,imageSize.width,imageSize.height);
	imgL->copyTo(showImage(originL));
	imgR->copyTo(showImage(originR));
	REimgL.copyTo(showImage(mapL));
	REimgR.copyTo(showImage(mapR));
	namedWindow("stereoRectify",0);
    for( int i = 0; i < 2 * imageSize.height; i += 64 )
	{
		line(showImage, Point(0, i), Point(2*imageSize.width, i), Scalar(0, 255, 0), 1, 8);
	}
	imshow("stereoRectify",showImage);
	while (waitKey() != 27);
	destroyWindow("stereoRectify");
	imwrite("stereoRectify.bmp",showImage);
	cout<<"Save stereoRectify.bmp!"<<endl;
}



#endif