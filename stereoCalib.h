#ifndef STEREOCALIB_H
#define STEREOCALIB_H
#include"Head.h"

class StereoCalibration
{
public:
	StereoCalibration() {};
	~StereoCalibration() {};
	//void loadfile(string str,int basisL,int basisR,int num);
	bool run(vector<string>* imagelist);


private:
	Mat cameraMatrix[2], distCoeffs[2];
	Mat R, T, E, F;

	int nimages;
	

};

bool StereoCalibration::run(vector<string>* imagelist)
{
	
	if( imagelist->size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
		return false;
    }
	nimages = imagelist->size() / 2;
    bool displayCorners = false;//true;
    const float squareSize = 285;  // Set this to your actual square size此处单位是mm对应深度输出是cm
    // ARRAY AND VECTOR STORAGE:

	Size imageSize;
    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<string> goodImageList;

	int i,j,k;
    for(i = j = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            string& filename = (*imagelist)[i*2+k];
            Mat img = imread(filename, 0);
            if(img.empty())
                break;
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }

            bool found = false;
            vector<Point2f>& corners = imagePoints[k][j];

			int maxScale = 2;
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
                found = findChessboardCorners(timg, boardSize, corners,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }

            if( displayCorners )
            {
                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf);
                imshow("corners", cimg1);
                char c = (char)waitKey(500);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if( !found )
                break;
            cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                         TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
        }
        if( k == 2 )
        {
            //goodImageList.push_back(imagelist[i*2]);
            //goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    }
    cout << endl << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        cout << "Error: too little pairs to run the calibration\n";
        return false;
    }

	imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);
	for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
			{
				objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
			}
    }

	cout << "Running stereo calibration ...\n";


    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);


    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_ASPECT_RATIO +
                    //CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH 
                    //CV_CALIB_RATIONAL_MODEL +
                    //CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5
					);
    cout << "done with RMS error=" << rms << endl;

    FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    fs.open("extrinsics.xml", cv::FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T ;
        fs.release();
    }
    else
        cout << "Error: can not save the extrinsics parameters\n";

	return true;
}

//void StereoCalibration::loadfile(string str,int basisL,int basisR,int num)
//{
//	nimages = num;
//	stringstream strTemp;
//	for(int i = 0;i < nimages;i++)
//	{
//		strTemp.clear();strTemp.str("");
//		strTemp<<str<<"\\left\\IMG_00"<< basisL + i << ".jpg";
//		imagelist.push_back(strTemp.str());
//
//		strTemp.clear();strTemp.str("");
//		strTemp<<str<<"\\right\\IMG_"<< basisR + i << ".jpg";
//		imagelist.push_back(strTemp.str());
//	}
//}


#endif 