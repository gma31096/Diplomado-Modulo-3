#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <time.h>


#include "opencv2/highgui.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>



#include <iostream>
using namespace cv;
using namespace std;

cv::Mat cameraMatrix, distCoeffs;
cv::Size imageSize;
cv::Mat frame;
cv::Mat map1, map2, dest;
cv::Mat camaraGris;

int scale = 1;
int delta = 0;
int ddepth = CV_16S;

void readCameraParams(cv::Mat &cameraMatrix,cv::Mat &distCoeffs){
	cv::FileStorage fs("Datos.txt", cv::FileStorage::READ );
	fs["camera_matrix"] >> cameraMatrix ;
	fs["distortion_coefficients"] >> distCoeffs ;
	fs.release () ;
}


int main( int argc, char** argv ){
	//despliege de la camara
	cv::VideoCapture miCamaraWeb;
	miCamaraWeb.open(1);	//inicia la camara 0, usualmente la webcam
	cv::Mat fotograma;
	int tecla;
	bool flag = true;

	while (tecla != 27) {
		miCamaraWeb.read(fotograma);
		tecla = cv::waitKey(1);
		readCameraParams(cameraMatrix, distCoeffs);
		miCamaraWeb.read(frame);
		imageSize = frame.size();
		cv::initUndistortRectifyMap(cameraMatrix,distCoeffs,cv::Mat(),
		getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,imageSize,1,imageSize,0),
					imageSize,CV_16SC2, map1, map2);
		cv::remap(frame,dest,map1,map2,cv::INTER_LINEAR);
		//imagen corregida
		namedWindow("Rect image", CV_WINDOW_AUTOSIZE );
		cv::imshow("Rect image", dest);
		/*----------------SObel---------------------*/
		cvtColor( dest, camaraGris, CV_BGR2GRAY );
		namedWindow("Gris", CV_WINDOW_AUTOSIZE );
		double timeSobel = 0.0;
		double timeLaplace = 0.0;
		double timeCanny = 0.0;
		clock_t inicio = clock();
		GaussianBlur( camaraGris, camaraGris, Size(3,3), 0, 0, BORDER_DEFAULT );
		/// Generate grad_x and grad_y
		Mat grad_x, grad_y;
		Mat abs_grad_x, abs_grad_y;
		/// Gradient X
		//Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
		Sobel(camaraGris, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
		convertScaleAbs( grad_x, abs_grad_x );
		/// Gradient Y
		//Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
		Sobel(camaraGris, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
		convertScaleAbs( grad_y, abs_grad_y );
		/// Total Gradient (approximate)
		cv::Mat camaraSobel;
		addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0,camaraSobel );
		clock_t fin = clock();
		timeSobel = (double)(fin - inicio) / CLOCKS_PER_SEC;
		printf("Sobel %f [ms]", 1000*timeSobel);
		imshow("Sobel",camaraSobel);
		/*---------------------Laplacian------------------------------*/
		/// Create window
		namedWindow("Laplace", CV_WINDOW_AUTOSIZE );
		/// Apply Laplace function
		cv::Mat camaraLaplaciano;
		int kernel_size = 3;
		inicio = clock();
		Laplacian( camaraGris, camaraLaplaciano, ddepth, kernel_size, scale, delta, BORDER_DEFAULT );
		convertScaleAbs(camaraLaplaciano,camaraLaplaciano );
		fin = clock();
		timeLaplace = (double)(fin - inicio) / CLOCKS_PER_SEC;
		printf(" Laplace %f [ms]", 1000*timeLaplace);
		/// Show what you got
		imshow("Laplace",camaraLaplaciano);
		/*--------------------------Canny-----------------------------*/
		cv::Mat camaraCanny;
		/// Reduce noise with a kernel 3x3
		inicio = clock();
		blur( camaraGris, camaraCanny, Size(3,3) );
		/// Canny detector
		int radio = 3;
		int low = 30;
		Canny(camaraCanny,camaraCanny, low, low*radio, kernel_size );
		fin = clock();
		timeCanny = (double)(fin - inicio) / CLOCKS_PER_SEC;
		printf(" Canny %f [ms]\n", 1000*timeCanny);
		// Using Canny's output as a mask, we display our result
		//outCanny = Scalar::all(0);
		//camaraGris.copyTo( outCanny,camaraCanny);
		imshow("Canny",camaraCanny);
		/*-----------------------Harris----------------------------------*/
	    int blockSize = 2;
	    int apertureSize = 3;
	    double k = 0.04;
	    cv::Mat Harris;
	    cv::Mat origen = camaraGris;
	    cornerHarris( origen, Harris, blockSize, apertureSize, k );
	    Mat Harris_scaled;
	    normalize( Harris,Harris, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
	    convertScaleAbs( Harris, Harris_scaled);
	    cv::Mat copyHarris;
	    copyHarris = dest.clone();
	    int thresh = 200;
	    int cornersCount = 0;
	    for( int i = 0; i < Harris.rows ; i++ )
	    {
	        for( int j = 0; j < Harris.cols; j++ )
	        {
	            if( (int) Harris.at<float>(i,j)>  thresh && cornersCount<500 && i>35 && i<350)
	            {
	            	cornersCount++;
	                circle(copyHarris, Point(j,i), 5,  Scalar(0), 2, 8, 0 );
	            }
	        }
	    }
	    namedWindow("Harrys");
	    imshow("Harrys", copyHarris);
	    /*-----------------------------------Shi-Thomas---------------------------*/
	    /// Parameters for Shi-Tomasi algorithm
	      vector<Point2f> corners;
	      double qualityLevel = 0.01;
	      double minDistance = 10;
	      //int blockSize = 3;
	      bool useHarrisDetector = true;
	      //double k = 0.04;

	      /// Copy the source image
	      //RNG rng(12345);
	      cv::Mat copyShi;
	      copyShi = dest.clone();
	      int maxCorners = 100;
	      /// Apply corner detection
	      goodFeaturesToTrack( camaraGris,
	                   corners,
	                   maxCorners,
	                   qualityLevel,
	                   minDistance,
	                   Mat(),
	                   blockSize,
	                   useHarrisDetector,
	                   k );


	      int r = 4;
	      for( int i = 0; i < corners.size(); i++ )
	         { circle( copyShi, corners[i], r, Scalar(255, 0,0), -1, 8, 0 ); }
	      imshow("Shi-Thomas", copyShi );
	      //https://docs.opencv.org/2.4/doc/tutorials/features2d/trackingmotion/generic_corner_detector/generic_corner_detector.html#generic-corner-detector

	      /*--------------------------------FAST-----------------------------------*/
	      cv::Mat copyFast;
	      copyFast = dest.clone();
	      vector<KeyPoint> keypointsD;
	      Ptr<FastFeatureDetector> detector=FastFeatureDetector::create();
	      vector<Mat> descriptor;

	      detector->detect(camaraGris,keypointsD,Mat());
	      drawKeypoints(copyFast, keypointsD, copyFast);
	      imshow("FAST",copyFast);
	}
	return 0;
}



