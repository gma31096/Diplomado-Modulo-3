#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <vector>


#include "opencv2/highgui.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>



#include <iostream>
using namespace cv;
using namespace std;

/// Global variables

int ejercicio; //por que me da pereza hacer el struct
cv::Mat cameraMatrix, distCoeffs;
cv::Size imageSize;
cv::Mat frame;
cv::Mat map1, map2, dest;
Scalar minimo = Scalar(0,0,0);
Scalar maximo = Scalar(180,255,255);
bool listo = false;

int valorMovil1 = 200;	//umbral
int valorMovil2 = 4;  //tipo de filtro


char* mensaje1 = "Tipo: \n 0:Binary \n 1:Binary Inverted \n 2:Truncate \n 3:To Zero \n 4:To Zero Inverted";
char* mensaje2 = "Valor";

char* mensajeAdaptativo = "0-> ADAPTIVE_THRESH_MEAN_C \n 1-> ADAPTIVE_THRESH_GAUSSIAN_C";
int valorMetodoAdaptativo = 0;

char* mensajeThresholdTipo = "0-> THRESH_BINARY   \m 1-> THRESH_BINARY_INV";
int valorThresholdTipo = 0;

char* mensajeTamanoBloque = "Radio de bloque";
int valorTamanoBloque = 1;

char* mensajeC = "Constante C";
int valorC = 5;

char* mensajeColor = "0->Naranja\n1->amatillo\n2->rosa\n3->azul\n4->verde\n5->blanco";
int valorColor = 3;
cv::Mat imagenHSV,imagenRango;
cv::Mat imagen;


cv::Mat imagenUniversoGris,imagenFiltrada,imagenFiltroAdaptativo;

void funcionEvento( int, void* )
{
	threshold(imagenUniversoGris, imagenFiltrada,  valorMovil1, 255,valorMovil2);
	imshow("Filtro",imagenFiltrada);
}
void filtroAdaptativo(int,void*){
	adaptiveThreshold(imagenUniversoGris, imagenFiltroAdaptativo, 255,
						valorMetodoAdaptativo,valorThresholdTipo,valorTamanoBloque*2+1+2, valorC);
	imshow("Filtro adaptativo",imagenFiltroAdaptativo);
}
void centroide(cv::Mat* imagen,int* Centroide){
	unsigned long long int sumax = 0;
	unsigned long long int sumay = 0;
	unsigned long long int conteo = 0;
	for(int x=0 ; x<(*imagen).rows ;x++){
		for(int y=0 ; y<(*imagen).cols ; y++){
			if((*imagen).at<cv::Vec3b>(x,y)[0] != 0 ){
				//printf("%d\n",(*imagen).at<cv::Vec3b>(10,10)[0]);
				//getchar();
				conteo++;
				sumax += x;
				sumay += y;
			}
		}
	}
	if(conteo > 200){
		sumax /= conteo;
		sumay /= conteo;
		Centroide[0] = sumax;
		Centroide[1] = sumay;

	}else{
		Centroide[0] = -100;
		Centroide[1] = -100;
	}


	//printf("%llu x:%llu y:%llu \n",conteo,Centroide[0],Centroide[1]);
}
bool getHSV(cv::Mat* imagen,int x,int y,int* HSV){
	 cv::Vec3b hsvChanel = (*imagen).at<cv::Vec3b>(y,x);
	 HSV[0] = hsvChanel.val[0];
	 HSV[1] = hsvChanel.val[1];
	 HSV[2] = hsvChanel.val[2];
	 return true;
}
void funcionColor( int, void* )
{
	Scalar Min = Scalar(0,0,0);
	Scalar Max = Scalar(180,255,255);
	switch(valorColor){
		case 0:
			//naranja
			 Min = Scalar(104,190,175);
			 Max = Scalar(110,255, 255); //110 ->111
			break;
		case 1:
			//amarillo
			 Min = Scalar(70,175,170);
			 Max = Scalar(103,255,255);
			break;
		case 2:
			//rosa
			 Min = Scalar(115,100,209);
			 Max = Scalar(120,255,255);
			break;
		case 3:
			//azul
			 Min = Scalar(10,0,0);
			 Max = Scalar(50,255,255);
			break;
		case 4:
			//verde
			 Min = Scalar(55,80,100);
			 Max = Scalar(95,200,205);
			break;
		case 5:
			 Min = Scalar(100,30,170);
			 Max = Scalar(110,60,198);
			break;
		default:
			 Min = Scalar(0,0,0);
			 Max = Scalar(180,255,255);
	}

	inRange(imagenHSV, Min,Max, imagenRango);
	//cv::imshow("Rango", imagenRango);
	cv::Mat imagenAND;
	cv::bitwise_and(imagen,imagen,imagenAND,imagenRango);
	cv::imshow("Filtro color", imagenAND);

}

void onMouse(int event, int x, int y, int what, void* param){
	 if( event != CV_EVENT_LBUTTONDOWN )
	        return;
	 if(ejercicio == 22){
		cv::Mat rangoModificado;
		int tam = 1;
		cv::Mat element = cv::getStructuringElement(0,cv::Size( 2*tam + 1, 2*tam+1 ),cv::Point( tam, tam ));
		cv::dilate( imagen, rangoModificado, element );
		cv::imshow("Dilatado", rangoModificado);
		cvtColor(rangoModificado, imagenHSV,  cv::COLOR_RGB2HSV);
		int HSV[3];
		getHSV(&imagenHSV,x,y,HSV);
		printf("x:%d y:%d -> H:%d S:%d V:%d \n",x,y,HSV[0],HSV[1],HSV[2]);
		Scalar Min = Scalar(HSV[0]-10,HSV[1]-100,HSV[2]-100);
		Scalar Max = Scalar(HSV[0]+10,HSV[1]+100,HSV[2]+100);
		inRange(imagenHSV, Min,Max, imagenRango);
		cv::imshow("Rango", imagenRango);
		cv::Mat imagenAND;
		cv::bitwise_and(imagen,imagen,imagenAND,imagenRango);
		cv::imshow("Filtro color", imagenAND);
	 }else if(ejercicio == 3){
		 imagen = dest;
		 cv::Mat rangoModificado;
		 int tam = 1;
		 cv::Mat element = cv::getStructuringElement(0,cv::Size( 2*tam + 1, 2*tam+1 ),cv::Point( tam, tam ));
		 cv::dilate( imagen, rangoModificado, element );
		 //cv::imshow("Dilatado", rangoModificado);
		 cvtColor(rangoModificado, imagenHSV,  cv::COLOR_RGB2HSV);
		 int HSV[3];
		 getHSV(&imagenHSV,x,y,HSV);
		 printf("x:%d y:%d -> H:%d S:%d V:%d \n",x,y,HSV[0],HSV[1],HSV[2]);
		 Scalar Min = Scalar(HSV[0]-5,HSV[1]-50,HSV[2]-50);
		 Scalar Max = Scalar(HSV[0]+5,HSV[1]+50,HSV[2]+50);
		 minimo = Min;
		 maximo = Max;
		 listo = true;
	 }


}
void readCameraParams(cv::Mat &cameraMatrix,cv::Mat &distCoeffs){
	cv::FileStorage fs("Datos.txt", cv::FileStorage::READ );
	fs["camera_matrix"] >> cameraMatrix ;
	fs["distortion_coefficients"] >> distCoeffs ;
	fs.release () ;
}

int main( int argc, char** argv )
{
	ejercicio = 3;
	if(ejercicio == 1){
		cv::Mat imagenUniverso = cv::imread("IMG_20190907_111552510.jpg");
		if(imagenUniverso.empty()){
			printf("Error al abrir la imagen.");
			return -1;
		}
		cv::imshow("UniversoOriginal", imagenUniverso);
		cvtColor(imagenUniverso, imagenUniversoGris, CV_BGR2GRAY );


		namedWindow("Filtro", CV_WINDOW_AUTOSIZE );
		namedWindow("Filtro adaptativo",CV_WINDOW_AUTOSIZE);
		//filtro hold
		createTrackbar( mensaje1,"Filtro", &valorMovil2,4, funcionEvento);
		createTrackbar(mensaje2,"Filtro", &valorMovil1,255, funcionEvento);
		funcionEvento( 0, 0 );
		//filtro adaptativo
		createTrackbar(mensajeAdaptativo,"Filtro adaptativo", &valorMetodoAdaptativo,1,filtroAdaptativo);
		createTrackbar(mensajeThresholdTipo,"Filtro adaptativo", &valorThresholdTipo,1,filtroAdaptativo);
		createTrackbar(mensajeTamanoBloque,"Filtro adaptativo", &valorTamanoBloque,10,filtroAdaptativo);
		createTrackbar(mensajeC,"Filtro adaptativo", &valorC,10,filtroAdaptativo);
		filtroAdaptativo(0,0);
	}else if(ejercicio == 2){
		imagen = cv::imread("Images/carasCubo_opt.jpg");
		if(imagen.empty()){
			printf("Error al abrir la imagen.");
			return -1;
		}
		cv::imshow("imagen Original", imagen);


		cv::Mat rangoModificado;
		int tam = 1;
		cv::Mat element = cv::getStructuringElement(0,cv::Size( 2*tam + 1, 2*tam+1 ),cv::Point( tam, tam ));
		cv::dilate( imagen, rangoModificado, element );


		//cv::dilate(imagen, rangoModificado,0,Point(-1,-1),1);
		cv::imshow("Dilatado", rangoModificado);

		cvtColor(rangoModificado, imagenHSV,  cv::COLOR_RGB2HSV);
		cv::imshow("HSV", imagenHSV);

		int HSV[3];
		getHSV(&imagenHSV,194,171,HSV);
		printf("x:%d y:%d -> H:%d S:%d V:%d \n",10,10,HSV[0],HSV[1],HSV[2]);


		namedWindow("Filtro color", CV_WINDOW_AUTOSIZE );
		createTrackbar(mensajeColor,"Filtro color", &valorColor,5,funcionColor);
		funcionColor(0,0);

	}else if(ejercicio == 22){
		imagen = cv::imread("Images/IMG_20190907_111547909.jpg");
		float scale = 0.125;
		cv::resize(imagen,imagen,cv::Size(imagen.cols*scale,imagen.rows*scale),0,0);
		if(imagen.empty()){
			printf("Error al abrir la imagen.");
			return -1;
		}
		namedWindow("imagen Original", CV_WINDOW_AUTOSIZE );
		cv::imshow("imagen Original", imagen);
		cv::setMouseCallback("imagen Original",onMouse);
	}else if(ejercicio == 3){
		//despliege de la camara
		cv::VideoCapture miCamaraWeb;
		miCamaraWeb.open(1);	//inicia la camara 0, usualmente la webcam
		cv::Mat fotograma;
		int tecla;
		bool flag = true;
		while (tecla != 27) {
			miCamaraWeb.read(fotograma);
			//cv::imshow("Esta es mi camara 0 \"Camara web\"", fotograma);
			tecla = cv::waitKey(1);

			//correccion de distorcion

			readCameraParams(cameraMatrix, distCoeffs);


			miCamaraWeb.read(frame);
			imageSize = frame.size();
			cv::initUndistortRectifyMap(cameraMatrix,distCoeffs,cv::Mat(),
			getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,imageSize,1,imageSize,0),
						imageSize,CV_16SC2, map1, map2);
			cv::remap(frame,dest,map1,map2,cv::INTER_LINEAR);
			//imagen corregida
			namedWindow("Rect image", CV_WINDOW_AUTOSIZE );

			if(flag){
				cv::setMouseCallback("Rect image",onMouse);
				flag = false;
			}
			int Centroide[2];
			if(listo){
				//refrescar la imagen
				imagen = frame;
				cv::Mat rangoModificado;
				int tam = 1;
				cv::Mat element = cv::getStructuringElement(0,cv::Size( 2*tam + 1, 2*tam+1 ),cv::Point( tam, tam ));
				cv::dilate( imagen, rangoModificado, element );
				//cv::imshow("Dilatado", rangoModificado);
				cvtColor(rangoModificado, imagenHSV,  cv::COLOR_RGB2HSV);
				inRange(imagenHSV, minimo,maximo, imagenRango);
				cv::imshow("Rango", imagenRango);


				//centroide
				cv::Mat imagenAND;
				cv::bitwise_and(imagen,imagen,imagenAND,imagenRango);
				cv::imshow("AND",imagenAND);
				centroide(&imagenAND,Centroide);

				//borde
				//bilateralFilter(imagenAND,imagenAND, 3, 3, 3,3);

				int tam2 = 5;
				cv::Mat element2 = cv::getStructuringElement(0,cv::Size( 2*tam2 + 1, 2*tam2+1 ),cv::Point( tam2, tam2 ));
				cv::dilate( imagenAND, imagenAND, element2 );
				//blur(imagenAND, imagenAND, Size(50,50));
				int tam3 = 5;
				cv::Mat element3 = cv::getStructuringElement(0,cv::Size( 2*tam3 + 1, 2*tam3+1 ),cv::Point( tam3, tam3 ));
				erode( imagenAND, imagenAND, element3 );
				cv::imshow("blurrr",imagenAND);
				int thresh = 50;
			    Mat canny_output;
			    Canny( imagenAND, canny_output, thresh, thresh*2,3 );


			    vector< vector<Point> > contours;
			    vector<Vec4i> hierarchy;
			    findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
			    Mat drawing = imagen;
			    int maxi = -1;
			    double maxArea = 0;
			    for( int i = 0; i<contours.size(); i++ )
			    {
			    	double epsilon = 0.0*arcLength(contours[i],true);
			    	approxPolyDP(contours[i],contours[i],epsilon,true);
			    	Scalar color = Scalar(255,255,0);
			    	//drawContours( imagen, contours,i, color, 2, LINE_8, hierarchy, 0 );

			        double area = contourArea(contours[i]);
			        if(area > maxArea){
			        	maxArea = area;
			        	maxi = i;
			        }
			        //printf("%lf \n",area);

			    }
			    	Scalar color = Scalar(255,255,0);
					drawContours( imagen, contours,maxi, color, 2, LINE_8, hierarchy, 0 );
					imshow( "Contours", drawing );
			}
			circle( dest, cv::Point(Centroide[1],Centroide[0]),10,Scalar(0,0,255),-1);
			cv::imshow("Rect image", dest);
		}
			return 0;
	}


	int c;
	while(true){
		c = waitKey(1);
		if( (char)c == 27 )
		  return 0;
	}
}





