/*
 * main.cpp
 *
 *  Created on: Jul 20, 2019
 *      Author: gerardo
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


int main(int argc, char** argv) {
//despliege de la imagen
	cv::Mat imagenUniverso = cv::imread("Images/universo.jpg");
	if(imagenUniverso.empty()){
		printf("Error al abrir la imagen.");
		return -1;
	}
	cv::imshow("UniversoOriginal", imagenUniverso);

 //despliege de la camara
	cv::VideoCapture miCamaraWeb;
	miCamaraWeb.open(0);	//inicia la camara 0, usualmente la webcam
	cv::Mat fotograma;
	int tecla;
	while (tecla != 27) {
		miCamaraWeb.read(fotograma);
		cv::imshow("Esta es mi camara 0 \"Camara web\"", fotograma);
		tecla = cv::waitKey(1);
	}
	return 0;
}


