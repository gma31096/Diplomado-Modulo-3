#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct datosMouse{
	cv::Mat* imagen;
	int NumeroEjercicio;
}datosRaton;
bool getBGR(cv::Mat* imagen,int x,int y,int* BGR){
	 cv::Vec3b bgrChanel = (*imagen).at<cv::Vec3b>(y,x);
	 BGR[0] = (int)bgrChanel.val[0];
	 BGR[1] = bgrChanel.val[1];
	 BGR[2] = bgrChanel.val[2];
	 return true;
}
void normalizaImagen(cv::Mat* imagen){
	for(int x=0 ; x<(*imagen).rows ;x++){
		for(int y=0 ; y<(*imagen).cols ; y++){
			int suma = 0;
			for(int i = 0 ; i<3 ; i++)
				suma += (*imagen).at<cv::Vec3b>(x,y)[i];
			for(int i = 0 ; i<3 ; i++){
				float mag = (*imagen).at<cv::Vec3b>(x,y)[i];
				mag *= 255;
				mag /= suma;
				(*imagen).at<cv::Vec3b>(x,y)[i] = (int)mag;
			}
		}
	}
}
void getChanelBGR(cv::Mat* imagen,int chanel){
	for(int x=0 ; x<(*imagen).rows ;x++){
		for(int y=0 ; y<(*imagen).cols ; y++){
			for(int i = 0 ; i<3 ; i++){
				if(i!=chanel)	(*imagen).at<cv::Vec3b>(x,y)[i] = 0;
			}
		}
	}
}
int esquina[2][2]={{-1,-1},{-1,-1}};
void onMouse(int event, int x, int y, int what, void* param){
	 if( event != CV_EVENT_LBUTTONDOWN )
	        return;

	 datosMouse* datos = (datosMouse*) param;
	 cv::Mat * imagenUniverso = datos->imagen;
	 if(datos->NumeroEjercicio == 1){
		 int BGR[3];
		 getBGR(imagenUniverso,x,y,BGR);
		 printf("x:%d y:%d -> B:%d G:%d R:%d \n",x,y,BGR[0],BGR[1],BGR[2]);
	 }else if(datos->NumeroEjercicio == 3){
		 if(esquina[0][0] == -1 && esquina[1][0] == -1){
			 //si no a hecho click asignalo a la primer esquina
			 esquina[0][0] = x;
			 esquina[0][1] = y;
		 }else if(esquina[0][0] != -1 && esquina[1][0] == -1){
			 //si ya a dado click en una esquina
			 esquina[1][0] = x;
			 esquina[1][1] = y;
			 //dibuja rectangulo
		 }else{
			 //si ya dio click mas de dos veces  la esquina anterior cambiala
			 esquina[0][0] = esquina[1][0];
			 esquina[0][1] = esquina[1][1];
			 esquina[1][0] = x;
			 esquina[1][1] = y;
			 //dibuja rectangulo
		 }
		 printf("(%d,%d) (%d,%d)\n",esquina[0][0],esquina[0][1],esquina[1][0],esquina[1][1]);
		 //cv::rectangle(imagenUniverso, cv::Point(5, 5), cv::Point(100, 100), cv::Scalar(0,255, 0), 2);
	 }
}
int main(int argc, char** argv) {
	int ejercicio = 1;
	if(argc>1)
		ejercicio = argv[1][0]-'0';
//despliege de la imagen
	cv::Mat imagenUniverso = cv::imread("Images/universo.jpg");
	if(imagenUniverso.empty()){
		printf("Error al abrir la imagen.");
		return -1;
	}
	//jalar de referencia

	if(ejercicio == 1){
		cv::namedWindow("Ventana ejercicio 1",1);
		datosRaton.imagen = &imagenUniverso;
		datosRaton.NumeroEjercicio=ejercicio;
		//cv::setMouseCallback("Ventana ejercicio 1",onMouse,&imagenUniverso);
		cv::setMouseCallback("Ventana ejercicio 1",onMouse,&datosRaton);
		cv::imshow("Ventana ejercicio 1", imagenUniverso);

	}else if(ejercicio == 2){
		cv::imshow("Original", imagenUniverso);
		// Azul
		cv::Mat blueUniverso;
		imagenUniverso.copyTo(blueUniverso);
		getChanelBGR(&blueUniverso,0);
		cv::imshow("Ventana Azul",blueUniverso);
		//Verde
		cv::Mat greenUniverso;
		imagenUniverso.copyTo(greenUniverso);
		getChanelBGR(&greenUniverso,1);
		cv::imshow("Ventana Verde",greenUniverso);
		//Verde
		cv::Mat redUniverso = ;
		imagenUniverso.copyTo(redUniverso);
		getChanelBGR(&redUniverso,2);
		cv::imshow("Ventana Roja",redUniverso);

	}else if(ejercicio == 3){
		cv::namedWindow("Ventana ejercicio 3",1);
		datosRaton.imagen = &imagenUniverso;
		datosRaton.NumeroEjercicio=ejercicio;
		cv::setMouseCallback("Ventana ejercicio 3",onMouse,&datosRaton);
		int tecla;
		while (tecla != 27) {
			cv::Mat imagenRectangulo;
			imagenUniverso.copyTo(imagenRectangulo);
			if(esquina[1][0] != -1){
				cv::rectangle(imagenRectangulo,cv::Point(esquina[0][0],esquina[0][1]), cv::Point(esquina[1][0], esquina[1][1]),cv::Scalar(0,255, 0),1,0);
				int y0 = esquina[0][0], y1 = esquina[1][0];
				int x0 = esquina[0][1], x1 = esquina[1][1];
				if(y0>y1){
					int aux = y0;
					y0 = y1;
					y1 = aux;
				}
				if(x0>x1){
					int aux = x0;
					x0 = x1;
					x1 = aux;
				}
				if(x0==x1)	x1++;
				if(y0==y1)	y1++;
				printf("%d %d : %d %d\n",x0,x1,y0,y1);
				cv::Range filas(x0,x1);
				cv::Range columnas(y0,y1);
				cv::Mat imagenRecotada = imagenUniverso(filas,columnas);
				cv::imshow("Ventana ejercicio 3: recorte", imagenRecotada);
			}
			cv::imshow("Ventana ejercicio 3", imagenRectangulo);
			tecla = cv::waitKey(1);
		}
		return 0;
	}else if(ejercicio == 4){
		cv::Mat imagenIA = cv::imread("Images/ia.jpg");
		if(imagenIA.empty()){
			printf("Error al abrir la imagen.");
			return -1;
		}
		int x = 300;
		int y = 250;
		cv::Rect rect(185, 40, x,y);
		cv::Mat recorteIA = imagenIA(rect);
		cv::imshow("Ventana ejercicio 4:IA", recorteIA);
		cv::Rect rect2(240, 50, x,y);
		cv::Mat recorteUniverso = imagenUniverso(rect2);
		cv::imshow("Ventana ejercicio 4:Universo", recorteUniverso);
		//-----------------------------AND--------------------------------
		cv::Mat imagenAND;
		cv::bitwise_and(recorteIA,recorteUniverso,imagenAND);
		cv::imshow("Ventana ejercicio 4:AND", imagenAND);
		//-----------------------------OR--------------------------------
		cv::Mat imagenOR;
		cv::bitwise_or(recorteIA,recorteUniverso,imagenOR);
		cv::imshow("Ventana ejercicio 4:OR", imagenOR);
		//-----------------------------NOT--------------------------------
		cv::Mat imagenNOTia;
		cv::bitwise_not(recorteIA,imagenNOTia);
		cv::imshow("Ventana ejercicio 4:NOT IA", imagenNOTia);
		cv::Mat imagenNOTuniverso;
		cv::bitwise_not(recorteUniverso,imagenNOTuniverso);
		cv::imshow("Ventana ejercicio 4:NOT Universo", imagenNOTuniverso);

		//-----------------------------XOR--------------------------------
		cv::Mat imagenXOR;
		cv::bitwise_xor(recorteIA,recorteUniverso,imagenXOR);
		cv::imshow("Ventana ejercicio 4:XOR", imagenXOR);


	}else if(ejercicio == 5){
		cv::imshow("Original", imagenUniverso);

		cv::Mat universoGris;
		cvtColor(imagenUniverso, universoGris, cv::COLOR_RGB2GRAY);
		cv::imshow("Gris", universoGris);

		cv::Mat universoHSV;
		cvtColor(imagenUniverso, universoHSV,  cv::COLOR_RGB2HSV);
		cv::imshow("HSV", universoHSV);

		cv::Mat universoXYZ;
		cvtColor(imagenUniverso, universoXYZ,cv::COLOR_RGB2XYZ);
		cv::imshow("XYZ", universoXYZ);

		cv::Mat universoYCC;
		cvtColor(imagenUniverso, universoYCC,cv::COLOR_RGB2YCrCb);
		cv::imshow("YCC", universoYCC);

		cv::Mat universoHLS;
		cvtColor(imagenUniverso, universoHLS,cv::COLOR_RGB2HLS);
		cv::imshow("HLS", universoHLS);

		cv::Mat universoLab;
		cvtColor(imagenUniverso, universoLab,cv::COLOR_RGB2Lab);
		cv::imshow("Lab", universoLab);

		cv::Mat universoLuv;
		cvtColor(imagenUniverso, universoLuv,cv::COLOR_RGB2Luv);
		cv::imshow("Luv", universoLuv);



	}else if(ejercicio == 6){
		cv::imshow("Original", imagenUniverso);
		cv::Mat universoNormalizado;
		imagenUniverso.copyTo(universoNormalizado);
		normalizaImagen(&universoNormalizado);
		cv::imshow("Imagen Normalizada",universoNormalizado);
	}
	int tecla;
	while (tecla != 27) {
		tecla = cv::waitKey(1);
	}
	printf("sali");
	return 0;
}

