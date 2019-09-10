#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

//Bandera para cámara tipo ojo de pescado
bool useFisheye = false;
int widthChessboard = 9 - 1;
int heightChessboard = 7 - 1;
int delay = 2000;
//Frames para la calibración
int nrFrames = 20;
float squareSize = 22;
int flag;
bool calibFixPrincipalPoint = true;
bool calibZeroTangentDist = true;
bool aspectRatio = true;
bool fixK1 = false;
bool fixK2 = false;
bool fixK3 = false;
bool fixK4 = true;
bool fixK5 = true;


bool flagSave = false;

//Modos
enum Mode {
	DETECTION = 0, CAPTURING = 1, CALIBRATED = 2,LOAD = 3
};

void saveCameraParams(cv::Mat &cameraMatrix,cv::Mat &distCoeffs){
	cv::FileStorage fs("Datos.txt", cv::FileStorage::WRITE ) ;
	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs ;
	fs.release();
}

void readCameraParams(cv::Mat &cameraMatrix,cv::Mat &distCoeffs){
	cv::FileStorage fs("Datos.txt", cv::FileStorage::READ );
	fs["camera_matrix"] >> cameraMatrix ;
	fs["distortion_coefficients"] >> distCoeffs ;
	fs.release () ;
}

void readCameraParamsMatlab(cv::Mat &cameraMatrix,cv::Mat &distCoeffs){
	cv::FileStorage fs("DatosMatlab.txt", cv::FileStorage::READ );
	fs["camera_matrix"] >> cameraMatrix ;
	fs["distortion_coefficients"] >> distCoeffs ;
	fs.release () ;
}

int main(int argc, char ** argv) {
	cv::VideoCapture capture;
	capture.open(1);
	cv::Mat frame;
	bool found;
	int key;
	const char ESC_KEY = 27;
	//Filtrado y normalización de la imagen
	int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH
			| cv::CALIB_CB_NORMALIZE_IMAGE;
	//Las detecciones para ojo de pescado debe ser más lento
	if (useFisheye == false) {
		// fast check erroneously fails with high distortions like fisheye
		chessBoardFlags |= cv::CALIB_CB_FAST_CHECK;
	}
	//Matriz que guarda los coeficientes de distorsión
	cv::Mat cameraMatrix, distCoeffs;
	cv::Mat cameraMatrixm, distCoeffsm;
	cv::Size imageSize;
	std::vector<std::vector<cv::Point2f> > imagePoints;
	cv::Size boardSize = cv::Size(widthChessboard, heightChessboard);
	clock_t prevTimestamp = 0;
	Mode mode = DETECTION;
	const cv::Scalar RED(0, 0, 255), GREEN(0, 255, 0);
	while (1) {
		bool blinkOutput = false;
		capture.read(frame);
		imageSize = frame.size();
		if (mode == CAPTURING && imagePoints.size() >= (size_t) nrFrames) {
			// Here put the code to calibrate the camera
			mode = CALIBRATED;
			//Aquí va la segunda parte
			std::vector<std::vector<cv::Point3f> > objectPoints(1);
			for (int i = 0; i < boardSize.height; ++i)
				for (int j = 0; j < boardSize.width; ++j)
					objectPoints[0].push_back(
							cv::Point3f(j * squareSize, i * squareSize, 0));
			objectPoints.resize(imagePoints.size(), objectPoints[0]);
			std::vector<cv::Mat> rvecs, tvecs;
			std::vector<float> reprojErrs;
			double totalAvgErr = 0;
			double rms;
			cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
			if (flag & cv::CALIB_FIX_ASPECT_RATIO)
				cameraMatrix.at<double>(0, 0) = aspectRatio;
			if (useFisheye) {
				distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
				cv::Mat _rvecs, _tvecs;
				rms = cv::fisheye::calibrate(objectPoints, imagePoints,
						imageSize, cameraMatrix, distCoeffs, _rvecs, _tvecs,
						flag);
				rvecs.reserve(_rvecs.rows);
				tvecs.reserve(_tvecs.rows);
				for (int i = 0; i < int(objectPoints.size()); i++) {
					rvecs.push_back(_rvecs.row(i));
					tvecs.push_back(_tvecs.row(i));
				}
			} else {
				distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
				rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
						cameraMatrix, distCoeffs, rvecs, tvecs, flag);
			}
			std::cout << "Errores de re-projection calibrateCamera: " << rms
					<< std::endl;
			bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
			if (ok)
				mode = CALIBRATED;
			else
				mode = DETECTION;
		}
		if (mode != CALIBRATED && mode != LOAD) {
			// Find feature points on the input format
			//Se crea vector que guarda los puntos de las esquinas
			std::vector<cv::Point2f> pointBuf;

			found = cv::findChessboardCorners(frame, boardSize, pointBuf,
					chessBoardFlags);
			if (found) {
				// improve the found corners coordinate accuracy for chessboard
				cv::Mat viewGray;
				//Se cambia a escala de grises
				cv::cvtColor(frame, viewGray, cv::COLOR_BGR2GRAY);
				//Función que
				cv::cornerSubPix(viewGray, pointBuf, cv::Size(11, 11),
						cv::Size(-1, -1),
						cv::TermCriteria(
								cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
								30, 0.1)); //0.1 es el error
				//Si está en modo captura, se introducen las esquinas de los frames a un vector
				if (mode == CAPTURING
						&& (clock() - prevTimestamp
								> delay * 1e-3 * CLOCKS_PER_SEC)) {
					imagePoints.push_back(pointBuf);
					prevTimestamp = clock();
					blinkOutput = true;
				}
// Draw the corners.
				cv::drawChessboardCorners(frame, boardSize, cv::Mat(pointBuf),
						found);
			}

		}else{

			if(mode == LOAD && flagSave == false){
				flagSave = true;
				readCameraParams(cameraMatrix, distCoeffs);
				readCameraParamsMatlab(cameraMatrixm, distCoeffsm);

			}else if(mode == CAPTURING && flagSave == false){
				flagSave = true;
				saveCameraParams(cameraMatrix, distCoeffs);
			}
			saveCameraParams(cameraMatrix, distCoeffs);
			cv::Mat map1, map2, dest;
			cv::initUndistortRectifyMap(cameraMatrix,distCoeffs,cv::Mat(),
			getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,imageSize,1,imageSize,0),
						imageSize,CV_16SC2, map1, map2);

			cv::remap(frame,dest,map1,map2,cv::INTER_LINEAR);
			cv::imshow("Rect image", dest);


			/*cv::Mat map1m, map2m, destm;
			cv::initUndistortRectifyMap(cameraMatrixm,distCoeffsm,cv::Mat(),
			getOptimalNewCameraMatrix(cameraMatrixm,distCoeffsm,imageSize,1,imageSize,0),
			imageSize,CV_16SC2, map1m, map2m);

			cv::remap(frame,destm,map1m,map2m,cv::INTER_LINEAR);
			cv::imshow("Rect image Matlab", destm);*/
		}
		//Mensajes que se despliegan en la pantalla
		std::string msg =
				(mode == CAPTURING) ? "100/100 " :
				mode == CALIBRATED ? "Calibrated" : "Press’g’tostart___Press'l'to_Load";
		int baseLine = 0;
		cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
		cv::Point textOrigin(frame.cols - 2 * textSize.width - 10,
				frame.rows - 2 * baseLine - 10);
		if (mode == CAPTURING)
			//Se le da formato
			msg = cv::format(" % d /% d ", (int) imagePoints.size(), nrFrames);
		//Se le asigna color al texto
		cv::putText(frame, msg, textOrigin, 1, 1,
				mode == CALIBRATED ? GREEN : RED);
		if (blinkOutput)
			//Hace un flashazo????
			cv::bitwise_not(frame, frame);
		cv::imshow(" \"Frame\" ", frame);
		key = cv::waitKey(1);
		if (key == (char) 'g') {
			mode = CAPTURING;
			imagePoints.clear();
		}else if(key == (char) 'l'){
			mode = LOAD;
		}
		if (key == ESC_KEY)
			break;
	}
	return 1;
}

