#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <KinectConnector.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

int main() {
	KinectConnector kin = KinectConnector();
	if (!kin.Connect()) return 1;

	while (true) {
		Mat depthImg;
		Mat colorImg;
		Mat indexImg;
		Mat pointImg;

		kin.GrabData(depthImg, colorImg, indexImg, pointImg);
		imshow("depthImg", depthImg);
		imshow("colorImg", colorImg);
		imshow("indexImg", indexImg);
		imshow("pointImg", pointImg);

		waitKey(100);
	}
	return 0;
}