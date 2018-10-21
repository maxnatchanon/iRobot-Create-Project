
#ifndef _KINECTCONNECTOR_H_
#define _KINECTCONNECTOR_H_

#include <iostream>
#include <NuiApi.h>

#include "opencv2/core/core.hpp"

class KinectConnector
{
public :
	KinectConnector()
	{
		isConnect = false;

		depthD16 = new USHORT[640*480];
		colorCoordinates = new LONG[640*480*2];
		colorRGBX = new BYTE[1280*960*4];
	}

	bool Connect()
	{
		if( isConnect )
			return true;

		HRESULT hr;

		//NUI initialize
		hr = NuiCreateSensorByIndex(0, &sensor);
		if( FAILED(hr) )
		{
			std::cout << "NuiCreateSensorByIndex failed." << std::endl;
			return false;
		}
		//WCHAR *id = m_pNuiSensor->NuiDeviceConnectionId();

		hr = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);
		if (FAILED(hr))
		{
			std::cout << "NuiInitialize failed." << std::endl;
			return false;
		}

		//create color stream
		hr = sensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_1280x960, 0, 2, NULL, &pColorStreamHandle );
		//hr = sensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, NULL, &pColorStreamHandle );
		if( FAILED(hr) )
		{
			std::cout << "NuiImageStreamOpen NUI_IMAGE_TYPE_COLOR failed." << std::endl;
			return false;
		}
		
		//create depth stream
		hr = sensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_STREAM_FLAG_DISTINCT_OVERFLOW_DEPTH_VALUES, 2, NULL, &pDepthStreamHandle);
		if( FAILED(hr) )
		{
			std::cout << "NuiImageStreamOpen NUI_IMAGE_TYPE_DEPTH failed." << std::endl;
			return false;
		}

		return isConnect = true;
	}

	void Disconnect()
	{
		isConnect = false;
		sensor->NuiShutdown();
	}

	bool IsConnected()
	{
		return isConnect;
	}

	void CheckStatus()
	{
		if( FAILED(sensor->NuiStatus()) )
		{
			Disconnect();
			Connect();
		}
	}

	bool GrabData(cv::Mat &depthImg, cv::Mat &colorImg, cv::Mat &indexImg, cv::Mat &pointImg)
	{
		bool isGrabData = false;
		colorImg.create(960, 1280, CV_8UC3);
		depthImg.create(480, 640, CV_16SC1);
		indexImg.create(960, 1280, CV_32SC1);
		pointImg.create(480, 640, CV_32FC3);

		HRESULT hr;
		//////////// color //////////////////////////
		hr = sensor->NuiImageStreamGetNextFrame( pColorStreamHandle, 5, &colorFrame );
		if( !FAILED(hr) )
		{
			// Lock the frame data so the Kinect knows not to modify it while we're reading it
			colorFrame.pFrameTexture->LockRect(0, &colorRect, NULL, 0);
			if( colorRect.Pitch != 0 )
			{
				cv::Mat tmpColorImg(colorImg.rows, colorImg.cols, CV_8UC4, colorRect.pBits);

				int val[6] = {0, 0, 1, 1, 2, 2};
				cv::mixChannels(tmpColorImg, colorImg, std::vector<int>(val, val+6));
				cv::flip(colorImg, colorImg, 1);
			}
			colorFrame.pFrameTexture->UnlockRect(0);
			sensor->NuiImageStreamReleaseFrame( pColorStreamHandle, &colorFrame );

			isGrabData = true;
		}

		////////////// depth //////////////////////////
		hr = sensor->NuiImageStreamGetNextFrame(pDepthStreamHandle, 5, &depthFrame );

		BOOL nearMode;
		INuiFrameTexture *pFrameTexture;

		hr = sensor->NuiImageFrameGetDepthImagePixelFrameTexture(pDepthStreamHandle, &depthFrame, &nearMode, &pFrameTexture );
		if( !FAILED(hr) )
		{
			// Lock the frame data so the Kinect knows not to modify it while we're reading it
			pFrameTexture->LockRect(0, &depthRect, NULL, 0);
			if( depthRect.Pitch != 0 )
			{
				unsigned short *p0 = (unsigned short*)depthImg.data;
				NUI_DEPTH_IMAGE_PIXEL *p1 = (NUI_DEPTH_IMAGE_PIXEL*)depthRect.pBits;

				for(int i = 0; i < depthRect.size; i+=4)
				{
					*p0 = (*p1).depth;
					p0++; p1++;
				}
				cv::flip(depthImg, depthImg, 1);
			}
			pFrameTexture->UnlockRect(0);
			sensor->NuiImageStreamReleaseFrame( pDepthStreamHandle, &depthFrame );

			isGrabData = true;
		}

		////////////// match //////////////////////////
		sensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
			NUI_IMAGE_RESOLUTION_1280x960, NUI_IMAGE_RESOLUTION_640x480, 
			640*480, depthD16, 640*480*2, colorCoordinates);

		indexImg.setTo(-1);
		// loop over each row and column of the color
		for(int i = 0; i < 960; i++)
		{
			for(int j = 0; j < 1280; j++)
			{
				// calculate index into depth array
				int depthIndex = j/2 + (i/2 * 640);
				int depth = depthImg.at<USHORT>(depthIndex);

				// retrieve the depth to color mapping for the current depth pixel
				int colorInDepthX = colorCoordinates[depthIndex * 2] + 0.028*depth - 15;
				int colorInDepthY = colorCoordinates[depthIndex * 2 + 1];

				// make sure the depth pixel maps to a valid point in color space
				if ( colorInDepthX >= 0 && colorInDepthX < 1280 && colorInDepthY >= 0 && colorInDepthY < 960 &&	0 < depth )
				{
					int colorIndex = colorInDepthX + colorInDepthY * 1280;
					indexImg.at<int>(colorIndex) = depthIndex;
				}
			}
		}

		////////////// point //////////////////////////
		for(int i = 0; i < 480; i++)
		{
			for(int j = 0; j < 640; j++)
			{
				Vector4 vec = NuiTransformDepthImageToSkeleton(j, i, depthImg.at<USHORT>(i, j), NUI_IMAGE_RESOLUTION_640x480);
				pointImg.at<cv::Vec3f>(i, j) = cv::Vec3f(-8000*vec.x, 8000*vec.y, 8000*vec.z);
			}
		}

		return isGrabData;
	}

private:
	INuiSensor *sensor;
	bool isConnect;

	HANDLE pDepthStreamHandle;
	NUI_IMAGE_FRAME depthFrame;
	NUI_LOCKED_RECT depthRect;

	HANDLE pColorStreamHandle;
	NUI_IMAGE_FRAME colorFrame;
	NUI_LOCKED_RECT colorRect;

    USHORT	*depthD16;
    BYTE	*colorRGBX;
    LONG	*colorCoordinates;
};

#endif // _KINECTCONNECTOR_H_