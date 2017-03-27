/**
 * Copyright 2017 National Chiao Tung University, Intelligent System and Control Integration Laboratory
 * Author: Cheng-Hei Wu 
 * Maintainer : Howard Chen 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include "KinectClass.h"

KinectClass::KinectClass()
{
	KinectStatus = false;
}

bool KinectClass::getKinectStatus()
{
	return KinectStatus;
}

IplImage* KinectClass::getpFrameBGR()
{
	return pFrameBGR;
}

IplImage* KinectClass::getpFrameDepth()
{
	return pFrameDepth;
}

//XnPoint3D* KinectClass::getpRealWorldCamaraPointSet()
//{
//	return pRealWorldCamaraPointSet;
//}

void KinectClass::CheckOpenNIError( string eStatus )
{
	eStatus = xnGetStatusString(eResult);
	string eMsg = "Error : " + eStatus;
	if ( eResult != XN_STATUS_OK ){
		std::cout << eMsg << std::endl;
	}
}

void KinectClass::KinectInitial()
{
	if (KinectStatus == false){
		// 1. Initial Context
		eResult = mContext.Init();
		CheckOpenNIError(" Initial Context ");

		// 2. Set Map_mode 
		/*std::stringstream out_yml_filename;
		cv::Mat Read_Depth;
		cv::Mat c16BitDepth;
		out_yml_filename.str( yml_name ); 
		cv::FileStorage fsd( out_yml_filename.str(), cv::FileStorage::READ);		
		fsd["frameCount"] >> c16BitDepth;		
		c16BitDepth.convertTo( Read_Depth, CV_32FC1);*/
		XnMapOutputMode mapMode;
		mapMode.nXRes = 640;
		mapMode.nYRes = 480;
		mapMode.nFPS = 30;

		// 3.a Create Depth_Generator
		eResult = mDepthGenerator.Create( mContext );
		CheckOpenNIError(" Create Depth Generator " );
		eResult = mDepthGenerator.SetMapOutputMode( mapMode );

		// 3.b Create Image_Generator
		eResult = mImageGenerator.Create( mContext );
		CheckOpenNIError( " Create Image Generator " );
		eResult = mImageGenerator.SetMapOutputMode( mapMode );

		 // 4. Correct view port
		eResult = mDepthGenerator.GetAlternativeViewPointCap().SetViewPoint( mImageGenerator );
		CheckOpenNIError( " Correct view port " );

		if (eResult == XN_STATUS_OK){
			std::cout << "正確：KinectSensor Intial Correct!" << std::endl;
			KinectStatus = true;
		}	
	}
	//else{
		//std::cout << "警告：請勿重複開啟Kinect!" << std::endl;
	//}
}

void KinectClass::CloseKinect()
{
	if (KinectStatus == true){
		eResult = mContext.StopGeneratingAll();
		
		if ( eResult == XN_STATUS_OK ){
			mContext.Shutdown();
			Sleep(10);
			std::cout << "正確：KinectSensor Shutdown!" << std::endl;
			KinectStatus = false;
		}
		else{
			std::cout << "StopGeneratingAll " << std::endl;
		}
	}
	else {
		std::cout << "警告：尚未開啟KinectSensor!" << std::endl;
	}
}

void KinectClass::CaptureImage()
{
	int idxShift, idx;
	int uPointNum = 640*480;
	static int FrameCount=0;
	//cv::namedWindow( "Depth Image", CV_WINDOW_AUTOSIZE );
    //cv::namedWindow( "Color Image", CV_WINDOW_AUTOSIZE );

	if ( KinectStatus == true )
	{
		// 4. start generate data
		eResult = mContext.StartGeneratingAll();

		// 5. read data
		eResult = mContext.WaitAndUpdateAll();

		// 6. get image data
		xn::ImageMetaData xColorData;
		mImageGenerator.GetMetaData( xColorData );

		// 7a. convert to OpenCV form
		cv::Mat cShowImg( xColorData.FullYRes(), xColorData.FullXRes(),
		CV_8UC3, (void*)xColorData.Data() );

		// 7b. convert from RGB to BGR
		cv::Mat cBGRImg;
		cv::cvtColor( cShowImg, cBGRImg, CV_RGB2BGR );
		//cv::imshow( "Color Image", cBGRImg );
			
		// 8. get depth data
		xn::DepthGenerator   xDepth;
		xDepth.Create( mContext );

		xn::DepthMetaData xDepthData;
		xDepth.GetMetaData( xDepthData );

		// 8a. convert to OpenCV form
		cv::Mat cDepthImg( xDepthData.FullYRes(), xDepthData.FullXRes(),
		CV_16UC1, (void*)xDepthData.Data() );

		// 8b. convert to 8 bit
		cv::Mat c8BitDepth;
		cDepthImg.convertTo( c8BitDepth, CV_8U, 255.0 / 7000 );
		//cv::namedWindow( "Depth Image", CV_WINDOW_AUTOSIZE );
		//cv::imshow( "Depth Image", c8BitDepth );
				
		// 9. save RGBD image data 	

		std::stringstream out_RGB_png_filename;
		out_RGB_png_filename << "Scene_"  << "_RGB"   << ".png";
		cv::imwrite(out_RGB_png_filename.str(), cBGRImg);

		std::stringstream out_D_png_filename;
		out_D_png_filename << "Scene_" << "_D"     << ".png";
		cv::imwrite(out_D_png_filename.str(), c8BitDepth);

		std::stringstream out_yml_filename;
		out_yml_filename << "Scene" << ".yml";

		Scene_ymlName = new char[out_yml_filename.str().length() + 1];
		strcpy(Scene_ymlName, out_yml_filename.str().c_str());

		cv::FileStorage fs( out_yml_filename.str(), cv::FileStorage::WRITE);
		fs << "frameCount" << cDepthImg;
		fs.release();
		
		FrameCount++;
		cv::waitKey(30);
	}

	else
	{
		std::cout << "錯誤 : 尚未開啟KinectSensor!!" << std::endl;
	}
}


void KinectClass::yml2realwordlByKinect(char *yml_name)
{

	std::stringstream out_yml_filename;
	cv::Mat Read_Depth;
	cv::Mat c16BitDepth;
	out_yml_filename.str( yml_name ); 
	cv::FileStorage fsd( out_yml_filename.str(), cv::FileStorage::READ);		
	fsd["frameCount"] >> c16BitDepth;		
	c16BitDepth.convertTo( Read_Depth, CV_32FC1);
		
	int idxShift, idx;
	XnPoint3D* pDepthPointSet = new XnPoint3D[ Read_Depth.cols * Read_Depth.rows];
	pRealWorldCamaraPointSet = new XnPoint3D[ Read_Depth.cols * Read_Depth.rows ];

	for ( int j = 0; j < Read_Depth.rows; ++j )
	{
		idxShift = j *  Read_Depth.cols;
		for ( int i = 0; i <   Read_Depth.cols; ++i)
		{
				idx = idxShift + i;
				pDepthPointSet[idx].X = (int)i;
				pDepthPointSet[idx].Y = (int)j;
				pDepthPointSet[idx].Z = (int)Read_Depth.at<float>(idx);
				
		}
	}
	mDepthGenerator.ConvertProjectiveToRealWorld( Read_Depth.cols * Read_Depth.rows, pDepthPointSet, pRealWorldCamaraPointSet );

	delete [] pDepthPointSet;
	//delete [] pRealWorldCamaraPointSet;
}


void KinectClass::SceneToPCDProcessing()
{
	if (!KinectStatus){
		KinectInitial();
	}

	if (KinectStatus)
	{
		CaptureImage();
		yml2realwordlByKinect(Scene_ymlName);
	}
	
}