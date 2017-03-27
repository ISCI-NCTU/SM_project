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

#ifndef KINECT_CLASS
#include <fstream> /* ifstream class */
#include <string>/* string class   */
#include <cstdlib> /* atoi   */
#include <sstream> /* istringstream class */
#include <XnCppWrapper.h>
#include <highgui.h>
#include <cv.h>
#include <cxcore.h>
#include <cvaux.h>
using namespace std;
using namespace xn;
#endif


class KinectClass
{
	private:
		bool KinectStatus;
		Context mContext;
		DepthGenerator mDepthGenerator;
		ImageGenerator mImageGenerator;
		XnStatus eResult;
		
		IplImage *pFrameBGR;
		IplImage *pFrameL;
		IplImage *pFrameDepth;
		IplImage *pFrameGray;



	public:
		KinectClass();
		bool getKinectStatus();
		char* Scene_ymlName;
		XnPoint3D* pRealWorldCamaraPointSet;
		IplImage* getpFrameBGR();
		IplImage* getpFrameDepth(); 
		void KinectInitial();
		void CheckOpenNIError( string eStatus );
		void CloseKinect();
		void CaptureImage();
		void yml2realwordlByKinect(char *yml_name);
		void SceneToPCDProcessing();
};
