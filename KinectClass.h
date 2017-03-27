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
