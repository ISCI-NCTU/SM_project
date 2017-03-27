#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>

class OpenNIViewerClass
{
	public:
		void showCloud (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
		void showRGBImage ( const boost::shared_ptr<openni_wrapper::Image> &image);
		void run();
		void capture_oneScene();
		void init();

		OpenNIViewerClass() 
			:Cloud_viewer("PCL OpenNI Viewer")
			,RGB_Viewer("PCL OpenNI RGB_Viewer")
			, rgb_data_size_(0)
			, rgb_data_ (0)
			, image_init(false)
			, cloud_init(false)
			, init_state(false)
		{
			interface = new pcl::OpenNIGrabber();

			pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_CaptureCloud (new pcl::PointCloud<pcl::PointXYZ>);
			Kinect_CaptureCloud = kinect_CaptureCloud;
		}


		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_;
		boost::shared_ptr<openni_wrapper::Image> image_;
		pcl::visualization::PCLVisualizer Cloud_viewer;
		pcl::visualization::ImageViewer RGB_Viewer;
		boost::mutex cloud_mutex_;
		boost::mutex image_mutex_;
		unsigned rgb_data_size_;
		unsigned char* rgb_data_;

		pcl::Grabber* interface;
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>  cloud_cb;
		boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb;
		boost::signals2::connection cloud_connection;
		boost::signals2::connection image_connection;
		bool image_init; 
		bool cloud_init;
		bool init_state;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_CaptureCloud;
};