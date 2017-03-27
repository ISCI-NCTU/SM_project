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