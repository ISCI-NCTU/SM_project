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
#include "PCL_KinectClass.h"

void OpenNIViewerClass::showCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
	boost::mutex::scoped_lock lock (cloud_mutex_);
	if ( !Cloud_viewer.wasStopped() )
	{
		cloud_ = cloud;
	}
}

void OpenNIViewerClass::showRGBImage( const boost::shared_ptr<openni_wrapper::Image> &image)
{
	 boost::mutex::scoped_lock lock (image_mutex_);
	 image_ = image;
     if (image->getEncoding () != openni_wrapper::Image::RGB)
     {
        if (rgb_data_size_ < image->getWidth () * image->getHeight ())
        {
          if (rgb_data_)
            delete [] rgb_data_;
          rgb_data_size_ = image->getWidth () * image->getHeight ();
          rgb_data_ = new unsigned char [rgb_data_size_ * 3];
        }
        image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);
		
     }
}


void OpenNIViewerClass::run()
{
	//pcl::Grabber* interface = new pcl::OpenNIGrabber();
	//
	//boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>  cloud_cb = boost::bind (&OpenNIViewerClass::showCloud, this, _1);
	//boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&OpenNIViewerClass::showRGBImage, this, _1);
	//boost::signals2::connection cloud_connection = interface->registerCallback (cloud_cb);
	//boost::signals2::connection image_connection = interface->registerCallback (image_cb);
	//bool image_init = false, cloud_init = false;

 //   interface->start ();


	//while (!Cloud_viewer.wasStopped())
 //   {
	//	boost::shared_ptr<openni_wrapper::Image> image;
	//	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;

	//	Cloud_viewer.spinOnce();
 //       if (cloud_mutex_.try_lock ())
 //       {
 //         cloud_.swap (cloud);
 //         cloud_mutex_.unlock ();
 //       }

 //       if (cloud)
 //       {
 //         if (!cloud_init)
 //         {
 //           Cloud_viewer.setPosition (0, 0);
 //           Cloud_viewer.setSize (cloud->width, cloud->height);
 //           cloud_init = !cloud_init;
 //         }

 //         if (!Cloud_viewer.updatePointCloud (cloud, "OpenNICloud"))
 //         {
 //           Cloud_viewer.addPointCloud (cloud, "OpenNICloud");
 //           Cloud_viewer.resetCameraViewpoint ("OpenNICloud");
 //         }          
 //       }

 //       // See if we can get an image
 //       if (image_mutex_.try_lock ())
 //       {
 //         image_.swap (image);
 //         image_mutex_.unlock ();
 //       }

 //       if (image)
 //       {
 //         if (!image_init && cloud && cloud->width != 0)
 //         {
 //           RGB_Viewer.setPosition (cloud->width, 0);
 //           RGB_Viewer.setSize (cloud->width, cloud->height);
 //           image_init = !image_init;
 //         }

 //         if (image->getEncoding() == openni_wrapper::Image::RGB)
	//		  RGB_Viewer.addRGBImage (image->getMetaData ().Data (), image->getWidth (), image->getHeight ());
 //         else
	//		  RGB_Viewer.addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
 //         RGB_Viewer.spinOnce ();
 //       }
 //   }

 //   interface->stop ();

	//cloud_connection.disconnect ();
 //   image_connection.disconnect ();
	//if (rgb_data_)
 //       delete[] rgb_data_;
	if ( init_state )
	{
		interface->start ();
		while (!Cloud_viewer.wasStopped())
		{
			boost::shared_ptr<openni_wrapper::Image> image;
			pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;

			Cloud_viewer.spinOnce();
			if (cloud_mutex_.try_lock ())
			{
				cloud_.swap (cloud);
				cloud_mutex_.unlock ();
			}

			if (cloud)
			{
				if (!cloud_init)
				{
				Cloud_viewer.setPosition (0, 0);
				Cloud_viewer.setSize (cloud->width, cloud->height);
				cloud_init = !cloud_init;
				}

				if (!Cloud_viewer.updatePointCloud (cloud, "OpenNICloud"))
				{
				Cloud_viewer.addPointCloud (cloud, "OpenNICloud");
				Cloud_viewer.resetCameraViewpoint ("OpenNICloud");
				}          
			}

				//See if we can get an image
			if (image_mutex_.try_lock ())
			{
				image_.swap (image);
				image_mutex_.unlock ();
			}

			if (image)
			{
				if (!image_init && cloud && cloud->width != 0)
				{
				RGB_Viewer.setPosition (cloud->width, 0);
				RGB_Viewer.setSize (cloud->width, cloud->height);
				image_init = !image_init;
				}

				if (image->getEncoding() == openni_wrapper::Image::RGB)
					RGB_Viewer.addRGBImage (image->getMetaData ().Data (), image->getWidth (), image->getHeight ());
				else
					RGB_Viewer.addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
				RGB_Viewer.spinOnce ();
			}
		}

		interface->stop ();
		if (rgb_data_)
			delete[] rgb_data_;
	}
}


void OpenNIViewerClass::init()
{
	if ( !init_state )
	{
		cloud_cb = boost::bind (&OpenNIViewerClass::showCloud, this, _1);
		image_cb = boost::bind (&OpenNIViewerClass::showRGBImage, this, _1);
		cloud_connection = interface->registerCallback (cloud_cb);
		image_connection = interface->registerCallback (image_cb);
		init_state = true;
		cout << "OpenNIViewerClass : Kinect init done!! \n";
	}
	else
	{
		cout << "OpenNIViewerClass : Kinect has already inited!!! \n";
	}
}

void OpenNIViewerClass::capture_oneScene()
{
	if ( init_state )
	{
		interface->start ();

		boost::shared_ptr<openni_wrapper::Image> image;
		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;

		Cloud_viewer.spinOnce();
		if (cloud_mutex_.try_lock ())
		{
			cloud_.swap (cloud);
			cloud_mutex_.unlock ();
		}

		if (cloud)
		{
			if (!cloud_init)
			{
			Cloud_viewer.setPosition (0, 0);
			Cloud_viewer.setSize (cloud->width, cloud->height);
			cloud_init = !cloud_init;
			}

			if (!Cloud_viewer.updatePointCloud (cloud, "OpenNICloud"))
			{
			Cloud_viewer.addPointCloud (cloud, "OpenNICloud");
			Cloud_viewer.resetCameraViewpoint ("OpenNICloud");
			}          
		}

			//See if we can get an image
		if (image_mutex_.try_lock ())
		{
			image_.swap (image);
			image_mutex_.unlock ();
		}

		if (image)
		{
			if (!image_init && cloud && cloud->width != 0)
			{
			RGB_Viewer.setPosition (cloud->width, 0);
			RGB_Viewer.setSize (cloud->width, cloud->height);
			image_init = !image_init;
			}

			if (image->getEncoding() == openni_wrapper::Image::RGB)
				RGB_Viewer.addRGBImage (image->getMetaData ().Data (), image->getWidth (), image->getHeight ());
			else
				RGB_Viewer.addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
			RGB_Viewer.spinOnce ();
		}

		pcl::copyPointCloud( *cloud, *Kinect_CaptureCloud);
		
		interface->stop ();
		if (rgb_data_)
			delete[] rgb_data_;
	}


}