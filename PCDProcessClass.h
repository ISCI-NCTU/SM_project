//#undef max 
//#undef min

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <cmath>
#include <cfloat>
#include <iostream>
#include <pcl\features\boundary.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\keypoints\uniform_sampling.h>
#include <pcl\features\fpfh.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\features\normal_3d.h>
#include <pcl\visualization\point_cloud_color_handlers.h>
#include <pcl\visualization\pcl_visualizer.h>
#include <pcl\visualization\histogram_visualizer.h>
#include <pcl\visualization\point_picking_event.h>
#include <Eigen/Geometry>
#include <boost/thread/thread.hpp>
#include <pcl\io\io.h>
#include <pcl\io\pcd_io.h>
#include <pcl\common\common.h>
#include <pcl\search\search.h>
#include <pcl\kdtree\kdtree.h>
#include <pcl\octree\octree.h>
#include <pcl\console\parse.h>
#include <pcl\console\print.h>
#include <pcl\console\time.h>
#include <pcl\range_image\range_image.h>
#include <pcl\impl\point_types.hpp>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZ PointT;

class PCDProcessClass
{
	private:
		pcl::PointCloud< pcl::PointXYZ>::Ptr CAD_PCData;
		pcl::PointCloud< pcl::PointXYZ>::Ptr CAD_ResamplePCData;
		pcl::PointCloud<pcl::Normal>::Ptr CAD_Normal;
		pcl::PointCloud<pcl::Boundary>::Ptr CAD_Boundary;
		pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_BoundaryPointXYZ;

	public:
		float **pairFeatures;
		int ***HashTable;
		int *HashTable_counter;
		pcl::PointCloud< pcl::PointXYZ>::Ptr getCAD_ResamplePCData();
		pcl::PointCloud<pcl::Normal>::Ptr getCAD_Normal();
		pcl::PointCloud<PointT>::Ptr getCAD_PCData();
		pcl::PointCloud<pcl::Boundary>::Ptr getCAD_Boundary();
		pcl::PointCloud<pcl::PointXYZ>::Ptr getCAD_BoundaryPointXYZ();

		void uniform_DownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, float voxel);
		void voxelGrid_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, float voxel);
		void showCAD_Boundary(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_BoundaryCloud);
		void computePairFeature(pcl::PointCloud<PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::Normal>::Ptr &input_normalCloud, float threshold_dist, float threshold_angle);
		void computeBoundary(pcl::PointCloud<PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::Normal>::Ptr &input_normalCloud, pcl::PointCloud<pcl::Boundary>::Ptr &output_boundaryCloud, pcl::PointCloud<PointXYZ>::Ptr &output_boundaryPointXYZ);
		PCDProcessClass();
};

//bool computePairFeatures (const Eigen::Vector4f &p1, const Eigen::Vector4f &n1, 
                          //const Eigen::Vector4f &p2, const Eigen::Vector4f &n2,
                          //float &f1, float &f2, float &f3, float &f4);

//int searchHashTable(int p, float** float_ptr, float* float_compare);

//float myself_round(float p, int order);