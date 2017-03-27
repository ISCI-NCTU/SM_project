#include "PCD_Function.h"

class VotingSchemePoseEstimationClass
{
	private:
		pcl::PointCloud<pcl::PointXYZ>::Ptr SceneCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr SceneSegmentationCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr Downsampling_SceneCloud;
		pcl::PointCloud<pcl::Normal>::Ptr SceneNormal;
		pcl::PointCloud<pcl::Normal>::Ptr Downsampling_SceneNormal;
		pcl::PointCloud<pcl::Boundary>::Ptr SceneBoundaryInf;
		pcl::PointCloud<pcl::PointXYZ>::Ptr SceneBoundaryCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr SceneReferenceCloud;
		Eigen::Matrix4f PickObject_Mat;
		
		
	public:
		VotingSchemePoseEstimationClass();
		pcl::PointCloud<pcl::PointXYZ>::Ptr getSceneCloud();
		pcl::PointCloud<pcl::PointXYZ>::Ptr getDownsampling_SceneCloud();
		pcl::PointCloud<pcl::Normal>::Ptr getDownsampling_SceneNormal();
		pcl::PointCloud<pcl::Normal>::Ptr getSceneNormal();
		pcl::PointCloud<pcl::PointXYZ>::Ptr getSceneBoundaryCloud();
		pcl::PointCloud<pcl::Boundary>::Ptr getSceneBoundaryInf();
		pcl::PointCloud<pcl::PointXYZ>::Ptr getSceneReferenceCloud();
		pcl::PointCloud<pcl::PointXYZ>::Ptr getSceneSegmentationCloud();
		std::vector<int> referencePoint_indices;
		Eigen::Matrix4f getPickObject_Mat();
};