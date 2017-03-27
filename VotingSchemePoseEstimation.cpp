#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include "VotingSchemePoseEstimation_Class.h"

VotingSchemePoseEstimationClass::VotingSchemePoseEstimationClass()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud (new pcl::PointCloud<pcl::PointXYZ>);
	SceneCloud = sceneCloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling_SceneCloud (new pcl::PointCloud<pcl::PointXYZ>);
	Downsampling_SceneCloud = downsampling_SceneCloud;

	pcl::PointCloud<pcl::Normal>::Ptr sceneNormal (new pcl::PointCloud<pcl::Normal>);
	SceneNormal = sceneNormal;

	pcl::PointCloud<pcl::Normal>::Ptr downsampling_SceneNormal (new pcl::PointCloud<pcl::Normal>);
	Downsampling_SceneNormal = downsampling_SceneNormal;

	pcl::PointCloud<pcl::Boundary>::Ptr sceneBoundaryInf (new pcl::PointCloud<pcl::Boundary>);
	SceneBoundaryInf = sceneBoundaryInf;

	pcl::PointCloud<pcl::PointXYZ>::Ptr sceneBoundaryCloud (new pcl::PointCloud<pcl::PointXYZ>);
	SceneBoundaryCloud = sceneBoundaryCloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr sceneReferenceCloud (new pcl::PointCloud<pcl::PointXYZ>);
	SceneReferenceCloud = sceneReferenceCloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr sceneSegmentationCloud (new pcl::PointCloud<pcl::PointXYZ>);
	SceneSegmentationCloud = sceneSegmentationCloud;



}

pcl::PointCloud<pcl::PointXYZ>::Ptr VotingSchemePoseEstimationClass::getSceneSegmentationCloud()
{
	return SceneSegmentationCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VotingSchemePoseEstimationClass::getSceneCloud()
{
	return SceneCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VotingSchemePoseEstimationClass::getDownsampling_SceneCloud()
{
	return Downsampling_SceneCloud;
}

pcl::PointCloud<pcl::Normal>::Ptr VotingSchemePoseEstimationClass::getSceneNormal()
{
	return SceneNormal;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VotingSchemePoseEstimationClass::getSceneBoundaryCloud()
{
	return SceneBoundaryCloud;
}

pcl::PointCloud<pcl::Boundary>::Ptr VotingSchemePoseEstimationClass::getSceneBoundaryInf()
{
	return SceneBoundaryInf;
}


pcl::PointCloud<pcl::Normal>::Ptr  VotingSchemePoseEstimationClass::getDownsampling_SceneNormal()
{
	return Downsampling_SceneNormal;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr VotingSchemePoseEstimationClass::getSceneReferenceCloud()
{
	return SceneReferenceCloud;
}


Eigen::Matrix4f VotingSchemePoseEstimationClass::getPickObject_Mat()
{
	return PickObject_Mat;
}