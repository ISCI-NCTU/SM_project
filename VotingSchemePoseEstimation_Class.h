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