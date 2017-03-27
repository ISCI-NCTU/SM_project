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
#ifndef _PCDFUNCTION_H
#define _PCDFUNCTION_H
#include <cmath>
#include "KinectClass.h"
#include <cfloat>
#include <iostream>
#include <XnCppWrapper.h>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <iomanip>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
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
#include <boost\thread\thread.hpp>
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
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/ppf.h>
#include <pcl/common/transforms.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/passthrough.h>
const float D2P = 3.14159 / 180;
typedef std::pair<std::string, std::vector<float> > vfh_model;


class PositionData 
{
public:
	float TCP_Position[6];
	Eigen::Matrix< float, 4, 1 > CameraPoint[6];
	int Target;
	float BaseObject_EulerAngle[3];
	pcl::PointXYZ Arm_PickPoint;
	int IndexOfOriginalPosition[5];
	int NoneZeroPosition;
};



#define SIGN(A)  ( (A) >= 0 ? 1: 0 )
#endif



void DisplyXYZPCDbyXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, const float *point_color, const float *background_color, int show_mode);
void DisplyXYZPCD_ReferencePCDbyXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &reference_cloud, const float *point_color,  const float *reference_color, const float *background_color);
void DisplyPCDNormalbyXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,pcl::PointCloud<pcl::Normal>::Ptr &input_normalCloud, const float *point_color, const float *normal_color,const float *background_color);
void ComputeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::Normal>::Ptr &output_normalCloud, float neighbor_radius);
void ComputeBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::Normal>::Ptr &input_normalCloud, pcl::PointCloud<pcl::Boundary>::Ptr &output_boundaryInf, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_boundaryCloud, float neighbor_radius);
void ply_mesh2pcd(char *ply_name, char *pcd_name);
void stl2pcd(char *stl_name, char *pcd_name);
void yml2pcd(char *yml_name, char *pcd_name,  KinectClass kinect_obj, pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, float Segmentation_Range[][2], int auto_type, int show_mode);
void LoadPCD(char *pcd_name, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);
void voxelGrid_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, float voxel);
void selectReferencePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, std::vector<int> &referencePoint_indices);
void estiblish_CADModelPointCloud( char *OriginalCADModel_Name, char *SaveCADModel_Name, float radius, float voxelGrid_Value, int is_Boundary, int show_mode);
void computePPF_TrainingModel(char *CADModel_Name, char *CADModel_HashMapName, float radius);
void compute_PPFNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &output_normalCloud, float neighbor_radius);
void compute_VotingEstimation_OffinePhase(int CADModel_Number, char **CADModel_FileName, float radius, float HashMapSearch_Position, float HashMapSearch_Rotation);
void compute_VotingEstimation_OnlinePhase( boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr &recognize_Scene, std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > &original_ObjectCAD,int CADModel_Number, float radius, float Clustter_Position, float Cluster_Rotation, float SamplingRate, pcl::PointXYZ &Arm_PickPoint, float TCP_Position[6], float BaseObject_EulerAngle[3],int &Grasp_ObjectType, bool &_IsPoseEstimationDonet);
void compute_SACSegmentationFromNormals( pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, float radius, int show_mode);
void computeIterativeClosestPoint( pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &Target_cloud, Eigen::Matrix4f &result_transformation);
void SavePCD( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, char *pcd_name);
void doExtractCluster_DivideObject( pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > &DivideObject_ClusterPCDResult, int &DivideObject_ClusterNumber);
void compute_VotingEstimation_OnlinePhase_VerifyPrecision( boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr &recognize_Scene, std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > &original_ObjectCAD,int CADModel_Number, float radius, float Clustter_Position, float Cluster_Rotation, float SamplingRate, pcl::PointXYZ &Arm_PickPoint, float TCP_Position[6], bool &_IsPoseEstimationDone, int CAD_Type, pcl::PointCloud<pcl::PointXYZ>::Ptr &recognized_CADModel);
void Base2Camera( pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, float TCP_Position[6], Eigen::Matrix4f &Object_PoseMatrix, float Base_Object_EulerAngle[3], float Camera_X, float Camera_Y, float Camera_Z, pcl::PointXYZ &RealWorld_Point );
void ComputeEulerAngle(Eigen::Matrix4f & PoseMatrix, float EulerAngle[3]);
void Compute_PercentOfOutlier( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_model, float &score, float dis_threshold);
void Find_GraspObject( float TCP_Position[6], std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr >  &determine_GraspObjectICPCloud, Eigen::Matrix< float, 4, 1 > &Camera_ObjectGraspPoint, int &determine_GraspObject );
void Verify_Precision( float TCP_Position[6], pcl::PointCloud<pcl::PointXYZ>::Ptr CADModel_PointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_PointCloud);
void PickPoint_Callback (const pcl::visualization::PointPickingEvent& event, void* viewer_void);
void TransferDatatoMain(PositionData *target, int cmd);
