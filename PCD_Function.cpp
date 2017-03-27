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
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include "PCD_Function.h"
#include <pcl/features/principal_curvatures.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/registration/impl/ppf_registration.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/cvfh.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <flann/flann.h>
#include <string.h>

#define SAME 1
#define NoSAME 0

PositionData PositionALL;
PositionData Position1;
PositionData Position2;
PositionData Position3;
PositionData Position4;
PositionData Position5;

int objectNum;
int FinalPositionIndex[5];


void TransferDatatoMain(PositionData *target, int cmd)
{
	if(cmd == 1)
	{
		*target = Position1;
	}
	else if(cmd == 2)
	{
		*target = Position2;
	}
	else if(cmd == 3)
	{
		*target = Position3;
	}
	else if(cmd == 4)
	{
		*target = Position4;
	}
	else
	{
		*target = Position5;
	}
}

void DisplyXYZPCDbyXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, const float *point_color, const float *background_color, int show_mode)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("OriginalPCD_XYZRGB"));
	viewer->setBackgroundColor (background_color[0], background_color[1], background_color[2]);
    viewer->addPointCloud<pcl::PointXYZ> (input_cloud,"cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, point_color[0], point_color[1], point_color[2], "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
	

	//viewer->addLine<pcl::PointXYZ> ( input_cloud->points[75], input_cloud->points[291], 255, 0, 0, "line_1");
	//viewer->addLine<pcl::PointXYZ> ( input_cloud->points[87], input_cloud->points[315], 255, 0, 0, "line_2");
	//viewer->addLine<pcl::PointXYZ> ( input_cloud->points[76], input_cloud->points[284], 255, 0, 0, "line_3");
	//viewer->addLine<pcl::PointXYZ> ( input_cloud->points[79], input_cloud->points[303], 255, 0, 0, "line_4");
	//viewer->addLine<pcl::PointXYZ> ( input_cloud->points[88], input_cloud->points[290], 255, 0, 0, "line_5");
	//viewer->addLine<pcl::PointXYZ> ( input_cloud->points[88], input_cloud->points[302], 255, 0, 0, "line_6");
	//viewer->addLine<pcl::PointXYZ> ( input_cloud->points[91], input_cloud->points[321], 255, 0, 0, "line_7");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "cloud");


	viewer->initCameraParameters ();
	if ( show_mode == 1 )
	{
		while (!viewer->wasStopped ()) {
			viewer->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
	}
	else if ( show_mode == 0 )
	{
		//viewer->spinOnce (500);
		//boost::this_thread::sleep (boost::posix_time::microseconds (1000000));
	}
}


void DisplyXYZPCD_ReferencePCDbyXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &reference_cloud, const float *point_color, const float *reference_color, const float *background_color)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("OriginalPCD_XYZRGB"));
	viewer->setBackgroundColor (background_color[0], background_color[1], background_color[2]);
    viewer->addPointCloud<pcl::PointXYZ> (input_cloud,"cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, point_color[0], point_color[1], point_color[2], "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

	viewer->addPointCloud<pcl::PointXYZ> (reference_cloud,"reference_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, reference_color[0], reference_color[1], reference_color[2], "reference_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "reference_cloud");

	viewer->initCameraParameters ();
    while (!viewer->wasStopped ()) {
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}

void DisplyPCDNormalbyXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,pcl::PointCloud<pcl::Normal>::Ptr &input_normalCloud, const float *point_color, const float *normal_color, const float *background_color)
{
	//Where "property" can be only: 
	//pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
	//pcl::visualization::PCL_VISUALIZER_OPACITY, 
	//pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
	//pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 
	//pcl::visualization::PCL_VISUALIZER_COLOR 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("OriginalPCDNormal_XYZRGB"));

    viewer->setBackgroundColor (background_color[0], background_color[1], background_color[2]);
    viewer->addPointCloud<pcl::PointXYZ> (input_cloud,"sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, point_color[0], point_color[1], point_color[2], "sample cloud");


	float nomal_length = 0.03;
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (input_cloud, input_normalCloud, 10, nomal_length, "normal cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, normal_color[1], normal_color[2], normal_color[3], "normal cloud");

    viewer->initCameraParameters ();
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}


// if radius = 0.03 ， 0.03 is 3cm
void ComputeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::Normal>::Ptr &output_normalCloud, float neighbor_radius)
{
	cout << "ComputeNormals : Create a NormalsEstimation.\n";
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud( input_cloud );

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	ne.setRadiusSearch(neighbor_radius);
	cout << "ComputeNormals : Computing nomals of input_cloud, waitting for a moment...\n";
	ne.compute(*output_normalCloud);
	cout << "ComputeNormals : Done.\n";

	// output_normalCloud->points.size () should have the same size as the input input_cloud->points.size ()*
}




void LoadPCD(char *pcd_name, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud)
{
	float background_color[3] = { 0, 0, 0 };
	float point_color[3] = { 255, 255, 255 };

	output_cloud->clear();
	cout << "LoadPCD : Loading " << pcd_name << " to PointXYZ of pcd type...\n";
	pcl::io::loadPCDFile(pcd_name, *output_cloud);
	cout << "LoadPCD : Size = " << output_cloud->size() << ".\n";
	//cout << "LoadPCD : Show Point Cloud!\n";
	//DisplyXYZPCDbyXYZRGB( output_cloud, point_color, background_color);
	cout << "LoadPCD : Done!!!" << endl; 

}

void SavePCD( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, char *pcd_name)
{
	cout << "SavePCD : Saving " << pcd_name << " to PointXYZ of pcd type...\n";
	pcl::io::savePCDFile (pcd_name, *input_cloud);
	cout << "SavePCD : Done!!!" << endl; 
}


void ply_mesh2pcd(char *ply_name, char *pcd_name)
{
	cout << "ply_mesh2pcd : Creating polydata!!\n";
	vtkSmartPointer<vtkPolyData> polydata;
	vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New ();
	cout << "ply_mesh2pcd : Loading " << ply_name << " in polydata!!\n"; 
	readerQuery->SetFileName (ply_name);
	polydata = readerQuery->GetOutput ();
	polydata->Update ();

	cout << "ply_mesh2pcd : Converting poly_data to pointcloud_data, wait for a moment....\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->clear();
	pcl::PointXYZ temp_point;
	double temp_double[3] = {0};
	for (size_t i = 0; i <  polydata->GetNumberOfPoints(); i++)
	{
		polydata->GetPoint(i,temp_double);
		temp_point.x = temp_double[0];
		temp_point.y = temp_double[1];
		temp_point.z = temp_double[2];
		cloud->push_back( temp_point );
	}

	cout << "ply_mesh2pcd : Save pointcloud_data in " << pcd_name << "\n";
	pcl::io::savePCDFile (pcd_name, *cloud);
}

void stl2pcd(char *stl_name, char *pcd_name)
{
	cout << "stl_mesh2pcd : Creating polydata!!\n";
	vtkSmartPointer<vtkPolyData> polydata;
	vtkSmartPointer<vtkSTLReader> readerQuery = vtkSmartPointer<vtkSTLReader>::New ();
	cout << "stl_mesh2pcd : Loading " << stl_name << " in polydata!!\n"; 
	readerQuery->SetFileName (stl_name);
	polydata = readerQuery->GetOutput ();
	polydata->Update ();

	cout << "stl_mesh2pcd : Converting poly_data to pointcloud_data, wait for a moment....\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	int counter = polydata->GetNumberOfPoints();
	cloud->clear();
	pcl::PointXYZ temp_point;
	double temp_double[3] = {0};
	for (size_t i = 0; i <  polydata->GetNumberOfPoints(); i++)
	{
		polydata->GetPoint(i,temp_double);
		temp_point.x = temp_double[0];
		temp_point.y = temp_double[1];
		temp_point.z = temp_double[2];
		cloud->push_back( temp_point );
	}

	cout << "stl_mesh2pcd : Save pointcloud_data in " << pcd_name << "\n";
	pcl::io::savePCDFile (pcd_name, *cloud);
}


void yml2pcd(char *yml_name, char *pcd_name,  KinectClass kinect_obj, pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, float Segmentation_Range[][2], int auto_type, int show_mode)
{
	input_cloud->clear();
	cout << "yml2pcd : yml2realwordlByKinect!!\n";
	kinect_obj.yml2realwordlByKinect(yml_name);
	
	std::stringstream out_yml_filename;
	cv::Mat Temp_Depth;
	cv::Mat Read_Depth;
	cv::Mat c16BitDepth;
	out_yml_filename.str( yml_name ); 
	cv::FileStorage fsd( out_yml_filename.str(), cv::FileStorage::READ);		
	fsd["frameCount"] >> c16BitDepth;		
	c16BitDepth.convertTo( Read_Depth, CV_32FC1);

	cout << "yml2pcd : Converting .yml to .pcd<PointXYZ>, wait for a moment...\n";
	pcl::PointXYZ temp_point;
	int idx = 0, idxShift = 0;
	

	XnPoint3D* pDepthPointSet = new XnPoint3D[ Read_Depth.cols * Read_Depth.rows];
	for ( int j = 0; j < Read_Depth.rows; ++j )
	{
		idxShift = j *  Read_Depth.cols;
		for ( int i = 0; i <   Read_Depth.cols; ++i)
		{
				idx = idxShift + i;
				pDepthPointSet[idx].X = (int)i;
				pDepthPointSet[idx].Y = (int)j;
				pDepthPointSet[idx].Z = (int)Read_Depth.at<float>(idx);		
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	char modify_mode;
	float background_color[3] = { 0, 0, 0 };
	float point_color[3] = { 255, 255, 255 };


	if ( auto_type == 0 )
	{
		float Segmentation_Range[3][2] = {0};
		while (true)
		{
		
			Temp_Depth.release();
			c16BitDepth.convertTo( Temp_Depth, CV_32FC1);

			save_cloud->clear();
			cout << "yml2pcd : Key in PassThroughFilter_Range modify mode : \n";
			cin >> modify_mode;

			switch (modify_mode)
			{
				case 'a':
					cout << "yml2pcd : Key in PassThroughFilter_Range XYZ range : \n";
					cin >> Segmentation_Range[0][0];
					cin >> Segmentation_Range[0][1];
					cin >> Segmentation_Range[1][0];
					cin >> Segmentation_Range[1][1];
					cin >> Segmentation_Range[2][0];
					cin >> Segmentation_Range[2][1];
					break;
				case 'x':
					cout << "yml2pcd : Key in PassThroughFilter_Range X range : \n";
					cin >> Segmentation_Range[0][0];
					cin >> Segmentation_Range[0][1];
					break;
				case 'y':
					cout << "yml2pcd : Key in PassThroughFilter_Range Y range : \n";
					cin >> Segmentation_Range[1][0];
					cin >> Segmentation_Range[1][1];
					break;
				case 'z' :
					cout << "yml2pcd : Key in PassThroughFilter_Range Z range : \n";
					cin >> Segmentation_Range[2][0];
					cin >> Segmentation_Range[2][1];
					break;
				default :
					break;
			}

			for (int y = 0; y < Read_Depth.rows; y++)
			{
				idxShift = y * Read_Depth.cols;
				for (int x = 0; x < Read_Depth.cols; x++)
				{

					idx = idxShift + x;
					if ( ( pDepthPointSet[idx].X  >  Segmentation_Range[0][0] && pDepthPointSet[idx].X  <  Segmentation_Range[0][1] ) &&
						 ( pDepthPointSet[idx].Y  >  Segmentation_Range[1][0] && pDepthPointSet[idx].Y  <  Segmentation_Range[1][1] ) &&
						 ( pDepthPointSet[idx].Z  >  Segmentation_Range[2][0] && pDepthPointSet[idx].Z  <  Segmentation_Range[2][1] ) )
					{

						temp_point.x = (( kinect_obj.pRealWorldCamaraPointSet + idx )->X);
						temp_point.y = (( kinect_obj.pRealWorldCamaraPointSet + idx )->Y);
						temp_point.z = (( kinect_obj.pRealWorldCamaraPointSet + idx )->Z);
						//save_cloud->push_back( temp_point );
						input_cloud->push_back( temp_point );

					}
					else
					{

						Temp_Depth.at<float>(idx) = 0.0;
					}
				}
			}
			cout << "yml2pcd : Save point cloud data size :  " << save_cloud->size() << "\n";	
			if (  modify_mode != 'a' && modify_mode != 'x' && modify_mode != 'y' && modify_mode != 'z' )
			{
				pcl::io::savePCDFile<pcl::PointXYZ> (pcd_name, *save_cloud );
				cout << "yml2pcd : Save point cloud data as " << pcd_name << ", please wait for a moment...\n";
				cout << "yml2pcd : Save point cloud data size :  " << save_cloud->size() << "\n";
				cout << "yml2pcd : done!!!" << endl;
				break;
			}
		}

	}
	else if( auto_type == 1 )
	{
		Temp_Depth.release();
		c16BitDepth.convertTo( Temp_Depth, CV_32FC1);

		for (int y = 0; y < Read_Depth.rows; y++)
		{
			idxShift = y * Read_Depth.cols;
			for (int x = 0; x < Read_Depth.cols; x++)
			{

				idx = idxShift + x;
				if ( ( pDepthPointSet[idx].X  >  Segmentation_Range[0][0] && pDepthPointSet[idx].X  <  Segmentation_Range[0][1] ) &&
					 ( pDepthPointSet[idx].Y  >  Segmentation_Range[1][0] && pDepthPointSet[idx].Y  <  Segmentation_Range[1][1] ) &&
					 ( pDepthPointSet[idx].Z  >  Segmentation_Range[2][0] && pDepthPointSet[idx].Z  <  Segmentation_Range[2][1] ) )
				{
				
						temp_point.x = (( kinect_obj.pRealWorldCamaraPointSet + idx )->X);
						temp_point.y = (( kinect_obj.pRealWorldCamaraPointSet + idx )->Y);
						temp_point.z = (( kinect_obj.pRealWorldCamaraPointSet + idx )->Z);

						//save_cloud->push_back( temp_point );
						input_cloud->push_back( temp_point );

				}
				else
				{
					Temp_Depth.at<float>(idx) = 0.0;
				}
			}
		}
		//pcl::io::savePCDFile<pcl::PointXYZ> (pcd_name, *save_cloud );
		cout << "yml2pcd : Save point cloud data as " << pcd_name << ", please wait for a moment...\n";
		cout << "yml2pcd : Save point cloud data size :  " << save_cloud->size() << "\n";
		cout << "yml2pcd : done!!!" << endl;

	}

}


void voxelGrid_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, float voxel)
{
	cout << "voxelGrid_Filter : Original PCD Size = " << input_cloud->size() << ".\n";
	cout << "voxelGrid_Filter : Downsampling using voxelGrid_Filter...\n";
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setInputCloud( input_cloud );
	voxel_grid.setLeafSize( voxel, voxel, voxel);
	voxel_grid.filter( *output_cloud );
	cout << "voxelGrid_Filter : Downsampling PCD Size = " << output_cloud->size() << ".\n";;
}

void ComputeBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::Normal>::Ptr &input_normalCloud, pcl::PointCloud<pcl::Boundary>::Ptr &output_boundaryInf,  pcl::PointCloud<pcl::PointXYZ>::Ptr &output_boundaryCloud,float neighbor_radius)
{
	cout << "ComputeBoundary : Create a BoundaryEstimation.\n";
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	est.setInputCloud( input_cloud );
	est.setInputNormals( input_normalCloud );
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	est.setSearchMethod (tree);
	est.setRadiusSearch ( neighbor_radius );

	cout << "ComputeBoundary : Computing BoundaryEdge of input_cloud, waitting for a moment...\n";
	est.compute( *output_boundaryInf );

	cout << "ComputeBoundary : Converting boundaryInf to boundayCloud, waitting for qa moment...\n";
	for (size_t i = 0; i < input_cloud->size(); ++i)
	{
		if (output_boundaryInf->points[i].boundary_point == 1)
		{
			output_boundaryCloud->push_back(input_cloud->points[i]);
		}
	}
	cout << "ComputeBoundary : Size = " << output_boundaryCloud->size() << ".\n";
	cout << "ComputeBoundary : Done.\n";
}


void selectReferencePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, std::vector<int> &referencePoint_indices)
{
	cout <<  "selectReferencePoints : Create a random_sample filter.\n";
	pcl::RandomSample<pcl::PointXYZ> random_sample(true);
	random_sample.setInputCloud( input_cloud );
	random_sample.setSample( ( input_cloud->size() / 10 )); // 1/5 th

	cout <<  "selectReferencePoints : Computing...., waitting for a moment...\n";
	random_sample.filter ( referencePoint_indices );
	random_sample.setSeed( rand() );
	pcl::PointCloud<pcl::PointXYZ> temp_cloud;
	random_sample.filter( temp_cloud );

	cout << "selectReferencePoints : The size of output_cloud = " << temp_cloud.size();
	cout << "selectReferencePoints : \nThe size of input_cloud = " << input_cloud->size() << '\n';
	cout << "selectReferencePoints : Copy Reference Point of input_cloud to output_cloud, waittin for a moment...\n";

	for (int i = 0; i < referencePoint_indices.size() - 1; i++)
	{
		cout << "indices = " << referencePoint_indices.at(i) << '\n';
		output_cloud->push_back( input_cloud->points[referencePoint_indices.at(i)] );
	}
	cout << "selectReferencePoints : The number of referenct point = " << referencePoint_indices.size() << "\n";
	cout << "selectReferencePoints : Done.\n";

}


void compute_PPFNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &output_normalCloud, float neighbor_radius)
{
	cout << "ComputeNormals : Create a NormalsEstimation.\n";
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud( input_cloud );

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	ne.setRadiusSearch(neighbor_radius);
	cout << "ComputeNormals : Computing nomals of input_cloud, waitting for a moment...\n";

	pcl::PointCloud< pcl::Normal >::Ptr cloud_subsampled_normals (new pcl::PointCloud< pcl::Normal > ());
	ne.compute(*cloud_subsampled_normals);
	cout << "computing cloud_subsampled_normals \n";
	concatenateFields (*input_cloud, *cloud_subsampled_normals, *output_normalCloud);
	cout << "concatenateFields \n";
	cout << "ComputeNormals : Done.\n";

	// output_normalCloud->points.size () should have the same size as the input input_cloud->points.size ()*
}

void estiblish_CADModelPointCloud( char *OriginalCADModel_Name, char *SaveCADModel_Name, float radius, float voxelGrid_Value, int is_Boundary, int show_mode)
{
	cout << "estiblish_CADModelPointCloud : Creating temp data....\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr Original_CADModel_PointCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Original_CADModel_DownsamplingCloud(new pcl::PointCloud<pcl::PointXYZ>);
	float background_color[3] = { 0, 0, 0 };
	float point_color[3] = { 255, 0, 0 };

	cout << "estiblish_CADModelPointCloud : Loading Original CADModel PCD...\n";
	LoadPCD( OriginalCADModel_Name, Original_CADModel_PointCloud);

	if ( is_Boundary == 1)
	{
		pcl::PointCloud<pcl::Normal>::Ptr Original_CADModel_PointCloud_Nromal (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::Boundary>::Ptr Original_CADModel_BoundaryInf (new pcl::PointCloud<pcl::Boundary>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr Original_CADModel_BoundaryCloud (new pcl::PointCloud<pcl::PointXYZ>);


		cout << "estiblish_CADModelPointCloud : Downsampling by voxel grid filter.\n";
		voxelGrid_Filter(  Original_CADModel_PointCloud, Original_CADModel_DownsamplingCloud, voxelGrid_Value);
		cout << "estiblish_CADModelPointCloud : Estimate normals of point cloud.\n";
		ComputeNormals(  Original_CADModel_DownsamplingCloud, Original_CADModel_PointCloud_Nromal, radius);
		cout << "estiblish_CADModelPointCloud : Estimate boundary of point cloud.\n";
		ComputeBoundary( Original_CADModel_DownsamplingCloud, Original_CADModel_PointCloud_Nromal,  Original_CADModel_BoundaryInf, Original_CADModel_BoundaryCloud, radius); 
		
		cout << "estiblish_CADModelPointCloud : Save result CADModel_PCD.\n";
		pcl::io::savePCDFile<pcl::PointXYZ> (SaveCADModel_Name, *Original_CADModel_BoundaryCloud );
		cout << "estiblish_CADModelPointCloud : Show result CADModel_PCD.\n";
		DisplyXYZPCDbyXYZRGB( Original_CADModel_BoundaryCloud, point_color,  background_color, show_mode);
		cout << "estiblish_CADModelPointCloud : Done!!!\n";
	}
	else
	{
		cout << "estiblish_CADModelPointCloud : Downsampling by voxel grid filter.\n";
		voxelGrid_Filter(  Original_CADModel_PointCloud, Original_CADModel_DownsamplingCloud, voxelGrid_Value);
		cout << "estiblish_CADModelPointCloud : Save result CADModel_PCD.\n";
		pcl::io::savePCDFile<pcl::PointXYZ> (SaveCADModel_Name, *Original_CADModel_DownsamplingCloud );
		cout << "estiblish_CADModelPointCloud : Show result CADModel_PCD.\n";
		DisplyXYZPCDbyXYZRGB( Original_CADModel_DownsamplingCloud, point_color,  background_color, show_mode);
		cout << "estiblish_CADModelPointCloud : Done!!!\n";
	}
}

void computePPF_TrainingModel(char *CADModel_Name, char *CADModel_HashMapName, float radius)
{
	cout << "computePPF_TrainingModel : Loding CADModel Data to PointCloud ...\n";
	pcl::PointCloud< pcl::PointNormal >::Ptr cloud_model_normal(new pcl::PointCloud<pcl::PointNormal>);
	 pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_model_input(new pcl::PointCloud<pcl::PointXYZ>);
	LoadPCD( CADModel_Name, cloud_model_input );
	cout << "computePPF_TrainingModel : Estimate normals of point cloud.\n\n";
	compute_PPFNormals( cloud_model_input, cloud_model_normal, radius);
	

	cout << "computePPF_TrainingModel : Training model ...\n";
    pcl::PointCloud< pcl::PPFSignature >::Ptr cloud_model_ppf (new pcl::PointCloud<pcl::PPFSignature> ());
    pcl::PPFEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature> ppf_estimator;
    ppf_estimator.setInputCloud (cloud_model_normal);
    ppf_estimator.setInputNormals (cloud_model_normal);
    ppf_estimator.compute (*cloud_model_ppf);


	cout << "computePPF_TrainingModel : Write HashMap ...\n";
	cout << "computePPF_TrainingModel : cloud_model_ppf->size() : " << cloud_model_ppf->size() << '\n';
	FILE * writeHashMap;
	writeHashMap = fopen( CADModel_HashMapName, "w" );
	fprintf( writeHashMap, "%d\n", cloud_model_ppf->size() );
	for (int i = 0; i < cloud_model_ppf->size(); i++)
	{
		fprintf( writeHashMap, "%f\t%f\t%f\t%f\t%f\n", cloud_model_ppf->at(i).alpha_m,  cloud_model_ppf->at(i).f1,  cloud_model_ppf->at(i).f2,  cloud_model_ppf->at(i).f3,  cloud_model_ppf->at(i).f4 );
	}
	cout << "computePPF_TrainingModel : Done!!\n";
}


vector<pcl::PointCloud<pcl::PointNormal>::Ptr > cloud_models_with_normals;
vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_models;
vector<pcl::PPFHashMapSearch::Ptr> hashmap_search_vector;

void compute_VotingEstimation_OffinePhase(int CADModel_Number, char **CADModel_FileName, float radius, float HashMapSearch_Position, float HashMapSearch_Rotation)
{
	cout << "compute_VotingEstimation_OffinePhase : Reading CADModel PointCloud...\n";
	pcl::PCDReader reader;
	for (int i = 0; i < CADModel_Number; i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> () );

		reader.read( *(CADModel_FileName+i), *cloud);
		cloud_models.push_back(cloud);
		cout << "compute_VotingEstimation_OffinePhase : Model read : " << *(CADModel_FileName+i) << "\n";
	}

	cout << "compute_VotingEstimation_OffinePhase : Training model ...\n";
	
	for (int i = 0; i < CADModel_Number; i++)
	{
		pcl::PointCloud< pcl::PointNormal >::Ptr cloud_model_normal (new pcl::PointCloud<pcl::PointNormal>);
		compute_PPFNormals( cloud_models[i], cloud_model_normal, radius); 
		cloud_models_with_normals.push_back(cloud_model_normal);

		pcl::PointCloud< pcl::PPFSignature >::Ptr cloud_model_ppf (new pcl::PointCloud<pcl::PPFSignature> ());
		pcl::PPFEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature> ppf_estimator;
		ppf_estimator.setInputCloud (cloud_model_normal);
		ppf_estimator.setInputNormals (cloud_model_normal);
		ppf_estimator.compute (*cloud_model_ppf);

		Eigen::Vector4f max_dis;
		pcl::getMaxDistance( *( cloud_models[i] ), cloud_models[i]->at(0).getVector4fMap(), max_dis);
		HashMapSearch_Position = (cloud_models[i]->at(0).getVector4fMap() - max_dis).norm() * 0.05;

		pcl::PPFHashMapSearch::Ptr hashmap_search ( new pcl::PPFHashMapSearch ( HashMapSearch_Rotation / 180.0f * float (M_PI),   HashMapSearch_Position )); //0.05, 12.0
		hashmap_search->setInputFeatureCloud (cloud_model_ppf);
		hashmap_search_vector.push_back(hashmap_search);
	}

	cout << "compute_VotingEstimation_OffinePhase : Done!!\n";
	

}

void compute_SACSegmentationFromNormals( pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, float radius, int show_Mode)
{

	output_cloud->clear();
	cout << "compute_SACSegmentationFromNormals : Computing scene normal.\n";
	pcl::PointCloud<pcl::Normal>::Ptr sceneNormal (new pcl::PointCloud<pcl::Normal>);
	ComputeNormals( input_cloud, sceneNormal, radius);


	cout << "compute_SACSegmentationFromNormals : Create the segmentation object...\n";
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(2.5);//2.5
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (20);//20
	seg.setInputCloud( input_cloud );
	seg.setInputNormals( sceneNormal );
	seg.segment (*inliers, *coefficients);

	cout << "compute_SACSegmentationFromNormals : Extrace result Segmentation Point Cloud...\n";
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setNegative (true);	
	extract.setInputCloud (input_cloud);
	extract.setIndices (inliers);
	extract.filter (*output_cloud);

	cout << "compute_SACSegmentationFromNormals : Show result Segmentation Point Cloud...\n";
	//float background_color[3] = { 0, 0, 0 };
	//float point_color[3] = { 255, 255, 255 };

	cout << "compute_SACSegmentationFromNormals : Done!!!\n";
}

void compute_VotingEstimation_OnlinePhase( boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr &recognize_Scene, std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > &original_ObjectCAD,int CADModel_Number, float radius, float Clustter_Position, float Cluster_Rotation, float SamplingRate, pcl::PointXYZ &Arm_PickPoint, float TCP_Position[6], float BaseObject_EulerAngle[3], int &grasp_objectType, bool &_IsPoseEstimationDonet)
{

	cout << "compute_VotingEstimation_OnlinePhase : Reading the segmentation PointCloud Scene\n";
	viewer->removeAllPointClouds();
    viewer->addPointCloud (cloud_scene);
	viewer->spinOnce (10);


	cout << "compute_VotingEstimation_OnlinePhase : Computing scene normal.\n";
	pcl::PointCloud< pcl::PointNormal >::Ptr recognize_Scene_normal (new pcl::PointCloud<pcl::PointNormal>);
	compute_PPFNormals( recognize_Scene, recognize_Scene_normal, radius); 
	
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > ICP_cloud;
	std::vector< float > Outlier_Score;
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > determine_GraspICP_Cloud;
	std::vector< Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > determine_GraspObjectMat;
	std::vector< int > model_type;

	int SuccessRecognitionCount = 0;

	float point_color[3][3] = { {128, 128, 0},
								{0,  255,  0},
								{255, 0,   0}
							  };
	for (int i = 0; i < CADModel_Number; i++)
	{
		

		cout << "compute_VotingEstimation_OnlinePhase : Creating ppf_registration...\n";
		pcl::PPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf_registration;

		
		// set parameters for the PPF registration procedure
		ppf_registration.setSceneReferencePointSamplingRate (SamplingRate); 
		ppf_registration.setPositionClusteringThreshold (Clustter_Position);
		ppf_registration.setRotationClusteringThreshold (Cluster_Rotation / 180.0f * float (M_PI));
		ppf_registration.setSearchMethod (hashmap_search_vector[i]);
		ppf_registration.setInputSource (cloud_models_with_normals[i]);
		ppf_registration.setInputTarget (recognize_Scene_normal);
	
		cout << "compute_VotingEstimation_OnlinePhase : ppf_registration align....\n";
		pcl::PointCloud<pcl::PointNormal> cloud_output_subsampled;//沒用到

		//做投票跟Cluster
		cout << "====================================================" << endl;
		ppf_registration.align (cloud_output_subsampled);
		cout << "====================================================" << endl;

		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output_subsampled_xyz (new pcl::PointCloud<pcl::PointXYZ> ());//沒用到
		//for ( i = 0; i < cloud_output_subsampled.points.size (); ++i)//沒用到
		//	cloud_output_subsampled_xyz->points.push_back ( pcl::PointXYZ (cloud_output_subsampled.points[i].x, cloud_output_subsampled.points[i].y, cloud_output_subsampled.points[i].z));//沒用到

		cout << "compute_VotingEstimation_OnlinePhase : Showing model...\n";
		Eigen::Matrix4f mat_1 = ppf_registration.results.at(0).pose.matrix();
		Eigen::Matrix4f mat_2 = ppf_registration.results.at(1).pose.matrix();
		Eigen::Matrix4f mat_3 = ppf_registration.results.at(2).pose.matrix();
		Eigen::Affine3f final_transformation_1 (mat_1);
		Eigen::Affine3f final_transformation_2 (mat_2);
		Eigen::Affine3f final_transformation_3 (mat_3);

		//  io::savePCDFileASCII ("output_subsampled_registered.pcd", cloud_output_subsampled);
		pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_output_1 (new pcl::PointCloud< pcl::PointXYZ > ());
		pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_output_2 (new pcl::PointCloud< pcl::PointXYZ > ());
		pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_output_3 (new pcl::PointCloud< pcl::PointXYZ > ());
		pcl::transformPointCloud (*(cloud_models[i]), *cloud_output_1, final_transformation_1);
		pcl::transformPointCloud (*(cloud_models[i]), *cloud_output_2, final_transformation_2);
		pcl::transformPointCloud (*(cloud_models[i]), *cloud_output_3, final_transformation_3);





		Eigen::Matrix4f ICP_mat[3];
		Eigen::Affine3f final_transformation_ICP[3];
		pcl::PointCloud< pcl::PointXYZ >::Ptr NewICP_Cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud< pcl::PointXYZ >::Ptr NewICP_Cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud< pcl::PointXYZ >::Ptr NewICP_Cloud_3(new pcl::PointCloud<pcl::PointXYZ>);
		ICP_cloud.push_back(NewICP_Cloud_1);
		ICP_cloud.push_back(NewICP_Cloud_2);
		ICP_cloud.push_back(NewICP_Cloud_3);

		computeIterativeClosestPoint( cloud_output_1, recognize_Scene, ICP_mat[0]); //ICP Algorithm
		final_transformation_ICP[0] = ICP_mat[0];
		pcl::transformPointCloud (*cloud_output_1, *( ICP_cloud.at(i * 3 + 0) ), final_transformation_ICP[0]);
		ICP_mat[0] = ICP_mat[0] * mat_1;

		computeIterativeClosestPoint( cloud_output_2, recognize_Scene, ICP_mat[1]);
		final_transformation_ICP[1] = ICP_mat[1];
		pcl::transformPointCloud (*cloud_output_2, *( ICP_cloud.at(i * 3 + 1) ), final_transformation_ICP[1]);
		ICP_mat[1] = ICP_mat[1] * mat_2;

		computeIterativeClosestPoint( cloud_output_3, recognize_Scene, ICP_mat[2]);
		final_transformation_ICP[2] = ICP_mat[2];
		pcl::transformPointCloud (*cloud_output_3, *( ICP_cloud.at(i * 3 + 2) ), final_transformation_ICP[2]);
		ICP_mat[2] = ICP_mat[2] * mat_3;


		//做outlier filter
		float score_1, score_2, score_3;
		Outlier_Score.push_back(score_1);
		Outlier_Score.push_back(score_2);
		Outlier_Score.push_back(score_3);
		Compute_PercentOfOutlier( recognize_Scene, ICP_cloud.at(i * 3 + 0), Outlier_Score.at(i * 3 + 0), 5.0);
		Compute_PercentOfOutlier( recognize_Scene, ICP_cloud.at(i * 3 + 1), Outlier_Score.at(i * 3 + 1), 5.0);
		Compute_PercentOfOutlier( recognize_Scene, ICP_cloud.at(i * 3 + 2), Outlier_Score.at(i * 3 + 2), 5.0);

		stringstream ss_1, ss_2, ss_3;
		ss_1 << "Model_ " << i << "_Voting_1";
		ss_2 << "Model_ " << i << "_Voting_2";
		ss_3 << "Model_ " << i << "_Voting_3";
		
		/*viewer.addPointCloud (cloud_output_1, ss_1.str ());
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, ss_1.str ());
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_1.str ());
		viewer.addPointCloud (cloud_output_2, ss_2.str ());
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, ss_2.str ());
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_2.str ());
		viewer.addPointCloud (cloud_output_3, ss_3.str ());
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, ss_3.str ());
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_3.str ());*/

		
		for ( int k = 0; k < 3; k++ )
		{
			stringstream ss_ICP;
			if ( i == 0)
			{
				if ( Outlier_Score.at(i * 3 + k)  < 0.15 )
				{
					ss_ICP << "Model_ " << i << "ICP_Voting_" << k;
					viewer->addPointCloud (ICP_cloud.at(i * 3 + k), ss_ICP.str ());
					viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_ICP.str ());
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, point_color[k][0], point_color[k][1], point_color[k][2], ss_ICP.str ());
					determine_GraspICP_Cloud.push_back( ICP_cloud.at(i * 3 + k) );
					determine_GraspObjectMat.push_back( ICP_mat[k] );
					model_type.push_back( 0 );
					SuccessRecognitionCount++;
				}
			}
			else if ( i == 1)
			{
				if ( Outlier_Score.at(i * 3 + k)  < 0.30 ) //0.35
				{
					ss_ICP << "Model_ " << i << "ICP_Voting_" << k;
					viewer->addPointCloud (ICP_cloud.at(i * 3 + k), ss_ICP.str ());
					viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_ICP.str ());
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, point_color[k][0], point_color[k][1], point_color[k][2], ss_ICP.str ());
					determine_GraspICP_Cloud.push_back( ICP_cloud.at(i * 3 + k) );
					determine_GraspObjectMat.push_back( ICP_mat[k] );
					model_type.push_back( 1 );
					SuccessRecognitionCount++;
				}
			}
			else if ( i == 2 )
			{
				if ( Outlier_Score.at(i * 3 + k)  < 0.20 ) 
				{
					ss_ICP << "Model_ " << i << "ICP_Voting_" << k;
					viewer->addPointCloud (ICP_cloud.at(i * 3 + k), ss_ICP.str ());
					viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_ICP.str ());
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, point_color[k][0], point_color[k][1], point_color[k][2], ss_ICP.str ());
					determine_GraspICP_Cloud.push_back( ICP_cloud.at(i * 3 + k) );
					determine_GraspObjectMat.push_back( ICP_mat[k] );
					model_type.push_back( 2 );
					SuccessRecognitionCount++;
				}
			}
		}

	}


	if ( SuccessRecognitionCount != 0 )

	{
		Eigen::Matrix< float, 4, 1 > Camera_ObjectGraspPoint;
		int WhichOneBeGrasp;
		Find_GraspObject( TCP_Position, determine_GraspICP_Cloud, Camera_ObjectGraspPoint, WhichOneBeGrasp);
			
//here
		int ii = 0;
		for( ii = 0; ii < PositionALL.NoneZeroPosition; ii++ )
		{
			printf("%d........ : ",ii );
			Camera_ObjectGraspPoint.x() = PositionALL.CameraPoint[ii].x();
			Camera_ObjectGraspPoint.y() = PositionALL.CameraPoint[ii].y();
			Camera_ObjectGraspPoint.z() = PositionALL.CameraPoint[ii].z();
			
			Base2Camera( determine_GraspICP_Cloud.at(ii), TCP_Position, determine_GraspObjectMat.at(ii), BaseObject_EulerAngle, Camera_ObjectGraspPoint.x(), Camera_ObjectGraspPoint.y(), Camera_ObjectGraspPoint.z(), Arm_PickPoint);
			
			if(ii == 0)
			{
				Position1.Target = model_type.at(PositionALL.IndexOfOriginalPosition[ii]);
				Position1.Arm_PickPoint = Arm_PickPoint;
				Position1.BaseObject_EulerAngle[0] = BaseObject_EulerAngle[0];
				Position1.BaseObject_EulerAngle[1] = BaseObject_EulerAngle[1];
				Position1.BaseObject_EulerAngle[2] = BaseObject_EulerAngle[2];
				Position1.NoneZeroPosition = PositionALL.NoneZeroPosition;
				printf(" done\n",ii );
			}
			else if(ii == 1)
			{
				Position2.Target = model_type.at(PositionALL.IndexOfOriginalPosition[ii]);
				Position2.Arm_PickPoint = Arm_PickPoint;
				Position2.BaseObject_EulerAngle[0] = BaseObject_EulerAngle[0];
				Position2.BaseObject_EulerAngle[1] = BaseObject_EulerAngle[1];
				Position2.BaseObject_EulerAngle[2] = BaseObject_EulerAngle[2];
				Position2.NoneZeroPosition = PositionALL.NoneZeroPosition;
				printf(" done\n",ii );
			}
			else if(ii == 2)
			{
				Position3.Target = model_type.at(PositionALL.IndexOfOriginalPosition[ii]);
				Position3.Arm_PickPoint = Arm_PickPoint;
				Position3.BaseObject_EulerAngle[0] = BaseObject_EulerAngle[0];
				Position3.BaseObject_EulerAngle[1] = BaseObject_EulerAngle[1];
				Position3.BaseObject_EulerAngle[2] = BaseObject_EulerAngle[2];
				Position3.NoneZeroPosition = PositionALL.NoneZeroPosition;
				printf(" done\n",ii );
			}
			else if(ii == 3)
			{
				Position4.Target = model_type.at(PositionALL.IndexOfOriginalPosition[ii]);
				Position4.Arm_PickPoint = Arm_PickPoint;
				Position4.BaseObject_EulerAngle[0] = BaseObject_EulerAngle[0];
				Position4.BaseObject_EulerAngle[1] = BaseObject_EulerAngle[1];
				Position4.BaseObject_EulerAngle[2] = BaseObject_EulerAngle[2];
				Position4.NoneZeroPosition = PositionALL.NoneZeroPosition;
				printf(" done\n",ii );
			}
			else
			{
				Position5.Target = model_type.at(PositionALL.IndexOfOriginalPosition[ii]);
				Position5.Arm_PickPoint = Arm_PickPoint;
				Position5.BaseObject_EulerAngle[0] = BaseObject_EulerAngle[0];
				Position5.BaseObject_EulerAngle[1] = BaseObject_EulerAngle[1];
				Position5.BaseObject_EulerAngle[2] = BaseObject_EulerAngle[2];
				Position5.NoneZeroPosition = PositionALL.NoneZeroPosition;
				printf(" done\n",ii );
			}
		}



         
		//grasp_objectType = model_type.at(WhichOneBeGrasp);
		_IsPoseEstimationDonet = true;
		
		/*stringstream ss_ICP;
		ss_ICP << "Z_HeightValue_CADModel";
		viewer->addPointCloud (determine_GraspICP_Cloud.at(WhichOneBeGrasp), ss_ICP.str ());
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_ICP.str ());
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, point_color[0][0], point_color[0][1], point_color[0][2], ss_ICP.str ());*/
	}


	viewer->spinOnce (10);
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));

}


void compute_VotingEstimation_OnlinePhase_VerifyPrecision( boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr &recognize_Scene, std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > &original_ObjectCAD,int CADModel_Number, float radius, float Clustter_Position, float Cluster_Rotation, float SamplingRate, pcl::PointXYZ &Arm_PickPoint, float TCP_Position[6], bool &_IsPoseEstimationDone, int CAD_Type, pcl::PointCloud<pcl::PointXYZ>::Ptr &recognized_CADModel)
{
	recognized_CADModel->clear();

	cout << "compute_VotingEstimation_OnlinePhase : Reading the segmentation PointCloud Scene\n";
	//pcl::visualization::PCLVisualizer viewer ("PPF Object Recognition - Results");
    //viewer.setBackgroundColor (0, 0, 0);
	viewer->removeAllPointClouds();
    viewer->addPointCloud (cloud_scene);
	viewer->spinOnce (10);
    //viewer.addPointCloud (cloud_scene);
    //viewer.spinOnce (10);

	cout << "compute_VotingEstimation_OnlinePhase : Computing scene normal.\n";
	pcl::PointCloud< pcl::PointNormal >::Ptr recognize_Scene_normal (new pcl::PointCloud<pcl::PointNormal>);
	compute_PPFNormals( recognize_Scene, recognize_Scene_normal, radius); 
	
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > ICP_cloud;
	std::vector< float > Outlier_Score;
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > determine_GraspICP_Cloud;
	std::vector< Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > determine_GraspObjectMat;
	std::vector< int > model_type;

	int SuccessRecognitionCount = 0;

	for (int i = 0; i < CADModel_Number; i++)
	{
		pcl::PointCloud< pcl::PointXYZ >::Ptr NewICP_Cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud< pcl::PointXYZ >::Ptr NewICP_Cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud< pcl::PointXYZ >::Ptr NewICP_Cloud_3(new pcl::PointCloud<pcl::PointXYZ>);
		ICP_cloud.push_back(NewICP_Cloud_1);
		ICP_cloud.push_back(NewICP_Cloud_2);
		ICP_cloud.push_back(NewICP_Cloud_3);

		cout << "compute_VotingEstimation_OnlinePhase : Creating ppf_registration...\n";
		pcl::PPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf_registration;

		
		// set parameters for the PPF registration procedure
		ppf_registration.setSceneReferencePointSamplingRate (SamplingRate); 
		ppf_registration.setPositionClusteringThreshold (Clustter_Position);
		ppf_registration.setRotationClusteringThreshold (Cluster_Rotation / 180.0f * float (M_PI));
		ppf_registration.setSearchMethod (hashmap_search_vector[i]);
		ppf_registration.setInputSource (cloud_models_with_normals[i]);
		ppf_registration.setInputTarget (recognize_Scene_normal);
	
		cout << "compute_VotingEstimation_OnlinePhase : ppf_registration align....\n";
		pcl::PointCloud<pcl::PointNormal> cloud_output_subsampled;
		ppf_registration.align (cloud_output_subsampled);

		/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output_subsampled_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
		for (size_t i = 0; i < cloud_output_subsampled.points.size (); ++i)
			cloud_output_subsampled_xyz->points.push_back ( pcl::PointXYZ (cloud_output_subsampled.points[i].x, cloud_output_subsampled.points[i].y, cloud_output_subsampled.points[i].z));*/

		cout << "compute_VotingEstimation_OnlinePhase : Showing model...\n";
		Eigen::Matrix4f mat_1 = ppf_registration.getFinalTransformation();
		Eigen::Matrix4f mat_2 = ppf_registration.results.at(1).pose.matrix();
		Eigen::Matrix4f mat_3 = ppf_registration.results.at(2).pose.matrix();
		Eigen::Affine3f final_transformation_1 (mat_1);
		Eigen::Affine3f final_transformation_2 (mat_2);
		Eigen::Affine3f final_transformation_3 (mat_3);

		//  io::savePCDFileASCII ("output_subsampled_registered.pcd", cloud_output_subsampled);
		pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_output_1 (new pcl::PointCloud< pcl::PointXYZ > ());
		pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_output_2 (new pcl::PointCloud< pcl::PointXYZ > ());
		pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_output_3 (new pcl::PointCloud< pcl::PointXYZ > ());
		pcl::transformPointCloud (*(cloud_models[i]), *cloud_output_1, final_transformation_1);
		pcl::transformPointCloud (*(cloud_models[i]), *cloud_output_2, final_transformation_2);
		pcl::transformPointCloud (*(cloud_models[i]), *cloud_output_3, final_transformation_3);


		Eigen::Matrix4f ICP_mat[3];
		Eigen::Affine3f final_transformation_ICP[3];

		computeIterativeClosestPoint( cloud_output_1, recognize_Scene, ICP_mat[0]);
		final_transformation_ICP[0] = ICP_mat[0];
		pcl::transformPointCloud (*cloud_output_1, *( ICP_cloud.at(i * 3 + 0) ), final_transformation_ICP[0]);
		ICP_mat[0] = ICP_mat[0] * mat_1;

		computeIterativeClosestPoint( cloud_output_2, recognize_Scene, ICP_mat[1]);
		final_transformation_ICP[1] = ICP_mat[1];
		pcl::transformPointCloud (*cloud_output_2, *( ICP_cloud.at(i * 3 + 1) ), final_transformation_ICP[1]);
		ICP_mat[1] = ICP_mat[1] * mat_2;

		computeIterativeClosestPoint( cloud_output_3, recognize_Scene, ICP_mat[2]);
		final_transformation_ICP[2] = ICP_mat[2];
		pcl::transformPointCloud (*cloud_output_3, *( ICP_cloud.at(i * 3 + 2) ), final_transformation_ICP[2]);
		ICP_mat[2] = ICP_mat[2] * mat_3;

		float score_1, score_2, score_3;
		Outlier_Score.push_back(score_1);
		Outlier_Score.push_back(score_2);
		Outlier_Score.push_back(score_3);
		Compute_PercentOfOutlier( recognize_Scene, ICP_cloud.at(i * 3 + 0), Outlier_Score.at(i * 3 + 0), 5.0);
		Compute_PercentOfOutlier( recognize_Scene, ICP_cloud.at(i * 3 + 1), Outlier_Score.at(i * 3 + 1), 5.0);
		Compute_PercentOfOutlier( recognize_Scene, ICP_cloud.at(i * 3 + 2), Outlier_Score.at(i * 3 + 2), 5.0);

		stringstream ss_1, ss_2, ss_3;
		ss_1 << "Model_ " << i << "_Voting_1";
		ss_2 << "Model_ " << i << "_Voting_2";
		ss_3 << "Model_ " << i << "_Voting_3";

		float point_color[3][3] = { {128, 128, 0},
									{0,  255,  0},
									{255, 0,   0}
								  };
		for ( int k = 0; k < 3; k++ )
		{
			stringstream ss_ICP;
			if ( CAD_Type == 0 && i == 0)
			{
				if ( Outlier_Score.at(i * 3 + k)  < 0.15 )
				{
					ss_ICP << "Model_ " << i << "ICP_Voting_" << k;
					viewer->addPointCloud (ICP_cloud.at(i * 3 + k), ss_ICP.str ());
					viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_ICP.str ());
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, point_color[k][0], point_color[k][1], point_color[k][2], ss_ICP.str ());
					determine_GraspICP_Cloud.push_back( ICP_cloud.at(i * 3 + k) );
					determine_GraspObjectMat.push_back( ICP_mat[k] );
					model_type.push_back( 0 );
					SuccessRecognitionCount++;
				}
			}
			else if ( CAD_Type == 1 && i == 1)
			{
				if ( Outlier_Score.at(i * 3 + k)  < 0.40 ) //0.35
				{
					ss_ICP << "Model_ " << i << "ICP_Voting_" << k;
					viewer->addPointCloud (ICP_cloud.at(i * 3 + k), ss_ICP.str ());
					viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_ICP.str ());
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, point_color[k][0], point_color[k][1], point_color[k][2], ss_ICP.str ());
					determine_GraspICP_Cloud.push_back( ICP_cloud.at(i * 3 + k) );
					determine_GraspObjectMat.push_back( ICP_mat[k] );
					model_type.push_back( 1 );
					SuccessRecognitionCount++;
				}
			}
			else if ( CAD_Type == 2 && i == 2 )
			{
				if ( Outlier_Score.at(i * 3 + k)  < 0.20 ) 
				{
					ss_ICP << "Model_ " << i << "ICP_Voting_" << k;
					viewer->addPointCloud (ICP_cloud.at(i * 3 + k), ss_ICP.str ());
					viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_ICP.str ());
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, point_color[k][0], point_color[k][1], point_color[k][2], ss_ICP.str ());
					determine_GraspICP_Cloud.push_back( ICP_cloud.at(i * 3 + k) );
					determine_GraspObjectMat.push_back( ICP_mat[k] );
					model_type.push_back( 2 );
					SuccessRecognitionCount++;
				}
			}
		}

	}


	if ( SuccessRecognitionCount != 0 )
	{
		Eigen::Matrix< float, 4, 1 > Camera_ObjectGraspPoint;
		int WhichOneBeGrasp;
		Find_GraspObject( TCP_Position, determine_GraspICP_Cloud, Camera_ObjectGraspPoint, WhichOneBeGrasp);
		pcl::copyPointCloud( *determine_GraspICP_Cloud.at(WhichOneBeGrasp) , *recognized_CADModel );
		_IsPoseEstimationDone = true;
	}
	// while (!viewer->wasStopped ()) {
		viewer->spinOnce (10);
	// }

}

void computeIterativeClosestPoint( pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &Target_cloud, Eigen::Matrix4f &result_transformation)
{
	cout << "computeIterativeClosestPoint : Creating ICP.....\n";
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> *ICP_reg;
	//ICP_reg = new pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>(); 
	ICP_reg = new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>();
	ICP_reg->setInputSource(input_cloud);
	ICP_reg->setInputTarget (Target_cloud);
	ICP_reg->setMaximumIterations (100);
	ICP_reg->setTransformationEpsilon (1e-7);
	//ICP_reg->setEuclideanFitnessEpsilon(0.1);

	cout << "computeIterativeClosestPoint : aligning.....\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output_subsampled (new pcl::PointCloud< pcl::PointXYZ > ());;
	ICP_reg->align(*cloud_output_subsampled);

	if ( ICP_reg->hasConverged() )
	{
		cout << "computeIterativeClosestPoint : ICP has converged!!!\n";
	}
	else
	{
		cout << "computeIterativeClosestPoint : ICP is not converged!!!\n";
	}

	result_transformation = ICP_reg->getFinalTransformation ();
	cout << "computeIterativeClosestPoint :Done!!\n";
}


void doExtractCluster_DivideObject( pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > &DivideObject_ClusterPCDResult, int &DivideObject_ClusterNumber)
{
	float background_color[3] = { 0, 0, 0 };
	float point_color[3] = { 255, 255, 255 };
	DivideObject_ClusterPCDResult.clear();
	
	cout << "doExtractCluster_DivideObject : Creating the KdTree object for the search method of the extraction.\n";
	pcl::search::KdTree<pcl::PointXYZ>::Ptr KD_Tree (new pcl::search::KdTree<pcl::PointXYZ>);
	KD_Tree->setInputCloud (input_cloud);

	cout << "doExtractCluster_DivideObject : Creating the EuclideanClusterExtraction object for cluster each object...\n";
	std::vector<pcl::PointIndices> DivideObject_ClusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (6.0); // 3.0	
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (1000);
	ec.setSearchMethod (KD_Tree);
	ec.setInputCloud (input_cloud);
	ec.extract (DivideObject_ClusterIndices);

	cout << "doExtractCluster_DivideObject : Saving each cluster to PCD...\n";
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = DivideObject_ClusterIndices.begin (); it != DivideObject_ClusterIndices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (input_cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		DivideObject_ClusterPCDResult.push_back(cloud_cluster);
		cout << "doExtractCluster_DivideObject : Cluster number " << j << " 'size is : " << DivideObject_ClusterPCDResult.at(j)->size() << "\n";
		//DisplyXYZPCDbyXYZRGB( cloud_cluster, point_color, background_color, 1);
		j++;
	}
	cout << "doExtractCluster_DivideObject : Cluster number is : " << j << "\n";
	DivideObject_ClusterNumber = j;

	cout << "doExtractCluster_DivideObject : Done!";

}

void Base2Camera( pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, float TCP_Position[6], Eigen::Matrix4f &Object_PoseMatrix, float Base_Object_EulerAngle[3], float Camera_X, float Camera_Y, float Camera_Z, pcl::PointXYZ &RealWorld_Point )
{ 
	float TCP_X = TCP_Position[0];
	float TCP_Y = TCP_Position[1];
	float TCP_Z = TCP_Position[2];
	float TCP_A = TCP_Position[3];
	float TCP_B = TCP_Position[4];
	float TCP_C = TCP_Position[5];

	float Tool2TCP_14 = 234.5585;
	float Tool2TCP_24 = 9.267568;
	float Tool2TCP_34 = -41.98946;

	float TCP2Camera_11 = 0.00112373,  TCP2Camera_12 = 0.00555237, TCP2Camera_13 = -0.999984, TCP2Camera_14 = 127.461;
	float TCP2Camera_21 = 0.999997,  TCP2Camera_22 = 0.00222004, TCP2Camera_23 = 0.00113607, TCP2Camera_24 = -6.08003;
	float TCP2Camera_31 = 0.00222631,  TCP2Camera_32 = -0.999982, TCP2Camera_33 = -0.00554986, TCP2Camera_34 = 58.0209;

	float x11 = cos( TCP_A  * D2P ) * cos( TCP_B * D2P );
	float x12 = cos( TCP_A  * D2P ) * sin( TCP_B * D2P ) * sin( TCP_C * D2P ) - sin( TCP_A * D2P ) * cos( TCP_C * D2P );
	float x13 = cos( TCP_A  * D2P ) * sin( TCP_B * D2P ) * cos( TCP_C * D2P ) + sin( TCP_A * D2P ) * sin( TCP_C * D2P );
	float x21 = sin( TCP_A  * D2P ) * cos( TCP_B * D2P );
	float x22 = sin( TCP_A  * D2P ) * sin( TCP_B * D2P ) * sin( TCP_C * D2P ) + cos( TCP_A * D2P ) * cos( TCP_C * D2P );
	float x23 = sin( TCP_A  * D2P ) * sin( TCP_B * D2P ) * cos( TCP_C * D2P ) - cos( TCP_A * D2P ) * sin( TCP_C * D2P );
	float x31 = -sin( TCP_B * D2P );
	float x32 = cos( TCP_B * D2P ) * sin( TCP_C * D2P );
	float x33 = cos( TCP_B * D2P ) * cos( TCP_C * D2P );
	float x14 = Tool2TCP_14 * x11 + Tool2TCP_24 * x12 + Tool2TCP_34 * x13 + TCP_X;
	float x24 = Tool2TCP_14 * x21 + Tool2TCP_24 * x22 + Tool2TCP_34 * x23 + TCP_Y;
	float x34 = Tool2TCP_14 * x31 + Tool2TCP_24 * x32 + Tool2TCP_34 * x33 + TCP_Z;

	float g11 = x11 * TCP2Camera_11 + x12 * TCP2Camera_21 + x13 * TCP2Camera_31;
	float g21 = x21 * TCP2Camera_11 + x22 * TCP2Camera_21 + x23 * TCP2Camera_31;
	float g31 = x31 * TCP2Camera_11 + x32 * TCP2Camera_21 + x33 * TCP2Camera_31;
	float g12 = x11 * TCP2Camera_12 + x12 * TCP2Camera_22 + x13 * TCP2Camera_32;
	float g22 = x21 * TCP2Camera_12 + x22 * TCP2Camera_22 + x23 * TCP2Camera_32;
	float g32 = x31 * TCP2Camera_12 + x32 * TCP2Camera_22 + x33 * TCP2Camera_32;
	float g13 = x11 * TCP2Camera_13 + x12 * TCP2Camera_23 + x13 * TCP2Camera_33;
	float g23 = x21 * TCP2Camera_13 + x22 * TCP2Camera_23 + x23 * TCP2Camera_33;
	float g33 = x31 * TCP2Camera_13 + x32 * TCP2Camera_23 + x33 * TCP2Camera_33;
	float g14 = x11 * TCP2Camera_14 + x12 * TCP2Camera_24 + x13 * TCP2Camera_34 + x14;
	float g24 = x21 * TCP2Camera_14 + x22 * TCP2Camera_24 + x23 * TCP2Camera_34 + x24;
	float g34 = x31 * TCP2Camera_14 + x32 * TCP2Camera_24 + x33 * TCP2Camera_34 + x34;

	Eigen::Matrix4f Base_CameraPoseMatrix;
	Base_CameraPoseMatrix(0, 0) = g11;
	Base_CameraPoseMatrix(0, 1) = g12;
	Base_CameraPoseMatrix(0, 2) = g13;
	Base_CameraPoseMatrix(0, 3) = g14;
	Base_CameraPoseMatrix(1, 0) = g21;
	Base_CameraPoseMatrix(1, 1) = g22;
	Base_CameraPoseMatrix(1, 2) = g23;
	Base_CameraPoseMatrix(1, 3) = g24;
	Base_CameraPoseMatrix(2, 0) = g31;
	Base_CameraPoseMatrix(2, 1) = g32;
	Base_CameraPoseMatrix(2, 2) = g33;
	Base_CameraPoseMatrix(2, 3) = g34;
	Base_CameraPoseMatrix(3, 0) = 0;
	Base_CameraPoseMatrix(3, 1) = 0;
	Base_CameraPoseMatrix(3, 2) = 0;
	Base_CameraPoseMatrix(3, 3) = 1;

	Eigen::Matrix4f Base_ObjectPoseMatrix = Base_CameraPoseMatrix * Object_PoseMatrix;
	ComputeEulerAngle( Base_ObjectPoseMatrix, Base_Object_EulerAngle );
	
	RealWorld_Point.x = g11 * Camera_X - g12 * Camera_Y + g13 * Camera_Z + g14;
	RealWorld_Point.y = g21 * Camera_X - g22 * Camera_Y + g23 * Camera_Z + g24;
	RealWorld_Point.z = g31 * Camera_X - g32 * Camera_Y + g33 * Camera_Z + g34;

	cout << "RealWorld_Point = Arm_pickpoint = " << RealWorld_Point << endl;

	//int CompareX = RealWorld_Point.x + 50;
	//int CompareY = RealWorld_Point.y + 0 ;
	//double CompareZ = RealWorld_Point.z - 330; 
}

void ComputeEulerAngle(Eigen::Matrix4f &PoseMatrix, float EulerAngle[3] )
{
	if ( PoseMatrix(2, 0) != 1 && PoseMatrix(2, 0) != -1 )
	{
		float euler_Y_1 = -asin( PoseMatrix(2, 0) );
		float euler_X_1 = atan2(  (PoseMatrix(2, 1) / cos(euler_Y_1) ), (PoseMatrix(2, 2) / cos(euler_Y_1) ) );
		float euler_Z_1 = atan2(  (PoseMatrix(1, 0) / cos(euler_Y_1) ), (PoseMatrix(0, 0) / cos(euler_Y_1) ) );

		EulerAngle[1] = euler_Y_1;
		EulerAngle[0] = euler_X_1;
		EulerAngle[2] = euler_Z_1;
	}
	else
	{

	}
}

void Compute_PercentOfOutlier( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_model, float &score, float dis_threshold)
{
	Eigen::Vector3f tempPoint_model;
	Eigen::Vector3f tempPoint_scene;
	float temp_distance = 0;
	float temp_score = 0;
	float min_dis;

	for (int i = 0; i < cloud_model->size(); i++ )
	{
		tempPoint_model = cloud_model->at(i).getVector3fMap();
		min_dis = 0;
		for (int j = 0; j < cloud_scene->size(); j++ )
		{
			tempPoint_scene = cloud_scene->at(j).getVector3fMap();
			temp_distance = ( tempPoint_model - tempPoint_scene ).norm();
			if (j == 0)
			{
				min_dis =  ( tempPoint_model - tempPoint_scene ).norm();
			}
			else if ( min_dis > temp_distance )
			{
				min_dis = temp_distance;
			}
		}

		if ( min_dis > dis_threshold )
		{
			temp_score++;
		}

	}

	score =  temp_score / cloud_model->size();
}


void Find_GraspObject( float TCP_Position[6], std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr >  &determine_GraspObjectICPCloud, Eigen::Matrix< float, 4, 1 > &Camera_ObjectGraspPoint, int &determine_GraspObject )
{
	cout << "=====================Find_GraspObject===========================\n";

	float TCP_X = TCP_Position[0];
	float TCP_Y = TCP_Position[1];
	float TCP_Z = TCP_Position[2];
	float TCP_A = TCP_Position[3];
	float TCP_B = TCP_Position[4];
	float TCP_C = TCP_Position[5];

	Position1.TCP_Position[0] = TCP_Position[0];
	Position1.TCP_Position[1] = TCP_Position[1];
	Position1.TCP_Position[2] = TCP_Position[2];
	Position1.TCP_Position[3] = TCP_Position[3];
	Position1.TCP_Position[4] = TCP_Position[4];
	Position1.TCP_Position[5] = TCP_Position[5];

	Position2.TCP_Position[0] = TCP_Position[0];
	Position2.TCP_Position[1] = TCP_Position[1];
	Position2.TCP_Position[2] = TCP_Position[2];
	Position2.TCP_Position[3] = TCP_Position[3];
	Position2.TCP_Position[4] = TCP_Position[4];
	Position2.TCP_Position[5] = TCP_Position[5];

	Position3.TCP_Position[0] = TCP_Position[0];
	Position3.TCP_Position[1] = TCP_Position[1];
	Position3.TCP_Position[2] = TCP_Position[2];
	Position3.TCP_Position[3] = TCP_Position[3];
	Position3.TCP_Position[4] = TCP_Position[4];
	Position3.TCP_Position[5] = TCP_Position[5];

	Position4.TCP_Position[0] = TCP_Position[0];
	Position4.TCP_Position[1] = TCP_Position[1];
	Position4.TCP_Position[2] = TCP_Position[2];
	Position4.TCP_Position[3] = TCP_Position[3];
	Position4.TCP_Position[4] = TCP_Position[4];
	Position4.TCP_Position[5] = TCP_Position[5];

	Position5.TCP_Position[0] = TCP_Position[0];
	Position5.TCP_Position[1] = TCP_Position[1];
	Position5.TCP_Position[2] = TCP_Position[2];
	Position5.TCP_Position[3] = TCP_Position[3];
	Position5.TCP_Position[4] = TCP_Position[4];
	Position5.TCP_Position[5] = TCP_Position[5];

	float Tool2TCP_14 = 234.5585;
	float Tool2TCP_24 = 9.267568;
	float Tool2TCP_34 = -41.98946;

	float TCP2Camera_11 = 0.00112373,  TCP2Camera_12 = 0.00555237, TCP2Camera_13 = -0.999984, TCP2Camera_14 = 127.461;
	float TCP2Camera_21 = 0.999997,  TCP2Camera_22 = 0.00222004, TCP2Camera_23 = 0.00113607, TCP2Camera_24 = -6.08003;
	float TCP2Camera_31 = 0.00222631,  TCP2Camera_32 = -0.999982, TCP2Camera_33 = -0.00554986, TCP2Camera_34 = 58.0209;

	float x11 = cos( TCP_A  * D2P ) * cos( TCP_B * D2P );
	float x12 = cos( TCP_A  * D2P ) * sin( TCP_B * D2P ) * sin( TCP_C * D2P ) - sin( TCP_A * D2P ) * cos( TCP_C * D2P );
	float x13 = cos( TCP_A  * D2P ) * sin( TCP_B * D2P ) * cos( TCP_C * D2P ) + sin( TCP_A * D2P ) * sin( TCP_C * D2P );
	float x21 = sin( TCP_A  * D2P ) * cos( TCP_B * D2P );
	float x22 = sin( TCP_A  * D2P ) * sin( TCP_B * D2P ) * sin( TCP_C * D2P ) + cos( TCP_A * D2P ) * cos( TCP_C * D2P );
	float x23 = sin( TCP_A  * D2P ) * sin( TCP_B * D2P ) * cos( TCP_C * D2P ) - cos( TCP_A * D2P ) * sin( TCP_C * D2P );
	float x31 = -sin( TCP_B * D2P );
	float x32 = cos( TCP_B * D2P ) * sin( TCP_C * D2P );
	float x33 = cos( TCP_B * D2P ) * cos( TCP_C * D2P );
	float x14 = Tool2TCP_14 * x11 + Tool2TCP_24 * x12 + Tool2TCP_34 * x13 + TCP_X;
	float x24 = Tool2TCP_14 * x21 + Tool2TCP_24 * x22 + Tool2TCP_34 * x23 + TCP_Y;
	float x34 = Tool2TCP_14 * x31 + Tool2TCP_24 * x32 + Tool2TCP_34 * x33 + TCP_Z;

	float g11 = x11 * TCP2Camera_11 + x12 * TCP2Camera_21 + x13 * TCP2Camera_31;
	float g21 = x21 * TCP2Camera_11 + x22 * TCP2Camera_21 + x23 * TCP2Camera_31;
	float g31 = x31 * TCP2Camera_11 + x32 * TCP2Camera_21 + x33 * TCP2Camera_31;
	float g12 = x11 * TCP2Camera_12 + x12 * TCP2Camera_22 + x13 * TCP2Camera_32;
	float g22 = x21 * TCP2Camera_12 + x22 * TCP2Camera_22 + x23 * TCP2Camera_32;
	float g32 = x31 * TCP2Camera_12 + x32 * TCP2Camera_22 + x33 * TCP2Camera_32;
	float g13 = x11 * TCP2Camera_13 + x12 * TCP2Camera_23 + x13 * TCP2Camera_33;
	float g23 = x21 * TCP2Camera_13 + x22 * TCP2Camera_23 + x23 * TCP2Camera_33;
	float g33 = x31 * TCP2Camera_13 + x32 * TCP2Camera_23 + x33 * TCP2Camera_33;
	float g14 = x11 * TCP2Camera_14 + x12 * TCP2Camera_24 + x13 * TCP2Camera_34 + x14;
	float g24 = x21 * TCP2Camera_14 + x22 * TCP2Camera_24 + x23 * TCP2Camera_34 + x24;
	float g34 = x31 * TCP2Camera_14 + x32 * TCP2Camera_24 + x33 * TCP2Camera_34 + x34;


	float tmepZ_Object = 0;
	float Z_maxObject = 0;
	int temp_one = 0;
	Eigen::Matrix< float, 4, 1 > Temp_Camera_ObjectGraspPoint;
	Eigen::Matrix< float, 4, 1 > Temp_Camera_ObjectGrasp[8];
	

	pcl::compute3DCentroid( *( determine_GraspObjectICPCloud.at(0) ), Temp_Camera_ObjectGraspPoint );
	Z_maxObject = g31 * Temp_Camera_ObjectGraspPoint.x() - g32 * Temp_Camera_ObjectGraspPoint.y() + g33 * Temp_Camera_ObjectGraspPoint.z() + g34;



	cout << "=======================determine_GraspObjectICPCloud.size() = "<<determine_GraspObjectICPCloud.size()<<"\n";


	//keypoint

	objectNum = determine_GraspObjectICPCloud.size(); 	//how many object been recognized
	//	Eigen::Matrix< float, 4, 1 > *Temp_Camera_ObjectGrasp;
	//	Temp_Camera_ObjectGrasp = new Eigen::Matrix< float, 4, 1 > [objectNum];//dynamic add element


	for (int i = 0; i < determine_GraspObjectICPCloud.size(); i++ )
	{
		pcl::compute3DCentroid( *( determine_GraspObjectICPCloud.at(i) ), Temp_Camera_ObjectGrasp[i] );  // count and record all the object that been detected
		cout << "Temp_Camera_ObjectGrasp[" << i << "] = " << Temp_Camera_ObjectGrasp[i].x() << "    "  << Temp_Camera_ObjectGrasp[i].y() << "           " << Temp_Camera_ObjectGrasp[i].z() << endl;


		
		pcl::compute3DCentroid( *( determine_GraspObjectICPCloud.at(i) ), Temp_Camera_ObjectGraspPoint );	//compare which one is the highest
		tmepZ_Object = g31 * Temp_Camera_ObjectGraspPoint.x() - g32 * Temp_Camera_ObjectGraspPoint.y() + g33 * Temp_Camera_ObjectGraspPoint.z() + g34;
		if ( Z_maxObject <= tmepZ_Object )
		{
			Z_maxObject = tmepZ_Object;
			temp_one = i;
		}
	}

	cout << "==========================================" << endl;



	float Xdiff;
	int j = 1;
	
	Eigen::Matrix< float, 4, 1 > Final_Camera_ObjectGrasp[8];
	memset(Final_Camera_ObjectGrasp, 0 ,8*sizeof( Eigen::Matrix< float, 4, 1 > ) );

	Final_Camera_ObjectGrasp[0].x() = Temp_Camera_ObjectGrasp[0].x();
	Final_Camera_ObjectGrasp[0].y() = Temp_Camera_ObjectGrasp[0].y();
	Final_Camera_ObjectGrasp[0].z() = Temp_Camera_ObjectGrasp[0].z();
	FinalPositionIndex[0] = 0;



	for (int i = 1; i < objectNum; i++ )
	{
		Xdiff = Temp_Camera_ObjectGrasp[i].x() - Final_Camera_ObjectGrasp[j-1].x();

		if( abs(Xdiff) > 1.1)
		{
			Final_Camera_ObjectGrasp[j].x() = Temp_Camera_ObjectGrasp[i].x();
			Final_Camera_ObjectGrasp[j].y() = Temp_Camera_ObjectGrasp[i].y();
			Final_Camera_ObjectGrasp[j].z() = Temp_Camera_ObjectGrasp[i].z();
			FinalPositionIndex[j] = i;
			j++;
		}
	}

	for (int i = 0; i < objectNum; i++ )
	{
		cout << "Final_Camera_ObjectGrasp[" << i << "] = " <<  Final_Camera_ObjectGrasp[i].x() << "    "  << Final_Camera_ObjectGrasp[i].y() << "           " << Final_Camera_ObjectGrasp[i].z() << endl;

	}

	cout << "==========================================" << endl;



	PositionALL.NoneZeroPosition = 0;

	for(int i = 0; i < objectNum; i++ )
	{
		if(Final_Camera_ObjectGrasp[i].x() != 0)
		{
			PositionALL.CameraPoint[i].x() = Final_Camera_ObjectGrasp[i].x();
			PositionALL.CameraPoint[i].y() = Final_Camera_ObjectGrasp[i].y();
			PositionALL.CameraPoint[i].z() = Final_Camera_ObjectGrasp[i].z();
			PositionALL.IndexOfOriginalPosition[i] = FinalPositionIndex[i];
			PositionALL.NoneZeroPosition++;
			cout << "PositionALL.CameraPoint[" << i << "] = " <<  PositionALL.CameraPoint[i].x() << "    "  << PositionALL.CameraPoint[i].y() << "           " << PositionALL.CameraPoint[i].z() << "   ::  " << PositionALL.IndexOfOriginalPosition[i] << endl;
		}	
	}
	cout << "PositionALL.NoneZeroPosition = " << PositionALL.NoneZeroPosition << endl;

	determine_GraspObject = temp_one;
}














fstream VerifyPrecision_File;
fstream VerifyPrecision_File_Reference;
fstream VerifyPrecision_File_Estimation;
bool CaptureImageAgain = true;
float CADModel_ReferencePoint[3] = {0};
float Scene_ReferencePoint[3] = {0};
float CADModel_ReferencePoint_2[3] = {0};
float Scene_ReferencePoint_2[3] = {0};
void Verify_Precision( float TCP_Position[6], pcl::PointCloud<pcl::PointXYZ>::Ptr CADModel_PointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_PointCloud)
{
	if ( !VerifyPrecision_File.is_open() )
	{
		int Is_OpenFile = 0;
		cout << "Verify_Precision : Create new file? ( 0:No, 1: Yes )" << endl;
		cin >> Is_OpenFile;
		if ( Is_OpenFile == 1 )
		{
			VerifyPrecision_File.open( "VerifyPrecision.txt", ios::out );
			VerifyPrecision_File_Reference.open( "VerifyPrecision_File_Reference.txt", ios::out );
			VerifyPrecision_File_Estimation.open( "VerifyPrecision_File_Estimation.txt", ios::out );
			VerifyPrecision_File <<"X\t" << "Y\t" << "Z\t" << "dis_sqrt\t" << "Rotation\t\n";
			VerifyPrecision_File_Reference <<"CX\t" << "CY\t" << "CZ\t"  << "R1X\t" << "R1Y\t" << "R1Z\t" << "R2X\t" << "R2Y\t" << "R2Z\t" << "VX\t" << "VY\t" << "VZ\n";
			VerifyPrecision_File_Estimation <<"CX\t" << "CY\t" << "CZ\t"  << "R1X\t" << "R1Y\t" << "R1Z\t" << "R2X\t" << "R2Y\t" << "R2Z\t" << "VX\t" << "VY\t" << "VZ\n";
			cout << "Verify_Precision : File already open! " << endl;
		}
		else
		{
			cout << "Verify_Precision : No create file, please restart the program!" << endl;
		}

	}
	else 
	{
		

		/*
		 *	 Convert image point cloud to real world point cloud
		 */

		float TCP_X = TCP_Position[0];
		float TCP_Y = TCP_Position[1];
		float TCP_Z = TCP_Position[2];
		float TCP_A = TCP_Position[3];
		float TCP_B = TCP_Position[4];
		float TCP_C = TCP_Position[5];

		float Tool2TCP_14 = 234.5585;
		float Tool2TCP_24 = 9.267568;
		float Tool2TCP_34 = -41.98946;

		float TCP2Camera_11 = 0.00112373,  TCP2Camera_12 = 0.00555237, TCP2Camera_13 = -0.999984, TCP2Camera_14 = 127.461;
		float TCP2Camera_21 = 0.999997,  TCP2Camera_22 = 0.00222004, TCP2Camera_23 = 0.00113607, TCP2Camera_24 = -6.08003;
		float TCP2Camera_31 = 0.00222631,  TCP2Camera_32 = -0.999982, TCP2Camera_33 = -0.00554986, TCP2Camera_34 = 58.0209;

		float x11 = cos( TCP_A  * D2P ) * cos( TCP_B * D2P );
		float x12 = cos( TCP_A  * D2P ) * sin( TCP_B * D2P ) * sin( TCP_C * D2P ) - sin( TCP_A * D2P ) * cos( TCP_C * D2P );
		float x13 = cos( TCP_A  * D2P ) * sin( TCP_B * D2P ) * cos( TCP_C * D2P ) + sin( TCP_A * D2P ) * sin( TCP_C * D2P );
		float x21 = sin( TCP_A  * D2P ) * cos( TCP_B * D2P );
		float x22 = sin( TCP_A  * D2P ) * sin( TCP_B * D2P ) * sin( TCP_C * D2P ) + cos( TCP_A * D2P ) * cos( TCP_C * D2P );
		float x23 = sin( TCP_A  * D2P ) * sin( TCP_B * D2P ) * cos( TCP_C * D2P ) - cos( TCP_A * D2P ) * sin( TCP_C * D2P );
		float x31 = -sin( TCP_B * D2P );
		float x32 = cos( TCP_B * D2P ) * sin( TCP_C * D2P );
		float x33 = cos( TCP_B * D2P ) * cos( TCP_C * D2P );
		float x14 = Tool2TCP_14 * x11 + Tool2TCP_24 * x12 + Tool2TCP_34 * x13 + TCP_X;
		float x24 = Tool2TCP_14 * x21 + Tool2TCP_24 * x22 + Tool2TCP_34 * x23 + TCP_Y;
		float x34 = Tool2TCP_14 * x31 + Tool2TCP_24 * x32 + Tool2TCP_34 * x33 + TCP_Z;

		float g11 = x11 * TCP2Camera_11 + x12 * TCP2Camera_21 + x13 * TCP2Camera_31;
		float g21 = x21 * TCP2Camera_11 + x22 * TCP2Camera_21 + x23 * TCP2Camera_31;
		float g31 = x31 * TCP2Camera_11 + x32 * TCP2Camera_21 + x33 * TCP2Camera_31;
		float g12 = x11 * TCP2Camera_12 + x12 * TCP2Camera_22 + x13 * TCP2Camera_32;
		float g22 = x21 * TCP2Camera_12 + x22 * TCP2Camera_22 + x23 * TCP2Camera_32;
		float g32 = x31 * TCP2Camera_12 + x32 * TCP2Camera_22 + x33 * TCP2Camera_32;
		float g13 = x11 * TCP2Camera_13 + x12 * TCP2Camera_23 + x13 * TCP2Camera_33;
		float g23 = x21 * TCP2Camera_13 + x22 * TCP2Camera_23 + x23 * TCP2Camera_33;
		float g33 = x31 * TCP2Camera_13 + x32 * TCP2Camera_23 + x33 * TCP2Camera_33;
		float g14 = x11 * TCP2Camera_14 + x12 * TCP2Camera_24 + x13 * TCP2Camera_34 + x14;
		float g24 = x21 * TCP2Camera_14 + x22 * TCP2Camera_24 + x23 * TCP2Camera_34 + x24;
		float g34 = x31 * TCP2Camera_14 + x32 * TCP2Camera_24 + x33 * TCP2Camera_34 + x34;

		cout << "Verify_Precision : Convert point cloud to real world, please waitting for a moment...." << endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr CADModel_RealWorldPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr Scene_RealWorldPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointXYZ temp_point;
		for (int i = 0; i < CADModel_PointCloud->size(); i++)
		{
			temp_point.x = g11 * CADModel_PointCloud->at(i).x - g12 *  CADModel_PointCloud->at(i).y + g13 *  CADModel_PointCloud->at(i).z + g14;
			temp_point.y = g21 * CADModel_PointCloud->at(i).x - g22 *  CADModel_PointCloud->at(i).y + g23 *  CADModel_PointCloud->at(i).z + g24;
			temp_point.z = g31 * CADModel_PointCloud->at(i).x - g32 *  CADModel_PointCloud->at(i).y + g33 *  CADModel_PointCloud->at(i).z + g34;
			CADModel_RealWorldPointCloud->push_back(temp_point);
		}
		for (int i = 0; i < scene_PointCloud->size(); i++)
		{
			temp_point.x = g11 * scene_PointCloud->at(i).x - g12 *  scene_PointCloud->at(i).y + g13 *  scene_PointCloud->at(i).z + g14;
			temp_point.y = g21 * scene_PointCloud->at(i).x - g22 *  scene_PointCloud->at(i).y + g23 *  scene_PointCloud->at(i).z + g24;
			temp_point.z = g31 * scene_PointCloud->at(i).x - g32 *  scene_PointCloud->at(i).y + g33 *  scene_PointCloud->at(i).z + g34;
			Scene_RealWorldPointCloud->push_back(temp_point);
		}
		

		/*
		 *	  Pick Up Real World Reference Point as  Scene_RealWorldReferencePoint and CADModel_RealWorldReferencePoint
		 */

		Eigen::Matrix< float, 4, 1 > CADModel_RealWorldReferencePoint;
		Eigen::Matrix< float, 4, 1 > Scene_RealWorldReferencePoint;
		Eigen::Matrix< float, 4, 1 > CADModel_RealWorldReferencePoint_2;
		Eigen::Matrix< float, 4, 1 > Scene_RealWorldReferencePoint_2;
		cout << "Verify_Precision : Please Pick point as CADModel_RealWorldReferencePoint..." << endl;
		CaptureImageAgain = true;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> SelectPoint_Viewer (new pcl::visualization::PCLVisualizer ("SelectPoint_Viewer"));
		SelectPoint_Viewer->removeAllPointClouds();
		SelectPoint_Viewer->addPointCloud( CADModel_RealWorldPointCloud);
		SelectPoint_Viewer->registerPointPickingCallback (PickPoint_Callback, (void*)&SelectPoint_Viewer); 
		while (CaptureImageAgain) 
		{ 
			SelectPoint_Viewer->spinOnce (100); 
			boost::this_thread::sleep (boost::posix_time::microseconds (100000)); 
		}

		CADModel_RealWorldReferencePoint(0) = CADModel_ReferencePoint[0];
		CADModel_RealWorldReferencePoint(1) = CADModel_ReferencePoint[1];
		CADModel_RealWorldReferencePoint(2) = CADModel_ReferencePoint[2];
		CADModel_RealWorldReferencePoint(3) = 1;
		CADModel_RealWorldReferencePoint_2(0) = CADModel_ReferencePoint_2[0];
		CADModel_RealWorldReferencePoint_2(1) = CADModel_ReferencePoint_2[1];
		CADModel_RealWorldReferencePoint_2(2) = CADModel_ReferencePoint_2[2];
		CADModel_RealWorldReferencePoint_2(3) = 1;

		cout << "Verify_Precision : Please Pick point as Scene_RealWorldReferencePoint..." << endl;
		CaptureImageAgain = true;
		SelectPoint_Viewer->removeAllPointClouds();
		SelectPoint_Viewer->addPointCloud( Scene_RealWorldPointCloud);
		while (CaptureImageAgain) 
		{ 
			SelectPoint_Viewer->spinOnce (100); 
			boost::this_thread::sleep (boost::posix_time::microseconds (100000)); 
		}

		Scene_RealWorldReferencePoint(0) = Scene_ReferencePoint[0];
		Scene_RealWorldReferencePoint(1) = Scene_ReferencePoint[1];
		Scene_RealWorldReferencePoint(2) = Scene_ReferencePoint[2];
		Scene_RealWorldReferencePoint(3) = 1;
		Scene_RealWorldReferencePoint_2(0) = Scene_ReferencePoint_2[0];
		Scene_RealWorldReferencePoint_2(1) = Scene_ReferencePoint_2[1];
		Scene_RealWorldReferencePoint_2(2) = Scene_ReferencePoint_2[2];
		Scene_RealWorldReferencePoint_2(3) = 1;

		/*
		 *	  Compute Real World Centroid Point as Scene_RealWorldCentroidPoint and CADModel_RealWorldCentroidPoint
		 *	  Compute Real World Vector as CADModel_Vector and Scene_Vector
		 */

		Eigen::Matrix< float, 4, 1 > CADModel_RealWorldCentroidPoint;
		Eigen::Matrix< float, 4, 1 > Scene_RealWorldCentroidPoint;
		Eigen::Matrix< float, 4, 1 > CADModel_RealWorldCentroidPoint_2;
		Eigen::Matrix< float, 4, 1 > Scene_RealWorldCentroidPoint_2;
		pcl::compute3DCentroid( *( CADModel_RealWorldPointCloud ), CADModel_RealWorldCentroidPoint );
		pcl::compute3DCentroid( *( Scene_RealWorldPointCloud ), Scene_RealWorldCentroidPoint );

		Eigen::Matrix< float, 4, 1 > CADModel_Vector;
		CADModel_Vector(0) = CADModel_RealWorldReferencePoint.x() - CADModel_RealWorldReferencePoint_2.x();
		CADModel_Vector(1) = CADModel_RealWorldReferencePoint.y() - CADModel_RealWorldReferencePoint_2.y();
		CADModel_Vector(2) = CADModel_RealWorldReferencePoint.z() - CADModel_RealWorldReferencePoint_2.z();
		CADModel_Vector(3) = 1;

		Eigen::Matrix< float, 4, 1 > Scene_Vector;
		Scene_Vector(0) = Scene_RealWorldReferencePoint.x() - Scene_RealWorldReferencePoint_2.x();
		Scene_Vector(1) = Scene_RealWorldReferencePoint.y() - Scene_RealWorldReferencePoint_2.y();
		Scene_Vector(2) = Scene_RealWorldReferencePoint.z() - Scene_RealWorldReferencePoint_2.z();
		Scene_Vector(3) = 1;

		/*
		 *	  Compute translation and rotation error
		 */

		float x_dis, y_dis, z_dis, dis_sqrt, rotation_angle;
		rotation_angle =  pcl::getAngle3D(Scene_Vector, CADModel_Vector) * 180 / float (M_PI);
		x_dis = Scene_RealWorldCentroidPoint(0) - CADModel_RealWorldCentroidPoint(0);
		y_dis = Scene_RealWorldCentroidPoint(1) - CADModel_RealWorldCentroidPoint(1);
		z_dis = Scene_RealWorldCentroidPoint(2) - CADModel_RealWorldCentroidPoint(2);
		Eigen::Matrix< float, 4, 1 > dis_sqrt_vector;
		dis_sqrt_vector(0) = x_dis;
		dis_sqrt_vector(1) = y_dis;
		dis_sqrt_vector(2) = z_dis;
		dis_sqrt_vector(3) = 1;
		dis_sqrt = dis_sqrt_vector.norm();
		int _IsSaveThisPoint = 1;
		cout << "Verify_Precision : do you want to save this point? (0:No, 1:Yes)\n";
		cin >> _IsSaveThisPoint;
		if ( _IsSaveThisPoint != 0 )
		{
			cout << "Verify_Precision : x_dis = " << x_dis << ", y_dis = " << y_dis << ", z_dis = " << z_dis << ", dis_sqrt = " << dis_sqrt << ", rotation_angle = " << rotation_angle << endl;
			VerifyPrecision_File << x_dis << '\t' << y_dis << '\t' << z_dis << '\t' << dis_sqrt << '\t' << rotation_angle << '\n';

			VerifyPrecision_File_Reference << Scene_RealWorldCentroidPoint(0) << '\t' << Scene_RealWorldCentroidPoint(1) << '\t' << Scene_RealWorldCentroidPoint(2) << '\t' 
										<< Scene_RealWorldReferencePoint.x() << '\t' << Scene_RealWorldReferencePoint.y() << '\t' << Scene_RealWorldReferencePoint.z() << '\t' 
										<< Scene_RealWorldReferencePoint_2.x() << '\t' << Scene_RealWorldReferencePoint_2.y() << '\t' << Scene_RealWorldReferencePoint_2.z() << '\t' 
										<< Scene_Vector(0) << '\t' << Scene_Vector(1) << '\t' << Scene_Vector(2) << '\n';

			VerifyPrecision_File_Estimation << CADModel_RealWorldCentroidPoint(0) << '\t' << CADModel_RealWorldCentroidPoint(1) << '\t' << CADModel_RealWorldCentroidPoint(2) << '\t' 
										    << CADModel_RealWorldReferencePoint.x() << '\t' << CADModel_RealWorldReferencePoint.y() << '\t' << CADModel_RealWorldReferencePoint.z() << '\t'
											 << CADModel_RealWorldReferencePoint_2.x() << '\t' << CADModel_RealWorldReferencePoint_2.y() << '\t' << CADModel_RealWorldReferencePoint_2.z() << '\t'
											<< CADModel_Vector(0) << '\t' << CADModel_Vector(1) << '\t' << CADModel_Vector(2) << '\n';
		}
		else
		{
			cout << "Verify_Precision :The point is not saved!!\n";
		}
	}
}

void PickPoint_Callback (const pcl::visualization::PointPickingEvent& event, void* viewer_void) 
{ 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void); 
	if (event.getPointIndex () == -1) 
	{ 
		return; 
	}
	int isPickThisPoint = 0;
	float px, py, pz;
	event.getPoint( px, py, pz );
	cout << "PickPoint_Callback : this point's position (" << px << ", " << py << ", "  << pz << ")" << std::endl;
	cout << "PickPoint_Callback : Save this point as reference point? \n(1.2:Yes->CADModel[3], 3.4:Yes->Scene, 0:No)" << endl;
	cin >> isPickThisPoint;

	if ( isPickThisPoint == 1 )
	{
		CADModel_ReferencePoint[0] = px;
		CADModel_ReferencePoint[1] = py;
		CADModel_ReferencePoint[2] = pz;
		cout << "PickPoint_Callback : Save this point as CADModel reference point." << endl;
		CaptureImageAgain = true;
	}
	else if ( isPickThisPoint == 2 )
	{
		CADModel_ReferencePoint_2[0] = px;
		CADModel_ReferencePoint_2[1] = py;
		CADModel_ReferencePoint_2[2] = pz;
		cout << "PickPoint_Callback : Save this point as Scene reference_2 point." << endl;
		CaptureImageAgain = false;
	}
	else if ( isPickThisPoint == 3 )
	{
		Scene_ReferencePoint[0] = px;
		Scene_ReferencePoint[1] = py;
		Scene_ReferencePoint[2] = pz;
		cout << "PickPoint_Callback : Save this point as Scene reference point." << endl;
		CaptureImageAgain = true;
	}
	else if ( isPickThisPoint == 4 )
	{
		Scene_ReferencePoint_2[0] = px;
		Scene_ReferencePoint_2[1] = py;
		Scene_ReferencePoint_2[2] = pz;
		cout << "PickPoint_Callback : Save this point as Scene reference_2 point." << endl;
		CaptureImageAgain = false;
	}
	else
	{
		cout << "PickPoint_Callback : Don't save anything picked point." << endl;
		CaptureImageAgain = true;
	}

	
} 