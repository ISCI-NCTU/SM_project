/**
 * Copyright 2017 National Chiao Tung University, Intelligent System and Control Integration Laboratory
 * Author: Cheng-Hei Wu 
 * Maintainer : Howard Chen 
 
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
#define  EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET 
#include "PCDProcessClass.h"


PCDProcessClass::PCDProcessClass()
{
	pcl::PointCloud<pcl::Normal>::Ptr NormalCloud (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr resample_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Boundary>::Ptr BoundaryCloud (new pcl::PointCloud<pcl::Boundary>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr BoundaryPointXYZ (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud< pcl::PointXYZ>::Ptr PCData (new pcl::PointCloud<pcl::PointXYZ>);
	CAD_ResamplePCData = resample_cloud;
	CAD_Normal = NormalCloud;
	CAD_Boundary = BoundaryCloud;
	CAD_BoundaryPointXYZ = BoundaryPointXYZ;
	CAD_PCData = PCData;
}


pcl::PointCloud<pcl::Normal>::Ptr PCDProcessClass::getCAD_Normal()
{
	return CAD_Normal;
}

pcl::PointCloud<pcl::Boundary>::Ptr PCDProcessClass::getCAD_Boundary()
{
	return CAD_Boundary;
}

pcl::PointCloud< pcl::PointXYZ>::Ptr PCDProcessClass::getCAD_ResamplePCData()
{
	return CAD_ResamplePCData;
}

pcl::PointCloud<PointT>::Ptr PCDProcessClass::getCAD_PCData()
{
	return CAD_PCData;
}

pcl::PointCloud< pcl::PointXYZ>::Ptr PCDProcessClass::getCAD_BoundaryPointXYZ()
{
	return CAD_BoundaryPointXYZ;
}



void PCDProcessClass::showCAD_Boundary(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_BoundaryCloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("ShowBoundary"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (input_BoundaryCloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters ();
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

//
//void PCDProcessClass::computePairFeature(pcl::PointCloud<PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::Normal>::Ptr &input_normalCloud, float threshold_dist, float threshold_angle)
//{
//	pcl::UniformSampling<PointXYZ> uniform_Sampling;
//	
//	Eigen::Vector4f p1;
//	Eigen::Vector4f p1_normal;
//	Eigen::Vector4f p2;
//	Eigen::Vector4f p2_normal;
//	float f1;
//	float f2;
//	float f3;
//	float f4;
//	float pi = 3.1415926;
//	int count_OK = 0;
//
//	for ( size_t i = 0; i < input_cloud->size(); ++i)
//	{
//		p1 = input_cloud->points[i].getVector4fMap();
//		p1_normal = input_normalCloud->points[i].getNormalVector4fMap();
//		for ( size_t j = i + 1; j < input_cloud->size(); ++j)
//		{
//			p2 = input_cloud->points[j].getVector4fMap();
//			p2_normal = input_normalCloud->points[j].getNormalVector4fMap();
//			//computePairFeatures( p1, p1_normal, p2, p2_normal, f1, f2, f3, f4);
//
//			if ( f1 > ( (2 *pi ) / (threshold_angle) ) && f4 > threshold_dist )
//			{
//				count_OK++;
//				//cout << "Count = " << count_OK << ": ( " << i << " , " << j << " ) " << '\n';
//				//cout << "f1 = " << f1 * (180 / 3.14) << '\t' << "f2 = " << f2 * (180 / 3.14) << '\t' << "f3 = " << f3 * (180 / 3.14) << '\t' << "f4 = " << f4 << '\n';
//			}
//		}
//
//	}
//	
//	cout << "count_OK = " << count_OK << endl;
//	pairFeatures = new float*[count_OK];
//	for ( int i = 0; i < count_OK; ++i)
//	{
//		*(pairFeatures + i) = new float[4];
//	}
//	HashTable_counter = new int[count_OK];
//	HashTable = new int**[count_OK];
//	for ( int i = 0; i < count_OK; ++i)
//	{
//		*(HashTable + i) = new int*[100];
//		for ( int j = 0; j < 100; ++j)
//		{
//			*(*(HashTable + i) +j) = new int[2];
//		}
//	}
//
//
//	float temp[4];
//	int check_SaveInHashTable;
//	count_OK = 0;
//	for ( size_t i = 0; i < input_cloud->size(); ++i)
//	{
//		p1 = input_cloud->points[i].getVector4fMap();
//		p1_normal = input_normalCloud->points[i].getNormalVector4fMap();
//		for ( size_t j = i + 1; j < input_cloud->size(); ++j)
//		{
//			p2 = input_cloud->points[j].getVector4fMap();
//			p2_normal = input_normalCloud->points[j].getNormalVector4fMap();
//			//computePairFeatures( p1, p1_normal, p2, p2_normal, f1, f2, f3, f4);
//			//cout << "f1 = " << f1 << '\t' << "f2 = " << f2 << '\t' << "f3 = " << f3 << '\t' << "f4 = " << f4 << '\n';
//			if ( f1 > ( (2 *pi ) / (threshold_angle) ) && f4 > threshold_dist )
//			{
//				temp[0] = f1;
//				temp[1] = f2;
//				temp[2] = f3;
//				temp[3] = f4;
//				check_SaveInHashTable = searchHashTable(count_OK, pairFeatures, temp);
//				if ( check_SaveInHashTable >= 0 )
//				{
//					HashTable[check_SaveInHashTable][ HashTable_counter[check_SaveInHashTable] ][0] = i;
//					HashTable[check_SaveInHashTable][ HashTable_counter[check_SaveInHashTable] ][1] = j;
//					HashTable_counter[check_SaveInHashTable]++;
//				}
//				else
//				{	
//					HashTable_counter[count_OK] = 0;
//					HashTable[count_OK][ HashTable_counter[count_OK] ][0] = i;
//					HashTable[count_OK][ HashTable_counter[count_OK] ][1] = j;
//					*(*(pairFeatures + count_OK) + 0) = (int)( f1 * (180 / 3.14));
//					*(*(pairFeatures + count_OK) + 1) = (int)( f2 * (180 / 3.14));
//					*(*(pairFeatures + count_OK) + 2) = (int)( f3 * (180 / 3.14));
//					*(*(pairFeatures + count_OK) + 3) = myself_round( f4, 1);
//					
//					//cout << "f1 = " << *(*(pairFeatures + count_OK) + 0) << '\t' << "f2 = " << *(*(pairFeatures + count_OK) + 1) << '\t' << "f3 = " << *(*(pairFeatures + count_OK) + 2) << '\t' << "f4 = " << *(*(pairFeatures + count_OK) + 3) << '\n';
//					HashTable_counter[count_OK]++;
//					count_OK++;
//				}
//			}
//		}
//
//	}
//	cout << "count_OK = " << count_OK << endl;
//
//
//}






////////////////////////////////////////////////////////////////////////////////////////////////
//bool computePairFeatures (const Eigen::Vector4f &p1, const Eigen::Vector4f &n1, 
//                          const Eigen::Vector4f &p2, const Eigen::Vector4f &n2,
//                          float &f1, float &f2, float &f3, float &f4)
//{
//  // Compute the Cartesian difference between the two points
//  Eigen::Vector4f delta = p2 - p1;
//  delta[3] = 0;
//
//  // Compute the Euclidean norm = || p_idx - q_idx ||
//  float distance_sqr = delta.squaredNorm ();
//
//  if (distance_sqr == 0)
//  {
//    f1 = f2 = f3 = f4 = 0;
//    return (false);
//  }
//
//  // Estimate f4 = || delta ||
//  f4 = sqrt (distance_sqr);
//
//  // Create a Darboux frame coordinate system u-v-w
//  // u = n1; v = (p_idx - q_idx) x u / || (p_idx - q_idx) x u ||; w = u x v
//
//  // Estimate f3 = u * delta / || delta ||
//  // delta[3] = 0 (line 59)
//  f3 = n1.dot (delta) / f4;
//
//  // v = delta * u
//  Eigen::Vector4f v = Eigen::Vector4f::Zero ();
//  v = delta.cross3 (n1);
//
//  distance_sqr = v.squaredNorm ();
//  if (distance_sqr == 0)
//  {
//    f1 = f2 = f3 = f4 = 0;
//    return (false);
//  }
//
//  // Copy the q_idx normal
//  Eigen::Vector4f nq = n2;
//  nq[3] = 0;
//
//  // Normalize the vector
//  v /= sqrt (distance_sqr);
//
//  // Compute delta (w) = u x v
//  delta = n1.cross3 (v);
//
//  // Compute f2 = v * n2;
//  // v[3] = 0 (line 82)
//  f2 = v.dot (nq);
//
//  // Compute f1 = arctan (w * n2, u * n2) i.e. angle of n2 in the x=u, y=w coordinate system
//  // delta[3] = 0 (line 59), nq[3] = 0 (line 97)
//  f1 = atan2f (delta.dot (nq), n1.dot (nq));       // @todo: optimize this
//
//  return (true);
//}

void PCDProcessClass::uniform_DownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, float voxel)
{
	pcl::PointCloud<int> sampled_indices;
	pcl::UniformSampling<PointXYZ> uni_sampling;

	uni_sampling.setInputCloud( input_cloud );
	uni_sampling.setRadiusSearch( voxel );
	uni_sampling.compute( sampled_indices );
	copyPointCloud( *input_cloud, sampled_indices.points, *output_cloud);
	cout << "Input Size = " << input_cloud->size() << "\nOutput Size = " << output_cloud->size() << endl;
	
}

void PCDProcessClass::voxelGrid_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, float voxel)
{
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setInputCloud( input_cloud );
	voxel_grid.setLeafSize( voxel, voxel, voxel);
	voxel_grid.filter( *output_cloud );
	cout << "Input Size = " << input_cloud->size() << "\nOutput Size = " << output_cloud->size() << endl;
}

void PCDProcessClass::computeBoundary(pcl::PointCloud<PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::Normal>::Ptr &input_normalCloud, pcl::PointCloud<pcl::Boundary>::Ptr &output_boundaryCloud, pcl::PointCloud<PointXYZ>::Ptr &output_boundaryPointXYZ)
{
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	est.setInputCloud(input_cloud);
	est.setInputNormals(input_normalCloud);
	est.setRadiusSearch(0.05);
	est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	est.compute(*output_boundaryCloud);

	int counter = 0;
	for (size_t i = 0; i < input_cloud->size(); ++i)
	{
		if (output_boundaryCloud->points[i].boundary_point == 1)
		{
			output_boundaryPointXYZ->push_back(input_cloud->points[i]);
			counter++;
		}
	}
	cout << "Boundary Point Size = " << counter << endl;
}

//int searchHashTable(int p, float** float_ptr, float* float_compare)
//{
//	for (int i = 0; i < p; i++)
//	{
//		//if ( myself_round( *(*(float_ptr+i)+0) * (180 / 3.14), 1 ) == myself_round( *(float_compare+0) * (180 / 3.14), 1) &&
//		//	 myself_round( *(*(float_ptr+i)+1) * (180 / 3.14), 1 ) == myself_round( *(float_compare+1) * (180 / 3.14), 1) &&
//		//	 myself_round( *(*(float_ptr+i)+2) * (180 / 3.14), 1 ) == myself_round( *(float_compare+2) * (180 / 3.14), 1) && 
//		//	 myself_round( *(*(float_ptr+i)+3), 1 ) == myself_round( *(float_compare+3), 1) )
//		//{
//		//	return i;
//		//}
//		if ( (int) (*(*(float_ptr+i)+0) * (180 / 3.14)) == (int)( *(float_compare+0) * (180 / 3.14)) &&
//			 (int) (*(*(float_ptr+i)+1) * (180 / 3.14)) == (int)( *(float_compare+1) * (180 / 3.14)) &&
//			 (int) (*(*(float_ptr+i)+2) * (180 / 3.14)) == (int)( *(float_compare+2) * (180 / 3.14)) && 
//			 myself_round( *(*(float_ptr+i)+3), 1 ) == myself_round( *(float_compare+3), 1) )
//		{
//			return i;
//		}
//	}
//	return -1;
//}
//
//float myself_round(float p, int order)
//{
//	float add = 5 / pow(10.0, order+1);
//	int A = (( p + add ) * pow(10.0, order) );
//	float C = (float) A / ( pow(10.0, order) );
//	return C;
//}