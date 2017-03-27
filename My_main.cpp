#define  EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET 
#include "PCD_Function.h"
#include "VotingSchemePoseEstimation_Class.h"
#include "CADDatabaseClass.h"
#include "PCL_KinectClass.h"
#include <Windows.h>
#include <process.h>
#include "MySocket.h"
#include <stdlib.h>
#include <stdio.h>

using namespace std;
/*
class Data 
{
public:
	float TCP_Position[6];
	Eigen::Matrix< float, 4, 1 > CameraPoint[6];
	int Target;
	float BaseObject_EulerAngle[3];
};
*/
PositionData  PositionData_Main;

std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > determine_GraspICP_Cloud;
std::vector< Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > determine_GraspObjectMat;
Eigen::Matrix< float, 4, 1 > Camera_ObjectGraspPoint;

int WhichOneBeGrasp = 0;

/*
 *   ª«¥ó
 */
	WgSocket MySocket;
	VotingSchemePoseEstimationClass PoseEstimationObj;
	CADDatabaseClass CADDatabaseObj;
	KinectClass KinectObj;

/*
 *   ¥þ°ì¨ç¼Æ
 */
	bool CreateClient(WgSocket &clientSocket);
	static unsigned __stdcall StartSocket_Thread(void * pThis);
	void StartSocket();
	void Auto_RecognitionFun();
	void Manual_RecognitionFun();
	void decode_TCPPosition( char * encode_TCP );
	void Manual_Fun();
	void PrintPosition();

/*
 *  ------------ Socket¸ê®Æ -----------------
 *
 *  KukaState = 0 -> µ¥«ÝKUKA¨ì¹FÂ^¨ú¼v¹³ÂI
 *	KukaState = 1 -> ¼v¹³Â^¨ú+«ººA¿ëÃÑ
 *  KukaState = 2 -> µ¥«ÝKUKA§ì¨ú¤u¥ó§¹¦¨
 *  KukaState = -1 -> Close
 */
	int KukaState = 0;
	float Xyzabc_CommandData[6] = {0};
	float TCP_PositionData[6] = {0};
	int PositionOrder = 1;
	int cmp;

/*
 *   ¥þ°ì°Ñ¼Æ
 */
	
	int show_Mode = 0;
	int CADModel_Number = 3;
	float CADModel_Normal_radius = 7.5;
	float CADModel_Voxel_radius = 5.0;//(1 = 1mm)
	float Scene_Voxel_radius = 6.0;
	float Scene_Normal_radius = 7.5;
	float SACSegmentationFromNormal_radius = 12;
	float HashMapSearch_Position = 20.0; // No use
	float HashMapSearch_Rotation = 15.0; 
	float Clustter_Position = 3.5;
	float Cluster_Rotation = 30.0;
	float SamplingRate = 20;
	int showPose_num = 0;
	int DivideObject_ClusterNumber = 0;
	pcl::PointXYZ Arm_PickPoint;
	//----------------------
	Eigen::Matrix< float, 4, 1 > Final_Camera_ObjectGrasp[6];
	float ObjectPose_EulerAngle[3];
	bool _IsPoseEstimationDone = true;
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > DivideObject_ClusterPCDResult;
	char *AllCADModel_pcdFileName[3] = { "VirtualObject_1_CADModel_PCD.pcd", "VirtualObject_3_CADModel_PCD.pcd", "VirtualObject_6_CADModel_PCD.pcd"};
	char *CADModel_pcdFileName[1] = { "VirtualObject_1_CADModel_PCD.pcd"};
	int Grasp_ObjectType;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> RecognitionPCD_Viewer (new pcl::visualization::PCLVisualizer ("RecognitionPCD_Viewer"));
	float segmentation_Range[3][2] =
	{
		{120, 385},
		{270, 440}, 
		{100, 800}
	};




int main()
{
	KinectObj.KinectInitial();

	/*HANDLE   SocketThread_Handel;
	unsigned  SocketThread_ID;  
	SocketThread_Handel = (HANDLE)_beginthreadex( NULL, 0, StartSocket_Thread, NULL, NULL,  &SocketThread_ID);
	WaitForSingleObject( SocketThread_Handel, INFINITE ); 
	CloseHandle( SocketThread_Handel );*/



	StartSocket();
	//Manual_Fun(); //Database_Fun():«Ø¥ßDatabaseªº«e¸m§@·~
	//Manual_RecognitionFun(); // Manual_Recognition:Åª¨ú¤@±i¼v¹³¿ëÃÑ(¥]§t«Ø¥ß¸ê®Æ®w»P¿ëÃÑ)
	//Auto_RecognitionFun();

	system("pause");
	return 0;
}


/*
 *	CreateClient() : ¶}±Ò¥»¦aClinet¡A¨Ã¸òServer³s½u
 */
bool CreateClient(WgSocket &clientSocket)
{
	if ( clientSocket.Open("100.100.100.1", 6008 ) )
	{
		cout << "Correct : Create  a virtual client and connect to server correct!!\n";
		return true;
	}
	else
	{
		return false;
	}
}

void StartSocket()
{
	bool check_OK;

	// Create a ClientSocket
	check_OK = CreateClient( MySocket );


	if ( check_OK )
	{
		char Sent_ClientData[1024] = {0};
		char Recieve_ServerData[1024] = {0};
		long ret_len;

		/*
		 *   ¦¹°Ï°ì«Ø¥ß¸ê®Æ®w
	     */
		     compute_VotingEstimation_OffinePhase( CADModel_Number, AllCADModel_pcdFileName, CADModel_Normal_radius, HashMapSearch_Position, HashMapSearch_Rotation);
		// Deliver message to Kuka server
		

		while( check_OK )
		{
			if ( KukaState == 0)
			{
				cout << "================================ KukaState = 0==========================\n";

				sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType); 
				MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));
				PrintPosition();

				cout << "Client_Socket : Waitting Kuka move to capture image position....\n";
				cout << "Client_Socket : Waitting Kuka sent TCP Position ....\n";
				MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
				if (  strncmp( Recieve_ServerData, "I", 1) == 0 ) // I : Require now TCP Position 
				{
						sprintf( Sent_ClientData, "STCP_GetE" );
						MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));

						MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
						decode_TCPPosition( Recieve_ServerData );

						sprintf( Sent_ClientData, "STCP_DoneE" );
						MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));

				}
				else 
				{
					cout << "Client_Socket : Something wrong!!!\n";
				}


				MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
				while ( KukaState == 0 )
				{
					if (  strncmp( Recieve_ServerData, "C", 1) == 0 ) // C : Capture Image 
					{
						 KukaState = 1;
					}
					else 
					{
						cout << "Client_Socket : Something wrong!!!\n";
					}
					Sleep(50);
				}
			}
			else if ( KukaState == 1)
			{
				
					cout <<"=========================== KukaState = 1 : image catching and pose estimation============================= \n";
					_IsPoseEstimationDone = false;
					int CaptureImage_Again = 1;
					int Times_Counter = 0;
					while ( !_IsPoseEstimationDone  )		// capture image and pose estimation
					{
						/*
						 *   ¦¹°Ï°ìÂ^¨ú¼v¹³
						 */
						
						 KinectObj.SceneToPCDProcessing();
						 yml2pcd( KinectObj.Scene_ymlName, "Scene.pcd", KinectObj, PoseEstimationObj.getSceneCloud(), segmentation_Range, 1, show_Mode);
						 delete [] KinectObj.Scene_ymlName;

						 voxelGrid_Filter( PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getDownsampling_SceneCloud(), Scene_Voxel_radius );
						 compute_SACSegmentationFromNormals( PoseEstimationObj.getDownsampling_SceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), SACSegmentationFromNormal_radius, 0);
						 if ( PoseEstimationObj.getSceneSegmentationCloud()->empty() || PoseEstimationObj.getSceneSegmentationCloud()->size() < 100 )
						 {
							 Times_Counter++;
							 if ( Times_Counter < 3 )
							 {
								 continue;
							 }
							 else
							 {
								 cout << "System : Capture Image again...? ( Enter : 1 -> again, 0 -> end ) \n";
								 cin >> CaptureImage_Again ;
								 if ( CaptureImage_Again == 1)
								 {
										continue;
								 }
								 else
								 {
							 		 KukaState = -1;
									 break;
								 }
							 }
								 
													
							 }
						
						/*
						 *   ¦¹°Ï°ì¿ëÃÑ«ººA
						 */						
						 compute_VotingEstimation_OnlinePhase( RecognitionPCD_Viewer, PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), CADDatabaseObj.getCADModel_OriginalPCDVector(), CADModel_Number, Scene_Normal_radius , Clustter_Position, Cluster_Rotation, SamplingRate, Arm_PickPoint, TCP_PositionData, ObjectPose_EulerAngle, Grasp_ObjectType, _IsPoseEstimationDone);

					     if ( !_IsPoseEstimationDone )
						 {
							 Times_Counter++;
							 if ( Times_Counter <= 3 )
							 {
								 continue;
							 }
							 else
							 {
								 cout << "System : Capture Image again...? ( Enter : 1 -> again, 0 -> end ) \n";
								 cin >> CaptureImage_Again ;
								 if ( CaptureImage_Again == 1)
								 {
										continue;
								 }
								 else
								 {
							 		 KukaState = -1;
									 break;
								 }
							 }
						 }
					}


				TransferDatatoMain(&PositionData_Main ,PositionOrder);

				cmp = PositionData_Main.NoneZeroPosition - PositionOrder;
				if(cmp < 0)
				{
					KukaState = 0;
					PositionOrder = 1;
				}
				else
				{
					PositionOrder++;
				}
					
				cout << "Client_Socket : Sent start pose estimation signal to Server....\n";
				sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType);
				MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));
						
				cout << "Client_Socket : Waitting Kuka move to grasp initial position....\n";
				MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
				
				ObjectPose_EulerAngle[0] = PositionData_Main.BaseObject_EulerAngle[0];
				ObjectPose_EulerAngle[1] = PositionData_Main.BaseObject_EulerAngle[1];
				ObjectPose_EulerAngle[2] = PositionData_Main.BaseObject_EulerAngle[2];
				TCP_PositionData[0] = PositionData_Main.TCP_Position[0];
				TCP_PositionData[1] = PositionData_Main.TCP_Position[1];
				TCP_PositionData[2] = PositionData_Main.TCP_Position[2];
				TCP_PositionData[3] = PositionData_Main.TCP_Position[3];
				
				Arm_PickPoint = PositionData_Main.Arm_PickPoint;
				Grasp_ObjectType = PositionData_Main.Target;


				while ( KukaState == 1 )	// point recalibration
				{
					if (  strncmp( Recieve_ServerData, "W", 1) == 0 && _IsPoseEstimationDone == true ) // G : Watting for pose estimation information
					{
						 
						 //±N²Ä¤»¶b¤¤¤ßÂI ²¾¦Ü §¨¤ö¤¤¤ßÂIªº·L½Õ
						 Xyzabc_CommandData[0] = Arm_PickPoint.x + 50;
						 Xyzabc_CommandData[1] = Arm_PickPoint.y + 0 ;
						 Xyzabc_CommandData[2] = Arm_PickPoint.z - 330; // -335
						 Xyzabc_CommandData[3] =  ( ( ObjectPose_EulerAngle[2] * 180.0f / float (M_PI) ) - TCP_PositionData[3] ) / 2.0;

						 cout << "Grasp_ObjectType = " << Grasp_ObjectType << endl;		
						 cout << "TCP_PositionData = " << TCP_PositionData[1] <<"       "<< TCP_PositionData[2] <<"        "<< TCP_PositionData[3] << endl;
						 cout << "ObjectPose_EulerAngle = " << ObjectPose_EulerAngle[1] << "      " << ObjectPose_EulerAngle[2] << "        " << ObjectPose_EulerAngle[3] << endl;					
						 cout << "Arm_PickPoint = " << Arm_PickPoint << endl;
						 PrintPosition();
						if ( Grasp_ObjectType == 0 || Grasp_ObjectType == 1 )
						{

							 if ( Grasp_ObjectType == 1 )
							 {
								   Xyzabc_CommandData[0] =  Xyzabc_CommandData[0] + 30;
								   Xyzabc_CommandData[1] =  Xyzabc_CommandData[1] + 5;
							 }
							 else if ( Grasp_ObjectType == 0 )
							 {
								 Xyzabc_CommandData[1] = Xyzabc_CommandData[1] + 5;	
								 Xyzabc_CommandData[0] =  Xyzabc_CommandData[0] + 30;
							 }

							if ( ( Xyzabc_CommandData[3] <= 0 && Xyzabc_CommandData[3] > -22.5 ) || ( Xyzabc_CommandData[3] <= -90 && Xyzabc_CommandData[3] > -112.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] * 2 ) - 90;
							}
							else if ( ( Xyzabc_CommandData[3] <= -22.5 && Xyzabc_CommandData[3] > -45 ) || ( Xyzabc_CommandData[3] <= -112.5 && Xyzabc_CommandData[3] > -135 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 22.5 ) * 2 - 45;
							}
							else if ( ( Xyzabc_CommandData[3] <= -45 && Xyzabc_CommandData[3] > -67.5 ) || ( Xyzabc_CommandData[3] <= -135 && Xyzabc_CommandData[3] > -157.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[0] += 20; 
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 45 ) * 2;
							}
							else if ( ( Xyzabc_CommandData[3] <= -67.5 && Xyzabc_CommandData[3] >= -90 ) || ( Xyzabc_CommandData[3] <= -157.5 && Xyzabc_CommandData[3] > -180 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 67.5 ) * 2 + 45;
								
							}
						}
						else if ( Grasp_ObjectType == 2 )
						{
							Xyzabc_CommandData[0] =  Xyzabc_CommandData[0] + 30;
							if ( ( Xyzabc_CommandData[3] <= 0 && Xyzabc_CommandData[3] > -22.5 ) || ( Xyzabc_CommandData[3] <= -90 && Xyzabc_CommandData[3] > -112.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] * 2 ) - 45;
							}
							else if ( ( Xyzabc_CommandData[3] <= -22.5 && Xyzabc_CommandData[3] > -45 ) || ( Xyzabc_CommandData[3] <= -112.5 && Xyzabc_CommandData[3] > -135 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 22.5 ) * 2;
							}
							else if ( ( Xyzabc_CommandData[3] <= -45 && Xyzabc_CommandData[3] > -67.5 ) || ( Xyzabc_CommandData[3] <= -135 && Xyzabc_CommandData[3] > -157.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 45 ) * 2 + 45;
							}
							else if ( ( Xyzabc_CommandData[3] <= -67.5 && Xyzabc_CommandData[3] >= -90 ) || ( Xyzabc_CommandData[3] <= -157.5 && Xyzabc_CommandData[3] > -180 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 67.5 ) * 2 - 90;
								
							}
						}

						KukaState = 2;
					}
					else if ( _IsPoseEstimationDone == false )
					{
						KukaState = 8;
					}
					else 
					{
						cout << "Client_Socket : Something wrong!!!\n";
					}
					Sleep(50);
				}			
			}
			else if ( KukaState == 2)
			{
				cout <<"=========================== KukaState = 2  : waiting KUKA finish picking============================= \n";
				cout << "Client_Socket : Sent Target Pose to Server....\n";
				PrintPosition();
				sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType);
				MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));
				

				cout << "Client_Socket : Waitting Kuka complete grasp target....\n";
				MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
				while ( KukaState == 2)
				{
					if (  strncmp( Recieve_ServerData, "CA", 2) == 0 ) // CA :¦AÂ^¨ú·s¼v¹³
					{
						 KukaState = 5;
					}
					else 
					{
						cout << "Client_Socket : Something wrong!!!\n";
					}
					Sleep(50);
				}
			}
			else if ( KukaState == 5 )
			{

				cout <<"=========================== KukaState = 5 ============================= \n";
				//PrintPosition();
				sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType); 
				MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));


				cout << "Client_Socket : Waitting Kuka move to capture image position....\n";
				cout << "Client_Socket : Waitting Kuka sent TCP Position ....\n";
				MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);

				if (  strncmp( Recieve_ServerData, "I", 1) == 0 ) // I : Require new TCP Position 
				{
						sprintf( Sent_ClientData, "STCP_GetE" );
						MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));

						MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
						decode_TCPPosition( Recieve_ServerData );

						sprintf( Sent_ClientData, "STCP_DoneE" );
						MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));
						MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);

						while ( KukaState == 5 )
						{
							if (  strncmp( Recieve_ServerData, "C", 1) == 0 ) // C : Capture Image 
							{
									_IsPoseEstimationDone = false;

										/*
										 *   capture image
										 */
											 //KinectObj.SceneToPCDProcessing();
											// yml2pcd( KinectObj.Scene_ymlName, "Scene.pcd", KinectObj, PoseEstimationObj.getSceneCloud(), segmentation_Range, 1, show_Mode);
											 //delete [] KinectObj.Scene_ymlName;

											 KukaState = 7;
											 cout <<"=========================== KukaState = 7 ============================= \n";
											 cout << "Client_Socket : Sent capture Image Done to Server....\n";
											 //PrintPosition();

											 sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType);	//put the object back
											 MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));

											 cout << "Client_Socket : Waitting Kuka sent Start Pick before object signal....\n";
											 MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);





											 while ( KukaState == 7 )
											 {
												 if (  strncmp( Recieve_ServerData, "G", 1) == 0 ) // G :Kuka Start to pick before object
												 {
													    break;
												 }
												 else 
												 {
														cout << "Client_Socket : Something wrong!!!\n";
												 }
												 Sleep(50);
											 }
										 
											 
											 /*voxelGrid_Filter( PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getDownsampling_SceneCloud(), Scene_Voxel_radius );
											 compute_SACSegmentationFromNormals( PoseEstimationObj.getDownsampling_SceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), SACSegmentationFromNormal_radius, 0);
											 if ( PoseEstimationObj.getSceneSegmentationCloud()->empty() || PoseEstimationObj.getSceneSegmentationCloud()->size() < 100 )
											 {
												  KukaState = 0;
												  break;
											 }*/
						
											/*
											 *   pose estimation
											 */
											
											/*compute_VotingEstimation_OnlinePhase( RecognitionPCD_Viewer, PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), CADDatabaseObj.getCADModel_OriginalPCDVector(), CADModel_Number, Scene_Normal_radius , Clustter_Position, Cluster_Rotation, SamplingRate, Arm_PickPoint, TCP_PositionData, ObjectPose_EulerAngle, Grasp_ObjectType, _IsPoseEstimationDone);
											if ( !_IsPoseEstimationDone )
											{
												 KukaState = 0;
												 break;
											}*/



									
														
								if (  KukaState != 0 )
								{
									KukaState = 6;
									break;
								}
						}
						else 
						{
							cout << "Client_Socket : Something wrong!!!\n";
						}
							Sleep(50);
					}


						

				}
				else 
				{
					cout << "Client_Socket : Something wrong!!!\n";
				}


			}
			else if ( KukaState == 6 )
			{
				cout <<"=========================== KukaState = 6 ============================= \n";	
				
				cout << "Client_Socket : Sent start pose estimation information to Server....\n";
				sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType);
				MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));

				cout << "Client_Socket : Waitting Kuka move to grasp initial position....\n";
				MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);


				cmp = PositionData_Main.NoneZeroPosition - PositionOrder;
				if(cmp < 0)
				{
					KukaState = 0;
					PositionOrder = 1;
				}
				else
				{
					TransferDatatoMain(&PositionData_Main ,PositionOrder);
					PositionOrder++;
				}

				int kk;


				ObjectPose_EulerAngle[0] = PositionData_Main.BaseObject_EulerAngle[0];
				ObjectPose_EulerAngle[1] = PositionData_Main.BaseObject_EulerAngle[1];
				ObjectPose_EulerAngle[2] = PositionData_Main.BaseObject_EulerAngle[2];
				TCP_PositionData[0] = PositionData_Main.TCP_Position[0];
				TCP_PositionData[1] = PositionData_Main.TCP_Position[1];
				TCP_PositionData[2] = PositionData_Main.TCP_Position[2];
				TCP_PositionData[3] = PositionData_Main.TCP_Position[3];

				Arm_PickPoint = PositionData_Main.Arm_PickPoint;
				Grasp_ObjectType = PositionData_Main.Target;	


				while ( KukaState == 6)
				{
					if (  strncmp( Recieve_ServerData, "W", 1) == 0 /*&& _IsPoseEstimationDone == true*/ ) // G : Watting for pose estimation information
					{
						 Xyzabc_CommandData[0] = Arm_PickPoint.x + 50;
						 Xyzabc_CommandData[1] = Arm_PickPoint.y + 0 ;
						 Xyzabc_CommandData[2] = Arm_PickPoint.z - 330; // -335
						 Xyzabc_CommandData[3] =  ( ( ObjectPose_EulerAngle[2] * 180.0f / float (M_PI) ) - TCP_PositionData[3] ) / 2.0;

						cout << "Grasp_ObjectType = " << Grasp_ObjectType << endl;		
						cout << "TCP_PositionData = " << TCP_PositionData[1] <<"       "<< TCP_PositionData[2] <<"        "<< TCP_PositionData[3] << endl;
						cout << "ObjectPose_EulerAngle = " << ObjectPose_EulerAngle[1] << "      " << ObjectPose_EulerAngle[2] << "        " << ObjectPose_EulerAngle[3] << endl;						 
						PrintPosition();
						 
						if ( Grasp_ObjectType == 0 || Grasp_ObjectType == 1 )
						{
							
							if ( Grasp_ObjectType == 1 )
							 {
								   Xyzabc_CommandData[0] =  Xyzabc_CommandData[0] + 30;
								   Xyzabc_CommandData[1] =  Xyzabc_CommandData[1] + 5;
							 }
							 else if ( Grasp_ObjectType == 0 )
							 {
								 Xyzabc_CommandData[1] = Xyzabc_CommandData[1] + 5;	
								 Xyzabc_CommandData[0] =  Xyzabc_CommandData[0] + 30;
							 }


							if ( ( Xyzabc_CommandData[3] <= 0 && Xyzabc_CommandData[3] > -22.5 ) || ( Xyzabc_CommandData[3] <= -90 && Xyzabc_CommandData[3] > -112.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] * 2 ) - 90;
							}
							else if ( ( Xyzabc_CommandData[3] <= -22.5 && Xyzabc_CommandData[3] > -45 ) || ( Xyzabc_CommandData[3] <= -112.5 && Xyzabc_CommandData[3] > -135 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 22.5 ) * 2 - 45;
							}
							else if ( ( Xyzabc_CommandData[3] <= -45 && Xyzabc_CommandData[3] > -67.5 ) || ( Xyzabc_CommandData[3] <= -135 && Xyzabc_CommandData[3] > -157.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								Xyzabc_CommandData[0] += 20; 
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 45 ) * 2;
							}
							else if ( ( Xyzabc_CommandData[3] <= -67.5 && Xyzabc_CommandData[3] >= -90 ) || ( Xyzabc_CommandData[3] <= -157.5 && Xyzabc_CommandData[3] > -180 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 67.5 ) * 2 + 45;
								
							}
						}


						else if ( Grasp_ObjectType == 2 )
						{
							Xyzabc_CommandData[0] =  Xyzabc_CommandData[0] + 30;
							if ( ( Xyzabc_CommandData[3] <= 0 && Xyzabc_CommandData[3] > -22.5 ) || ( Xyzabc_CommandData[3] <= -90 && Xyzabc_CommandData[3] > -112.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] * 2 ) - 45;
							}
							else if ( ( Xyzabc_CommandData[3] <= -22.5 && Xyzabc_CommandData[3] > -45 ) || ( Xyzabc_CommandData[3] <= -112.5 && Xyzabc_CommandData[3] > -135 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 22.5 ) * 2;
							}
							else if ( ( Xyzabc_CommandData[3] <= -45 && Xyzabc_CommandData[3] > -67.5 ) || ( Xyzabc_CommandData[3] <= -135 && Xyzabc_CommandData[3] > -157.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 45 ) * 2 + 45;
							}
							else if ( ( Xyzabc_CommandData[3] <= -67.5 && Xyzabc_CommandData[3] >= -90 ) || ( Xyzabc_CommandData[3] <= -157.5 && Xyzabc_CommandData[3] > -180 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 67.5 ) * 2 - 90;
								
							}
						}

						 KukaState = 2;
					}
					else 
					{
						cout << "Client_Socket : Something wrong!!!\n";
					}
					Sleep(50);
				}
			}
			else if ( KukaState == -1 )
			{
				cout << "Client_Socket : Exit\n";
				sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType);
				MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));
				break;
			}
		}

		/*while( check_OK )
		{
			if ( KukaState == 0)
			{
				cout << "================================ KukaState = 0==========================\n";

				sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType); 
				MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));
				PrintPosition();

				cout << "Client_Socket : Waitting Kuka move to capture image position....\n";
				cout << "Client_Socket : Waitting Kuka sent TCP Position ....\n";
				MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
				if (  strncmp( Recieve_ServerData, "I", 1) == 0 ) // I : Require now TCP Position 
				{
						sprintf( Sent_ClientData, "STCP_GetE" );
						MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));

						MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
						decode_TCPPosition( Recieve_ServerData );

						sprintf( Sent_ClientData, "STCP_DoneE" );
						MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));

				}
				else 
				{
					cout << "Client_Socket : Something wrong!!!\n";
				}


				MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
				while ( KukaState == 0 )
				{
					if (  strncmp( Recieve_ServerData, "C", 1) == 0 ) // C : Capture Image 
					{
						 KukaState = 1;
					}
					else 
					{
						cout << "Client_Socket : Something wrong!!!\n";
					}
					Sleep(50);
				}
			}
			else if ( KukaState == 1)
			{
				
					cout <<"=========================== KukaState = 1 : image catching and pose estimation============================= \n";
					_IsPoseEstimationDone = false;
					int CaptureImage_Again = 1;
					int Times_Counter = 0;
					while ( !_IsPoseEstimationDone  )
					{
						
						 //   ¦¹°Ï°ìÂ^¨ú¼v¹³
						 
						
						 KinectObj.SceneToPCDProcessing();
						 yml2pcd( KinectObj.Scene_ymlName, "Scene.pcd", KinectObj, PoseEstimationObj.getSceneCloud(), segmentation_Range, 1, show_Mode);
						 delete [] KinectObj.Scene_ymlName;

						 voxelGrid_Filter( PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getDownsampling_SceneCloud(), Scene_Voxel_radius );
						 compute_SACSegmentationFromNormals( PoseEstimationObj.getDownsampling_SceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), SACSegmentationFromNormal_radius, 0);
						 if ( PoseEstimationObj.getSceneSegmentationCloud()->empty() || PoseEstimationObj.getSceneSegmentationCloud()->size() < 100 )
						 {
							 Times_Counter++;
							 if ( Times_Counter < 3 )
							 {
								 continue;
							 }
							 else
							 {
								 cout << "System : Capture Image again...? ( Enter : 1 -> again, 0 -> end ) \n";
								 cin >> CaptureImage_Again ;
								 if ( CaptureImage_Again == 1)
								 {
										continue;
								 }
								 else
								 {
							 		 KukaState = -1;
									 break;
								 }
							 }
								 
													
							 }
						
						
						  //  ¦¹°Ï°ì¿ëÃÑ«ººA
						 					
						 compute_VotingEstimation_OnlinePhase( RecognitionPCD_Viewer, PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), CADDatabaseObj.getCADModel_OriginalPCDVector(), CADModel_Number, Scene_Normal_radius , Clustter_Position, Cluster_Rotation, SamplingRate, Arm_PickPoint, TCP_PositionData, ObjectPose_EulerAngle, Grasp_ObjectType, _IsPoseEstimationDone);

					     if ( !_IsPoseEstimationDone )
						 {
							 Times_Counter++;
							 if ( Times_Counter <= 3 )
							 {
								 continue;
							 }
							 else
							 {
								 cout << "System : Capture Image again...? ( Enter : 1 -> again, 0 -> end ) \n";
								 cin >> CaptureImage_Again ;
								 if ( CaptureImage_Again == 1)
								 {
										continue;
								 }
								 else
								 {
							 		 KukaState = -1;
									 break;
								 }
							 }
						 }
					}

				TransferDatatoMain(&PositionData_Main ,1);
					
				cout << "Client_Socket : Sent start pose estimation signal to Server....\n";
				sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType);
				MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));
						
				cout << "Client_Socket : Waitting Kuka move to grasp initial position....\n";
				MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
				
				ObjectPose_EulerAngle[0] = PositionData_Main.BaseObject_EulerAngle[0];
				ObjectPose_EulerAngle[1] = PositionData_Main.BaseObject_EulerAngle[1];
				ObjectPose_EulerAngle[2] = PositionData_Main.BaseObject_EulerAngle[2];
				Arm_PickPoint = PositionData_Main.Arm_PickPoint;
				Grasp_ObjectType = PositionData_Main.Target;


				while ( KukaState == 1 )	// point recalibration
				{
					if (  strncmp( Recieve_ServerData, "W", 1) == 0 && _IsPoseEstimationDone == true ) // G : Watting for pose estimation information
					{
						 
						 //±N²Ä¤»¶b¤¤¤ßÂI ²¾¦Ü §¨¤ö¤¤¤ßÂIªº·L½Õ
						 Xyzabc_CommandData[0] = Arm_PickPoint.x + 50;
						 Xyzabc_CommandData[1] = Arm_PickPoint.y + 0 ;
						 Xyzabc_CommandData[2] = Arm_PickPoint.z - 330; // -335
						 Xyzabc_CommandData[3] =  ( ( ObjectPose_EulerAngle[2] * 180.0f / float (M_PI) ) - TCP_PositionData[3] ) / 2.0;

						 cout << "Grasp_ObjectType = " << Grasp_ObjectType << endl;		
						 cout << "TCP_PositionData = " << TCP_PositionData[1] <<"       "<< TCP_PositionData[2] <<"        "<< TCP_PositionData[3] << endl;
						 cout << "ObjectPose_EulerAngle = " << ObjectPose_EulerAngle[1] << "      " << ObjectPose_EulerAngle[2] << "        " << ObjectPose_EulerAngle[3] << endl;					
						 cout << "Arm_PickPoint = " << Arm_PickPoint << endl;

						if ( Grasp_ObjectType == 0 || Grasp_ObjectType == 1 )
						{

							 if ( Grasp_ObjectType == 1 )
							 {
								   Xyzabc_CommandData[0] =  Xyzabc_CommandData[0] + 30;
								   Xyzabc_CommandData[1] =  Xyzabc_CommandData[1] + 5;
							 }
							 else if ( Grasp_ObjectType == 0 )
							 {
								 Xyzabc_CommandData[1] = Xyzabc_CommandData[1] + 5;	
								 Xyzabc_CommandData[0] =  Xyzabc_CommandData[0] + 30;
							 }

							if ( ( Xyzabc_CommandData[3] <= 0 && Xyzabc_CommandData[3] > -22.5 ) || ( Xyzabc_CommandData[3] <= -90 && Xyzabc_CommandData[3] > -112.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] * 2 ) - 90;
							}
							else if ( ( Xyzabc_CommandData[3] <= -22.5 && Xyzabc_CommandData[3] > -45 ) || ( Xyzabc_CommandData[3] <= -112.5 && Xyzabc_CommandData[3] > -135 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 22.5 ) * 2 - 45;
							}
							else if ( ( Xyzabc_CommandData[3] <= -45 && Xyzabc_CommandData[3] > -67.5 ) || ( Xyzabc_CommandData[3] <= -135 && Xyzabc_CommandData[3] > -157.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[0] += 20; 
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 45 ) * 2;
							}
							else if ( ( Xyzabc_CommandData[3] <= -67.5 && Xyzabc_CommandData[3] >= -90 ) || ( Xyzabc_CommandData[3] <= -157.5 && Xyzabc_CommandData[3] > -180 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 67.5 ) * 2 + 45;
								
							}
						}
						else if ( Grasp_ObjectType == 2 )
						{
							Xyzabc_CommandData[0] =  Xyzabc_CommandData[0] + 30;
							if ( ( Xyzabc_CommandData[3] <= 0 && Xyzabc_CommandData[3] > -22.5 ) || ( Xyzabc_CommandData[3] <= -90 && Xyzabc_CommandData[3] > -112.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] * 2 ) - 45;
							}
							else if ( ( Xyzabc_CommandData[3] <= -22.5 && Xyzabc_CommandData[3] > -45 ) || ( Xyzabc_CommandData[3] <= -112.5 && Xyzabc_CommandData[3] > -135 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 22.5 ) * 2;
							}
							else if ( ( Xyzabc_CommandData[3] <= -45 && Xyzabc_CommandData[3] > -67.5 ) || ( Xyzabc_CommandData[3] <= -135 && Xyzabc_CommandData[3] > -157.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 45 ) * 2 + 45;
							}
							else if ( ( Xyzabc_CommandData[3] <= -67.5 && Xyzabc_CommandData[3] >= -90 ) || ( Xyzabc_CommandData[3] <= -157.5 && Xyzabc_CommandData[3] > -180 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 67.5 ) * 2 - 90;
								
							}
						}

						KukaState = 2;
					}
					else if ( _IsPoseEstimationDone == false )
					{
						KukaState = 8;
					}
					else 
					{
						cout << "Client_Socket : Something wrong!!!\n";
					}
					Sleep(50);
				}

			cout << "============= after calibration   ========"<< endl;
			PrintPosition();

			}
			else if ( KukaState == 2)
			{
				cout <<"=========================== KukaState = 2  : waiting KUKA finish picking============================= \n";
				cout << "Client_Socket : Sent Target Pose to Server....\n";
				sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType);
				MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));
				PrintPosition();
				

				cout << "Client_Socket : Waitting Kuka complete grasp target....\n";
				MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
				while ( KukaState == 2)
				{
					if (  strncmp( Recieve_ServerData, "CA", 2) == 0 ) // CA :¦AÂ^¨ú·s¼v¹³
					{//--   5--->7
						 KukaState = 5;
					}
					else 
					{
						cout << "Client_Socket : Something wrong!!!\n";
					}
					Sleep(50);
				}
			}
			else if ( KukaState == 5 )
			{

				cout <<"=========================== KukaState = 5 ============================= \n";
				sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType); 
				MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));

				PrintPosition();

				cout << "Client_Socket : Waitting Kuka move to capture image position....\n";
				cout << "Client_Socket : Waitting Kuka sent TCP Position ....\n";
				MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
				if (  strncmp( Recieve_ServerData, "I", 1) == 0 ) // I : Require new TCP Position 
				{
						sprintf( Sent_ClientData, "STCP_GetE" );
						MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));

						MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
						decode_TCPPosition( Recieve_ServerData );

						sprintf( Sent_ClientData, "STCP_DoneE" );
						MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));

						MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
						while ( KukaState == 5 )
						{
							if (  strncmp( Recieve_ServerData, "C", 1) == 0 ) // C : Capture Image 
							{
									_IsPoseEstimationDone = false;

										
										 //   capture image
										 
											 KinectObj.SceneToPCDProcessing();
											 yml2pcd( KinectObj.Scene_ymlName, "Scene.pcd", KinectObj, PoseEstimationObj.getSceneCloud(), segmentation_Range, 1, show_Mode);
											 delete [] KinectObj.Scene_ymlName;

											 KukaState = 7;
											 cout << "Client_Socket : Sent capture Image Done to Server....\n";
											 sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType);	//put the object back
											 MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));

											 cout <<"=========================== KukaState = 7 ============================= \n";
											 PrintPosition();

											 cout << "Client_Socket : Waitting Kuka sent Start Pick before object signal....\n";
											 MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);
											 while ( KukaState == 7 )
											 {
												 if (  strncmp( Recieve_ServerData, "G", 1) == 0 ) // G :Kuka Start to pick before object
												 {
													    break;
												 }
												 else 
												 {
														cout << "Client_Socket : Something wrong!!!\n";
												 }
												 Sleep(50);
											 }
										 
											 
											 voxelGrid_Filter( PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getDownsampling_SceneCloud(), Scene_Voxel_radius );
											 compute_SACSegmentationFromNormals( PoseEstimationObj.getDownsampling_SceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), SACSegmentationFromNormal_radius, 0);
											 if ( PoseEstimationObj.getSceneSegmentationCloud()->empty() || PoseEstimationObj.getSceneSegmentationCloud()->size() < 100 )
											 {
												  KukaState = 0;
												  break;
											 }
						
											
											 //   pose estimation
											 
											
											compute_VotingEstimation_OnlinePhase( RecognitionPCD_Viewer, PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), CADDatabaseObj.getCADModel_OriginalPCDVector(), CADModel_Number, Scene_Normal_radius , Clustter_Position, Cluster_Rotation, SamplingRate, Arm_PickPoint, TCP_PositionData, ObjectPose_EulerAngle, Grasp_ObjectType, _IsPoseEstimationDone);

											if ( !_IsPoseEstimationDone )
											{
												 KukaState = 0;
												 break;
											}
									
														
								if (  KukaState != 0 )
								{
									KukaState = 6;
									break;
								}
						}
						else 
						{
							cout << "Client_Socket : Something wrong!!!\n";
						}
							Sleep(50);
					}


						

				}
				else 
				{
					cout << "Client_Socket : Something wrong!!!\n";
				}


			}
			else if ( KukaState == 6 )
			{
				cout <<"=========================== KukaState = 6 ============================= \n";		

				cout << "Client_Socket : Sent start pose estimation information to Server....\n";
				sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType);
				MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));

				PrintPosition();

				cout << "Client_Socket : Waitting Kuka move to grasp initial position....\n";
				MySocket.Read( MySocket.getMySocket(), Recieve_ServerData, 1024, ret_len);

				while ( KukaState == 6)
				{
					if (  strncmp( Recieve_ServerData, "W", 1) == 0 && _IsPoseEstimationDone == true ) // G : Watting for pose estimation information
					{
						 Xyzabc_CommandData[0] = Arm_PickPoint.x + 50;
						 Xyzabc_CommandData[1] = Arm_PickPoint.y + 0 ;
						 Xyzabc_CommandData[2] = Arm_PickPoint.z - 330; // -335
						 Xyzabc_CommandData[3] =  ( ( ObjectPose_EulerAngle[2] * 180.0f / float (M_PI) ) - TCP_PositionData[3] ) / 2.0;

						cout << "Grasp_ObjectType = " << Grasp_ObjectType << endl;		
						cout << "TCP_PositionData = " << TCP_PositionData[1] <<"       "<< TCP_PositionData[2] <<"        "<< TCP_PositionData[3] << endl;
						cout << "ObjectPose_EulerAngle = " << ObjectPose_EulerAngle[1] << "      " << ObjectPose_EulerAngle[2] << "        " << ObjectPose_EulerAngle[3] << endl;						 
						
						 
						if ( Grasp_ObjectType == 0 || Grasp_ObjectType == 1 )
						{
							
							if ( Grasp_ObjectType == 1 )
							 {
								   Xyzabc_CommandData[0] =  Xyzabc_CommandData[0] + 30;
								   Xyzabc_CommandData[1] =  Xyzabc_CommandData[1] + 5;
							 }
							 else if ( Grasp_ObjectType == 0 )
							 {
								 Xyzabc_CommandData[1] = Xyzabc_CommandData[1] + 5;	
								 Xyzabc_CommandData[0] =  Xyzabc_CommandData[0] + 30;
							 }


							if ( ( Xyzabc_CommandData[3] <= 0 && Xyzabc_CommandData[3] > -22.5 ) || ( Xyzabc_CommandData[3] <= -90 && Xyzabc_CommandData[3] > -112.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] * 2 ) - 90;
							}
							else if ( ( Xyzabc_CommandData[3] <= -22.5 && Xyzabc_CommandData[3] > -45 ) || ( Xyzabc_CommandData[3] <= -112.5 && Xyzabc_CommandData[3] > -135 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 22.5 ) * 2 - 45;
							}
							else if ( ( Xyzabc_CommandData[3] <= -45 && Xyzabc_CommandData[3] > -67.5 ) || ( Xyzabc_CommandData[3] <= -135 && Xyzabc_CommandData[3] > -157.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								Xyzabc_CommandData[0] += 20; 
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 45 ) * 2;
							}
							else if ( ( Xyzabc_CommandData[3] <= -67.5 && Xyzabc_CommandData[3] >= -90 ) || ( Xyzabc_CommandData[3] <= -157.5 && Xyzabc_CommandData[3] > -180 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 67.5 ) * 2 + 45;
								
							}
						}


						else if ( Grasp_ObjectType == 2 )
						{
							Xyzabc_CommandData[0] =  Xyzabc_CommandData[0] + 30;
							if ( ( Xyzabc_CommandData[3] <= 0 && Xyzabc_CommandData[3] > -22.5 ) || ( Xyzabc_CommandData[3] <= -90 && Xyzabc_CommandData[3] > -112.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] * 2 ) - 45;
							}
							else if ( ( Xyzabc_CommandData[3] <= -22.5 && Xyzabc_CommandData[3] > -45 ) || ( Xyzabc_CommandData[3] <= -112.5 && Xyzabc_CommandData[3] > -135 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 22.5 ) * 2;
							}
							else if ( ( Xyzabc_CommandData[3] <= -45 && Xyzabc_CommandData[3] > -67.5 ) || ( Xyzabc_CommandData[3] <= -135 && Xyzabc_CommandData[3] > -157.5 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 45 ) * 2 + 45;
							}
							else if ( ( Xyzabc_CommandData[3] <= -67.5 && Xyzabc_CommandData[3] >= -90 ) || ( Xyzabc_CommandData[3] <= -157.5 && Xyzabc_CommandData[3] > -180 ) )
							{
								if ( fabs( Xyzabc_CommandData[3] ) > 90 )
								{
									Xyzabc_CommandData[3] += 90;
								}
								
								Xyzabc_CommandData[3] = -( Xyzabc_CommandData[3] + 67.5 ) * 2 - 90;
								
							}
						}

						 KukaState = 2;
					}
					else 
					{
						cout << "Client_Socket : Something wrong!!!\n";
					}
					Sleep(50);
				}
			}
			else if ( KukaState == -1 )
			{
				cout << "Client_Socket : Exit\n";
				sprintf( Sent_ClientData, "X%3.3fY%3.3fZ%3.3fA%3.3fB%3.3fC%3.3fS%dO%dE", Xyzabc_CommandData[0], Xyzabc_CommandData[1], Xyzabc_CommandData[2], Xyzabc_CommandData[3], Xyzabc_CommandData[4], Xyzabc_CommandData[5], KukaState, Grasp_ObjectType);
				MySocket.Write( MySocket.getMySocket(), Sent_ClientData, sizeof( Sent_ClientData ));
				break;
			}
		}*/

	}
	else
	{
		cout << "Error : Create a virtual client socket fail!!!\n";
	}


}

static unsigned __stdcall StartSocket_Thread(void * pThis) 
{
	StartSocket();
	return 1;
}




void Auto_RecognitionFun()
{
	float background_color[3] = { 0, 0, 0 };
	float point_color[3] = { 255, 255, 255 };
	int CaptureImage_Again = 1;

	compute_VotingEstimation_OffinePhase( CADModel_Number, AllCADModel_pcdFileName, CADModel_Normal_radius, HashMapSearch_Position, HashMapSearch_Rotation);
	while (CaptureImage_Again==1)
	{
		

		/*
		 *   catch the image
		 */
				KinectObj.SceneToPCDProcessing();
				yml2pcd( KinectObj.Scene_ymlName, "Scene.pcd", KinectObj, PoseEstimationObj.getSceneCloud(), segmentation_Range, 1, show_Mode);
				delete [] KinectObj.Scene_ymlName;
				
				voxelGrid_Filter( PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getDownsampling_SceneCloud(), Scene_Voxel_radius );
				compute_SACSegmentationFromNormals( PoseEstimationObj.getDownsampling_SceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), SACSegmentationFromNormal_radius, 1);

		
		/*
		 *   pose estimating
		 */
		
		compute_VotingEstimation_OnlinePhase( RecognitionPCD_Viewer, PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), CADDatabaseObj.getCADModel_OriginalPCDVector(), CADModel_Number, Scene_Normal_radius , Clustter_Position, Cluster_Rotation, SamplingRate, Arm_PickPoint, TCP_PositionData, ObjectPose_EulerAngle, Grasp_ObjectType, _IsPoseEstimationDone);
		Xyzabc_CommandData[0] = Arm_PickPoint.x + 50;
		Xyzabc_CommandData[1] = Arm_PickPoint.y + 0 ;
		Xyzabc_CommandData[2] = Arm_PickPoint.z - 330; // -335
		Xyzabc_CommandData[3] =  ( ( ObjectPose_EulerAngle[2] * 180.0f / float (M_PI) ) - TCP_PositionData[3] ) / 2.0;

		cout << "Grasp_ObjectType = " << Grasp_ObjectType << endl;		
		cout << "TCP_PositionData = " << TCP_PositionData[1] <<"       "<< TCP_PositionData[2] <<"        "<< TCP_PositionData[3] << endl;
		cout << "ObjectPose_EulerAngle = " << ObjectPose_EulerAngle[1] << "      " << ObjectPose_EulerAngle[2] << "        " << ObjectPose_EulerAngle[3] << endl;					
		cout << "Arm_PickPoint = " << Arm_PickPoint << endl;

		PrintPosition();

		cout << " CaptureImage_Again : (1)->Yes, 0->No" << endl;
		cin >> CaptureImage_Again;
	}
}

void Manual_RecognitionFun()
{
	float background_color[3] = { 0, 0, 0 };
	float point_color[3] = { 255, 255, 255 };
	float reference_point_color[3] = { 255, 0, 0 };
	int CAD_Type = 1;

	/*
	 *  generaing database
	 */
	compute_VotingEstimation_OffinePhase( CADModel_Number, CADModel_pcdFileName, CADModel_Normal_radius, HashMapSearch_Position, HashMapSearch_Rotation);
	//compute_VotingEstimation_OffinePhase( CADModel_Number, AllCADModel_pcdFileName, CADModel_Normal_radius, HashMapSearch_Position, HashMapSearch_Rotation);


	/*
	 *   pose estimation and recognition
	 */	

	//KinectObj.SceneToPCDProcessing();
	yml2pcd( "Scene.yml", "Scene.pcd", KinectObj, PoseEstimationObj.getSceneCloud(), segmentation_Range, 1, 1);
	//yml2pcd( KinectObj.Scene_ymlName, "Scene.pcd", KinectObj, PoseEstimationObj.getSceneCloud(), segmentation_Range, 1, show_Mode);
	

	//delete [] KinectObj.Scene_ymlName;
	voxelGrid_Filter( PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getDownsampling_SceneCloud(), Scene_Voxel_radius );
	compute_SACSegmentationFromNormals( PoseEstimationObj.getDownsampling_SceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), SACSegmentationFromNormal_radius, 0);			
	//compute_VotingEstimation_OnlinePhase_VerifyPrecision( RecognitionPCD_Viewer, PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), CADDatabaseObj.getCADModel_OriginalPCDVector(), CADModel_Number, Scene_Normal_radius , Clustter_Position, Cluster_Rotation, SamplingRate, Arm_PickPoint, TCP_PositionData, _IsPoseEstimationDone, CAD_Type, CADDatabaseObj.getCADModelCloud());			
	compute_VotingEstimation_OnlinePhase( RecognitionPCD_Viewer, PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), CADDatabaseObj.getCADModel_OriginalPCDVector(), CADModel_Number, Scene_Normal_radius , Clustter_Position, Cluster_Rotation, SamplingRate, Arm_PickPoint, TCP_PositionData, ObjectPose_EulerAngle, Grasp_ObjectType, _IsPoseEstimationDone);

	Xyzabc_CommandData[0] = Arm_PickPoint.x + 50;
	Xyzabc_CommandData[1] = Arm_PickPoint.y + 0 ;
	Xyzabc_CommandData[2] = Arm_PickPoint.z - 330; // -335
	Xyzabc_CommandData[3] =  ( ( ObjectPose_EulerAngle[2] * 180.0f / float (M_PI) ) - TCP_PositionData[3] ) / 2.0;

	cout << "Grasp_ObjectType = " << Grasp_ObjectType << endl;		
	cout << "TCP_PositionData = " << TCP_PositionData[1] <<"       "<< TCP_PositionData[2] <<"        "<< TCP_PositionData[3] << endl;
	cout << "ObjectPose_EulerAngle = " << ObjectPose_EulerAngle[1] << "      " << ObjectPose_EulerAngle[2] << "        " << ObjectPose_EulerAngle[3] << endl;					
	cout << "Arm_PickPoint = " << Arm_PickPoint << endl;

	PrintPosition();
}

void Manual_Fun()
{
	float background_color[3] = { 0, 0, 0 };
	float point_color[3] = { 255, 255, 255 };
	float reference_point_color[3] = { 255, 0, 0 };


	/*
	 *   此區域建立資料庫
	 */
	ply_mesh2pcd("VirtualScene_2_theta_0_phi_0.ply", "VirtualScene_1_theta_0_phi_0.pcd");
	LoadPCD("VirtualScene_1_theta_0_phi_0.pcd", PoseEstimationObj.getSceneCloud() );
	//yml2pcd(  "VirtualScene_1_theta_0_phi_0.yml", "VirtualScene_1_theta_0_phi_0.pcd", KinectObj, PoseEstimationObj.getSceneCloud(), segmentation_Range, 1, show_Mode);
	compute_VotingEstimation_OffinePhase( CADModel_Number, AllCADModel_pcdFileName, CADModel_Normal_radius, HashMapSearch_Position, HashMapSearch_Rotation);
	

	voxelGrid_Filter( PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getDownsampling_SceneCloud(), Scene_Voxel_radius );
	//compute_SACSegmentationFromNormals( PoseEstimationObj.getDownsampling_SceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), SACSegmentationFromNormal_radius, 0);
	compute_VotingEstimation_OnlinePhase( RecognitionPCD_Viewer, PoseEstimationObj.getSceneCloud(),  PoseEstimationObj.getDownsampling_SceneCloud(), CADDatabaseObj.getCADModel_OriginalPCDVector(), CADModel_Number, Scene_Normal_radius , Clustter_Position, Cluster_Rotation, SamplingRate, Arm_PickPoint, TCP_PositionData, ObjectPose_EulerAngle, Grasp_ObjectType, _IsPoseEstimationDone);
	//DisplyXYZPCDbyXYZRGB( PoseEstimationObj.getSceneCloud(), point_color,  background_color,1);

	Xyzabc_CommandData[0] = Arm_PickPoint.x + 50;
	Xyzabc_CommandData[1] = Arm_PickPoint.y + 0 ;
	Xyzabc_CommandData[2] = Arm_PickPoint.z - 330; // -335
	Xyzabc_CommandData[3] =  ( ( ObjectPose_EulerAngle[2] * 180.0f / float (M_PI) ) - TCP_PositionData[3] ) / 2.0;

	cout << "Grasp_ObjectType = " << Grasp_ObjectType << endl;		
	cout << "TCP_PositionData = " << TCP_PositionData[1] <<"       "<< TCP_PositionData[2] <<"        "<< TCP_PositionData[3] << endl;
	cout << "ObjectPose_EulerAngle = " << ObjectPose_EulerAngle[1] << "      " << ObjectPose_EulerAngle[2] << "        " << ObjectPose_EulerAngle[3] << endl;					
	cout << "Arm_PickPoint = " << Arm_PickPoint << endl;

	PrintPosition();
}

void decode_TCPPosition( char * encode_TCP )
{
	char* Xdummy = strchr( encode_TCP, 'X');
	char* Ydummy = strchr( encode_TCP, 'Y');
	char* Zdummy = strchr( encode_TCP, 'Z');
	char* Adummy = strchr( encode_TCP, 'A');
	char* Bdummy = strchr( encode_TCP, 'B');
	char* Cdummy = strchr( encode_TCP, 'C');
	char* Edummy = strchr( encode_TCP, 'E');

	size_t X_count = strlen(Xdummy) - strlen(Ydummy) - 1;
	size_t Y_count = strlen(Ydummy) - strlen(Zdummy) - 1;
	size_t Z_count = strlen(Zdummy) - strlen(Adummy) - 1;
	size_t A_count = strlen(Adummy) - strlen(Bdummy) - 1;
	size_t B_count = strlen(Bdummy) - strlen(Cdummy) - 1;
	size_t C_count = strlen(Cdummy) - strlen(Edummy) - 1;

	char *TCP_X = new char[X_count];
	char *TCP_Y = new char[Y_count];
	char *TCP_Z = new char[Z_count];
	char *TCP_A = new char[A_count];
	char *TCP_B = new char[B_count];
	char *TCP_C = new char[C_count];

	memcpy( TCP_X, Xdummy + 1, X_count );
	memcpy( TCP_Y, Ydummy + 1, Y_count );
	memcpy( TCP_Z, Zdummy + 1, Z_count );
	memcpy( TCP_A, Adummy + 1, A_count );
	memcpy( TCP_B, Bdummy + 1, B_count );
	memcpy( TCP_C, Cdummy + 1, C_count );

	TCP_PositionData[0] = atof(TCP_X);
	TCP_PositionData[1] = atof(TCP_Y);
	TCP_PositionData[2] = atof(TCP_Z);
	TCP_PositionData[3] = atof(TCP_A);
	TCP_PositionData[4] = atof(TCP_B);
	TCP_PositionData[5] = atof(TCP_C);

}

void PrintPosition()
{
	printf("x = %f \n",Xyzabc_CommandData[0]);
	printf("y = %f \n",Xyzabc_CommandData[1]);
	printf("z = %f \n",Xyzabc_CommandData[2]);
	printf("a = %f \n",Xyzabc_CommandData[3]);
	printf("b = %f \n",Xyzabc_CommandData[4]);
	printf("c = %f \n",Xyzabc_CommandData[5]);
}