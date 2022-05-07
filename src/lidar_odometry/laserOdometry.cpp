// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>
#include <tf/transform_listener.h>

#include "common.h"
#include "tic_toc.h"
#include "lidarFactor.hpp"
#include "utility.h"

#define DISTORTION 0


int corner_correspondence = 0, plane_correspondence = 0;

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

int skipFrameNum = 5;
bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;

// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
std::mutex mBuf;

//----------------------------------//D
bool VinsInited = false;
bool VO_LO_equal =false;
bool VO_LO_Comb =true;  
bool VO_LO_skip=false;

double timeVinsHeader=0;     
double timeVinsCurrent=0;    
double timeLaserBuffer=0;     
double timeVinsBuffer=0;     
double LaserHeadertime=0;     
double LaserRate=10.0;     
double CameraRate=10.0;   
double IMURate=100.0;   

//D- visual odometry 
Eigen::Quaterniond vins_w_q(1, 0, 0, 0);
Eigen::Vector3d vins_w_t(0, 0, 0);

Eigen::Quaterniond vins_w_q_prev(1, 0, 0, 0);
Eigen::Vector3d vins_w_t_prev(0, 0, 0);

Eigen::Quaterniond vins_w_q_init(1, 0, 0, 0);
Eigen::Vector3d vins_w_t_init(0, 0, 0);  	

std::queue<nav_msgs::Odometry::ConstPtr> visualOdometryBuf;

Eigen::Quaterniond q_vo_ff; 
Eigen::Vector3d t_vo_ff;
Eigen::Quaterniond q_lo_ff; 
Eigen::Vector3d t_lo_ff;

Eigen::Vector3d q_vo_ff_vec;
Eigen::Vector3d q_lo_ff_vec;

Eigen::Matrix4d VELO_T_CAM;
Eigen::Matrix4d CAM_T_VELO;
Eigen::Matrix4d CAM_T_IMU;
Eigen::Matrix4d IMU_T_CAM;
Eigen::Matrix4d VELO_T_IMU;
Eigen::Matrix4d VELO_T_IMUy;
Eigen::Matrix4d VELO_T_IMUx;

Eigen::Quaterniond q_vo_init; 
Eigen::Vector3d t_vo_init;
Eigen::Vector3d q_vo_init_vec;
Eigen::Vector3d q_lo_init_vec;


// undistort lidar point
void TransformToStart(PointType const *const pi, PointType *const po)
{
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    //s = 1;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame

void TransformToEnd(PointType const *const pi, PointType *const po)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}

void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    mBuf.lock();
    cornerSharpBuf.push(cornerPointsSharp2);
    mBuf.unlock();
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}

//receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
    LaserHeadertime=laserCloudFullRes2->header.stamp.toSec();
    ROS_INFO("\n -------------Scan_Header_time =, %f", LaserHeadertime);
}

void VinsOdometryHandler(const nav_msgs::Odometry::ConstPtr &pose_msg)  //D
{

    mBuf.lock();
    visualOdometryBuf.push(pose_msg);
    mBuf.unlock();
    timeVinsHeader = pose_msg->header.stamp.toSec(); 
    //ROS_INFO("\n -------------Vins_Header_time =, %f", timeVinsHeader);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    nh.param<int>("mapping_skip_frame", skipFrameNum, 2);

    printf("Mapping %d Hz \n", 10 / skipFrameNum);

    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);

    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);

    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);

    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);

    ros::Subscriber subVinsOdometryTime = nh.subscribe<nav_msgs::Odometry>("viloam/vins/odometry/odometry", 100, VinsOdometryHandler);        //D

    //ros::Subscriber subVinsOdometryTime = nh.subscribe<nav_msgs::Odometry>("vins_estimator/odometry", 100, VinsOdometryHandler);        //D

    //ros::Subscriber subVinsOdometryTime = nh.subscribe<nav_msgs::Odometry>("viloam/vins/odometry/imu_propagate", 2000, VinsOdometryHandler);        //D

    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);

    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);

    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

    ros::Publisher pubVisualOdometry = nh.advertise<nav_msgs::Odometry>("/visual_odom_last", 100);  //D


    nav_msgs::Path laserPath;

    int frameCount = 0;
    ros::Rate rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
	ParamServer Param;
        
	VELO_T_IMU=Param.VELO_T_IMU;


	//LVI 
	/*VELO_T_IMU <<  -1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, -1, 0,
              		0, 0, 0, 1;*/


	//Payload-Down
	
	/*VELO_T_IMU <<   0, 0, 1, 0,
			0, -1, 0, 0,
			1, 0, 0, 0,
              		0, 0, 0, 1;*/

	/*IMU_T_CAM<< -0.0009792, 0.00685726, 0.99997601, 0.18648448,
		     0.99931723, -0.03692628, 0.00123178, -0.04199414,
	             0.03693384, 0.99929446, -0.00681642, -0.03693199,
		     0, 0, 0, 1;

        CAM_T_IMU = IMU_T_CAM.inverse();      

   	VELO_T_CAM<< -0.0062, -0.0009, 1.000, 0.0923,
          	      -0.9992, 0.0391, -0.0061, 0.0388,
          	     -0.0391,  -0.9992, -0.0012, -0.0740,
         	      0, 0, 0, 1;

	CAM_T_VELO=VELO_T_CAM.inverse();


 	VELO_T_IMU=VELO_T_CAM*CAM_T_IMU;*/


	/*tf::TransformListener listener;
	tf::StampedTransform transforml;
	try{
	    //listener.waitForTransform("vins_world", "camera_init", ros::Time(0), ros::Duration(0.1));
	    listener.lookupTransform("vins_world", "camera_init", ros::Time(0), transforml);
	} 
	catch (tf::TransformException ex){
	    ROS_ERROR("no tf");
   	    //continue;
	}


	std::cout <<"-------x-------- "<<transforml.getOrigin().x()<< std::endl;
	std::cout <<"-------y-------- "<<transforml.getOrigin().y()<< std::endl;
	std::cout <<"-------z-------- "<<transforml.getOrigin().z()<< std::endl;
	std::cout <<"-------R-------- "<<transforml.getRotation().x()<< std::endl;
	std::cout <<"-------R-------- "<<transforml.getRotation().y()<< std::endl;
	std::cout <<"-------R-------- "<<transforml.getRotation().z()<< std::endl;
	std::cout <<"-------R-------- "<<transforml.getRotation().w()<< std::endl;*/


        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty() && !visualOdometryBuf.empty())
	{
	 	
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
	    ROS_INFO("\n -------------lidar buffer time -------------%f", timeLaserCloudFullRes);	

	

	mBuf.lock();
	for(int i = 0; i < (signed)visualOdometryBuf.size(); ++i)
     	{
		timeVinsBuffer=visualOdometryBuf.front()->header.stamp.toSec();
		if(fabs(timeVinsBuffer-timeLaserCloudFullRes)<1/(CameraRate*2))
		{
			ROS_INFO("\n ------------- Vins Buffer time --------------%f", timeVinsBuffer);	
			ROS_INFO("\n -----------------------------------Equal VO and LO times --------------------------------------- ");
			VO_LO_equal=true;
			VO_LO_skip=false;
		    	vins_w_t.x() = visualOdometryBuf.front()->pose.pose.position.x;                   //D - store VIO previous frame pose
		    	vins_w_t.y() = visualOdometryBuf.front()->pose.pose.position.y;                   
		    	vins_w_t.z() = visualOdometryBuf.front()->pose.pose.position.z;
		    	vins_w_q.w() = visualOdometryBuf.front()->pose.pose.orientation.w;
		    	vins_w_q.x() = visualOdometryBuf.front()->pose.pose.orientation.x;
		    	vins_w_q.y() = visualOdometryBuf.front()->pose.pose.orientation.y;
		    	vins_w_q.z() = visualOdometryBuf.front()->pose.pose.orientation.z;
			break;
		}

		else if(timeVinsBuffer-timeLaserCloudFullRes>1/(CameraRate*2))
		{

            		cornerSharpBuf.pop();
            		cornerLessSharpBuf.pop();
            		surfFlatBuf.pop();
            		surfLessFlatBuf.pop();
            		fullPointsBuf.pop();
			VO_LO_equal=false;
			VO_LO_skip=true;					
			break;	
		}

 		else
		{
		    	vins_w_t_prev.x() = visualOdometryBuf.front()->pose.pose.position.x;                   //D - store VIO previous frame pose
		    	vins_w_t_prev.y() = visualOdometryBuf.front()->pose.pose.position.y;                   
		    	vins_w_t_prev.z() = visualOdometryBuf.front()->pose.pose.position.z;
		    	vins_w_q_prev.w() = visualOdometryBuf.front()->pose.pose.orientation.w;
		    	vins_w_q_prev.x() = visualOdometryBuf.front()->pose.pose.orientation.x;
		    	vins_w_q_prev.y() = visualOdometryBuf.front()->pose.pose.orientation.y;
		    	vins_w_q_prev.z() = visualOdometryBuf.front()->pose.pose.orientation.z;
			visualOdometryBuf.pop();
			VO_LO_skip=true;
		}
	}



		/*std::cout <<"-------vins_w_t_prev.x "<<vins_w_t_prev.x()<< std::endl;        
		std::cout <<"-------vins_w_t_prev.y "<<vins_w_t_prev.y()<< std::endl; 
		std::cout <<"-------vins_w_t_prev.z "<<vins_w_t_prev.z()<< std::endl;		
	
		std::cout <<"-------vins_w_q_prev.x "<<vins_w_q_prev.x()<< std::endl;        
		std::cout <<"-------vins_w_q_prev.y "<<vins_w_q_prev.y()<< std::endl; 
		std::cout <<"-------vins_w_q_prev.z "<<vins_w_q_prev.z()<< std::endl;					
		std::cout <<"-------vins_w_q_prev.w "<<vins_w_q_prev.w()<< std::endl;

		std::cout <<"-------vins_w_t.x "<<vins_w_t.x()<< std::endl;        
		std::cout <<"-------vins_w_t.y "<<vins_w_t.y()<< std::endl; 
		std::cout <<"-------vins_w_t.z "<<vins_w_t.z()<< std::endl;		
	
		std::cout <<"-------vins_w_q.x "<<vins_w_q.x()<< std::endl;        
		std::cout <<"-------vins_w_q.y "<<vins_w_q.y()<< std::endl; 
		std::cout <<"-------vins_w_q.z "<<vins_w_q.z()<< std::endl;					
		std::cout <<"-------vins_w_q.w "<<vins_w_q.w()<< std::endl;*/




        mBuf.unlock();

	if (VO_LO_Comb)											 //D - LO initialization from VIO 
	{

		//Rotation matrices method
		Eigen::Matrix3d R_w_p = vins_w_q_prev.toRotationMatrix();
		Eigen::Matrix3d R_w_c = vins_w_q.toRotationMatrix();
		Eigen::Matrix3d R_p_c = R_w_p.inverse()*R_w_c;

		Eigen::Matrix3d Rff_L= VELO_T_IMU.block<3, 3>(0, 0)*R_p_c*VELO_T_IMU.block<3, 3>(0, 0).inverse();

		t_vo_ff=R_w_p.inverse()*(vins_w_t-vins_w_t_prev);
		q_lo_ff = Eigen::Quaterniond(Rff_L);
		q_lo_ff = q_lo_ff.normalized();  
	    	t_lo_ff = VELO_T_IMU.block<3, 3>(0, 0)*t_vo_ff+VELO_T_IMU.block<3, 1>(0, 3);

		//quat method
		/*t_vo_ff=vins_w_q_prev.inverse()*(vins_w_t-vins_w_t_prev);
		q_vo_ff=vins_w_q_prev.inverse()*vins_w_q;
		q_vo_ff_vec<<q_vo_ff.x(),q_vo_ff.y(), q_vo_ff.z();

		//Transforming to lidar frame
		q_lo_ff_vec=VELO_T_IMU.block<3, 3>(0, 0)*q_vo_ff_vec;
		q_lo_ff.x()= q_lo_ff_vec[0];
		q_lo_ff.y()= q_lo_ff_vec[1];
		q_lo_ff.z()= q_lo_ff_vec[2];
		q_lo_ff.w()= q_vo_ff.w();
	    	t_lo_ff = VELO_T_IMU.block<3, 3>(0, 0)*t_vo_ff+VELO_T_IMU.block<3, 1>(0, 3);	*/


		para_t[0] = t_lo_ff.x();
		para_t[1] = t_lo_ff.y();
		para_t[2] = t_lo_ff.z();

		para_q[0] = q_lo_ff.x();
		para_q[1] = q_lo_ff.y();
		para_q[2] = q_lo_ff.z();
		para_q[3] = q_lo_ff.w();

		/*std::cout <<"-------t_vo_ff.x "<<t_vo_ff.x()<< std::endl;        
		std::cout <<"-------t_vo_ff.y "<<t_vo_ff.y()<< std::endl; 
		std::cout <<"-------t_vo_ff.z "<<t_vo_ff.z()<< std::endl;		
	

		std::cout <<"-------t_lo_ff.x "<<t_lo_ff.x()<< std::endl;        
		std::cout <<"-------t_lo_ff.y "<<t_lo_ff.y()<< std::endl; 
		std::cout <<"-------t_lo_ff.z "<<t_lo_ff.z()<< std::endl;		
	
		std::cout <<"-------q_lo_ff.x "<<q_lo_ff.x()<< std::endl;        
		std::cout <<"-------q_lo_ff.y "<<q_lo_ff.y()<< std::endl; 
		std::cout <<"-------q_lo_ff.z "<<q_lo_ff.z()<< std::endl;					
		std::cout <<"-------q_lo_ff.w "<<q_lo_ff.w()<< std::endl;*/

	}


	  if(!VO_LO_skip)
          {


            if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                printf("unsync messeage!");
                ROS_BREAK();
            }

            mBuf.lock();
            cornerPointsSharp->clear();
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            cornerSharpBuf.pop();

            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();

            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();

            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();

            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            fullPointsBuf.pop();
            mBuf.unlock();

            TicToc t_whole;
            // initializing
            if (!systemInited)
            {
                systemInited = true;
                std::cout << "Initialization finished \n";

		/*Eigen::Matrix3d R_w_p = vins_w_q_init.toRotationMatrix();
		Eigen::Matrix3d R_w_c = vins_w_q.toRotationMatrix();
		Eigen::Matrix3d R_p_c = R_w_p.inverse()*R_w_c;

		Eigen::Matrix3d Rff_L= VELO_T_IMU.block<3, 3>(0, 0)*R_p_c;

		t_vo_ff=R_w_p.inverse()*(vins_w_t-vins_w_t_init);
		q_lo_ff = Eigen::Quaterniond(Rff_L);
		q_lo_ff = q_lo_ff.normalized();  
	    	t_lo_ff = VELO_T_IMU.block<3, 3>(0, 0)*t_vo_ff+VELO_T_IMU.block<3, 1>(0, 3);*/

		Eigen::Matrix3d R_w_c = vins_w_q.toRotationMatrix();
		Eigen::Matrix3d R= VELO_T_IMU.block<3, 3>(0, 0)*R_w_c*VELO_T_IMU.block<3, 3>(0, 0).inverse();
	    	t_lo_ff = VELO_T_IMU.block<3, 3>(0, 0)*vins_w_t+VELO_T_IMU.block<3, 1>(0, 3);
		q_lo_ff = Eigen::Quaterniond(R);
		q_lo_ff = q_lo_ff.normalized();  


		t_w_curr=t_lo_ff;
		q_w_curr=q_lo_ff;

		//t_w_curr=vins_w_t;
		//q_w_curr=vins_w_q;
            }
            else
            {
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();

                TicToc t_opt;
                for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
                {
                    corner_correspondence = 0;
                    plane_correspondence = 0;

                    //ceres::LossFunction *loss_function = NULL;
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    ceres::LocalParameterization *q_parameterization =
                        new ceres::EigenQuaternionParameterization();
                    ceres::Problem::Options problem_options;

                    ceres::Problem problem(problem_options);
                    problem.AddParameterBlock(para_q, 4, q_parameterization);
                    problem.AddParameterBlock(para_t, 3);

                    pcl::PointXYZI pointSel;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;

		   //pose constraints from visual odometry  //D
		   if (VO_LO_Comb)
		    {
                    	
			ceres::CostFunction* vo_function = RelativeRTError::Create(t_lo_ff.x(), t_lo_ff.y(), t_lo_ff.z(),
                                                                                q_lo_ff.w(), q_lo_ff.x(), q_lo_ff.y(), q_lo_ff.z(),
                                                                                0.01, 0.001);

			problem.AddResidualBlock(vo_function, NULL, para_q, para_t);
		    }

                    TicToc t_data;
                    // find correspondence for corner features
                    for (int i = 0; i < cornerPointsSharpNum; ++i)
                    {
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                        }
                        if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                        {
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);

                            double s;
                            if (DISTORTION)
                                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                            else
                                s = 1.0;
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            corner_correspondence++;
                        }
                    }

                    // find correspondence for plane features
                    for (int i = 0; i < surfPointsFlatNum; ++i)
                    {
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];

                            // get closest point's scan ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or lower scan line
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                // if in the higher scan line
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or higher scan line
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    // find nearer point
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            if (minPointInd2 >= 0 && minPointInd3 >= 0)
                            {

                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                            surfPointsFlat->points[i].y,
                                                            surfPointsFlat->points[i].z);
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                                laserCloudSurfLast->points[closestPointInd].y,
                                                                laserCloudSurfLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                                laserCloudSurfLast->points[minPointInd2].y,
                                                                laserCloudSurfLast->points[minPointInd2].z);
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                                laserCloudSurfLast->points[minPointInd3].y,
                                                                laserCloudSurfLast->points[minPointInd3].z);

                                double s;
                                if (DISTORTION)
                                    s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                                else
                                    s = 1.0;
                                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                plane_correspondence++;
                            }
                        }
                    }

                    //printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                    printf("data association time %f ms \n", t_data.toc());

                    if ((corner_correspondence + plane_correspondence) < 10)
                    {
                        printf("less correspondence! *************************************************\n");
                    }

                    TicToc t_solver;
                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR;
                    options.max_num_iterations = 4;
                    options.minimizer_progress_to_stdout = false;
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);
                    printf("solver time %f ms \n", t_solver.toc());
                }
                printf("optimization twice time %f \n", t_opt.toc());

                t_w_curr = t_w_curr + q_w_curr * t_last_curr;
                q_w_curr = q_w_curr * q_last_curr;
            }


		/*std::cout <<"-------para_t[0] "<<para_t[0]<< std::endl;        
		std::cout <<"-------para_t[1] "<<para_t[1]<< std::endl; 
		std::cout <<"-------para_t[2] "<<para_t[2]<< std::endl;		
	
		std::cout <<"-------para_q[0]"<<para_q[0]<< std::endl;        
		std::cout <<"-------para_q[0]"<<para_q[1]<< std::endl; 
		std::cout <<"-------para_q[0]"<<para_q[2]<< std::endl;					
		std::cout <<"-------para_q[0]"<<para_q[3]<< std::endl;*/

		/*std::cout <<"-------t_w_curr.x "<<t_w_curr.x()<< std::endl;        
		std::cout <<"-------t_w_curr.y "<<t_w_curr.y()<< std::endl; 
		std::cout <<"-------t_w_curr.z "<<t_w_curr.z()<< std::endl;		
	
		std::cout <<"-------q_w_curr.x "<<q_w_curr.x()<< std::endl;        
		std::cout <<"-------q_w_curr.y "<<q_w_curr.y()<< std::endl; 
		std::cout <<"-------q_w_curr.z "<<q_w_curr.z()<< std::endl;					
		std::cout <<"-------q_w_curr.w "<<q_w_curr.w()<< std::endl;*/


            TicToc t_pub;

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "/odom	";
            laserOdometry.child_frame_id = "/laser_odom";
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            pubLaserOdometry.publish(laserOdometry);

            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "/odom";
            pubLaserPath.publish(laserPath);

       	    // publis visual odometry	
	    nav_msgs::Odometry VisualOdometry;
	    VisualOdometry.header.frame_id = "/odom";
	    VisualOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
	    VisualOdometry.pose.pose.orientation.x = vins_w_q.x();
	    VisualOdometry.pose.pose.orientation.y = vins_w_q.y();
	    VisualOdometry.pose.pose.orientation.z = vins_w_q.z();
	    VisualOdometry.pose.pose.orientation.w = vins_w_q.w();
	    VisualOdometry.pose.pose.position.x = vins_w_t.x();
	    VisualOdometry.pose.pose.position.y = vins_w_t.y();
	    VisualOdometry.pose.pose.position.z = vins_w_t.z();
	    pubVisualOdometry.publish(VisualOdometry);

            // transform corner features and plane features to the scan end point
            if (0)
            {
                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
                for (int i = 0; i < cornerPointsLessSharpNum; i++)
                {
                    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
                }

                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
                for (int i = 0; i < surfPointsLessFlatNum; i++)
                {
                    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
                }

                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }
            }

            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;
            laserCloudCornerLast = laserCloudTemp;

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;

            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();

            // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

            if (VO_LO_equal)
            {
		    vins_w_t_prev.x() = visualOdometryBuf.front()->pose.pose.position.x;                   //D - store VIO previous frame pose
		    vins_w_t_prev.y() = visualOdometryBuf.front()->pose.pose.position.y;                   
		    vins_w_t_prev.z() = visualOdometryBuf.front()->pose.pose.position.z;
		    vins_w_q_prev.w() = visualOdometryBuf.front()->pose.pose.orientation.w;
		    vins_w_q_prev.x() = visualOdometryBuf.front()->pose.pose.orientation.x;
		    vins_w_q_prev.y() = visualOdometryBuf.front()->pose.pose.orientation.y;
		    vins_w_q_prev.z() = visualOdometryBuf.front()->pose.pose.orientation.z;
		    visualOdometryBuf.pop();
	    }

            if (frameCount % skipFrameNum == 0 && VO_LO_equal)
            {
                frameCount = 0;

                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = "/camera";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }
            printf("publication time %f ms \n", t_pub.toc());
            printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
            if(t_whole.toc() > 100)
                ROS_WARN("odometry process over 100ms");

            frameCount++;
	}
        }
       
        rate.sleep();
    }
    return 0;
}
