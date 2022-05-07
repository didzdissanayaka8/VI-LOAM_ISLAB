#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/NavSatFix.h>
#include </home/didula/GVLOAM/src/src/lidar_odometry/geodetic_conv.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <fstream>

geodetic_converter::GeodeticConverter g_geodetic_converter;

double latitude, longitude, altitude;
bool initGPS=false;
double x, y, z;
double tempx, tempy, tempz;
int GTframeCount = 0;

Eigen::Quaterniond vins_w_q_init(1, 0, 0, 0);
Eigen::Vector3d vins_w_t_init(0, 0, 0);
Eigen::Vector3d t_world_vocurr;
Eigen::Quaterniond q_world_vocurr;   	
Eigen::Vector3d q_world_vocurr_vec;
Eigen::Vector3d q_wodom_vocurr_vec;
Eigen::Vector3d t_wodom_locurr;
Eigen::Quaterniond q_wodom_locurr;
Eigen::Quaterniond q_world_locurr;   	   	
Eigen::Vector3d q_wodom_locurr_vec;
Eigen::Vector3d q_world_locurr_vec; 
Eigen::Quaterniond q_wodom_curr;

Eigen::Matrix4d VELO_T_GPSx;
Eigen::Matrix4d VELO_T_GPSy;
Eigen::Matrix4d VELO_T_GPSz;
Eigen::Matrix4d VELO_T_GPS;
Eigen::Vector3d temp;
Eigen::Vector3d GT;

//double angle=-44;		           //Handheld LVI
//double angle=-15;		           //Handheld
//double angle=-123;		           //Jackal
double angle=202;		           //Handheld
//double angle=-90;		           //Jackal
//double angle=-23;		           //Handheld - no depth
//double angle=-100;		           //Payload-down-lighthouse

#define PI 3.14159265

nav_msgs::Path GPSpath;
ros::Publisher pubGPSpath;

nav_msgs::Path TFpath;
ros::Publisher pub_Tpath;

Eigen::Quaterniond q_lo_curr;

void GPSHandler(const sensor_msgs::NavSatFixConstPtr& msg)
{
  
  if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) 
  {
    	ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
    	return;
  }

  latitude=msg->latitude;
  longitude=msg->longitude;
  altitude=msg->altitude;
  
  if (!initGPS) 
  {
	g_geodetic_converter.initialiseReference(latitude, longitude, altitude);
	initGPS=true;
        ROS_INFO("\n ------------------------------------GPS------------------------------------------------"); 
	return;
  }

  else 
  {
  	g_geodetic_converter.geodetic2Enu(latitude, longitude, altitude, &x, &y, &z);


	temp<<x,y,z;
	VELO_T_GPSz<< cos(angle*PI/180), -sin(angle*PI/180), 0, 0,
		     sin(angle*PI/180), cos(angle*PI/180), 0, 0,
	             0, 0, 1, 0,
		     0, 0, 0, 1;
	VELO_T_GPSy<< cos(180*PI/180), 0, sin(180*PI/180), 0,
		     0, 1, 0, 0,	
		     -sin(180*PI/180), 0, cos(180*PI/180), 0,
		     0, 0, 0, 1;

	VELO_T_GPSx<< 1,0,0,0,
		      0, cos(180*PI/180), -sin(180*PI/180), 0,
		      0, sin(180*PI/180), cos(180*PI/180), 0,
		      0, 0, 0, 1;
	
	//LVI-SAM
	VELO_T_GPS=VELO_T_GPSz*VELO_T_GPSy;
	
	//Payload
	//VELO_T_GPS=VELO_T_GPSz;

   	//GT=VELO_T_GPS.block<3, 3>(0, 0)*temp;
	GT=temp;

	//Transform to Lidar frame 
	tempx=GT[0];			//ToDo: Find the correct calibration 
	tempy=GT[1];
	tempz=GT[2];
  }

        // publish Ground truth path
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.header.frame_id = "/odom";
	pose_stamped.pose.position.x=tempx;
	pose_stamped.pose.position.y=tempy;  
	pose_stamped.pose.position.z=tempz; 

        GPSpath.header = msg->header;
        GPSpath.header.frame_id = "/odom";
        GPSpath.poses.push_back(pose_stamped);
        pubGPSpath.publish(GPSpath);

    	Eigen::Matrix3d tmp_R = q_lo_curr.toRotationMatrix();   //Lidar quat to rotation matrix conversion
	
	//Write results to a file 
	std::ofstream foutC("/home/didula/GVLOAM/src/Results/GT.txt", std::ios::app);             //Delete the previous file -ToDO --- check how to replace
    	foutC.setf(std::ios::fixed, std::ios::floatfield);
    	//foutC.precision(0);
    	//foutC << header.stamp.toSec() * 1e9 << ",";
    	foutC.precision(6);
    	foutC << tmp_R(0,0) << " "
            << tmp_R(0,1) << " "
            << tmp_R(0,2) << " "
            << tempx << " "
            << tmp_R(1,0) << " "
            << tmp_R(1,1) << " "
            << tmp_R(1,2) << " "
            << tempy << " "
            << tmp_R(2,0) << " "
            << tmp_R(2,1) << " "
            << tmp_R(2,2) << " "
            << tempz<< std::endl;
    	/*foutC << tempx << " "
              << tempy << " "
              << tempz<< std::endl;*/
	foutC.close();
}


void VinsOdometryHandler(const nav_msgs::Odometry::ConstPtr &pose_msg)  //D
{
	
        Eigen::Vector3d odometry_pose;
	Eigen::Vector3d TFodometry_pose;
	Eigen::Quaterniond odometry_quat; 

	odometry_pose.x()= pose_msg->pose.pose.position.x;
	odometry_pose.y()= pose_msg->pose.pose.position.y;
	odometry_pose.z()= pose_msg->pose.pose.position.z;
	odometry_quat.x()= pose_msg->pose.pose.orientation.x;
	odometry_quat.y()= pose_msg->pose.pose.orientation.y;
	odometry_quat.z()= pose_msg->pose.pose.orientation.z;
	odometry_quat.w()= pose_msg->pose.pose.orientation.w;
	
	Eigen::Matrix4d VELO_T_CAM;
	Eigen::Matrix4d CAM_T_VELO;
	Eigen::Matrix4d CAM_T_IMU;
	Eigen::Matrix4d IMU_T_CAM;
	Eigen::Matrix4d VELO_T_IMU;


//Payload - transform to lidar frame  
	IMU_T_CAM<< -0.0009792, 0.00685726, 0.99997601, 0.18648448,
		     0.99931723, -0.03692628, 0.00123178, -0.04199414,
	             0.03693384, 0.99929446, -0.00681642, -0.03693199,
		     0, 0, 0, 1;

        CAM_T_IMU = IMU_T_CAM.inverse();      

   	VELO_T_CAM<< -0.0062, -0.0009, 1.000, 0.0923,
          	      -0.9992, 0.0391, -0.0061, 0.0388,
          	     -0.0391,  -0.9992, -0.0012, -0.0740,
         	      0, 0, 0, 1;

	CAM_T_VELO=VELO_T_CAM.inverse();


 	VELO_T_IMU=VELO_T_CAM*CAM_T_IMU;

//KITTI 
        /*CAM_T_VELO <<  7.027555e-03, -9.999753e-01, 2.599616e-05, -7.137748e-03,                    //ToDo: Read from config file
          		-2.254837e-03, -4.184312e-05, -9.999975e-01, -7.482656e-02,
           		9.999728e-01, 7.027479e-03, -2.255075e-03, -3.336324e-01,
                        0,             0,             0,             1; 

        VELO_T_IMU <<  9.999976e-01, 7.553071e-04, -2.035826e-03, -8.086759e-01,                    //ToDo: Read from config file
          		-7.854027e-04, 9.998898e-01, -1.482298e-02, 3.195559e-01,
           		2.024406e-03, 1.482454e-02, 9.998881e-01, -7.997231e-01,
                        0,             0,             0,             1; 

	VELO_T_IMU = CAM_T_VELO.inverse();*/


	t_world_vocurr=vins_w_q_init.inverse()*(odometry_pose-vins_w_t_init);                              //world - vo world frame and wodom - lidar world frame
	q_world_vocurr=vins_w_q_init.inverse()*odometry_quat;
	q_world_vocurr_vec<<q_world_vocurr.x(),q_world_vocurr.y(), q_world_vocurr.z();


	TFodometry_pose=VELO_T_IMU.block<3, 3>(0, 0)*t_world_vocurr+ VELO_T_IMU.block<3, 1>(0, 3);
	q_wodom_vocurr_vec=VELO_T_IMU.block<3, 3>(0, 0)*q_world_vocurr_vec; 

    	q_wodom_curr.x()=q_wodom_vocurr_vec[0];
	q_wodom_curr.y()=q_wodom_vocurr_vec[1];
	q_wodom_curr.z()=q_wodom_vocurr_vec[2];
    	q_wodom_curr.w()=q_world_vocurr.w();

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = pose_msg->header;
        pose_stamped.header.frame_id = "/camera_init";
	pose_stamped.pose.position.x=TFodometry_pose.x();
	pose_stamped.pose.position.y=TFodometry_pose.y();  
	pose_stamped.pose.position.z=TFodometry_pose.z(); 
        TFpath.header = pose_msg->header;
        TFpath.header.frame_id = "/camera_init";
        TFpath.poses.push_back(pose_stamped);
        pub_Tpath.publish(TFpath);

	Eigen::Matrix3d R=q_wodom_curr.toRotationMatrix();

        /*Eigen::Matrix3d temp_R = odometry_quat.toRotationMatrix();
	Eigen::Matrix3d R;
	R=VELO_T_IMU.block<3, 3>(0, 0)*temp_R;
        Eigen::Quaterniond Q_curr;
        Q_curr = Eigen::Quaterniond(R);*/


	/*std::cout <<"-------VO T_curr.x() "<<TFodometry_pose.x()<< std::endl; 
	std::cout <<"-------VO T_curr.y() "<<TFodometry_pose.y()<< std::endl;					
	std::cout <<"-------VO T_curr.z() "<<TFodometry_pose.z()<< std::endl;

	std::cout <<"-------VO Q_w_curr.x() "<<Q_curr.x()<< std::endl;        
	std::cout <<"-------VO Q_w_curr.y() "<<Q_curr.y()<< std::endl; 
	std::cout <<"-------VO Q_w_curr.z() "<<Q_curr.z()<< std::endl;					
	std::cout <<"-------VO Q_w_curr.w() "<<Q_curr.w()<< std::endl;*/



        if (GTframeCount % 2 == 0)      //Save at 10hz 
	{
  	GTframeCount=0;
	std::ofstream foutC("/home/didula/GVLOAM/src/Results/VO.txt", std::ios::app);             //Delete the previous file -ToDO --- check how to replace
    	foutC.setf(std::ios::fixed, std::ios::floatfield);
    	//foutC.precision(0);
    	//foutC << header.stamp.toSec() * 1e9 << ",";
    	foutC.precision(6);
    	foutC << R(0,0) << " "
            << R(0,1) << " "
            << R(0,2) << " "
            << TFodometry_pose.x() << " "
            << R(1,0) << " "
            << R(1,1) << " "
            << R(1,2) << " "
            << TFodometry_pose.y() << " "
            << R(2,0) << " "
            << R(2,1) << " "
            << R(2,2) << " "
            << TFodometry_pose.z()<< std::endl;
    	foutC.close();
	}
	GTframeCount++;

}


void LidarOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)  //D
{
	q_lo_curr.x() = laserOdometry->pose.pose.orientation.x;
	q_lo_curr.y() = laserOdometry->pose.pose.orientation.y;
	q_lo_curr.z() = laserOdometry->pose.pose.orientation.z;
	q_lo_curr.w() = laserOdometry->pose.pose.orientation.w;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPStoPosenode");
    ros::NodeHandle nh;


    //ros::Subscriber sub_GPS = nh.subscribe("/fix", 100, GPSHandler);
    
    ros::Subscriber sub_GPS = nh.subscribe("/gps/fix", 100, GPSHandler);

    ros::Subscriber subVinsOdometry = nh.subscribe<nav_msgs::Odometry>("vins_estimator/odometry", 100, VinsOdometryHandler);
    
    ros::Subscriber subLidarOdometry = nh.subscribe<nav_msgs::Odometry>("aft_mapped_to_init", 100, LidarOdometryHandler);            

    //ros::Publisher pubGroundTruth = nh.advertise<nav_msgs::Odometry>("/GPS_GroundTruth", 100);

    pubGPSpath = nh.advertise<nav_msgs::Path>("/GPS_GroundTruth_path", 100);
    
    pub_Tpath = nh.advertise<nav_msgs::Path>("TFpath", 1000);		      

    ros::spin();
}
