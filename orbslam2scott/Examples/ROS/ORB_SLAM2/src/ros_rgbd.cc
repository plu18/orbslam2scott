/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <mutex>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/common/impl/angles.hpp>


#include "std_msgs/String.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include "../../../include/System.h"
#include "../../../include/pointcloudmapping.h"
//#include "../lib/SAC_MethodXYZ.h"

using namespace std;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
	
};

int rgb1 = 255<< 16 | 0 << 8 | 0;
int rgb2 = 255<< 16 | 255 << 8 | 255;
cv::Mat myPose;
ros::Publisher _pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr PC_filtered (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgbtmp (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgbout (new pcl::PointCloud<pcl::PointXYZRGB>);

//SAC_MethodXYZ sac_method;

void generateSparsePC(const pcl::PointCloud<pcl::PointXYZ>::Ptr in, const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
	cloud_in->is_dense = false;
	//pcl::PCDWriter writer;
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	//sensor_msgs::PointCloud2 cloud_filtered;
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(in);
	sor.setLeafSize (0.05f, 0.05f, 0.05f);
	sor.filter (*out);
	
}


void pcCallback(const sensor_msgs::PointCloud2ConstPtr& points)
{
	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromROSMsg (*points, *cloud_in);
	generateSparsePC(cloud_in, PC_filtered);
	

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ>);
	//remove NAN points from the cloud
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*PC_filtered,*PC_filtered, indices);


	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());	

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	//seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.05);
	//ground plane
	//seg.setAxis(Eigen::Vector3f(0.0,1.0,0.0));
	//front plane???
	//seg.setAxis(Eigen::Vector3f(1.0,0.0,0.0));
	//side plane
	seg.setAxis(Eigen::Vector3f(0.0,0.0,1.0));
	seg.setEpsAngle (pcl::deg2rad(90.0f));


	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

  	int i = 0, nr_points = (int) PC_filtered->points.size ();
	while (PC_filtered->points.size () > 0.3 * nr_points && i < 1)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (PC_filtered);
		seg.segment (*inliers, *coefficients);
		

		if (inliers->indices.size () == 0)
		{
		  	std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		  	break;
		}

		// Extract the inliers
		extract.setInputCloud (PC_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_p);
		*cloud_tmp += *cloud_p;
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		//std::stringstream ss;
		//ss << "new_pcl" << i << ".pcd";
		//writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_p, false);

		// Create the filtering object
		extract.setNegative (true);
		extract.filter (*cloud_f);
		PC_filtered.swap (cloud_f);

		i++;
	}

	*cloud_out = *PC_filtered;
	*cloud_out += *cloud_tmp;
	pcl::copyPointCloud(*cloud_out,*cloud_rgbtmp);

	for (size_t i = 0; i < cloud_out->points.size(); i++)
	{
		if (i < PC_filtered->points.size())
		{
			cloud_rgbtmp->points[i].rgb = rgb2;
		}
		else
		{
			cloud_rgbtmp->points[i].rgb = rgb1;
		}
	}


	//std::cout << "Saved " << cloud_tmp->points.size () << " data points to input" << std::endl;

	//icp
	//cloud_rgbout.swap(cloud_rgbtmp);
	//pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	//icp.setInputSource(cloud_rgbtmp);
	//icp.setInputTarget(cloud_rgbout);
	//pcl::PointCloud<pcl::PointXYZRGB> Final;

	//icp.align(Final);
	//std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	//std::cout << icp.getFinalTransformation() << std::endl;

	// Convert the pcl::PointCloud to sensor_msgs/PointCloud2
	sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
	
	pcl::toROSMsg( *cloud_rgbtmp, *output );

	// Publish the map

	//_pub.publish(output);
	//std::cout << "output " << Final.size () << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    std::mutex mMutexCamera;
	ros::NodeHandle nh;
	//MapDrawer mapDrawer;
	
	//ORB_SLAM2::MapDrawer* mapDrawer;

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);


    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/sw_registered/image_rect_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
	
	//subscribe pointcloud from camera
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, pcCallback);
	
	//publish tf

	tf::TransformBroadcaster br;
	tf::Transform transform;

	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
	PointCloud::Ptr mPointMapMsg (new PointCloud);
	
	//publish point cloud
	_pub = nh.advertise<PointCloud> ("myPointCloud233",100);
	
	
	ros::Rate loop_rate(30); 

	while (nh.ok()){
		SLAM.mpPointCloudMapping->GetPointCloud(mPointMapMsg);
		mPointMapMsg->header.frame_id = "map";
		mPointMapMsg->header.stamp = ros::Time::now().toNSec()/1e3;
		_pub.publish(mPointMapMsg);
		//cout<<"generate point cloud for globalmap "<<mPointMapMsg->points.size()<<endl;

		tf::Vector3 origin;
		tf::Matrix3x3 tf3d;
		tf::Quaternion q;
		if (!SLAM.mpMap->GetAllKeyFrames().empty()){
			//publish tf
			SLAM.mpMapDrawer->GetCameraPose(myPose);
			unique_lock<mutex> lock(mMutexCamera);
			cv::Mat mRwc = myPose.rowRange(0,3).colRange(0,3).t();
			cv::Mat mTwc = -mRwc*myPose.rowRange(0,3).col(3);

			origin.setValue(mTwc.at<float>(2),0.0,-mTwc.at<float>(1));

			tf3d.setValue(mRwc.at<float>(0,0),mRwc.at<float>(0,2),0.0,
						mRwc.at<float>(2,0),mRwc.at<float>(2,2),0.0,
						0.0,0.0,1.0);
			
			//tf3d.setValue(mRwc.at<float>(0,0),mRwc.at<float>(0,1),mRwc.at<float>(0,2),
			//			mRwc.at<float>(1,0),mRwc.at<float>(1,1),mRwc.at<float>(1,2),
			//			mRwc.at<float>(2,0),mRwc.at<float>(2,1),mRwc.at<float>(2,2));


			transform.setOrigin(origin);
			tf3d.getRotation(q);
			
		}else{
			transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
			q.setRPY(0.0f, 0.0f, 0.0f);
		}

		transform.setRotation(q);
		
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "camera_link"));


		ros::spinOnce();
		loop_rate.sleep();

	}


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}



void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
	
	//cout << mpSLAM.size();
	//cout << "\n";

}


