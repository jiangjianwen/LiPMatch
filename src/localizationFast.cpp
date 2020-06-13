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

#include <LiPMatch.h>
#include <math.h>
#include <vector>
#include <aloam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>  // tf::Matrix3x3, tf::createQuaternionMsgFromRollPitchYaw, tf::Quaternion
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "lidarFactor.hpp"
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "RangenetAPI.hpp"

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace LiPMatch_ns;

int frameCount = 0;
double timeLaserOdometry = 0;

double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);

Eigen::Vector3d t_w_prev(0.0,0.0,0.0);

pcl::VoxelGrid<PointType> downSizeFilterKF;

std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;

ros::Publisher pubLaserCloudFullRes, pubOdomAftMapped, pubLaserAfterMappedPath;

ros::Publisher pubLaserCloudFullResbef;

ros::Publisher pubMatchedPoints1;

ros::Publisher pubMatchedPoints2;

ros::Publisher pubEachFrameLaserCloud;

ros::Publisher pubTargetMapLaserCloud;

ros::Publisher marker_keyframe_pub;

ros::Publisher m_pub_laser_aft_loopclosure_path;

nav_msgs::Path laserAfterMappedPath;

visualization_msgs::Marker points;

visualization_msgs::Marker line_list;

visualization_msgs::Marker line_list2;

LiPMatch lipmatch;

std::list<m_keyframe> m_keyframe_of_updating_list;
std::list<m_keyframe> m_keyframe_need_precession_list;

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}



void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}

void process()
{

//    sensor_msgs::PointCloud2 TargetPointCloudOutMsg;
//    pcl::toROSMsg(lipmatch.mapToShow, TargetPointCloudOutMsg);
//    TargetPointCloudOutMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//    TargetPointCloudOutMsg.header.frame_id = "/camera_init";
//    pubTargetMapLaserCloud.publish(TargetPointCloudOutMsg);

//    std::ifstream timestamp_file("/home/jjwen/data/KITTI/odometry/00/pose.txt", std::ifstream::in);

    std::ifstream timestamp_file("/media/jjwen/SBPD1_ddy/bk ubuntu18 6月7号/data/KITTI/odometry/05.txt", std::ifstream::in);

    std::string line;
    std::size_t line_num = 0;

    ros::Rate r(10.0);

    while (std::getline(timestamp_file, line) && ros::ok())
    {
//        std::stringstream pose_stream(line);
//        std::string s;
//        for (std::size_t i = 0; i < 7; ++i)
//        {
//            std::getline(pose_stream, s, ' ');
//            parameters[i] = stof(s);
//        }

        std::vector<double> vdata;
        std::stringstream pose_stream(line);
        std::string s;
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                vdata.push_back(stof(s));
            }
        }

        Eigen::Matrix3d rotation_matrix;
        rotation_matrix<<vdata[0],vdata[1],vdata[2],vdata[4],vdata[5],vdata[6],vdata[8],vdata[9],vdata[10];
        Eigen::Quaterniond quat(rotation_matrix);

        parameters[0] = quat.z();
        parameters[1] = -quat.x();
        parameters[2] = -quat.y();
        parameters[3] = quat.w();
        parameters[4] = vdata[11];
        parameters[5] = -vdata[3];
        parameters[6] = -vdata[7];


        pcl::PointCloud<pcl::PointXYZI> laserCloudIn;


        string binpath = "/media/jjwen/SBPD1_ddy/bk ubuntu18 6月7号/data/KITTI/odometry/05/semantic/"+to_string(line_num)+".bin";

        line_num++;


        // read lidar point cloud
        std::vector<float> lidar_data = read_lidar_data(binpath);
//        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laserCloudIn.push_back(point);
        }

        uint32_t num_points = laserCloudIn.points.size();

        pcl::PointCloud<pcl::PointXYZI>::Ptr structure_points(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr vehicle_points(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr nature_points(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());


        for (uint32_t i = 0; i < num_points; ++i) {

            int label = int(laserCloudIn.points[i].intensity);

            if (label == 40 || label == 44 || label == 48 || label == 49)
            {
                ground_points->points.push_back(laserCloudIn.points[i]);
            }
            //structure
            else if (label == 50 || label == 51 || label == 52 || label == 60) {
                structure_points->points.push_back(laserCloudIn.points[i]);
            }
                //vehicle
            else if (label == 10 || label == 13 || label == 18 || label == 16) {
                vehicle_points->points.push_back(laserCloudIn.points[i]);
            }
                //cylinder
            else if (label == 71 || label == 80) {
                nature_points->points.push_back(laserCloudIn.points[i]);
            }
        }

        downSizeFilterKF.setInputCloud(structure_points);
        downSizeFilterKF.filter(*structure_points);

        downSizeFilterKF.setInputCloud(vehicle_points);
        downSizeFilterKF.filter(*vehicle_points);

        downSizeFilterKF.setInputCloud(nature_points);
        downSizeFilterKF.filter(*nature_points);

        downSizeFilterKF.setInputCloud(ground_points);
        downSizeFilterKF.filter(*ground_points);


        for (int i = 0; i < structure_points->points.size(); i++)
        {
            pointAssociateToMap(&structure_points->points[i], &structure_points->points[i]);
        }

        for (int i = 0; i < vehicle_points->points.size(); i++)
        {
            pointAssociateToMap(&vehicle_points->points[i], &vehicle_points->points[i]);
        }

        for (int i = 0; i < nature_points->points.size(); i++)
        {
            pointAssociateToMap(&nature_points->points[i], &nature_points->points[i]);
        }

        for (int i = 0; i < ground_points->points.size(); i++)
        {
            pointAssociateToMap(&ground_points->points[i], &ground_points->points[i]);
        }

        for(auto it = m_keyframe_of_updating_list.begin(); it != m_keyframe_of_updating_list.end(); it++ )
        {
            *(it->structurelaserCloud) += *structure_points;
            *(it->vehiclelaserCloud) += *vehicle_points;
            *(it->naturelaserCloud) += *nature_points;
            *(it->g_laserCloud) += *ground_points;
            it->framecount++;
            it->travel_length += sqrt((t_w_curr[0]-t_w_prev[0])*(t_w_curr[0]-t_w_prev[0])+
                                      (t_w_curr[1]-t_w_prev[1])*(t_w_curr[1]-t_w_prev[1])+(t_w_curr[2]-t_w_prev[2])*(t_w_curr[2]-t_w_prev[2]));
        }

        t_w_prev = t_w_curr;

        //初始化
        points.header.frame_id = "/camera_init";
        points.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        points.ns = "points_and_lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        //分配3个id
        points.id = 0;
        //初始化形状
        points.type = visualization_msgs::Marker::POINTS;
        //初始化大小
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 2.75;
        points.scale.y = 2.75;
        //初始化颜色
        // Points are green
        points.color.r = 1.0f;
        points.color.a = 1.0;

        //30 32
        if ( m_keyframe_of_updating_list.front().travel_length >= 60.0 )
        {
            m_keyframe_of_updating_list.front().m_ending_frame_idx = frameCount;
            m_keyframe_of_updating_list.front().m_pose_q = q_w_curr;
            m_keyframe_of_updating_list.front().m_pose_t =  t_w_curr;
            m_keyframe_need_precession_list.push_back( m_keyframe_of_updating_list.front() );
            m_keyframe_of_updating_list.pop_front();

            lipmatch.frameQueue.push_back(m_keyframe_need_precession_list.back());

            geometry_msgs::Point p;
            p.x = t_w_curr.x();
            p.y = t_w_curr.y();
            p.z = t_w_curr.z();
            points.points.push_back(p);

            marker_keyframe_pub.publish(points);
        }

        //25 26
        if ( m_keyframe_of_updating_list.back().travel_length >= 55.0 )
        {
            m_keyframe tk1;
            m_keyframe_of_updating_list.push_back(tk1);
        }

        sensor_msgs::PointCloud2 matchedCloudOutMsg1;
        pcl::toROSMsg(lipmatch.laserCloudOri_m1, matchedCloudOutMsg1);
        matchedCloudOutMsg1.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        matchedCloudOutMsg1.header.frame_id = "/camera_init";
        pubMatchedPoints1.publish(matchedCloudOutMsg1);

//        sensor_msgs::PointCloud2 matchedCloudOutMsg2;
//        pcl::toROSMsg(lipmatch.laserCloudOri_m2_1, matchedCloudOutMsg2);
//        matchedCloudOutMsg2.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//        matchedCloudOutMsg2.header.frame_id = "/camera_init";
//        pubMatchedPoints2.publish(matchedCloudOutMsg2);

        sensor_msgs::PointCloud2 matchedCloudOutMsg2;
        pcl::toROSMsg(lipmatch.laserCloudOri_m2, matchedCloudOutMsg2);
        matchedCloudOutMsg2.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        matchedCloudOutMsg2.header.frame_id = "/camera_init";
        pubMatchedPoints2.publish(matchedCloudOutMsg2);

        sensor_msgs::PointCloud2 eachPointCloudOutMsg;
        pcl::toROSMsg(*vehicle_points, eachPointCloudOutMsg);
        eachPointCloudOutMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        eachPointCloudOutMsg.header.frame_id = "/camera_init";
        pubEachFrameLaserCloud.publish(eachPointCloudOutMsg);

        //初始化
        line_list.header.frame_id = "/camera_init";
        line_list.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        line_list.ns = "points_and_lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        //分配3个id
        line_list.id = 1;
        //初始化形状
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        //初始化大小
        // POINTS markers use x and y scale for width/height respectively
        line_list.scale.x = 2.75;
        line_list.scale.y = 2.75;
        //初始化颜色
        // Points are green
        line_list.color.r = 1.0f;
        line_list.color.g = 1.0f;
        line_list.color.a = 1.0;

        for (map<size_t, size_t>::iterator it = lipmatch.loop_closure_matchedid.begin(); it != lipmatch.loop_closure_matchedid.end(); it++) {
            geometry_msgs::Point p1 = points.points[it->first];
            geometry_msgs::Point p2 = points.points[it->second];
            line_list.points.push_back(p1);
            line_list.points.push_back(p2);
        }

        marker_keyframe_pub.publish(line_list);

//        sensor_msgs::PointCloud2 laserCloudFullRes3;
//        pcl::toROSMsg(lipmatch.refined_pt, laserCloudFullRes3);
//        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//        laserCloudFullRes3.header.frame_id = "/camera_init";
//        pubLaserCloudFullRes.publish(laserCloudFullRes3);

        sensor_msgs::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(lipmatch.same_laserCloud, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        laserCloudFullRes3.header.frame_id = "/camera_init";
        pubLaserCloudFullRes.publish(laserCloudFullRes3);

        sensor_msgs::PointCloud2 laserCloudFullRes4;
        pcl::toROSMsg(lipmatch.same_laserCloud2, laserCloudFullRes4);
        laserCloudFullRes4.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        laserCloudFullRes4.header.frame_id = "/camera_init";
        pubLaserCloudFullResbef.publish(laserCloudFullRes4);

        nav_msgs::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = "/camera_init";
        odomAftMapped.child_frame_id = "/aft_mapped";
        odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
        odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
        odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
        odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
        odomAftMapped.pose.pose.position.x = t_w_curr.x();
        odomAftMapped.pose.pose.position.y = t_w_curr.y();
        odomAftMapped.pose.pose.position.z = t_w_curr.z();
        pubOdomAftMapped.publish(odomAftMapped);

        geometry_msgs::PoseStamped laserAfterMappedPose;
        laserAfterMappedPose.header = odomAftMapped.header;
        laserAfterMappedPose.pose = odomAftMapped.pose.pose;
        laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
        laserAfterMappedPath.header.frame_id = "/camera_init";
        laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
        pubLaserAfterMappedPath.publish(laserAfterMappedPath);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(t_w_curr(0), t_w_curr(1), t_w_curr(2)));
        q.setW(q_w_curr.w());
        q.setX(q_w_curr.x());
        q.setY(q_w_curr.y());
        q.setZ(q_w_curr.z());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));

        r.sleep();
    }

    //初始化
    line_list2.header.frame_id = "/camera_init";
    line_list2.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    line_list2.ns = "points_and_lines";
    line_list2.action = visualization_msgs::Marker::ADD;
    line_list2.pose.orientation.w = 1.0;
    //分配3个id
    line_list2.id = 1;
    //初始化形状
    line_list2.type = visualization_msgs::Marker::LINE_LIST;
    //初始化大小
    // POINTS markers use x and y scale for width/height respectively
    line_list2.scale.x = 2.75;
    line_list2.scale.y = 2.75;
    //初始化颜色
    // Points are green
    line_list2.color.r = 1.0f;
    line_list2.color.g = 1.0f;
    line_list2.color.a = 1.0;

    for (map<size_t, size_t>::iterator it = lipmatch.loop_closure_matchedid.begin(); it != lipmatch.loop_closure_matchedid.end(); it++) {
        geometry_msgs::Point p1 = points.points[it->first];
        geometry_msgs::Point p2 = points.points[it->second];
        line_list2.points.push_back(p1);
        line_list2.points.push_back(p2);
    }

    marker_keyframe_pub.publish(line_list2);

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

//	float planeRes = 0.8;
    downSizeFilterKF.setLeafSize(0.4,0.4,0.4);

    pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 300);

    pubLaserCloudFullResbef = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered_bef", 300);

    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 300);

    pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 300);

    pubMatchedPoints1 = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_matchedpoints1", 300);

    pubMatchedPoints2 = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_matchedpoints2", 300);

    marker_keyframe_pub = nh.advertise<visualization_msgs::Marker>("/visualization_keyframe_marker", 300);

    pubTargetMapLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_target_map", 300);

    pubEachFrameLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_each_frame", 300);

    m_pub_laser_aft_loopclosure_path = nh.advertise<nav_msgs::Path>( "/aft_loopclosure_path", 30000 );


    m_keyframe tk;

    m_keyframe_of_updating_list.push_back(tk);

    process();

    ros::spin();

    return 0;
}