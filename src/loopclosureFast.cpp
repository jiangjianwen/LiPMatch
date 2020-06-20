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
#include <tf/tf.h>  
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

const double scanPeriod = 0.1;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

pcl::PointCloud<PointType> cornerPointsSharp;
pcl::PointCloud<PointType> cornerPointsLessSharp;
pcl::PointCloud<PointType> surfPointsFlat;
pcl::PointCloud<PointType> surfPointsLessFlat;

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

ros::Publisher marker_keyframe_pub;

ros::Publisher m_pub_laser_aft_loopclosure_path;

nav_msgs::Path laserAfterMappedPath;

visualization_msgs::Marker points;

visualization_msgs::Marker line_list;

visualization_msgs::Marker line_list2;

// LiPMatch lipmatch;

std::list<tools::m_keyframe> m_keyframe_of_updating_list;
std::list<tools::m_keyframe> m_keyframe_need_precession_list;

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

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void laserCloudHandler(pcl::PointCloud<pcl::PointXYZI>& laserCloudIn)
{
    cornerPointsSharp.clear();
    cornerPointsLessSharp.clear();
    surfPointsFlat.clear();
    surfPointsLessFlat.clear();

    int N_SCANS = 64;
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, 5.0);

    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (angle >= -8.83)
            scanID = int((2 - angle) * 3.0 + 0.5);
        else
            scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);
        // use [0 50]  > 50 remove outlies
        if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
        {
            count--;
            continue;
        }

        //printf("angle %f scanID %d \n", angle, scanID);

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        {
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + scanPeriod * relTime;
        laserCloudScans[scanID].push_back(point);
    }

    cloudSize = count;
//    printf("points size %d \n", cloudSize);

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS; i++)
    {
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }

//    printf("prepare time %f \n", t_prepare.toc());

    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }


    TicToc t_pts;

    float t_q_sort = 0;
    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        for (int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)
                    {
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1;
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }

}


void loop_closure_pub_optimzed_path( const LiPMatch::MapOfPoses &pose3d_aft_loopclosure )
{
    nav_msgs::Path m_laser_after_loopclosure_path;
    m_laser_after_loopclosure_path.header.stamp = ros::Time::now();
    m_laser_after_loopclosure_path.header.frame_id = "/camera_init";
    for ( auto it = pose3d_aft_loopclosure.begin(); it != pose3d_aft_loopclosure.end(); it++ )
    {
        geometry_msgs::PoseStamped  pose_stamp;
        LiPMatch::Pose3d pose_3d = it->second;
        pose_stamp.pose.orientation.x = pose_3d.q.x();
        pose_stamp.pose.orientation.y = pose_3d.q.y();
        pose_stamp.pose.orientation.z = pose_3d.q.z();
        pose_stamp.pose.orientation.w = pose_3d.q.w();

        pose_stamp.pose.position.x = pose_3d.p( 0 );
        pose_stamp.pose.position.y = pose_3d.p( 1 );
        pose_stamp.pose.position.z = pose_3d.p( 2 );

        pose_stamp.header.frame_id = "/camera_init";

        m_laser_after_loopclosure_path.poses.push_back( pose_stamp );
    }

    m_pub_laser_aft_loopclosure_path.publish( m_laser_after_loopclosure_path );

}

//localize

//void process()
//{
//
//    std::ifstream timestamp_file("/home/jjwen/data/KITTI/odometry/05.txt", std::ifstream::in);
//
//    Eigen::Matrix3d R_transform;
//    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
////    R_transform << 0, 0, -1, 1, 0, 0, 0, -1, 0;
//    Eigen::Quaterniond q_transform(R_transform);
//
//    std::string line;
//    std::size_t line_num = 0;
//
//    ros::Rate r(15.0);
//
//    while (std::getline(timestamp_file, line) && ros::ok())
//    {
////        std::stringstream pose_st    std::cout<<"pointSearchInd: "<<pointSearchInd[0]<<std::endl;
//
//
//
////    if (pointSearchSqDis[0] > 10.0)
////    {
////        return;
////    }ream(line);
////        std::string s;
////        for (std::size_t i = 0; i < 7; ++i)
////        {
////            std::getline(pose_stream, s, ' ');
////            parameters[i] = stof(s);
////        }
//
////        std::stringstream pose_stream(line);
////        std::string s;
////        Eigen::Matrix<double, 3, 4> gt_pose;
////        for (std::size_t i = 0; i < 3; ++i)
////        {
////            for (std::size_t j = 0; j < 4; ++j)
////            {
////                std::getline(pose_stream, s, ' ');
////                gt_pose(i, j) = stof(s);
////            }
////        }
////        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
////        Eigen::Quaterniond q = q_transform * q_w_i;
//////        Eigen::Quaterniond q = q_w_i;
////        q.normalize();
////        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();
////
////        parameters[0] = q.z();
////        parameters[1] = q.y();
////        parameters[2] = q.x();
////        parameters[3] = q.w();
////        parameters[4] = t(0);
////        parameters[5] = t(1);
////        parameters[6] = t(2);
//
//        std::vector<double> vdata;
//        std::stringstream pose_stream(line);
//        std::string s;
//        for (std::size_t i = 0; i < 3; ++i)
//        {
//            for (std::size_t j = 0; j < 4; ++j)
//            {
//                std::getline(pose_stream, s, ' ');
//                vdata.push_back(stof(s));
//            }
//        }
//        double roll, pitch, yaw;
//        Eigen::Matrix4f tform;
//        tf::Matrix3x3 tf_mat;
//
//        Eigen::Matrix3d rotation_matrix;
//        rotation_matrix<<vdata[0],vdata[1],vdata[2],vdata[4],vdata[5],vdata[6],vdata[8],vdata[9],vdata[10];
//        Eigen::Quaterniond quat(rotation_matrix);
//
//
////        Eigen::Vector3d eulerAngle=rotation_matrix.eulerAngles(2,1,0);
////        tf_mat.getRPY(roll, pitch, yaw);
////        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
////        tf_mat.setRotation(tf::Quaternion(quat.z(), -quat.x(), -quat.y(), quat.w()));
////        tform(0, 0) = tf_mat[0][0]; tform(0, 1) = tf_mat[0][1]; tform(0, 2) = tf_mat[0][2]; tform(0, 3) = vdata[11];
////        tform(1, 0) = tf_mat[1][0]; tform(1, 1) = tf_mat[1][1]; tform(1, 2) = tf_mat[1][2]; tform(1, 3) = -vdata[3];
////        tform(2, 0) = tf_mat[2][0]; tform(2, 1) = tf_mat[2][1]; tform(2, 2) = tf_mat[2][2]; tform(2, 3) = -vdata[7];
////        tform(3, 0) = 0; tform(3, 1) = 0; tform(3, 2) = 0; tform(3, 3) = 1;
////        Eigen::Quaterniond q_w_i(tform.topLeftCorner<3, 3>());
////        Eigen::Quaterniond q = q_w_i;
//////        Eigen::Quaterniond q = q_w_i;
////        q.normalize();
////        Eigen::Vector3d t = tform.topRightCorner<3, 1>();
//
//
//        parameters[0] = quat.z();
//        parameters[1] = -quat.x();
//        parameters[2] = -quat.y();
//        parameters[3] = quat.w();
//        parameters[4] = vdata[11];
//        parameters[5] = -vdata[3];
//        parameters[6] = -vdata[7];
//
//
//
////        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
////        Eigen::Quaterniond q = q_transform * q_w_i;
//////        Eigen::Quaterniond q = q_w_i;
////        q.normalize();
////        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();
////
////        parameters[0] = q.z();
////        parameters[1] = q.y();
////        parameters[2] = q.x();
////        parameters[3] = q.w();
////        parameters[4] = t(0);
////        parameters[5] = t(1);
////        parameters[6] = t(2);
//
//
//
//
//
//
//
//
//
////        std::cout<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<" "<<t(0)<<" "<<t(1)<<" "<<t(2)<<std::endl;
//
//        pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
//
////        string pcdpath = "/home/jjwen/datasemantic/"+to_string(line_num)+".pcd";
////        pcl::io::loadPCDFile<pcl::PointXYZI>(pcdpath, laserCloudIn);
////        uint32_t num_points = laserCloudIn.points.size();
//
//
//
//        string binpath = "/home/jjwen/data/KITTI/odometry/05/semantic/"+to_string(line_num)+".bin";
//
//        line_num++;
//
//
////        std::fstream output(binpath.c_str(), std::ios::out | std::ios::binary);
////        output.seekg(0, std::ios::beg);
////        for (auto point : laserCloudIn.points) {
////                output.write((char*)&point.x, sizeof(float));
////                output.write((char*)&point.y, sizeof(float));
////                output.write((char*)&point.z, sizeof(float));
////                output.write((char*)&point.intensity, sizeof(float));
////        }
////        output.close();
//
//
//
//        // read lidar point cloud
//        std::vector<float> lidar_data = read_lidar_data(binpath);
////        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";
//        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
//        {
//            pcl::PointXYZI point;
//            point.x = lidar_data[i];
//            point.y = lidar_data[i + 1];
//            point.z = lidar_data[i + 2];
//            point.intensity = lidar_data[i + 3];
//            laserCloudIn.push_back(point);
//        }
//
//        uint32_t num_points = laserCloudIn.points.size();
//
//        pcl::PointCloud<pcl::PointXYZI>::Ptr structure_points(new pcl::PointCloud<pcl::PointXYZI>());
//        pcl::PointCloud<pcl::PointXYZI>::Ptr vehicle_points(new pcl::PointCloud<pcl::PointXYZI>());
//        pcl::PointCloud<pcl::PointXYZI>::Ptr nature_points(new pcl::PointCloud<pcl::PointXYZI>());
//        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());
//
//
//        for (uint32_t i = 0; i < num_points; ++i) {
//
//            int label = int(laserCloudIn.points[i].intensity);
//
//            if (label == 40 || label == 44 || label == 48 || label == 49)
//            {
//                ground_points->points.push_back(laserCloudIn.points[i]);
//            }
//                //structure
//            else if (label == 50 || label == 51 || label == 52 || label == 60) {
//                structure_points->points.push_back(laserCloudIn.points[i]);
//            }
//                //vehicle
//            else if (label == 10 || label == 13 || label == 18 || label == 16) {
//                vehicle_points->points.push_back(laserCloudIn.points[i]);
//            }
//                //cylinder
//            else if (label == 71 || label == 80) {
//                nature_points->points.push_back(laserCloudIn.points[i]);
//            }
//        }
//
//        downSizeFilterKF.setInputCloud(structure_points);
//        downSizeFilterKF.filter(*structure_points);
//
//        downSizeFilterKF.setInputCloud(vehicle_points);
//        downSizeFilterKF.filter(*vehicle_points);
//
//        downSizeFilterKF.setInputCloud(nature_points);
//        downSizeFilterKF.filter(*nature_points);
//
//        downSizeFilterKF.setInputCloud(ground_points);
//        downSizeFilterKF.filter(*ground_points);
//
//        for (int i = 0; i < structure_points->points.size(); i++)
//        {
//            pointAssociateToMap(&structure_points->points[i], &structure_points->points[i]);
//        }
//
//        for (int i = 0; i < vehicle_points->points.size(); i++)
//        {
//            pointAssociateToMap(&vehicle_points->points[i], &vehicle_points->points[i]);
//        }
//
//        for (int i = 0; i < nature_points->points.size(); i++)
//        {
//            pointAssociateToMap(&nature_points->points[i], &nature_points->points[i]);
//        }
//
//        for (int i = 0; i < ground_points->points.size(); i++)
//        {
//            pointAssociateToMap(&ground_points->points[i], &ground_points->points[i]);
//        }
//
//        laserCloudHandler(laserCloudIn);
//
//        downSizeFilterKF.setInputCloud(cornerPointsLessSharp.makeShared());
//        downSizeFilterKF.filter(cornerPointsLessSharp);
//
//        downSizeFilterKF.setInputCloud(surfPointsLessFlat.makeShared());
//        downSizeFilterKF.filter(surfPointsLessFlat);
//
//        for (int i = 0; i < cornerPointsLessSharp.points.size(); i++)
//        {
//            pointAssociateToMap(&cornerPointsLessSharp.points[i], &cornerPointsLessSharp.points[i]);
//        }
//
//        for (int i = 0; i < surfPointsLessFlat.points.size(); i++)
//        {
//            pointAssociateToMap(&surfPointsLessFlat.points[i], &surfPointsLessFlat.points[i]);
//        }
//
//        for(auto it = m_keyframe_of_updating_list.begin(); it != m_keyframe_of_updating_list.end(); it++ )
//        {
//            *(it->structurelaserCloud) += *structure_points;
//            *(it->vehiclelaserCloud) += *vehicle_points;
//            *(it->naturelaserCloud) += *nature_points;
//            *(it->g_laserCloud) += *ground_points;
//
//            *(it->linelaserCloud) += cornerPointsLessSharp;
//            *(it->surflaserCloud) += surfPointsLessFlat;
//
//            it->framecount++;
//            it->travel_length += sqrt((t_w_curr[0]-t_w_prev[0])*(t_w_curr[0]-t_w_prev[0])+
//                                      (t_w_curr[1]-t_w_prev[1])*(t_w_curr[1]-t_w_prev[1])+(t_w_curr[2]-t_w_prev[2])*(t_w_curr[2]-t_w_prev[2]));
//        }
//
//        t_w_prev = t_w_curr;
//
//        //初始化
//        points.header.frame_id = "/camera_init";
//        points.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//        points.ns = "points_and_lines";
//        points.action = visualization_msgs::Marker::ADD;
//        points.pose.orientation.w = 1.0;
//        //分配3个id
//        points.id = 0;
//        //初始化形状
//        points.type = visualization_msgs::Marker::POINTS;
//        //初始化大小
//        // POINTS markers use x and y scale for width/height respectively
//        points.scale.x = 2.75;
//        points.scale.y = 2.75;
//        //初始化颜色
//        // Points are green
//        points.color.r = 1.0f;
//        points.color.a = 1.0;
//
//        //30 32
//        if ( m_keyframe_of_updating_list.front().travel_length >= 40.0 )
//        {
//            m_keyframe_of_updating_list.front().m_ending_frame_idx = frameCount;
//            m_keyframe_of_updating_list.front().m_pose_q = q_w_curr;
//            m_keyframe_of_updating_list.front().m_pose_t =  t_w_curr;
//            m_keyframe_need_precession_list.push_back( m_keyframe_of_updating_list.front() );
//            m_keyframe_of_updating_list.pop_front();
//
//            lipmatch.frameQueue.push_back(m_keyframe_need_precession_list.back());
//
//            geometry_msgs::Point p;
//            p.x = t_w_curr.x();
//            p.y = t_w_curr.y();
//            p.z = t_w_curr.z();
//            points.points.push_back(p);
//
//            marker_keyframe_pub.publish(points);
//        }
//
//        //25 26
//        if ( m_keyframe_of_updating_list.back().travel_length >= 35.0 )
//        {
//            m_keyframe tk1;
//            m_keyframe_of_updating_list.push_back(tk1);
//        }
//
//        sensor_msgs::PointCloud2 matchedCloudOutMsg1;
//        pcl::toROSMsg(lipmatch.laserCloudOri_m1, matchedCloudOutMsg1);
//        matchedCloudOutMsg1.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//        matchedCloudOutMsg1.header.frame_id = "/camera_init";
//        pubMatchedPoints1.publish(matchedCloudOutMsg1);
//
////        sensor_msgs::PointCloud2 matchedCloudOutMsg2;
////        pcl::toROSMsg(lipmatch.laserCloudOri_m2_1, matchedCloudOutMsg2);
////        matchedCloudOutMsg2.header.stamp = ros::Time().fromSec(timeLaserOdometry);
////        matchedCloudOutMmarker_keyframe_pubsg2.header.frame_id = "/camera_init";
////        pubMatchedPoints2.publish(matchedCloudOutMsg2);
//
//        sensor_msgs::PointCloud2 matchedCloudOutMsg2;
//        pcl::toROSMsg(lipmatch.laserCloudOri_m2, matchedCloudOutMsg2);
//        matchedCloudOutMsg2.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//        matchedCloudOutMsg2.header.frame_id = "/camera_init";
//        pubMatchedPoints2.publish(matchedCloudOutMsg2);
//
//        sensor_msgs::PointCloud2 eachPointCloudOutMsg;
//        pcl::toROSMsg(*vehicle_points, eachPointCloudOutMsg);
//        eachPointCloudOutMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//        eachPointCloudOutMsg.header.frame_id = "/camera_init";
//        pubEachFrameLaserCloud.publish(eachPointCloudOutMsg);
//
//        //初始化
//        line_list.header.frame_id = "/camera_init";
//        line_list.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//        line_list.ns = "points_and_lines";
//        line_list.action = visualization_msgs::Marker::ADD;
//        line_list.pose.orientation.w = 1.0;
//        //分配3个id
//        line_list.id = 1;
//        //初始化形状
//        line_list.type = visualization_msgs::Marker::LINE_LIST;
//        //初始化大小
//        // POINTS markers use x and y scale for width/height respectively
//        line_list.scale.x = 2.75;
//        line_list.scale.y = 2.75;
//        //初始化颜色
//        // Points are green
//        line_list.color.r = 1.0f;
//        line_list.color.g = 1.0f;
//        line_list.color.a = 1.0;
//
//        for (map<size_t, size_t>::iterator it = lipmatch.loop_closure_matchedid.begin(); it != lipmatch.loop_closure_matchedid.end(); it++) {
//            geometry_msgs::Point p1 = points.points[it->first];
//            geometry_msgs::Point p2 = points.points[it->second];
//            line_list.points.push_back(p1);
//            line_list.points.push_back(p2);
//        }
//
//        marker_keyframe_pub.publish(line_list);
//
////        sensor_msgs::PointCloud2 laserCloudFullRes3;
////        pcl::toROSMsg(lipmatch.refined_pt, laserCloudFullRes3);
////        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
////        laserCloudFullRes3.header.frame_id = "/camera_init";
////        pubLaserCloudFullRes.publish(laserCloudFullRes3);
//
////        loop_closure_pub_optimzed_path(lipmatch.optimized_pose3d_map);
//
//
//        sensor_msgs::PointCloud2 laserCloudFullRes3;
//        pcl::toROSMsg(lipmatch.same_laserCloud, laserCloudFullRes3);
//        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//        laserCloudFullRes3.header.frame_id = "/camera_init";
//        pubLaserCloudFullRes.publish(laserCloudFullRes3);
//
//        sensor_msgs::PointCloud2 laserCloudFullRes4;
//        pcl::toROSMsg(lipmatch.same_laserCloud2, laserCloudFullRes4);
//        laserCloudFullRes4.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//        laserCloudFullRes4.header.frame_id = "/camera_init";
//        pubLaserCloudFullResbef.publish(laserCloudFullRes4);
//
//        nav_msgs::Odometry odomAftMapped;
//        odomAftMapped.header.frame_id = "/camera_init";
//        odomAftMapped.child_frame_id = "/aft_mapped";
//        odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//        odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
//        odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
//        odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
//        odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
//        odomAftMapped.pose.pose.position.x = t_w_curr.x();
//        odomAftMapped.pose.pose.position.y = t_w_curr.y();
//        odomAftMapped.pose.pose.position.z = t_w_curr.z();
//        pubOdomAftMapped.publish(odomAftMapped);
//
//        geometry_msgs::PoseStamped laserAfterMappedPose;
//        laserAfterMappedPose.header = odomAftMapped.header;
//        laserAfterMappedPose.pose = odomAftMapped.pose.pose;
//        laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
//        laserAfterMappedPath.header.frame_id = "/camera_init";
//        laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
//        pubLaserAfterMappedPath.publish(laserAfterMappedPath);
//
//        static tf::TransformBroadcaster br;
//        tf::Transform transform;
//        tf::Quaternion q1;
//        transform.setOrigin(tf::Vector3(t_w_curr(0), t_w_curr(1), t_w_curr(2)));
//        q1.setW(q_w_curr.w());
//        q1.setX(q_w_curr.x());
//        q1.setY(q_w_curr.y());
//        q1.setZ(q_w_curr.z());
//        transform.setRotation(q1);
//        br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));
//
//        r.sleep();
//    }
//
//    ros::Rate r1(1.0);
//    for (int i = 0 ; i < 3 ; i++)
//    {
//        std::cout<<i<<std::endl;
//        r1.sleep();
//    }
//
////    loop_closure_pub_optimzed_path(lipmatch.optimized_pose3d_map);
//
//    sensor_msgs::PointCloud2 matchedCloudOutMsg4;
//    pcl::toROSMsg(lipmatch.laserCloudOri_m2, matchedCloudOutMsg4);
//    matchedCloudOutMsg4.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//    matchedCloudOutMsg4.header.frame_id = "/camera_init";
//    pubMatchedPoints2.publish(matchedCloudOutMsg4);
//
//    //初始化
//    line_list2.header.frame_id = "/camera_init";
//    line_list2.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//    line_list2.ns = "points_and_lines";
//    line_list2.action = visualization_msgs::Marker::ADD;
//    line_list2.pose.orientation.w = 1.0;
//    //分配3个id
//    line_list2.id = 1;
//    //初始化形状
//    line_list2.type = visualization_msgs::Marker::LINE_LIST;
//    //初始化大小
//    // POINTS markers use x and y scale for width/height respectively
//    line_list2.scale.x = 2.75;
//    line_list2.scale.y = 2.75;
//    //初始化颜色
//    // Points are green
//    line_list2.color.r = 1.0f;
//    line_list2.color.g = 1.0f;
//    line_list2.color.a = 1.0;
//
//    for (map<size_t, size_t>::iterator it = lipmatch.loop_closure_matchedid.begin(); it != lipmatch.loop_closure_matchedid.end(); it++) {
//        geometry_msgs::Point p1 = points.points[it->first];
//        geometry_msgs::Point p2 = points.points[it->second];
//        line_list2.points.push_back(p1);
//        line_list2.points.push_back(p2);
//    }
//
//    marker_keyframe_pub.publish(line_list2);
//
//}



// loop closure
void process()
{

    std::ifstream timestamp_file("/media/jjwen/SBPD1_ddy/bk ubuntu18 6月7号/data/KITTI/odometry/00.txt", std::ifstream::in);

    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_transform(R_transform);

    std::string line;
    std::size_t line_num = 0;

    ros::Rate r(18.0);

    while (std::getline(timestamp_file, line) && ros::ok())
    {
        std::stringstream pose_stream(line);
        std::string s;
        for (std::size_t i = 0; i < 7; ++i)
        {
            std::getline(pose_stream, s, ' ');
            parameters[i] = stof(s);
        }

        pcl::PointCloud<pcl::PointXYZI> laserCloudIn;



        string binpath = "/media/jjwen/SBPD1_ddy/bk ubuntu18 6月7号/data/KITTI/odometry/00/semantic/"+to_string(line_num)+".bin";

                line_num++;


//        std::fstream output(binpath.c_str(), std::ios::out | std::ios::binary);
//        output.seekg(0, std::ios::beg);
//        for (auto point : laserCloudIn.points) {
//                output.write((char*)&point.x, sizeof(float));
//                output.write((char*)&point.y, sizeof(float));
//                output.write((char*)&point.z, sizeof(float));
//                output.write((char*)&point.intensity, sizeof(float));
//        }
//        output.close();

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

        laserCloudHandler(laserCloudIn);

        downSizeFilterKF.setInputCloud(cornerPointsLessSharp.makeShared());
        downSizeFilterKF.filter(cornerPointsLessSharp);

        downSizeFilterKF.setInputCloud(surfPointsLessFlat.makeShared());
        downSizeFilterKF.filter(surfPointsLessFlat);

        for (int i = 0; i < cornerPointsLessSharp.points.size(); i++)
        {
            pointAssociateToMap(&cornerPointsLessSharp.points[i], &cornerPointsLessSharp.points[i]);
        }

        for (int i = 0; i < surfPointsLessFlat.points.size(); i++)
        {
            pointAssociateToMap(&surfPointsLessFlat.points[i], &surfPointsLessFlat.points[i]);
        }

        for(auto it = m_keyframe_of_updating_list.begin(); it != m_keyframe_of_updating_list.end(); it++ )
        {
            *(it->structurelaserCloud) += *structure_points;
            *(it->vehiclelaserCloud) += *vehicle_points;
            *(it->naturelaserCloud) += *nature_points;
            *(it->g_laserCloud) += *ground_points;

            *(it->linelaserCloud) += cornerPointsLessSharp;
            *(it->surflaserCloud) += surfPointsLessFlat;

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
        if ( m_keyframe_of_updating_list.front().travel_length >= 30.0 )
        {
            m_keyframe_of_updating_list.front().m_ending_frame_idx = frameCount;
            m_keyframe_of_updating_list.front().m_pose_q = q_w_curr;
            m_keyframe_of_updating_list.front().m_pose_t =  t_w_curr;
            m_keyframe_need_precession_list.push_back( m_keyframe_of_updating_list.front() );
            m_keyframe_of_updating_list.pop_front();

            // lipmatch.frameQueue.push_back(m_keyframe_need_precession_list.back());

            geometry_msgs::Point p;
            p.x = t_w_curr.x();
            p.y = t_w_curr.y();
            p.z = t_w_curr.z();
            points.points.push_back(p);

            marker_keyframe_pub.publish(points);
        }

        //25 26
        if ( m_keyframe_of_updating_list.back().travel_length >= 25.0 )
        {
            tools::m_keyframe tk1;
            m_keyframe_of_updating_list.push_back(tk1);
        }

        // sensor_msgs::PointCloud2 matchedCloudOutMsg1;
        // pcl::toROSMsg(lipmatch.laserCloudOri_mp1, matchedCloudOutMsg1);
        // matchedCloudOutMsg1.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        // matchedCloudOutMsg1.header.frame_id = "/camera_init";
        // pubMatchedPoints1.publish(matchedCloudOutMsg1);

//        sensor_msgs::PointCloud2 matchedCloudOutMsg2;
//        pcl::toROSMsg(lipmatch.laserCloudOri_m2_1, matchedCloudOutMsg2);
//        matchedCloudOutMsg2.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//        matchedCloudOutMsg2.header.frame_id = "/camera_init";
//        pubMatchedPoints2.publish(matchedCloudOutMsg2);

        // sensor_msgs::PointCloud2 matchedCloudOutMsg2;
        // pcl::toROSMsg(lipmatch.laserCloudOri_m2, matchedCloudOutMsg2);
        // matchedCloudOutMsg2.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        // matchedCloudOutMsg2.header.frame_id = "/camera_init";
        // pubMatchedPoints2.publish(matchedCloudOutMsg2);

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

        // for (map<size_t, size_t>::iterator it = lipmatch.loop_closure_matchedid.begin(); it != lipmatch.loop_closure_matchedid.end(); it++) {
        //     geometry_msgs::Point p1 = points.points[it->first];
        //     geometry_msgs::Point p2 = points.points[it->second];
        //     line_list.points.push_back(p1);
        //     line_list.points.push_back(p2);
        // }

        // marker_keyframe_pub.publish(line_list);

//        sensor_msgs::PointCloud2 laserCloudFullRes3;
//        pcl::toROSMsg(lipmatch.refined_pt, laserCloudFullRes3);
//        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//        laserCloudFullRes3.header.frame_id = "/camera_init";
//        pubLaserCloudFullRes.publish(laserCloudFullRes3);

        // loop_closure_pub_optimzed_path(lipmatch.optimized_pose3d_map);


        // sensor_msgs::PointCloud2 laserCloudFullRes3;
        // pcl::toROSMsg(lipmatch.same_laserCloud, laserCloudFullRes3);
        // laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        // laserCloudFullRes3.header.frame_id = "/camera_init";
        // pubLaserCloudFullRes.publish(laserCloudFullRes3);

        // sensor_msgs::PointCloud2 laserCloudFullRes4;
        // pcl::toROSMsg(lipmatch.same_laserCloud2, laserCloudFullRes4);
        // laserCloudFullRes4.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        // laserCloudFullRes4.header.frame_id = "/camera_init";
        // pubLaserCloudFullResbef.publish(laserCloudFullRes4);

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

    // for (map<size_t, size_t>::iterator it = lipmatch.loop_closure_matchedid.begin(); it != lipmatch.loop_closure_matchedid.end(); it++) {
    //     geometry_msgs::Point p1 = points.points[it->first];
    //     geometry_msgs::Point p2 = points.points[it->second];
    //     line_list2.points.push_back(p1);
    //     line_list2.points.push_back(p2);
    // }

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

    pubEachFrameLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_each_frame", 300);

    m_pub_laser_aft_loopclosure_path = nh.advertise<nav_msgs::Path>( "/aft_loopclosure_path", 30000 );


    tools::m_keyframe tk;

    m_keyframe_of_updating_list.push_back(tk);

    process();

    ros::spin();

    return 0;
}
