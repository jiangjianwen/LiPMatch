// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_helper");
    ros::NodeHandle n("~");
    std::string dataset_folder, sequence_number, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);
    n.getParam("sequence_number", sequence_number);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);
    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);

    image_transport::ImageTransport it(n);

    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = "/camera_init";
    odomGT.child_frame_id = "/ground_truth";

    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);
    nav_msgs::Path pathGT;
    pathGT.header.frame_id = "/camera_init";

    std::ifstream timestamp_file("/home/jjwen/data/KITTI/odometry/00/times.txt", std::ifstream::in);

    std::ifstream ground_truth_file("/home/jjwen/data/KITTI/odometry/00.txt", std::ifstream::in);

    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);

    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_transform(R_transform);

    std::string line;
    std::size_t line_num = 0;

    ros::Rate r(10.0 / publish_delay);
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        float timestamp = stof(line);

        //cv::imshow("test",left_image);

        std::getline(ground_truth_file, line);
        std::stringstream pose_stream(line);
        std::string s;
        Eigen::Matrix<double, 3, 4> gt_pose;
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = stof(s);
            }
        }

        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
        Eigen::Quaterniond q = q_transform * q_w_i;
        q.normalize();
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();

        odomGT.header.stamp = ros::Time().fromSec(timestamp);
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT.publish(odomGT);

        geometry_msgs::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        pubPathGT.publish(pathGT);

        // read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << "/home/jjwen/data/KITTI/odometry/00/velodyne/"
                        << std::setfill('0') << std::setw(6) << line_num << ".bin";
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

        std::vector<Eigen::Vector3d> lidar_points;
        std::vector<float> lidar_intensities;
        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
            lidar_intensities.push_back(lidar_data[i+3]);

            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point);
        }

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        laser_cloud_msg.header.frame_id = "/camera_init";
        pub_laser_cloud.publish(laser_cloud_msg);

        if (to_bag)
        {
            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
            bag_out.write("/path_gt", ros::Time::now(), pathGT);
            bag_out.write("/odometry_gt", ros::Time::now(), odomGT);
        }

        line_num ++;
        r.sleep();
    }

    bag_out.close();
    std::cout << "Done \n";


    return 0;
}

//
//// Author:   Tong Qin               qintonguav@gmail.com
//// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk
//
//#include <iostream>
//#include <fstream>
//#include <iterator>
//#include <string>
//#include <vector>
//#include <opencv2/opencv.hpp>
//#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <nav_msgs/Odometry.h>
//#include <nav_msgs/Path.h>
//#include <ros/ros.h>
//#include <rosbag/bag.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <eigen3/Eigen/Dense>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include "RangenetAPI.hpp"
//
//using namespace std;
//
//std::string model_path;
//std::shared_ptr<RangenetAPI> net;
//std::vector<int> label_map_;
//namespace cl = rangenet::segmentation;
//
//
//std::vector<float> read_lidar_data(const std::string lidar_data_path)
//{
//    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
//    lidar_data_file.seekg(0, std::ios::end);
//    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
//    lidar_data_file.seekg(0, std::ios::beg);
//
//    std::vector<float> lidar_data_buffer(num_elements);
//    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
//    return lidar_data_buffer;
//}
//
//
//int main(int argc, char** argv)
//{
//    model_path = "/home/jjwen/model/darknet53";
//    net = std::shared_ptr<RangenetAPI> (new RangenetAPI(model_path));
//    label_map_ = net->getLabelMap();
//
//    ros::init(argc, argv, "kitti_helper");
//    ros::NodeHandle n("~");
//    std::string dataset_folder, sequence_number, output_bag_file;
//    n.getParam("dataset_folder", dataset_folder);
//    n.getParam("sequence_number", sequence_number);
//    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
//    bool to_bag;
//    n.getParam("to_bag", to_bag);
//    if (to_bag)
//        n.getParam("output_bag_file", output_bag_file);
//    int publish_delay;
//    n.getParam("publish_delay", publish_delay);
//    publish_delay = publish_delay <= 0 ? 1 : publish_delay;
//
//    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
//
//    image_transport::ImageTransport it(n);
//
//    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
//    nav_msgs::Odometry odomGT;
//    odomGT.header.frame_id = "/camera_init";
//    odomGT.child_frame_id = "/ground_truth";
//
//    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);
//    nav_msgs::Path pathGT;
//    pathGT.header.frame_id = "/camera_init";
//
//    std::ifstream timestamp_file("/home/jjwen/data/KITTI/odometry/00/times.txt", std::ifstream::in);
//
//    std::ifstream ground_truth_file("/home/jjwen/data/KITTI/odometry/00.txt", std::ifstream::in);
//
//    rosbag::Bag bag_out;
//    if (to_bag)
//        bag_out.open(output_bag_file, rosbag::bagmode::Write);
//
//    Eigen::Matrix3d R_transform;
//    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
//    Eigen::Quaterniond q_transform(R_transform);
//
//    std::string line;
//    std::size_t line_num = 0;
//
//    ros::Rate r(10.0 / publish_delay);
//    while (std::getline(timestamp_file, line) && ros::ok())
//    {
//        float timestamp = stof(line);
//
//        //cv::imshow("test",left_image);
//
//        std::getline(ground_truth_file, line);
//        std::stringstream pose_stream(line);
//        std::string s;
//        Eigen::Matrix<double, 3, 4> gt_pose;
//        for (std::size_t i = 0; i < 3; ++i)
//        {
//            for (std::size_t j = 0; j < 4; ++j)
//            {
//                std::getline(pose_stream, s, ' ');
//                gt_pose(i, j) = stof(s);
//            }
//        }
//
//        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
//        Eigen::Quaterniond q = q_transform * q_w_i;
//        q.normalize();
//        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();
//
//        odomGT.header.stamp = ros::Time().fromSec(timestamp);
//        odomGT.pose.pose.orientation.x = q.x();
//        odomGT.pose.pose.orientation.y = q.y();
//        odomGT.pose.pose.orientation.z = q.z();
//        odomGT.pose.pose.orientation.w = q.w();
//        odomGT.pose.pose.position.x = t(0);
//        odomGT.pose.pose.position.y = t(1);
//        odomGT.pose.pose.position.z = t(2);
//        pubOdomGT.publish(odomGT);
//
//        geometry_msgs::PoseStamped poseGT;
//        poseGT.header = odomGT.header;
//        poseGT.pose = odomGT.pose.pose;
//        pathGT.header.stamp = odomGT.header.stamp;
//        pathGT.poses.push_back(poseGT);
//        pubPathGT.publish(pathGT);
//
//        // read lidar point cloud
//        std::stringstream lidar_data_path;
//        lidar_data_path << "/home/jjwen/data/KITTI/odometry/05/velodyne/"
//                        << std::setfill('0') << std::setw(6) << line_num << ".bin";
//        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
//        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";
//
//        std::vector<Eigen::Vector3d> lidar_points;
//        std::vector<float> lidar_intensities;
//        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
//        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
//        {
//            lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
//            lidar_intensities.push_back(lidar_data[i+3]);
//
//            pcl::PointXYZI point;
//            point.x = lidar_data[i];
//            point.y = lidar_data[i + 1];
//            point.z = lidar_data[i + 2];
//            point.intensity = lidar_data[i + 3];
//            laser_cloud.push_back(point);
//        }
//
//        ///////////////////////////////
//
//        uint32_t num_points = laser_cloud.points.size();
//        std::vector<float> values(4 * num_points);
//        float max_remission = 0;
//        for (size_t i = 0 ; i < num_points ; ++i)
//        {
//            values[4 * i] = laser_cloud.points[i].x;
//            values[4 * i + 1] = laser_cloud.points[i].y;
//            values[4 * i + 2] = laser_cloud.points[i].z;
//            values[4 * i + 3] = laser_cloud.points[i].intensity;
//            max_remission = std::max(values[4 * i + 3], max_remission);
//        }
//
//        for (size_t i = 0; i < num_points; ++i)
//        {
//            values[4 * i + 3] /= max_remission;
//        }
//
//        // for semantic map
//        std::vector<float> color_mask;
//        std::vector<std::vector<float>> semantic_points = net->infer(values, num_points);
//
//        for (auto point : semantic_points) {
//            for (float ele : point){
//                color_mask.push_back(ele);
//            }
//        }
//
//        std::vector<float> labels;
//        std::vector<float> labels_prob;
//        labels.resize(num_points);
//        labels_prob.resize(num_points);
//
//        for (uint32_t i = 0; i < num_points; ++i) {
//            labels[i] = 0;
//            labels_prob[i] = 0;
//            for (uint32_t j = 0; j < 20; ++j)
//            {
//                if (labels_prob[i] <= color_mask[i*20+j])
//                {
//                    labels[i] = label_map_[j];
//                    labels_prob[i] = color_mask[i*20+j];
//                }
//            }
//        }
//
//        for (uint32_t i = 0; i < num_points; ++i) {
//            if (labels_prob[i] < 0.7) {
//                laser_cloud.points[i].intensity = 0;
//                continue;
//            }
//
//            laser_cloud.points[i].intensity = labels[i];
//        }
//
//        laser_cloud.width = 1;
//        laser_cloud.height = laser_cloud.points.size();
//
//        string binpath = "/home/jjwen/data/KITTI/odometry/05/semantic/"+to_string(line_num)+".bin";
//
//        std::fstream output(binpath.c_str(), std::ios::out | std::ios::binary);
//        output.seekg(0, std::ios::beg);
//        for (auto point : laser_cloud.points) {
//                output.write((char*)&point.x, sizeof(float));
//                output.write((char*)&point.y, sizeof(float));
//                output.write((char*)&point.z, sizeof(float));
//                output.write((char*)&point.intensity, sizeof(float));
//        }
//        output.close();
//
//        sensor_msgs::PointCloud2 laser_cloud_msg;
//        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
//        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
//        laser_cloud_msg.header.frame_id = "/camera_init";
//        pub_laser_cloud.publish(laser_cloud_msg);
//
//        if (to_bag)
//        {
//            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
//            bag_out.write("/path_gt", ros::Time::now(), pathGT);
//            bag_out.write("/odometry_gt", ros::Time::now(), odomGT);
//        }
//
//        line_num ++;
//        r.sleep();
//    }
//    bag_out.close();
//    std::cout << "Done \n";
//
//
//    return 0;
//}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
////// Author:   Tong Qin               qintonguav@gmail.com
////// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk
////
////#include <iostream>
////#include <fstream>
////#include <iterator>
////#include <string>
////#include <vector>
////#include <opencv2/opencv.hpp>
////#include <image_transport/image_transport.h>
////#include <opencv2/highgui/highgui.hpp>
////#include <nav_msgs/Odometry.h>
////#include <nav_msgs/Path.h>
//#include <ros/ros.h>
//#include <rosbag/bag.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <eigen3/Eigen/Dense>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <sensor_msgs/PointCloud2.h>
//#include<pcl/io/pcd_io.h>
//
//
//std::vector<float> read_lidar_data(const std::string lidar_data_path)
//{
//    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
//    lidar_data_file.seekg(0, std::ios::end);
//    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
//    lidar_data_file.seekg(0, std::ios::beg);
//
//    std::vector<float> lidar_data_buffer(num_elements);
//    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
//    return lidar_data_buffer;
//}
//
//
//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "kitti_helper");
//    ros::NodeHandle n("~");
//    std::string dataset_folder, sequence_number, output_bag_file;
//    n.getParam("dataset_folder", dataset_folder);
//    n.getParam("sequence_number", sequence_number);
//    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
//    bool to_bag;
//    n.getParam("to_bag", to_bag);
//    if (to_bag)
//        n.getParam("output_bag_file", output_bag_file);
//    int publish_delay;
//    n.getParam("publish_delay", publish_delay);
//    publish_delay = publish_delay <= 0 ? 1 : publish_delay;
//
//    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
//
//    image_transport::ImageTransport it(n);
//    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);
//    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2);
//
//    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
//    nav_msgs::Odometry odomGT;
//    odomGT.header.frame_id = "/camera_init";
//    odomGT.child_frame_id = "/ground_truth";
//
//    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);
//    nav_msgs::Path pathGT;
//    pathGT.header.frame_id = "/camera_init";
//
//    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
//    std::ifstream timestamp_file("/home/jjwen/data/KITTI/odometry/05/times.txt", std::ifstream::in);
//
//    std::string ground_truth_path = "results/" + sequence_number + ".txt";
//    std::ifstream ground_truth_file("/home/jjwen/data/KITTI/odometry/05.txt", std::ifstream::in);
//
//    rosbag::Bag bag_out;
//    if (to_bag)
//        bag_out.open(output_bag_file, rosbag::bagmode::Write);
//
//    Eigen::Matrix3d R_transform;
//    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
//    Eigen::Quaterniond q_transform(R_transform);
//
//    std::string line;
//    std::size_t line_num = 0;
//
//    ros::Rate r(10.0 / publish_delay);
//    while (std::getline(timestamp_file, line) && ros::ok())
//    {
//        float timestamp = stof(line);
//        std::stringstream left_image_path, right_image_path;
//
//        left_image_path << "/home/jjwen/data/KITTI/odometry/05/image_2/" << std::setfill('0') << std::setw(6) << line_num << ".png";
//        cv::Mat left_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
//        right_image_path << "/home/jjwen/data/KITTI/odometry/05/image_3/" << std::setfill('0') << std::setw(6) << line_num << ".png";
//        cv::Mat right_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
//
//        //std::cout<<"left_image_path "<<left_image_path.str()<<std::endl;
//
//        //cv::imshow("test",left_image);
//
//        std::getline(ground_truth_file, line);
//        std::stringstream pose_stream(line);
//        std::string s;
//        Eigen::Matrix<double, 3, 4> gt_pose;
//        for (std::size_t i = 0; i < 3; ++i)
//        {
//            for (std::size_t j = 0; j < 4; ++j)
//            {
//                std::getline(pose_stream, s, ' ');
//                gt_pose(i, j) = stof(s);
//            }
//        }
//
//        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
//        Eigen::Quaterniond q = q_transform * q_w_i;
//        q.normalize();
//        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();
//
//        odomGT.header.stamp = ros::Time().fromSec(timestamp);
//        odomGT.pose.pose.orientation.x = q.x();
//        odomGT.pose.pose.orientation.y = q.y();
//        odomGT.pose.pose.orientation.z = q.z();
//        odomGT.pose.pose.orientation.w = q.w();
//        odomGT.pose.pose.position.x = t(0);
//        odomGT.pose.pose.position.y = t(1);
//        odomGT.pose.pose.position.z = t(2);
//        pubOdomGT.publish(odomGT);
//
//        geometry_msgs::PoseStamped poseGT;
//        poseGT.header = odomGT.header;
//        poseGT.pose = odomGT.pose.pose;
//        pathGT.header.stamp = odomGT.header.stamp;
//        pathGT.poses.push_back(poseGT);
//        pubPathGT.publish(pathGT);
//
//        // read lidar point cloud
//        std::stringstream lidar_data_path;
//        lidar_data_path << "/home/jjwen/data/KITTI/odometry/05/velodyne/"
//                        << std::setfill('0') << std::setw(6) << line_num << ".bin";
//        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
//        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";
//
//        std::vector<Eigen::Vector3d> lidar_points;
//        std::vector<float> lidar_intensities;
//        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
//        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
//        {
//            lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
//            lidar_intensities.push_back(lidar_data[i+3]);
//
//            pcl::PointXYZI point;
//            point.x = lidar_data[i];
//            point.y = lidar_data[i + 1];
//            point.z = lidar_data[i + 2];
//            point.intensity = lidar_data[i + 3];
//            laser_cloud.push_back(point);
//        }
//
//        sensor_msgs::PointCloud2 laser_cloud_msg;
//        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
//        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
//        laser_cloud_msg.header.frame_id = "/camera_init";
//        pub_laser_cloud.publish(laser_cloud_msg);
//
//        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();
//        sensor_msgs::ImagePtr image_right_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image).toImageMsg();
//        pub_image_left.publish(image_left_msg);
//        pub_image_right.publish(image_right_msg);
//
//        if (to_bag)
//        {
//            bag_out.write("/image_left", ros::Time::now(), image_left_msg);
//            bag_out.write("/image_right", ros::Time::now(), image_right_msg);
//            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
//            bag_out.write("/path_gt", ros::Time::now(), pathGT);
//            bag_out.write("/odometry_gt", ros::Time::now(), odomGT);
//        }
//
//        line_num ++;
//        r.sleep();
//    }
//    bag_out.close();
//    std::cout << "Done \n";
//
//
//    return 0;
//}
//




//
//
//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "kitti_helper");
//    ros::NodeHandle n("~");
//    std::string dataset_folder, sequence_number, output_bag_file;
//    n.getParam("dataset_folder", dataset_folder);
//    n.getParam("sequence_number", sequence_number);
//    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
//    bool to_bag;
//    n.getParam("to_bag", to_bag);
//    if (to_bag)
//        n.getParam("output_bag_file", output_bag_file);
//    int publish_delay;
//    n.getParam("publish_delay", publish_delay);
//    publish_delay = publish_delay <= 0 ? 1 : publish_delay;
//
//    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
//
//    image_transport::ImageTransport it(n);
//    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);
//    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2);
//
//    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
//    nav_msgs::Odometry odomGT;
//    odomGT.header.frame_id = "/camera_init";
//    odomGT.child_frame_id = "/ground_truth";
//
//    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);
//    nav_msgs::Path pathGT;
//    pathGT.header.frame_id = "/camera_init";
//
//    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
//    std::ifstream timestamp_file("/home/jjwen/data/KITTI/odometry/05/times.txt", std::ifstream::in);
//
//    std::string ground_truth_path = "results/" + sequence_number + ".txt";
//    std::ifstream ground_truth_file("/home/jjwen/data/KITTI/odometry/05.txt", std::ifstream::in);
//
//    rosbag::Bag bag_out;
//    if (to_bag)
//        bag_out.open(output_bag_file, rosbag::bagmode::Write);
//
//    Eigen::Matrix3d R_transform;
//    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
//    Eigen::Quaterniond q_transform(R_transform);
//
//    std::string line;
//    std::size_t line_num = 0;
//
//    ros::Rate r(10.0 / publish_delay);
//    while (line_num < 2513 && ros::ok())
//    {
//        // read lidar point cloud
//        std::stringstream lidar_data_path;
//        lidar_data_path << "/home/jjwen/Desktop/mapper/build/"
//                        << std::setfill('0') << std::setw(5) << line_num << ".pcd";
//
//        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
//
//        pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_data_path.str(), laser_cloud);
//
//        sensor_msgs::PointCloud2 laser_cloud_msg;
//        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
//        laser_cloud_msg.header.stamp = ros::Time::now();
//        laser_cloud_msg.header.frame_id = "/camera_init";
//        pub_laser_cloud.publish(laser_cloud_msg);
//
//
//        line_num ++;
//        r.sleep();
//    }
//    bag_out.close();
//    std::cout << "Done \n";
//
//
//    return 0;
//}