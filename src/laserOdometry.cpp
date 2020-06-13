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

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"

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

Eigen::Matrix3d oldCov;

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
}


void calculate_ICP_COV(pcl::PointCloud<pcl::PointNormal> &data_pi, pcl::PointCloud<pcl::PointNormal> &model_qi,
                       Eigen::Matrix4f &transform, Eigen::MatrixXd &ICP_COV) {

    double Tx = transform(0, 3);
    double Ty = transform(1, 3);
    double Tz = transform(2, 3);
    double roll = atan2f(transform(2, 1), transform(2, 2));
    double pitch = asinf(-transform(2, 0));
    double yaw = atan2f(transform(1, 0), transform(0, 0));

//    std::cout<<"============="<<std::endl;
//    std::cout<<yaw<<"  "<<pitch<<"  "<<roll<<std::endl;
//    std::cout<<"============="<<std::endl;

    double x, y, z, a, b, c;
    x = Tx;
    y = Ty;
    z = Tz;
    a = yaw;
    b = pitch;
    c = roll;// important // According to the rotation matrix I used and after verification, it is Yaw Pitch ROLL = [a,b,c]== [R] matrix used in the MatLab also :)

    /* Flushing out in the form of XYZ ABC */
    std::cout << "\nPrinting out [x, y, z, a, b, c] =  " << x << "    " << y << "    " << z << "    "
                                                         << a << "    " << b << "    " << c << std::endl;

    //Matrix initialization
    Eigen::MatrixXd d2J_dX2(6, 6);
    d2J_dX2 = Eigen::MatrixXd::Zero(6, 6);

    /****  Calculating d2J_dX2  ****/
    for (size_t s = 0; s < data_pi.points.size(); ++s) {
        double pix = data_pi.points[s].x;
        double piy = data_pi.points[s].y;
        double piz = data_pi.points[s].z;
        double qix = model_qi.points[s].x;
        double qiy = model_qi.points[s].y;
        double qiz = model_qi.points[s].z;

        double nix = model_qi[s].normal_x;
        double niy = model_qi[s].normal_y;
        double niz = model_qi[s].normal_z;

        if (niz != niz)// for nan removal in the input point cloud data:)
            continue;

        /***********************************************************************
        d2J_dX2 -- X is the [R|T] in the form of [x, y, z, a, b, c]
        x, y, z is the translation part
        a, b, c is the rotation part in Euler format
        [x, y, z, a, b, c] is acquired from the Transformation Matrix returned by ICP.

        Now d2J_dX2 is a 6x6 matrix of the form

        d2J_dx2
        d2J_dxdy    d2J_dy2
        d2J_dxdz    d2J_dydz    d2J_dz2
        d2J_dxda    d2J_dyda    d2J_dzda   d2J_da2
        d2J_dxdb    d2J_dydb    d2J_dzdb   d2J_dadb   d2J_db2
        d2J_dxdc    d2J_dydc    d2J_dzdc   d2J_dadc   d2J_dbdc   d2J_dc2
        *************************************************************************/

        double  d2J_dx2, d2J_dydx, d2J_dzdx, d2J_dadx, d2J_dbdx, d2J_dcdx,
                d2J_dxdy, d2J_dy2, d2J_dzdy, d2J_dady, d2J_dbdy, d2J_dcdy,
                d2J_dxdz, d2J_dydz, d2J_dz2, d2J_dadz, d2J_dbdz, d2J_dcdz,
                d2J_dxda, d2J_dyda, d2J_dzda, d2J_da2, d2J_dbda, d2J_dcda,
                d2J_dxdb, d2J_dydb, d2J_dzdb, d2J_dadb, d2J_db2, d2J_dcdb,
                d2J_dxdc, d2J_dydc, d2J_dzdc, d2J_dadc, d2J_dbdc, d2J_dc2;

        // These terms are generated from the provided Matlab scipts. We just have to copy
        // the expressions from the matlab output with two very simple changes.
        // The first one being the the sqaure of a number 'a' is shown as a^2 in matlab,
        // which is converted to pow(a,2) in the below expressions.
        // The second change is to add ';' at the end of each expression :)
        // In this way, matlab can be used to generate these terms for various objective functions of ICP
        // and they can simply be copied to the C++ files and with appropriate changes to ICP estimation,
        // its covariance can be easily estimated.

        d2J_dx2 = 2 * pow(nix, 2);
        d2J_dy2 = 2 * pow(niy, 2);
        d2J_dz2 = 2 * pow(niz, 2);
        d2J_dydx = 2 * nix * niy;
        d2J_dxdy = 2 * nix * niy;
        d2J_dzdx = 2 * nix * niz;
        d2J_dxdz = 2 * nix * niz;
        d2J_dydz = 2 * niy * niz;
        d2J_dzdy = 2 * niy * niz;
        d2J_da2 = (niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                   piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) - nix *
                           (piy *
                                                                                                       (cos(a) *
                                                                                                        cos(c) +
                                                                                                        sin(a) *
                                                                                                        sin(b) *
                                                                                                        sin(c)) -
                                                                                                       piz *
                                                                                                       (cos(a) *
                                                                                                        sin(c) -
                                                                                                        cos(c) *
                                                                                                        sin(a) *
                                                                                                        sin(b)) +
                                                                                                       pix *
                                                                                                       cos(b) *
                                                                                                       sin(a))) *
                (2 * niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                            piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                 2 * nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                            piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a))) -
                (2 * nix * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                            piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) +
                 2 * niy * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                            piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a))) *
                (nix * (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
                        piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) + pix * cos(a) * cos(b)) + niy *
                                                                                                      (y - qiy +
                                                                                                       piy *
                                                                                                       (cos(a) *
                                                                                                        cos(c) +
                                                                                                        sin(a) *
                                                                                                        sin(b) *
                                                                                                        sin(c)) -
                                                                                                       piz *
                                                                                                       (cos(a) *
                                                                                                        sin(c) -
                                                                                                        cos(c) *
                                                                                                        sin(a) *
                                                                                                        sin(b)) +
                                                                                                       pix *
                                                                                                       cos(b) *
                                                                                                       sin(a)) +
                 niz * (z - qiz - pix * sin(b) + piz * cos(b) * cos(c) + piy * cos(b) * sin(c)));


        d2J_db2 =

                (niy *
                 (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) + piy * cos(b) * sin(a) * sin(c)) -
                 niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + nix *
                                                                                        (piz * cos(a) * cos(b) *
                                                                                         cos(c) -
                                                                                         pix * cos(a) * sin(b) +
                                                                                         piy * cos(a) * cos(b) *
                                                                                         sin(c))) * (2 * niy *
                                                                                                     (piz *
                                                                                                      cos(b) *
                                                                                                      cos(c) *
                                                                                                      sin(a) -
                                                                                                      pix *
                                                                                                      sin(a) *
                                                                                                      sin(b) +
                                                                                                      piy *
                                                                                                      cos(b) *
                                                                                                      sin(a) *
                                                                                                      sin(c)) -
                                                                                                     2 * niz *
                                                                                                     (pix *
                                                                                                      cos(b) +
                                                                                                      piz *
                                                                                                      cos(c) *
                                                                                                      sin(b) +
                                                                                                      piy *
                                                                                                      sin(b) *
                                                                                                      sin(c)) +
                                                                                                     2 * nix *
                                                                                                     (piz *
                                                                                                      cos(a) *
                                                                                                      cos(b) *
                                                                                                      cos(c) -
                                                                                                      pix *
                                                                                                      cos(a) *
                                                                                                      sin(b) +
                                                                                                      piy *
                                                                                                      cos(a) *
                                                                                                      cos(b) *
                                                                                                      sin(c))) -
                (2 * niy *
                 (pix * cos(b) * sin(a) + piz * cos(c) * sin(a) * sin(b) + piy * sin(a) * sin(b) * sin(c)) +
                 2 * niz * (piz * cos(b) * cos(c) - pix * sin(b) + piy * cos(b) * sin(c)) + 2 * nix *
                                                                                            (pix * cos(a) *
                                                                                             cos(b) +
                                                                                             piz * cos(a) *
                                                                                             cos(c) * sin(b) +
                                                                                             piy * cos(a) *
                                                                                             sin(b) * sin(c))) *
                (nix * (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
                        piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) + pix * cos(a) * cos(b)) + niy *
                                                                                                      (y - qiy +
                                                                                                       piy *
                                                                                                       (cos(a) *
                                                                                                        cos(c) +
                                                                                                        sin(a) *
                                                                                                        sin(b) *
                                                                                                        sin(c)) -
                                                                                                       piz *
                                                                                                       (cos(a) *
                                                                                                        sin(c) -
                                                                                                        cos(c) *
                                                                                                        sin(a) *
                                                                                                        sin(b)) +
                                                                                                       pix *
                                                                                                       cos(b) *
                                                                                                       sin(a)) +
                 niz * (z - qiz - pix * sin(b) + piz * cos(b) * cos(c) + piy * cos(b) * sin(c)));


        d2J_dc2 =

                (nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                        piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - niy * (piy * (cos(a) * sin(c) -
                                                                                            cos(c) * sin(a) *
                                                                                            sin(b)) + piz *
                                                                                                      (cos(a) *
                                                                                                       cos(c) +
                                                                                                       sin(a) *
                                                                                                       sin(b) *
                                                                                                       sin(c))) +
                 niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c))) * (2 * nix * (piy * (sin(a) * sin(c) +
                                                                                             cos(a) * cos(c) *
                                                                                             sin(b)) + piz *
                                                                                                       (cos(c) *
                                                                                                        sin(a) -
                                                                                                        cos(a) *
                                                                                                        sin(b) *
                                                                                                        sin(c))) -
                                                                           2 * niy * (piy * (cos(a) * sin(c) -
                                                                                             cos(c) * sin(a) *
                                                                                             sin(b)) + piz *
                                                                                                       (cos(a) *
                                                                                                        cos(c) +
                                                                                                        sin(a) *
                                                                                                        sin(b) *
                                                                                                        sin(c))) +
                                                                           2 * niz * (piy * cos(b) * cos(c) -
                                                                                      piz * cos(b) * sin(c))) -
                (2 * niy * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                            piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b))) - 2 * nix * (piy *
                                                                                             (cos(c) * sin(a) -
                                                                                              cos(a) * sin(b) *
                                                                                              sin(c)) - piz *
                                                                                                        (sin(a) *
                                                                                                         sin(c) +
                                                                                                         cos(a) *
                                                                                                         cos(c) *
                                                                                                         sin(b))) +
                 2 * niz * (piz * cos(b) * cos(c) + piy * cos(b) * sin(c))) * (nix * (x - qix - piy * (cos(c) *
                                                                                                       sin(a) -
                                                                                                       cos(a) *
                                                                                                       sin(b) *
                                                                                                       sin(c)) +
                                                                                      piz * (sin(a) * sin(c) +
                                                                                             cos(a) * cos(c) *
                                                                                             sin(b)) +
                                                                                      pix * cos(a) * cos(b)) +
                                                                               niy * (y - qiy + piy * (cos(a) *
                                                                                                       cos(c) +
                                                                                                       sin(a) *
                                                                                                       sin(b) *
                                                                                                       sin(c)) -
                                                                                      piz * (cos(a) * sin(c) -
                                                                                             cos(c) * sin(a) *
                                                                                             sin(b)) +
                                                                                      pix * cos(b) * sin(a)) +
                                                                               niz * (z - qiz - pix * sin(b) +
                                                                                      piz * cos(b) * cos(c) +
                                                                                      piy * cos(b) * sin(c)));

        d2J_dxda =

                nix * (2 * niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                                  piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                       2 * nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                                  piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a)));


        d2J_dadx =

                2 * nix * (niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                                  piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                           nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                                  piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a)));


        d2J_dyda =

                niy * (2 * niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                                  piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                       2 * nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                                  piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a)));


        d2J_dady =

                2 * niy * (niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                                  piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                           nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                                  piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a)));


        d2J_dzda =

                niz * (2 * niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                                  piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                       2 * nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                                  piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a)));


        d2J_dadz =

                2 * niz * (niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                                  piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                           nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                                  piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a)));


        d2J_dxdb =

                nix * (2 * niy * (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) +
                                  piy * cos(b) * sin(a) * sin(c)) -
                       2 * niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + 2 * nix *
                                                                                                  (piz *
                                                                                                   cos(a) *
                                                                                                   cos(b) *
                                                                                                   cos(c) -
                                                                                                   pix *
                                                                                                   cos(a) *
                                                                                                   sin(b) +
                                                                                                   piy *
                                                                                                   cos(a) *
                                                                                                   cos(b) *
                                                                                                   sin(c)));


        d2J_dbdx =

                2 * nix * (niy * (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) +
                                  piy * cos(b) * sin(a) * sin(c)) -
                           niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + nix * (piz *
                                                                                                         cos(a) *
                                                                                                         cos(b) *
                                                                                                         cos(c) -
                                                                                                         pix *
                                                                                                         cos(a) *
                                                                                                         sin(b) +
                                                                                                         piy *
                                                                                                         cos(a) *
                                                                                                         cos(b) *
                                                                                                         sin(c)));


        d2J_dydb =

                niy * (2 * niy * (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) +
                                  piy * cos(b) * sin(a) * sin(c)) -
                       2 * niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + 2 * nix *
                                                                                                  (piz *
                                                                                                   cos(a) *
                                                                                                   cos(b) *
                                                                                                   cos(c) -
                                                                                                   pix *
                                                                                                   cos(a) *
                                                                                                   sin(b) +
                                                                                                   piy *
                                                                                                   cos(a) *
                                                                                                   cos(b) *
                                                                                                   sin(c)));


        d2J_dbdy =

                2 * niy * (niy * (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) +
                                  piy * cos(b) * sin(a) * sin(c)) -
                           niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + nix * (piz *
                                                                                                         cos(a) *
                                                                                                         cos(b) *
                                                                                                         cos(c) -
                                                                                                         pix *
                                                                                                         cos(a) *
                                                                                                         sin(b) +
                                                                                                         piy *
                                                                                                         cos(a) *
                                                                                                         cos(b) *
                                                                                                         sin(c)));

        d2J_dzdb =

                niz * (2 * niy * (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) +
                                  piy * cos(b) * sin(a) * sin(c)) -
                       2 * niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + 2 * nix *
                                                                                                  (piz *
                                                                                                   cos(a) *
                                                                                                   cos(b) *
                                                                                                   cos(c) -
                                                                                                   pix *
                                                                                                   cos(a) *
                                                                                                   sin(b) +
                                                                                                   piy *
                                                                                                   cos(a) *
                                                                                                   cos(b) *
                                                                                                   sin(c)));


        d2J_dbdz =

                2 * niz * (niy * (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) +
                                  piy * cos(b) * sin(a) * sin(c)) -
                           niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + nix * (piz *
                                                                                                         cos(a) *
                                                                                                         cos(b) *
                                                                                                         cos(c) -
                                                                                                         pix *
                                                                                                         cos(a) *
                                                                                                         sin(b) +
                                                                                                         piy *
                                                                                                         cos(a) *
                                                                                                         cos(b) *
                                                                                                         sin(c)));


        d2J_dxdc =

                nix * (2 * nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                                  piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - 2 * niy * (piy *
                                                                                                   (cos(a) *
                                                                                                    sin(c) -
                                                                                                    cos(c) *
                                                                                                    sin(a) *
                                                                                                    sin(b)) +
                                                                                                   piz *
                                                                                                   (cos(a) *
                                                                                                    cos(c) +
                                                                                                    sin(a) *
                                                                                                    sin(b) *
                                                                                                    sin(c))) +
                       2 * niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c)));


        d2J_dcdx =

                2 * nix * (nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                                  piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - niy * (piy * (cos(a) *
                                                                                                      sin(c) -
                                                                                                      cos(c) *
                                                                                                      sin(a) *
                                                                                                      sin(b)) +
                                                                                               piz * (cos(a) *
                                                                                                      cos(c) +
                                                                                                      sin(a) *
                                                                                                      sin(b) *
                                                                                                      sin(c))) +
                           niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c)));


        d2J_dydc =

                niy * (2 * nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                                  piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - 2 * niy * (piy *
                                                                                                   (cos(a) *
                                                                                                    sin(c) -
                                                                                                    cos(c) *
                                                                                                    sin(a) *
                                                                                                    sin(b)) +
                                                                                                   piz *
                                                                                                   (cos(a) *
                                                                                                    cos(c) +
                                                                                                    sin(a) *
                                                                                                    sin(b) *
                                                                                                    sin(c))) +
                       2 * niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c)));

        d2J_dcdy =

                2 * niy * (nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                                  piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - niy * (piy * (cos(a) *
                                                                                                      sin(c) -
                                                                                                      cos(c) *
                                                                                                      sin(a) *
                                                                                                      sin(b)) +
                                                                                               piz * (cos(a) *
                                                                                                      cos(c) +
                                                                                                      sin(a) *
                                                                                                      sin(b) *
                                                                                                      sin(c))) +
                           niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c)));


        d2J_dzdc =

                niz * (2 * nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                                  piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - 2 * niy * (piy *
                                                                                                   (cos(a) *
                                                                                                    sin(c) -
                                                                                                    cos(c) *
                                                                                                    sin(a) *
                                                                                                    sin(b)) +
                                                                                                   piz *
                                                                                                   (cos(a) *
                                                                                                    cos(c) +
                                                                                                    sin(a) *
                                                                                                    sin(b) *
                                                                                                    sin(c))) +
                       2 * niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c)));


        d2J_dcdz =

                2 * niz * (nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                                  piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - niy * (piy * (cos(a) *
                                                                                                      sin(c) -
                                                                                                      cos(c) *
                                                                                                      sin(a) *
                                                                                                      sin(b)) +
                                                                                               piz * (cos(a) *
                                                                                                      cos(c) +
                                                                                                      sin(a) *
                                                                                                      sin(b) *
                                                                                                      sin(c))) +
                           niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c)));


        d2J_dadb =

                (niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                        piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) - nix *
                                                                                                      (piy *
                                                                                                       (cos(a) *
                                                                                                        cos(c) +
                                                                                                        sin(a) *
                                                                                                        sin(b) *
                                                                                                        sin(c)) -
                                                                                                       piz *
                                                                                                       (cos(a) *
                                                                                                        sin(c) -
                                                                                                        cos(c) *
                                                                                                        sin(a) *
                                                                                                        sin(b)) +
                                                                                                       pix *
                                                                                                       cos(b) *
                                                                                                       sin(a))) *
                (2 * niy *
                 (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) + piy * cos(b) * sin(a) * sin(c)) -
                 2 * niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + 2 * nix *
                                                                                            (piz * cos(a) *
                                                                                             cos(b) * cos(c) -
                                                                                             pix * cos(a) *
                                                                                             sin(b) +
                                                                                             piy * cos(a) *
                                                                                             cos(b) * sin(c))) -
                (2 * nix *
                 (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) + piy * cos(b) * sin(a) * sin(c)) -
                 2 * niy *
                 (piz * cos(a) * cos(b) * cos(c) - pix * cos(a) * sin(b) + piy * cos(a) * cos(b) * sin(c))) *
                (nix * (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
                        piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) + pix * cos(a) * cos(b)) + niy *
                                                                                                      (y - qiy +
                                                                                                       piy *
                                                                                                       (cos(a) *
                                                                                                        cos(c) +
                                                                                                        sin(a) *
                                                                                                        sin(b) *
                                                                                                        sin(c)) -
                                                                                                       piz *
                                                                                                       (cos(a) *
                                                                                                        sin(c) -
                                                                                                        cos(c) *
                                                                                                        sin(a) *
                                                                                                        sin(b)) +
                                                                                                       pix *
                                                                                                       cos(b) *
                                                                                                       sin(a)) +
                 niz * (z - qiz - pix * sin(b) + piz * cos(b) * cos(c) + piy * cos(b) * sin(c)));


        d2J_dbda =

                (2 * niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                            piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                 2 * nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                            piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a))) *
                (niy *
                 (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) + piy * cos(b) * sin(a) * sin(c)) -
                 niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + nix *
                                                                                        (piz * cos(a) * cos(b) *
                                                                                         cos(c) -
                                                                                         pix * cos(a) * sin(b) +
                                                                                         piy * cos(a) * cos(b) *
                                                                                         sin(c))) - (2 * nix *
                                                                                                     (piz *
                                                                                                      cos(b) *
                                                                                                      cos(c) *
                                                                                                      sin(a) -
                                                                                                      pix *
                                                                                                      sin(a) *
                                                                                                      sin(b) +
                                                                                                      piy *
                                                                                                      cos(b) *
                                                                                                      sin(a) *
                                                                                                      sin(c)) -
                                                                                                     2 * niy *
                                                                                                     (piz *
                                                                                                      cos(a) *
                                                                                                      cos(b) *
                                                                                                      cos(c) -
                                                                                                      pix *
                                                                                                      cos(a) *
                                                                                                      sin(b) +
                                                                                                      piy *
                                                                                                      cos(a) *
                                                                                                      cos(b) *
                                                                                                      sin(c))) *
                                                                                                    (nix *
                                                                                                     (x - qix -
                                                                                                      piy *
                                                                                                      (cos(c) *
                                                                                                       sin(a) -
                                                                                                       cos(a) *
                                                                                                       sin(b) *
                                                                                                       sin(c)) +
                                                                                                      piz *
                                                                                                      (sin(a) *
                                                                                                       sin(c) +
                                                                                                       cos(a) *
                                                                                                       cos(c) *
                                                                                                       sin(b)) +
                                                                                                      pix *
                                                                                                      cos(a) *
                                                                                                      cos(b)) +
                                                                                                     niy *
                                                                                                     (y - qiy +
                                                                                                      piy *
                                                                                                      (cos(a) *
                                                                                                       cos(c) +
                                                                                                       sin(a) *
                                                                                                       sin(b) *
                                                                                                       sin(c)) -
                                                                                                      piz *
                                                                                                      (cos(a) *
                                                                                                       sin(c) -
                                                                                                       cos(c) *
                                                                                                       sin(a) *
                                                                                                       sin(b)) +
                                                                                                      pix *
                                                                                                      cos(b) *
                                                                                                      sin(a)) +
                                                                                                     niz *
                                                                                                     (z - qiz -
                                                                                                      pix *
                                                                                                      sin(b) +
                                                                                                      piz *
                                                                                                      cos(b) *
                                                                                                      cos(c) +
                                                                                                      piy *
                                                                                                      cos(b) *
                                                                                                      sin(c)));


        d2J_dbdc =

                (2 * nix * (piy * cos(a) * cos(b) * cos(c) - piz * cos(a) * cos(b) * sin(c)) -
                 2 * niz * (piy * cos(c) * sin(b) - piz * sin(b) * sin(c)) +
                 2 * niy * (piy * cos(b) * cos(c) * sin(a) - piz * cos(b) * sin(a) * sin(c))) * (nix *
                                                                                                 (x - qix -
                                                                                                  piy *
                                                                                                  (cos(c) *
                                                                                                   sin(a) -
                                                                                                   cos(a) *
                                                                                                   sin(b) *
                                                                                                   sin(c)) +
                                                                                                  piz *
                                                                                                  (sin(a) *
                                                                                                   sin(c) +
                                                                                                   cos(a) *
                                                                                                   cos(c) *
                                                                                                   sin(b)) +
                                                                                                  pix * cos(a) *
                                                                                                  cos(b)) +
                                                                                                 niy *
                                                                                                 (y - qiy +
                                                                                                  piy *
                                                                                                  (cos(a) *
                                                                                                   cos(c) +
                                                                                                   sin(a) *
                                                                                                   sin(b) *
                                                                                                   sin(c)) -
                                                                                                  piz *
                                                                                                  (cos(a) *
                                                                                                   sin(c) -
                                                                                                   cos(c) *
                                                                                                   sin(a) *
                                                                                                   sin(b)) +
                                                                                                  pix * cos(b) *
                                                                                                  sin(a)) +
                                                                                                 niz *
                                                                                                 (z - qiz -
                                                                                                  pix * sin(b) +
                                                                                                  piz * cos(b) *
                                                                                                  cos(c) +
                                                                                                  piy * cos(b) *
                                                                                                  sin(c))) +
                (niy *
                 (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) + piy * cos(b) * sin(a) * sin(c)) -
                 niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + nix *
                                                                                        (piz * cos(a) * cos(b) *
                                                                                         cos(c) -
                                                                                         pix * cos(a) * sin(b) +
                                                                                         piy * cos(a) * cos(b) *
                                                                                         sin(c))) * (2 * nix *
                                                                                                     (piy *
                                                                                                      (sin(a) *
                                                                                                       sin(c) +
                                                                                                       cos(a) *
                                                                                                       cos(c) *
                                                                                                       sin(b)) +
                                                                                                      piz *
                                                                                                      (cos(c) *
                                                                                                       sin(a) -
                                                                                                       cos(a) *
                                                                                                       sin(b) *
                                                                                                       sin(c))) -
                                                                                                     2 * niy *
                                                                                                     (piy *
                                                                                                      (cos(a) *
                                                                                                       sin(c) -
                                                                                                       cos(c) *
                                                                                                       sin(a) *
                                                                                                       sin(b)) +
                                                                                                      piz *
                                                                                                      (cos(a) *
                                                                                                       cos(c) +
                                                                                                       sin(a) *
                                                                                                       sin(b) *
                                                                                                       sin(c))) +
                                                                                                     2 * niz *
                                                                                                     (piy *
                                                                                                      cos(b) *
                                                                                                      cos(c) -
                                                                                                      piz *
                                                                                                      cos(b) *
                                                                                                      sin(c)));


        d2J_dcdb =

                (2 * nix * (piy * cos(a) * cos(b) * cos(c) - piz * cos(a) * cos(b) * sin(c)) -
                 2 * niz * (piy * cos(c) * sin(b) - piz * sin(b) * sin(c)) +
                 2 * niy * (piy * cos(b) * cos(c) * sin(a) - piz * cos(b) * sin(a) * sin(c))) * (nix *
                                                                                                 (x - qix -
                                                                                                  piy *
                                                                                                  (cos(c) *
                                                                                                   sin(a) -
                                                                                                   cos(a) *
                                                                                                   sin(b) *
                                                                                                   sin(c)) +
                                                                                                  piz *
                                                                                                  (sin(a) *
                                                                                                   sin(c) +
                                                                                                   cos(a) *
                                                                                                   cos(c) *
                                                                                                   sin(b)) +
                                                                                                  pix * cos(a) *
                                                                                                  cos(b)) +
                                                                                                 niy *
                                                                                                 (y - qiy +
                                                                                                  piy *
                                                                                                  (cos(a) *
                                                                                                   cos(c) +
                                                                                                   sin(a) *
                                                                                                   sin(b) *
                                                                                                   sin(c)) -
                                                                                                  piz *
                                                                                                  (cos(a) *
                                                                                                   sin(c) -
                                                                                                   cos(c) *
                                                                                                   sin(a) *
                                                                                                   sin(b)) +
                                                                                                  pix * cos(b) *
                                                                                                  sin(a)) +
                                                                                                 niz *
                                                                                                 (z - qiz -
                                                                                                  pix * sin(b) +
                                                                                                  piz * cos(b) *
                                                                                                  cos(c) +
                                                                                                  piy * cos(b) *
                                                                                                  sin(c))) +
                (2 * niy *
                 (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) + piy * cos(b) * sin(a) * sin(c)) -
                 2 * niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + 2 * nix *
                                                                                            (piz * cos(a) *
                                                                                             cos(b) * cos(c) -
                                                                                             pix * cos(a) *
                                                                                             sin(b) +
                                                                                             piy * cos(a) *
                                                                                             cos(b) * sin(c))) *
                (nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                        piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - niy * (piy * (cos(a) * sin(c) -
                                                                                            cos(c) * sin(a) *
                                                                                            sin(b)) + piz *
                                                                                                      (cos(a) *
                                                                                                       cos(c) +
                                                                                                       sin(a) *
                                                                                                       sin(b) *
                                                                                                       sin(c))) +
                 niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c)));


        d2J_dcda =

                (2 * niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                            piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                 2 * nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                            piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a))) *
                (nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                        piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - niy * (piy * (cos(a) * sin(c) -
                                                                                            cos(c) * sin(a) *
                                                                                            sin(b)) + piz *
                                                                                                      (cos(a) *
                                                                                                       cos(c) +
                                                                                                       sin(a) *
                                                                                                       sin(b) *
                                                                                                       sin(c))) +
                 niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c))) + (2 * nix * (piy * (cos(a) * sin(c) -
                                                                                             cos(c) * sin(a) *
                                                                                             sin(b)) + piz *
                                                                                                       (cos(a) *
                                                                                                        cos(c) +
                                                                                                        sin(a) *
                                                                                                        sin(b) *
                                                                                                        sin(c))) +
                                                                           2 * niy * (piy * (sin(a) * sin(c) +
                                                                                             cos(a) * cos(c) *
                                                                                             sin(b)) + piz *
                                                                                                       (cos(c) *
                                                                                                        sin(a) -
                                                                                                        cos(a) *
                                                                                                        sin(b) *
                                                                                                        sin(c)))) *
                                                                          (nix * (x - qix - piy *
                                                                                            (cos(c) * sin(a) -
                                                                                             cos(a) * sin(b) *
                                                                                             sin(c)) + piz *
                                                                                                       (sin(a) *
                                                                                                        sin(c) +
                                                                                                        cos(a) *
                                                                                                        cos(c) *
                                                                                                        sin(b)) +
                                                                                  pix * cos(a) * cos(b)) + niy *
                                                                                                           (y -
                                                                                                            qiy +
                                                                                                            piy *
                                                                                                            (cos(a) *
                                                                                                             cos(c) +
                                                                                                             sin(a) *
                                                                                                             sin(b) *
                                                                                                             sin(c)) -
                                                                                                            piz *
                                                                                                            (cos(a) *
                                                                                                             sin(c) -
                                                                                                             cos(c) *
                                                                                                             sin(a) *
                                                                                                             sin(b)) +
                                                                                                            pix *
                                                                                                            cos(b) *
                                                                                                            sin(a)) +
                                                                           niz * (z - qiz - pix * sin(b) +
                                                                                  piz * cos(b) * cos(c) +
                                                                                  piy * cos(b) * sin(c)));


        d2J_dadc =

                (niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                        piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) - nix *
                                                                                                      (piy *
                                                                                                       (cos(a) *
                                                                                                        cos(c) +
                                                                                                        sin(a) *
                                                                                                        sin(b) *
                                                                                                        sin(c)) -
                                                                                                       piz *
                                                                                                       (cos(a) *
                                                                                                        sin(c) -
                                                                                                        cos(c) *
                                                                                                        sin(a) *
                                                                                                        sin(b)) +
                                                                                                       pix *
                                                                                                       cos(b) *
                                                                                                       sin(a))) *
                (2 * nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                            piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - 2 * niy * (piy *
                                                                                             (cos(a) * sin(c) -
                                                                                              cos(c) * sin(a) *
                                                                                              sin(b)) + piz *
                                                                                                        (cos(a) *
                                                                                                         cos(c) +
                                                                                                         sin(a) *
                                                                                                         sin(b) *
                                                                                                         sin(c))) +
                 2 * niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c))) + (2 * nix * (piy *
                                                                                          (cos(a) * sin(c) -
                                                                                           cos(c) * sin(a) *
                                                                                           sin(b)) + piz *
                                                                                                     (cos(a) *
                                                                                                      cos(c) +
                                                                                                      sin(a) *
                                                                                                      sin(b) *
                                                                                                      sin(c))) +
                                                                               2 * niy * (piy *
                                                                                          (sin(a) * sin(c) +
                                                                                           cos(a) * cos(c) *
                                                                                           sin(b)) + piz *
                                                                                                     (cos(c) *
                                                                                                      sin(a) -
                                                                                                      cos(a) *
                                                                                                      sin(b) *
                                                                                                      sin(c)))) *
                                                                              (nix * (x - qix - piy * (cos(c) *
                                                                                                       sin(a) -
                                                                                                       cos(a) *
                                                                                                       sin(b) *
                                                                                                       sin(c)) +
                                                                                      piz * (sin(a) * sin(c) +
                                                                                             cos(a) * cos(c) *
                                                                                             sin(b)) +
                                                                                      pix * cos(a) * cos(b)) +
                                                                               niy * (y - qiy + piy * (cos(a) *
                                                                                                       cos(c) +
                                                                                                       sin(a) *
                                                                                                       sin(b) *
                                                                                                       sin(c)) -
                                                                                      piz * (cos(a) * sin(c) -
                                                                                             cos(c) * sin(a) *
                                                                                             sin(b)) +
                                                                                      pix * cos(b) * sin(a)) +
                                                                               niz * (z - qiz - pix * sin(b) +
                                                                                      piz * cos(b) * cos(c) +
                                                                                      piy * cos(b) * sin(c)));


        Eigen::MatrixXd d2J_dX2_temp(6, 6);

        d2J_dX2_temp << d2J_dx2, d2J_dydx, d2J_dzdx, d2J_dadx, d2J_dbdx, d2J_dcdx,
                d2J_dxdy, d2J_dy2, d2J_dzdy, d2J_dady, d2J_dbdy, d2J_dcdy,
                d2J_dxdz, d2J_dydz, d2J_dz2, d2J_dadz, d2J_dbdz, d2J_dcdz,
                d2J_dxda, d2J_dyda, d2J_dzda, d2J_da2, d2J_dbda, d2J_dcda,
                d2J_dxdb, d2J_dydb, d2J_dzdb, d2J_dadb, d2J_db2, d2J_dcdb,
                d2J_dxdc, d2J_dydc, d2J_dzdc, d2J_dadc, d2J_dbdc, d2J_dc2;


        d2J_dX2 = d2J_dX2 + d2J_dX2_temp;

    }// End of the FOR loop!!!

    std::cout << "\n**************\n Successfully Computed d2J_dX2 \n**************\n" << std::endl;

    // Now its time to calculate d2J_dZdX , where Z are the measurements Pi and Qi, X = [x,y,z,a,b,c]

    // n is the number of correspondences
    int n = data_pi.points.size();

    /*  Here we check if the number of correspondences between the source and the target point clouds are greater than 200.
        if yes, we only take the first 200 correspondences to calculate the covariance matrix
        You can try increasing it but if its too high, the system may run out of memory and give an exception saying
        terminate called after throwing an instance of 'std::bad_alloc' what():  std::bad_alloc Aborted (core dumped)
    */

    std::cout<<"n "<<n<<std::endl;

    if (n > 400)
        n = 400;////////////****************************IMPORTANT CHANGE***** but may not affect********************/////////////////////////////////////////

    std::cout << "\nNumber of Correspondences used for ICP's covariance estimation = " << n << std::endl;

    Eigen::MatrixXd d2J_dZdX(6, 6 * n);

    for (int k = 0; k < n; ++k) // row
    {
        //here the current correspondences are loaded into Pi and Qi
        double pix = data_pi.points[k].x;
        double piy = data_pi.points[k].y;
        double piz = data_pi.points[k].z;
        double qix = model_qi.points[k].x;
        double qiy = model_qi.points[k].y;
        double qiz = model_qi.points[k].z;

        double nix = model_qi[k].normal_x;
        double niy = model_qi[k].normal_y;
        double niz = model_qi[k].normal_z;

        if (niz != niz) // for nan removal in input point cloud data
            continue;

        Eigen::MatrixXd d2J_dZdX_temp(6, 6);


        double d2J_dpix_dx, d2J_dpiy_dx, d2J_dpiz_dx, d2J_dqix_dx, d2J_dqiy_dx, d2J_dqiz_dx,
                d2J_dpix_dy, d2J_dpiy_dy, d2J_dpiz_dy, d2J_dqix_dy, d2J_dqiy_dy, d2J_dqiz_dy,
                d2J_dpix_dz, d2J_dpiy_dz, d2J_dpiz_dz, d2J_dqix_dz, d2J_dqiy_dz, d2J_dqiz_dz,
                d2J_dpix_da, d2J_dpiy_da, d2J_dpiz_da, d2J_dqix_da, d2J_dqiy_da, d2J_dqiz_da,
                d2J_dpix_db, d2J_dpiy_db, d2J_dpiz_db, d2J_dqix_db, d2J_dqiy_db, d2J_dqiz_db,
                d2J_dpix_dc, d2J_dpiy_dc, d2J_dpiz_dc, d2J_dqix_dc, d2J_dqiy_dc, d2J_dqiz_dc;


        d2J_dpix_dx =

                2 * nix * (nix * cos(a) * cos(b) - niz * sin(b) + niy * cos(b) * sin(a));


        d2J_dpix_dy =

                2 * niy * (nix * cos(a) * cos(b) - niz * sin(b) + niy * cos(b) * sin(a));


        d2J_dpix_dz =

                2 * niz * (nix * cos(a) * cos(b) - niz * sin(b) + niy * cos(b) * sin(a));


        d2J_dpix_da =

                (2 * niy * cos(a) * cos(b) - 2 * nix * cos(b) * sin(a)) * (nix * (x - qix - piy *
                                                                                            (cos(c) * sin(a) -
                                                                                             cos(a) * sin(b) *
                                                                                             sin(c)) + piz *
                                                                                                       (sin(a) *
                                                                                                        sin(c) +
                                                                                                        cos(a) *
                                                                                                        cos(c) *
                                                                                                        sin(b)) +
                                                                                  pix * cos(a) * cos(b)) + niy *
                                                                                                           (y -
                                                                                                            qiy +
                                                                                                            piy *
                                                                                                            (cos(a) *
                                                                                                             cos(c) +
                                                                                                             sin(a) *
                                                                                                             sin(b) *
                                                                                                             sin(c)) -
                                                                                                            piz *
                                                                                                            (cos(a) *
                                                                                                             sin(c) -
                                                                                                             cos(c) *
                                                                                                             sin(a) *
                                                                                                             sin(b)) +
                                                                                                            pix *
                                                                                                            cos(b) *
                                                                                                            sin(a)) +
                                                                           niz * (z - qiz - pix * sin(b) +
                                                                                  piz * cos(b) * cos(c) +
                                                                                  piy * cos(b) * sin(c))) +
                (2 * niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                            piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                 2 * nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                            piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a))) *
                (nix * cos(a) * cos(b) - niz * sin(b) + niy * cos(b) * sin(a));


        d2J_dpix_db =

                (2 * niy *
                 (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) + piy * cos(b) * sin(a) * sin(c)) -
                 2 * niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + 2 * nix *
                                                                                            (piz * cos(a) *
                                                                                             cos(b) * cos(c) -
                                                                                             pix * cos(a) *
                                                                                             sin(b) +
                                                                                             piy * cos(a) *
                                                                                             cos(b) * sin(c))) *
                (nix * cos(a) * cos(b) - niz * sin(b) + niy * cos(b) * sin(a)) -
                (2 * niz * cos(b) + 2 * nix * cos(a) * sin(b) + 2 * niy * sin(a) * sin(b)) * (nix * (x - qix -
                                                                                                     piy *
                                                                                                     (cos(c) *
                                                                                                      sin(a) -
                                                                                                      cos(a) *
                                                                                                      sin(b) *
                                                                                                      sin(c)) +
                                                                                                     piz *
                                                                                                     (sin(a) *
                                                                                                      sin(c) +
                                                                                                      cos(a) *
                                                                                                      cos(c) *
                                                                                                      sin(b)) +
                                                                                                     pix *
                                                                                                     cos(a) *
                                                                                                     cos(b)) +
                                                                                              niy * (y - qiy +
                                                                                                     piy *
                                                                                                     (cos(a) *
                                                                                                      cos(c) +
                                                                                                      sin(a) *
                                                                                                      sin(b) *
                                                                                                      sin(c)) -
                                                                                                     piz *
                                                                                                     (cos(a) *
                                                                                                      sin(c) -
                                                                                                      cos(c) *
                                                                                                      sin(a) *
                                                                                                      sin(b)) +
                                                                                                     pix *
                                                                                                     cos(b) *
                                                                                                     sin(a)) +
                                                                                              niz * (z - qiz -
                                                                                                     pix *
                                                                                                     sin(b) +
                                                                                                     piz *
                                                                                                     cos(b) *
                                                                                                     cos(c) +
                                                                                                     piy *
                                                                                                     cos(b) *
                                                                                                     sin(c)));


        d2J_dpix_dc =

                (2 * nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                            piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - 2 * niy * (piy *
                                                                                             (cos(a) * sin(c) -
                                                                                              cos(c) * sin(a) *
                                                                                              sin(b)) + piz *
                                                                                                        (cos(a) *
                                                                                                         cos(c) +
                                                                                                         sin(a) *
                                                                                                         sin(b) *
                                                                                                         sin(c))) +
                 2 * niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c))) *
                (nix * cos(a) * cos(b) - niz * sin(b) + niy * cos(b) * sin(a));


        d2J_dpiy_dx =

                2 * nix * (niy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                           nix * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + niz * cos(b) * sin(c));


        d2J_dpiy_dy =

                2 * niy * (niy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                           nix * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + niz * cos(b) * sin(c));


        d2J_dpiy_dz =

                2 * niz * (niy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                           nix * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + niz * cos(b) * sin(c));


        d2J_dpiy_da =

                (2 * niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                            piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                 2 * nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                            piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a))) *
                (niy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                 nix * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + niz * cos(b) * sin(c)) -
                (2 * nix * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) +
                 2 * niy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) * (nix * (x - qix - piy *
                                                                                             (cos(c) * sin(a) -
                                                                                              cos(a) * sin(b) *
                                                                                              sin(c)) + piz *
                                                                                                        (sin(a) *
                                                                                                         sin(c) +
                                                                                                         cos(a) *
                                                                                                         cos(c) *
                                                                                                         sin(b)) +
                                                                                   pix * cos(a) * cos(b)) +
                                                                            niy * (y - qiy + piy *
                                                                                             (cos(a) * cos(c) +
                                                                                              sin(a) * sin(b) *
                                                                                              sin(c)) - piz *
                                                                                                        (cos(a) *
                                                                                                         sin(c) -
                                                                                                         cos(c) *
                                                                                                         sin(a) *
                                                                                                         sin(b)) +
                                                                                   pix * cos(b) * sin(a)) +
                                                                            niz * (z - qiz - pix * sin(b) +
                                                                                   piz * cos(b) * cos(c) +
                                                                                   piy * cos(b) * sin(c)));


        d2J_dpiy_db =

                (2 * niy *
                 (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) + piy * cos(b) * sin(a) * sin(c)) -
                 2 * niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + 2 * nix *
                                                                                            (piz * cos(a) *
                                                                                             cos(b) * cos(c) -
                                                                                             pix * cos(a) *
                                                                                             sin(b) +
                                                                                             piy * cos(a) *
                                                                                             cos(b) * sin(c))) *
                (niy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                 nix * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + niz * cos(b) * sin(c)) +
                (2 * nix * cos(a) * cos(b) * sin(c) - 2 * niz * sin(b) * sin(c) +
                 2 * niy * cos(b) * sin(a) * sin(c)) * (nix * (x - qix - piy * (cos(c) * sin(a) -
                                                                                cos(a) * sin(b) * sin(c)) +
                                                               piz *
                                                               (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                                                               pix * cos(a) * cos(b)) + niy * (y - qiy + piy *
                                                                                                         (cos(a) *
                                                                                                          cos(c) +
                                                                                                          sin(a) *
                                                                                                          sin(b) *
                                                                                                          sin(c)) -
                                                                                               piz * (cos(a) *
                                                                                                      sin(c) -
                                                                                                      cos(c) *
                                                                                                      sin(a) *
                                                                                                      sin(b)) +
                                                                                               pix * cos(b) *
                                                                                               sin(a)) + niz *
                                                                                                         (z -
                                                                                                          qiz -
                                                                                                          pix *
                                                                                                          sin(b) +
                                                                                                          piz *
                                                                                                          cos(b) *
                                                                                                          cos(c) +
                                                                                                          piy *
                                                                                                          cos(b) *
                                                                                                          sin(c)));


        d2J_dpiy_dc =

                (2 * nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                            piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - 2 * niy * (piy *
                                                                                             (cos(a) * sin(c) -
                                                                                              cos(c) * sin(a) *
                                                                                              sin(b)) + piz *
                                                                                                        (cos(a) *
                                                                                                         cos(c) +
                                                                                                         sin(a) *
                                                                                                         sin(b) *
                                                                                                         sin(c))) +
                 2 * niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c))) *
                (niy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                 nix * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + niz * cos(b) * sin(c)) +
                (2 * nix * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                 2 * niy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + 2 * niz * cos(b) * cos(c)) * (nix *
                                                                                                        (x -
                                                                                                         qix -
                                                                                                         piy *
                                                                                                         (cos(c) *
                                                                                                          sin(a) -
                                                                                                          cos(a) *
                                                                                                          sin(b) *
                                                                                                          sin(c)) +
                                                                                                         piz *
                                                                                                         (sin(a) *
                                                                                                          sin(c) +
                                                                                                          cos(a) *
                                                                                                          cos(c) *
                                                                                                          sin(b)) +
                                                                                                         pix *
                                                                                                         cos(a) *
                                                                                                         cos(b)) +
                                                                                                        niy *
                                                                                                        (y -
                                                                                                         qiy +
                                                                                                         piy *
                                                                                                         (cos(a) *
                                                                                                          cos(c) +
                                                                                                          sin(a) *
                                                                                                          sin(b) *
                                                                                                          sin(c)) -
                                                                                                         piz *
                                                                                                         (cos(a) *
                                                                                                          sin(c) -
                                                                                                          cos(c) *
                                                                                                          sin(a) *
                                                                                                          sin(b)) +
                                                                                                         pix *
                                                                                                         cos(b) *
                                                                                                         sin(a)) +
                                                                                                        niz *
                                                                                                        (z -
                                                                                                         qiz -
                                                                                                         pix *
                                                                                                         sin(b) +
                                                                                                         piz *
                                                                                                         cos(b) *
                                                                                                         cos(c) +
                                                                                                         piy *
                                                                                                         cos(b) *
                                                                                                         sin(c)));


        d2J_dpiz_dx =

                2 * nix * (nix * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                           niy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + niz * cos(b) * cos(c));


        d2J_dpiz_dy =

                2 * niy * (nix * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                           niy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + niz * cos(b) * cos(c));


        d2J_dpiz_dz =

                2 * niz * (nix * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                           niy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + niz * cos(b) * cos(c));


        d2J_dpiz_da =

                (2 * niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                            piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                 2 * nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                            piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a))) *
                (nix * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                 niy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + niz * cos(b) * cos(c)) +
                (2 * nix * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
                 2 * niy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b))) * (nix * (x - qix - piy *
                                                                                             (cos(c) * sin(a) -
                                                                                              cos(a) * sin(b) *
                                                                                              sin(c)) + piz *
                                                                                                        (sin(a) *
                                                                                                         sin(c) +
                                                                                                         cos(a) *
                                                                                                         cos(c) *
                                                                                                         sin(b)) +
                                                                                   pix * cos(a) * cos(b)) +
                                                                            niy * (y - qiy + piy *
                                                                                             (cos(a) * cos(c) +
                                                                                              sin(a) * sin(b) *
                                                                                              sin(c)) - piz *
                                                                                                        (cos(a) *
                                                                                                         sin(c) -
                                                                                                         cos(c) *
                                                                                                         sin(a) *
                                                                                                         sin(b)) +
                                                                                   pix * cos(b) * sin(a)) +
                                                                            niz * (z - qiz - pix * sin(b) +
                                                                                   piz * cos(b) * cos(c) +
                                                                                   piy * cos(b) * sin(c)));


        d2J_dpiz_db =

                (2 * niy *
                 (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) + piy * cos(b) * sin(a) * sin(c)) -
                 2 * niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + 2 * nix *
                                                                                            (piz * cos(a) *
                                                                                             cos(b) * cos(c) -
                                                                                             pix * cos(a) *
                                                                                             sin(b) +
                                                                                             piy * cos(a) *
                                                                                             cos(b) * sin(c))) *
                (nix * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                 niy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + niz * cos(b) * cos(c)) +
                (2 * nix * cos(a) * cos(b) * cos(c) - 2 * niz * cos(c) * sin(b) +
                 2 * niy * cos(b) * cos(c) * sin(a)) * (nix * (x - qix - piy * (cos(c) * sin(a) -
                                                                                cos(a) * sin(b) * sin(c)) +
                                                               piz *
                                                               (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                                                               pix * cos(a) * cos(b)) + niy * (y - qiy + piy *
                                                                                                         (cos(a) *
                                                                                                          cos(c) +
                                                                                                          sin(a) *
                                                                                                          sin(b) *
                                                                                                          sin(c)) -
                                                                                               piz * (cos(a) *
                                                                                                      sin(c) -
                                                                                                      cos(c) *
                                                                                                      sin(a) *
                                                                                                      sin(b)) +
                                                                                               pix * cos(b) *
                                                                                               sin(a)) + niz *
                                                                                                         (z -
                                                                                                          qiz -
                                                                                                          pix *
                                                                                                          sin(b) +
                                                                                                          piz *
                                                                                                          cos(b) *
                                                                                                          cos(c) +
                                                                                                          piy *
                                                                                                          cos(b) *
                                                                                                          sin(c)));


        d2J_dpiz_dc =

                (2 * nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                            piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - 2 * niy * (piy *
                                                                                             (cos(a) * sin(c) -
                                                                                              cos(c) * sin(a) *
                                                                                              sin(b)) + piz *
                                                                                                        (cos(a) *
                                                                                                         cos(c) +
                                                                                                         sin(a) *
                                                                                                         sin(b) *
                                                                                                         sin(c))) +
                 2 * niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c))) *
                (nix * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                 niy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + niz * cos(b) * cos(c)) -
                (2 * niy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                 2 * nix * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + 2 * niz * cos(b) * sin(c)) * (nix *
                                                                                                        (x -
                                                                                                         qix -
                                                                                                         piy *
                                                                                                         (cos(c) *
                                                                                                          sin(a) -
                                                                                                          cos(a) *
                                                                                                          sin(b) *
                                                                                                          sin(c)) +
                                                                                                         piz *
                                                                                                         (sin(a) *
                                                                                                          sin(c) +
                                                                                                          cos(a) *
                                                                                                          cos(c) *
                                                                                                          sin(b)) +
                                                                                                         pix *
                                                                                                         cos(a) *
                                                                                                         cos(b)) +
                                                                                                        niy *
                                                                                                        (y -
                                                                                                         qiy +
                                                                                                         piy *
                                                                                                         (cos(a) *
                                                                                                          cos(c) +
                                                                                                          sin(a) *
                                                                                                          sin(b) *
                                                                                                          sin(c)) -
                                                                                                         piz *
                                                                                                         (cos(a) *
                                                                                                          sin(c) -
                                                                                                          cos(c) *
                                                                                                          sin(a) *
                                                                                                          sin(b)) +
                                                                                                         pix *
                                                                                                         cos(b) *
                                                                                                         sin(a)) +
                                                                                                        niz *
                                                                                                        (z -
                                                                                                         qiz -
                                                                                                         pix *
                                                                                                         sin(b) +
                                                                                                         piz *
                                                                                                         cos(b) *
                                                                                                         cos(c) +
                                                                                                         piy *
                                                                                                         cos(b) *
                                                                                                         sin(c)));


        d2J_dqix_dx =

                -2 * pow(nix, 2);


        d2J_dqix_dy =

                -2 * nix * niy;


        d2J_dqix_dz =

                -2 * nix * niz;


        d2J_dqix_da =

                -nix * (2 * niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                                   piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                        2 * nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                                   piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a)));


        d2J_dqix_db =

                -nix * (2 * niy * (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) +
                                   piy * cos(b) * sin(a) * sin(c)) -
                        2 * niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + 2 * nix *
                                                                                                   (piz *
                                                                                                    cos(a) *
                                                                                                    cos(b) *
                                                                                                    cos(c) -
                                                                                                    pix *
                                                                                                    cos(a) *
                                                                                                    sin(b) +
                                                                                                    piy *
                                                                                                    cos(a) *
                                                                                                    cos(b) *
                                                                                                    sin(c)));


        d2J_dqix_dc =

                -nix * (2 * nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                                   piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - 2 * niy * (piy *
                                                                                                    (cos(a) *
                                                                                                     sin(c) -
                                                                                                     cos(c) *
                                                                                                     sin(a) *
                                                                                                     sin(b)) +
                                                                                                    piz *
                                                                                                    (cos(a) *
                                                                                                     cos(c) +
                                                                                                     sin(a) *
                                                                                                     sin(b) *
                                                                                                     sin(c))) +
                        2 * niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c)));


        d2J_dqiy_dx =

                -2 * nix * niy;


        d2J_dqiy_dy =

                -2 * pow(niy, 2);


        d2J_dqiy_dz =

                -2 * niy * niz;


        d2J_dqiy_da =

                -niy * (2 * niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                                   piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                        2 * nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                                   piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a)));


        d2J_dqiy_db =

                -niy * (2 * niy * (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) +
                                   piy * cos(b) * sin(a) * sin(c)) -
                        2 * niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + 2 * nix *
                                                                                                   (piz *
                                                                                                    cos(a) *
                                                                                                    cos(b) *
                                                                                                    cos(c) -
                                                                                                    pix *
                                                                                                    cos(a) *
                                                                                                    sin(b) +
                                                                                                    piy *
                                                                                                    cos(a) *
                                                                                                    cos(b) *
                                                                                                    sin(c)));


        d2J_dqiy_dc =

                -niy * (2 * nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                                   piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - 2 * niy * (piy *
                                                                                                    (cos(a) *
                                                                                                     sin(c) -
                                                                                                     cos(c) *
                                                                                                     sin(a) *
                                                                                                     sin(b)) +
                                                                                                    piz *
                                                                                                    (cos(a) *
                                                                                                     cos(c) +
                                                                                                     sin(a) *
                                                                                                     sin(b) *
                                                                                                     sin(c))) +
                        2 * niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c)));


        d2J_dqiz_dx =

                -2 * nix * niz;


        d2J_dqiz_dy =

                -2 * niy * niz;


        d2J_dqiz_dz =

                -2 * pow(niz, 2);


        d2J_dqiz_da =

                -niz * (2 * niy * (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                                   piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) + pix * cos(a) * cos(b)) -
                        2 * nix * (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                                   piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) + pix * cos(b) * sin(a)));


        d2J_dqiz_db =

                -niz * (2 * niy * (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) +
                                   piy * cos(b) * sin(a) * sin(c)) -
                        2 * niz * (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) + 2 * nix *
                                                                                                   (piz *
                                                                                                    cos(a) *
                                                                                                    cos(b) *
                                                                                                    cos(c) -
                                                                                                    pix *
                                                                                                    cos(a) *
                                                                                                    sin(b) +
                                                                                                    piy *
                                                                                                    cos(a) *
                                                                                                    cos(b) *
                                                                                                    sin(c)));


        d2J_dqiz_dc =

                -niz * (2 * nix * (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
                                   piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) - 2 * niy * (piy *
                                                                                                    (cos(a) *
                                                                                                     sin(c) -
                                                                                                     cos(c) *
                                                                                                     sin(a) *
                                                                                                     sin(b)) +
                                                                                                    piz *
                                                                                                    (cos(a) *
                                                                                                     cos(c) +
                                                                                                     sin(a) *
                                                                                                     sin(b) *
                                                                                                     sin(c))) +
                        2 * niz * (piy * cos(b) * cos(c) - piz * cos(b) * sin(c)));


        d2J_dZdX_temp << d2J_dpix_dx, d2J_dpiy_dx, d2J_dpiz_dx, d2J_dqix_dx, d2J_dqiy_dx, d2J_dqiz_dx,
                d2J_dpix_dy, d2J_dpiy_dy, d2J_dpiz_dy, d2J_dqix_dy, d2J_dqiy_dy, d2J_dqiz_dy,
                d2J_dpix_dz, d2J_dpiy_dz, d2J_dpiz_dz, d2J_dqix_dz, d2J_dqiy_dz, d2J_dqiz_dz,
                d2J_dpix_da, d2J_dpiy_da, d2J_dpiz_da, d2J_dqix_da, d2J_dqiy_da, d2J_dqiz_da,
                d2J_dpix_db, d2J_dpiy_db, d2J_dpiz_db, d2J_dqix_db, d2J_dqiy_db, d2J_dqiz_db,
                d2J_dpix_dc, d2J_dpiy_dc, d2J_dpiz_dc, d2J_dqix_dc, d2J_dqiy_dc, d2J_dqiz_dc;


        d2J_dZdX.block<6, 6>(0, 6 * k) = d2J_dZdX_temp;

    }


    //By reaching here both the matrices d2J_dX2 and d2J_dZdX are calculated and lets print those values out;

    //std::cout << "\n Finally here are the two matrices \n\n" << "d2J_dX2 = \n " << d2J_dX2 <<std::endl;
    //std::cout << "\n\n\n" << "d2J_dZdX = \n " << d2J_dZdX <<std::endl;




    /**************************************
*
* Here we create the matrix cov(z) as mentioned in Section 3.3 in the paper, "Covariance of ICP with 3D Point to Point and Point to Plane Error Metrics"
*
* ************************************/
    Eigen::MatrixXd cov_z(6 * n, 6 * n);
    cov_z = 0.01 * Eigen::MatrixXd::Identity(6 * n, 6 * n);


    ICP_COV = d2J_dX2.inverse() * d2J_dZdX * cov_z * d2J_dZdX.transpose() * d2J_dX2.inverse();


    std::cout << "\n\n********************** \n\n" << "ICP_COV = \n" << ICP_COV << "\n*******************\n\n"
              << std::endl;

    std::cout << "\nSuccessfully Computed the ICP's Covariance !!!\n" << std::endl;

}


Eigen::Matrix3d updatePoseCovariance(const Eigen::Vector3d &oldPose, const Eigen::Matrix3d &oldCov,
                                     const Eigen::Matrix<double, 6, 1> &motion, const Eigen::Matrix<double, 6, 6> &motionCov) {

            Eigen::Matrix3d Jp;

            Eigen::Matrix<double, 3, 6> Jm;

            double srx = sin(motion[3]);
            double crx = cos(motion[3]);
            double sry = sin(motion[4]);
            double cry = cos(motion[4]);
            double srz = sin(motion[5]);
            double crz = cos(motion[5]);

            Eigen::Vector3d eulerAngle(motion[3], motion[4], motion[5]);

            Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));

            Eigen::Matrix3d rotation_matrix;
            rotation_matrix = yawAngle * pitchAngle * rollAngle;

//            /* Jacobian wrt previous pose */
//            Jp <<
//               1, 0, -motion[1] * c - motion[0] * s,
//                    0, 1, -motion[1] * s + motion[0] * c,
//                    0, 0, 1;

            /* Jacobian wrt motion delta */
            Jm <<
                    1, 0, 0, (crx * sry * srz * oldPose[0] + crx * crz * sry * oldPose[1] - srx * sry * oldPose[2]), ((cry * srx * srz - crz * sry) * oldPose[0] + (sry * srz + cry * crz * srx) * oldPose[1] +
                    crx * cry * oldPose[2]), ((crz * srx * sry - cry * srz) * oldPose[0] +
                                              (-cry * crz - srx * sry * srz) * oldPose[1]),
                    0, 1, 0, (-srx * srz * oldPose[0] - crz * srx * oldPose[1] - crx * oldPose[2]), 0, (
                    crx * crz * oldPose[0] - crx * srz * oldPose[1]),
                    0, 0, 1, (crx * cry * srz * oldPose[0] + crx * cry * crz * oldPose[1] - cry * srx * oldPose[2]), (
                    (-cry * crz - srx * sry * srz) * oldPose[0] + (cry * srz - crz * srx * sry) * oldPose[1] -
                    crx * sry * oldPose[2]), ((sry * srz + cry * crz * srx) * oldPose[0] +
                                              (crz * sry - cry * srx * srz) * oldPose[1]);

            return rotation_matrix * oldCov * rotation_matrix.transpose() + Jm * motionCov * Jm.transpose();
        }




int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    nh.param<int>("mapping_skip_frame", skipFrameNum, 2);

//    printf("Mapping %d Hz \n", 10 / skipFrameNum);

    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 300, laserCloudSharpHandler);

    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 300, laserCloudLessSharpHandler);

    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 300, laserCloudFlatHandler);

    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 300, laserCloudLessFlatHandler);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 300, laserCloudFullResHandler);

    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 300);

    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 300);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 300);

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 300);

    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 300);

    nav_msgs::Path laserPath;

    int frameCount = 0;
    ros::Rate rate(100);

    oldCov << 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0,
              0.0, 0.0, 0.0;

    while (ros::ok())
    {
        ros::spinOnce();

        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty())
        {
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();

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
            }
            else
            {
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();

                TicToc t_opt;

                pcl::PointCloud<pcl::PointNormal> data_pi;
                pcl::PointCloud<pcl::PointNormal> model_qi;

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

//                    pcl::PointCloud<pcl::PointNormal> data_pi;
                    data_pi.points.clear();
//                    pcl::PointCloud<pcl::PointNormal> model_qi;
                    model_qi.points.clear();

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

                                pcl::PointNormal d_p;
                                d_p.x = curr_point[0]; d_p.y = curr_point[1]; d_p.z = curr_point[2];
                                data_pi.points.push_back(d_p);

                                pcl::PointNormal m_qi;
                                m_qi.x = last_point_a[0];
                                m_qi.y = last_point_a[1];
                                m_qi.z = last_point_a[2];

                                Eigen::Vector3d ljm_norm;
                                ljm_norm = (last_point_a - last_point_b).cross(last_point_a - last_point_c);
                                ljm_norm.normalize();
                                m_qi.normal_x = ljm_norm.x();
                                m_qi.normal_y = ljm_norm.y();
                                m_qi.normal_z = ljm_norm.z();

                                model_qi.points.push_back(m_qi);
                            }
                        }
                    }

                    //printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
//                    printf("data association time %f ms \n", t_data.toc());

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
//                    printf("solver time %f ms \n", t_solver.toc());
                }
//                printf("optimization twice time %f \n", t_opt.toc());


//                Eigen::Matrix3d rotation_matrix;
//                rotation_matrix = q_last_curr.matrix();
//
//                Eigen::Matrix4f transformT;
//                transformT << rotation_matrix(0,0),rotation_matrix(0,1),rotation_matrix(0,2),t_last_curr[0],
//                              rotation_matrix(1,0),rotation_matrix(1,1),rotation_matrix(1,2),t_last_curr[1],
//                              rotation_matrix(2,0),rotation_matrix(2,1),rotation_matrix(2,2),t_last_curr[2],
//                              0                            ,0                    ,0                            ,1;
//                Eigen::MatrixXd ICP_COV;
//
//                calculate_ICP_COV(data_pi,model_qi,transformT,ICP_COV);
//
////                Eigen::Matrix3d oldCov;
////                oldCov << ICP_COV(0,0), ICP_COV(0,1), ICP_COV(0,2),
////                          ICP_COV(1,0), ICP_COV(1,1), ICP_COV(1,2),
////                          ICP_COV(2,0), ICP_COV(2,1), ICP_COV(2,2);
//
//                Eigen::Matrix<double, 6, 1> motion;
//
//                Eigen::Vector3d eulerAngle = q_last_curr.matrix().eulerAngles(2,1,0);
//
////                std::cout<<"============="<<std::endl;
////                std::cout<<eulerAngle(0)<<"  "<<eulerAngle(1)<<"  "<<eulerAngle(2)<<std::endl;
////                std::cout<<"============="<<std::endl;
//
//
//                motion << t_last_curr(0), t_last_curr(1), t_last_curr(2), eulerAngle(0), eulerAngle(1), eulerAngle(2);
//
//                Eigen::Vector3d t_w_curr1(0, 0, 0);
//
//                oldCov = updatePoseCovariance(t_w_curr1, oldCov, motion, ICP_COV);
//
//                double maxaxis = oldCov(0,0);
//
//                if (maxaxis < oldCov(1,1))
//                    maxaxis = oldCov(1,1);
//
//                if (maxaxis < oldCov(2,2))
//                    maxaxis = oldCov(2,2);
//
//                std::cout<<"maxaxis "<<3 * sqrt(maxaxis)<<std::endl;

                t_w_curr = t_w_curr + q_w_curr * t_last_curr;
                q_w_curr = q_w_curr * q_last_curr;

            }

            TicToc t_pub;

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "/camera_init";
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
            laserPath.header.frame_id = "/camera_init";
            pubLaserPath.publish(laserPath);

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

            if (frameCount % skipFrameNum == 0)
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
//            printf("publication time %f ms \n", t_pub.toc());
//            printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
            if(t_whole.toc() > 100)
                ROS_WARN("odometry process over 100ms");

            frameCount++;
        }
        rate.sleep();
    }
    return 0;
}