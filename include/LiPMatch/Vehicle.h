//
// Created by jjwen on 2020/2/28.
//

#ifndef MRPT_VEHICLE_H
#define MRPT_VEHICLE_H

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <set>
#include <map>


namespace LiPMatch_ns {

        class Vehicle
        {
        public:
            Vehicle() : matched(false), VehiclePointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>)
            {}

            void calcCenterAndElongation()
            {
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*VehiclePointCloudPtr, centroid);
                v3center(0) = centroid(0); v3center(1) = centroid(1);
//                v3center(2) = 0.0;
                v3center(2) = centroid(2);
                pcl::PCA< pcl::PointXYZI > pca;
                pca.setInputCloud(VehiclePointCloudPtr);
                eigenVal = pca.getEigenValues();
            }

            unsigned id;
            unsigned keyFrameId;
            std::map<unsigned,unsigned> neighborVehicles;

            Eigen::Vector3f v3center;
            Eigen::Vector3f eigenVal;
            bool matched;

            pcl::PointCloud<pcl::PointXYZI>::Ptr VehiclePointCloudPtr;
        };
     } // End of namespaces











#endif //MRPT_VEHICLE_H
