//
// Created by jjwen on 2020/3/2.
//

#ifndef LiPMatch_POLE_H
#define LiPMatch_POLE_H


#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <set>
#include <map>


namespace LiPMatch_ns {

        class Pole
        {
        public:
            Pole() : PolePointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>)
            {}

            void calcCenterAndElongation()
            {
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*PolePointCloudPtr, centroid);
                v3center(0) = centroid(0); v3center(1) = centroid(1); v3center(2) = centroid(2);
                pcl::PCA< pcl::PointXYZI > pca;
                pca.setInputCloud(PolePointCloudPtr);
                eigenVal = pca.getEigenValues();
                elongation = sqrt(eigenVal[0] / eigenVal[1]);
            }

            unsigned id;
            unsigned keyFrameId;

            float elongation;

            Eigen::Vector3f v3center;
            Eigen::Vector3f eigenVal;

            pcl::PointCloud<pcl::PointXYZI>::Ptr PolePointCloudPtr;
        };
     } // End of namespaces



#endif 
