#ifndef __MAKEMAP_H
#define __MAKEMAP_H

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <scene_alignment.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/time.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>

#include <iostream>
#include <ceres/ceres.h>
#include <Vehicle.h>
#include <Pole.h>
#include <Plane.h>
#include <thread>
#include <set>
#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <fstream>
#include <Vehicle.h>
#include <Pole.h>
#include <thread>
#include <string>
#include <sys/time.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <tools.h>


namespace Makemap_ns {
    


  class Makemap
  {
   public:









    Makemap();


        void transformationfromMatches(std::vector<Eigen::Vector3d> m1, std::vector<Eigen::Vector3d> m2, Eigen::Matrix<float, 4, 4> *transform, Eigen::Vector3d & trans);


          ~Makemap();


   void genPlaneMap();

    void genCylinderMap();

    void genVehicleMap();

    void run();

    bool Makemap_stop;

    bool Makemap_finished;

    bool stop_Makemap();

    std::vector<tools::m_keyframe> frameQueue;

    pcl::PointCloud<pcl::PointXYZI> laserCloudOri_m2;
    pcl::PointCloud<pcl::PointXYZI> laserCloudOri_mp1;

    pcl::PointCloud<pcl::PointXYZI> allPlanesBef;
    pcl::PointCloud<pcl::PointXYZI> allVehiclesBef;
    pcl::PointCloud<pcl::PointXYZI> allCylindersBef;





  private:





      void detectPlanesCloud( tools::m_keyframe &c_keyframe, int keyFrameCount);




    tools::TThreadHandle Makemap_hd;

    protected:

    std::set<unsigned> observedPlanes;

    std::set<unsigned> observedVehicles;

    std::set<unsigned> observedPoles;


    };

 } // End of namespaces


#endif
