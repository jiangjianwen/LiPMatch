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



#	define	TIMEVAL_NUMS			reinterpret_cast<struct timeval*>(largeInts)

namespace Makemap_ns {
    
    class CTicTac{
        public:
        CTicTac()
        {
            ::memset( largeInts, 0, sizeof(largeInts) );
            static_assert( sizeof( largeInts ) > 2*sizeof(struct timeval), "sizeof(struct timeval) failed!");
            Tic();
        }

        void Tic()
        {
            struct timeval* ts = TIMEVAL_NUMS;
            gettimeofday( &ts[0], NULL);
        }

        double Tac()
        {
            struct timeval* ts = TIMEVAL_NUMS;
            gettimeofday( &ts[1], NULL);
            return ( ts[1].tv_sec - ts[0].tv_sec) + 1e-6*(  ts[1].tv_usec - ts[0].tv_usec );
        }
        private:
        unsigned long long largeInts[8];
    };

    static void sleep( int time_ms )
    {
        CTicTac tictac;
        tictac.Tic();
        int timeLeft_ms = time_ms - (int)(tictac.Tac()*1000);
        while ( timeLeft_ms>0 )
        {
            usleep( timeLeft_ms * 1000 );
            timeLeft_ms = time_ms - (int)(tictac.Tac()*1000);
        }
    }

    struct TThreadHandle
    {
        std::shared_ptr<std::thread> m_thread;

        TThreadHandle() : m_thread(std::make_shared<std::thread>()) {}
        ~TThreadHandle() { clear(); }

        /** Mark the handle as invalid.
          * \sa isClear
          */
        void clear()
        {
            if (m_thread && m_thread->joinable())
                m_thread->detach();
            m_thread = std::make_shared<std::thread>();
        }
        /** Returns true if the handle is uninitialized */
        bool isClear() const { return !m_thread || !m_thread->joinable(); }
    };

    //! \overload
    template <typename CLASS>
    TThreadHandle createThreadFromObjectMethod(CLASS *obj, void (CLASS::*func)(void))	{
        TThreadHandle h;
        h.m_thread = std::make_shared<std::thread>(func, obj);
        return h;
    }

    static void joinThread( TThreadHandle &threadHandle )
    {
        if (threadHandle.m_thread && threadHandle.m_thread->joinable())
            threadHandle.m_thread->join();
    }

    class m_keyframe
    {
    public:
        m_keyframe() : structurelaserCloud(new pcl::PointCloud<pcl::PointXYZI>()), vehiclelaserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
                       naturelaserCloud(new pcl::PointCloud<pcl::PointXYZI>()), objectlaserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
                       orilaserCloud(new pcl::PointCloud<pcl::PointXYZI>()), surflaserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
                       linelaserCloud(new pcl::PointCloud<pcl::PointXYZI>()), g_laserCloud(new pcl::PointCloud<pcl::PointXYZI>())
        {
            framecount = 0;
            m_ending_frame_idx = 0;
            travel_length = 0.0;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr structurelaserCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr vehiclelaserCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr naturelaserCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr objectlaserCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr orilaserCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr surflaserCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr linelaserCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr g_laserCloud;

        int framecount = 0;
        int m_ending_frame_idx = 0;
        double travel_length = 0.0;
    };



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

    std::vector<m_keyframe> frameQueue;

    pcl::PointCloud<pcl::PointXYZI> laserCloudOri_m2;
    pcl::PointCloud<pcl::PointXYZI> laserCloudOri_mp1;

    pcl::PointCloud<pcl::PointXYZI> allPlanesBef;
    pcl::PointCloud<pcl::PointXYZI> allVehiclesBef;
    pcl::PointCloud<pcl::PointXYZI> allCylindersBef;





  private:





      void detectPlanesCloud( m_keyframe &c_keyframe, int keyFrameCount);




    TThreadHandle Makemap_hd;

    protected:

    std::set<unsigned> observedPlanes;

    std::set<unsigned> observedVehicles;

    std::set<unsigned> observedPoles;


    };

 } // End of namespaces


#endif
