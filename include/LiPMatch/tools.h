#pragma once


#include <iostream>
#include <thread>
#include <set>
#include <fstream>
#include <string>
#include <sys/time.h>


#define TIMEVAL_NUMS reinterpret_cast<struct timeval*>(largeInts)

namespace tools{
    
    static double parameters1[7] = {0, 0, 0, 1, 0, 0, 0};

    class m_keyframe
    {
    public:
        m_keyframe() : structurelaserCloud(new pcl::PointCloud<pcl::PointXYZI>()), vehiclelaserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
                       naturelaserCloud(new pcl::PointCloud<pcl::PointXYZI>()), objectlaserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
                       orilaserCloud(new pcl::PointCloud<pcl::PointXYZI>()), surflaserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
                       linelaserCloud(new pcl::PointCloud<pcl::PointXYZI>()), g_laserCloud(new pcl::PointCloud<pcl::PointXYZI>())
//                       same_laserCloud(new pcl::PointCloud<pcl::PointXYZI>())
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

//        pcl::PointCloud<pcl::PointXYZI>::Ptr same_laserCloud;

        int framecount = 0;
        int m_ending_frame_idx = 0;
        double travel_length = 0.0;


        Eigen::Map<Eigen::Quaterniond> m_pose_q = Eigen::Map<Eigen::Quaterniond>( parameters1 );
        Eigen::Map<Eigen::Vector3d>    m_pose_t = Eigen::Map<Eigen::Vector3d>( parameters1 + 4 );

    };




    
    class CTicTac{
        public:
        CTicTac(){
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
    
    inline void sleep( int time_ms )
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
        
        void clear()
        { 
            if (m_thread && m_thread->joinable())
            m_thread->detach();
            m_thread = std::make_shared<std::thread>();
        }
        bool isClear() const { return !m_thread || !m_thread->joinable(); }
    };
    
    template <typename CLASS>
    inline TThreadHandle createThreadFromObjectMethod(CLASS *obj, void (CLASS::*func)(void)){
        TThreadHandle h;
        h.m_thread = std::make_shared<std::thread>(func, obj);
        return h;
    }
    
    inline void joinThread( TThreadHandle &threadHandle )
    {
        if (threadHandle.m_thread && threadHandle.m_thread->joinable())
        threadHandle.m_thread->join();
    }

}
