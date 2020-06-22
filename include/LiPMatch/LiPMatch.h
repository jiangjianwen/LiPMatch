#ifndef __LiPMatch_H
#define __LiPMatch_H

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <scene_alignment.h>
#include <SubgraphMatcher.h>
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
#include <pcl/recognition/cg/geometric_consistency.h> //几何一致性
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>

#include <tools.h>

namespace LiPMatch_ns
{

    template <typename DATA_TYPE>
    class Maps_keyframe
    {
    public:
        double m_pose_buffer[7] = {0, 0, 0, 1, 0, 0, 0};

        Eigen::Map<Eigen::Quaterniond> m_pose_q = Eigen::Map<Eigen::Quaterniond>(m_pose_buffer);
        Eigen::Map<Eigen::Vector3d> m_pose_t = Eigen::Map<Eigen::Vector3d>(m_pose_buffer + 4);

        pcl::PointCloud<pcl::PointXYZI> m_accumulated_point_cloud; // The pointcloud sampled from current frame to last frame

        pcl::PointCloud<pcl::PointXYZI> m_accumulated_surf_pc; // The pointcloud sampled from current frame to last frame
        pcl::PointCloud<pcl::PointXYZI> m_accumulated_line_pc; // The pointcloud sampled from current frame to last frame

        pcl::PointCloud<pcl::PointXYZI> m_accumulated_ng_pc;

        pcl::PointCloud<pcl::PointXYZI> m_accumulated_g_pc;
        pcl::PointCloud<pcl::PointXYZI> m_accumulated_structure_pc;
        pcl::PointCloud<pcl::PointXYZI> m_accumulated_vehicle_pc;

        unsigned int m_accumulate_frames = 0;
        int m_ending_frame_idx;
        int m_keyframe_idx;

        std::vector<Plane> vPlanes;

        std::vector<Vehicle> vVehicles;

        std::vector<Pole> vPoles;

        Maps_keyframe()
        {
            m_keyframe_idx = 0;
            vPlanes.clear();
            vPoles.clear();
        };

        ~Maps_keyframe(){};
    };

    class LiPMatch
    {
    public:
        struct Pose3d
        {
            Eigen::Matrix<double, 3, 1> p;
            Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

            // The name of the data type in the g2o file format.
            static std::string name()
            {
                return "VERTEX_SE3:QUAT";
            }

            Pose3d() = default;

            Pose3d(Eigen::Quaterniond &in_q, Eigen::Vector3d &in_t) : p(in_t), q(in_q){};

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };

        struct Constraint3d
        {
            int id_begin;
            int id_end;

            // The transformation that represents the pose of the end frame E w.r.t. the
            // begin frame B. In other words, it transforms a vector in the E frame to
            // the B frame.
            Pose3d t_be;

            // The inverse of the covariance matrix for the measurement. The order of the
            // entries are x, y, z, delta orientation.
            Eigen::Matrix<double, 6, 6> information;

            Constraint3d()
            {
                information.setIdentity();
            };

            Constraint3d(int id_begin, int id_end, Eigen::Quaterniond &q, Eigen::Vector3d &p) : id_begin(id_begin),
                                                                                                id_end(id_end),
                                                                                                t_be(q, p)
            {
                information.setIdentity();
            };

            // The name of the data type in the g2o file format.
            static std::string name()
            {
                return "EDGE_SE3:QUAT";
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        typedef std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Pose3d>>> MapOfPoses;
        typedef std::vector<Constraint3d, Eigen::aligned_allocator<Constraint3d>> VectorOfConstraints;
        typedef std::vector<Pose3d, Eigen::aligned_allocator<Pose3d>> VectorOfPose;

        class Mapping_refine
        {
        public:
            pcl::VoxelGrid<pcl::PointXYZI> m_down_sample_filter;

            MapOfPoses pose3d_map_oir, pose3d_map_opm;
            pcl::PointCloud<pcl::PointXYZI> m_pts_aft_refind;
            float m_down_sample_res = 0.2;
            int m_step_skip = 2;
            Mapping_refine()
            {
                set_down_sample_resolution(0.2);
            };

            void set_down_sample_resolution(float res)
            {
                m_down_sample_res = res;
                m_down_sample_filter.setLeafSize(res, res, res);
                //m_pts_aft_refind.reserve(1e8);
            }

            template <typename T, typename PointType>
            vector<Eigen::Matrix<T, 3, 1>> pcl_pts_to_eigen_pts(const typename pcl::PointCloud<PointType>::Ptr input_cloud)
            {
                vector<Eigen::Matrix<T, 3, 1>> out_res;
                size_t pc_size = input_cloud->size();
                out_res.resize(pc_size);
                for (size_t i = 0; i < pc_size; i++)
                {
                    //out_res[ i ] << input_cloud->points[ i ]x, input_cloud->points[ i ].y, input_cloud->points[ i ].z;
                    out_res[i] << input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z;
                }
                return out_res;
            }

            template <typename TT, typename T>
            TT eigen_to_pcl_pt(const T &pt)
            {
                TT res_pt;
                res_pt.x = pt(0);
                res_pt.y = pt(1);
                res_pt.z = pt(2);
                return res_pt;
            }

            template <typename PointType, typename T>
            pcl::PointCloud<PointType> eigen_pt_to_pcl_pointcloud(const vector<T> &eigen_pt_vec)
            {
                pcl::PointCloud<PointType> pcl_pc_vec;
                pcl_pc_vec.resize(eigen_pt_vec.size());
                for (size_t i = 0; i < eigen_pt_vec.size(); i++)
                {
                    pcl_pc_vec[i] = eigen_to_pcl_pt<PointType>(eigen_pt_vec[i]);
                }
                return pcl_pc_vec;
            }

            template <typename T, typename TT>
            static std::vector<Eigen::Matrix<T, 3, 1>> refine_pts(std::vector<Eigen::Matrix<T, 3, 1>> &raw_pt_vec,
                                                                  const Eigen::Matrix<TT, 3, 3> &R_mat_ori, Eigen::Matrix<TT, 3, 1> &t_vec_ori,
                                                                  const Eigen::Matrix<TT, 3, 3> &R_mat_opm, Eigen::Matrix<TT, 3, 1> &t_vec_opm)
            {
                std::vector<Eigen::Matrix<float, 3, 1>> opm_pt_vec;
                opm_pt_vec.resize(raw_pt_vec.size());
                Eigen::Matrix<T, 3, 3> R_aff = (R_mat_opm * R_mat_ori.transpose()).template cast<T>();
                Eigen::Matrix<T, 3, 1> T_aff = (R_aff.template cast<TT>() * (-t_vec_ori) + t_vec_opm).template cast<T>();

                for (size_t i = 0; i < raw_pt_vec.size(); i++)
                {
                    opm_pt_vec[i] = R_aff * raw_pt_vec[i] + T_aff;
                }
                return opm_pt_vec;
            }

            pcl::PointCloud<pcl::PointXYZI> refine_pointcloud(std::map<int, pcl::PointCloud<pcl::PointXYZI>> &map_idx_pc,
                                                              MapOfPoses &pose3d_map_oir,
                                                              MapOfPoses &pose3d_map_opm,
                                                              int idx = 0)
            {
                assert(map_idx_pc.size() == pose3d_map_oir.size());
                assert(map_idx_pc.size() == pose3d_map_opm.size());
                pcl::PointCloud<pcl::PointXYZI> pcl_pts;
                auto it = std::next(map_idx_pc.begin(), idx);
                if (it == map_idx_pc.end())
                {
                    return pcl_pts;
                }
                else
                {
                    int id = it->first;
                    std::vector<Eigen::Matrix<float, 3, 1>> pts_vec_ori, pts_vec_opm;
                    Pose3d pose_ori = pose3d_map_oir.find(id)->second;
                    Pose3d pose_opm = pose3d_map_opm.find(id)->second;
                    auto pc_in = map_idx_pc.find(id)->second.makeShared();

                    //                      m_down_sample_filter.setInputCloud( pc_in );
                    //                      m_down_sample_filter.filter( *pc_in );

                    pts_vec_ori = pcl_pts_to_eigen_pts<float, pcl::PointXYZI>(pc_in);

                    pts_vec_opm = refine_pts(pts_vec_ori, pose_ori.q.toRotationMatrix(), pose_ori.p,
                                             pose_opm.q.toRotationMatrix(), pose_opm.p);
                    pcl_pts = eigen_pt_to_pcl_pointcloud<pcl::PointXYZI>(pts_vec_opm);

                    return pcl_pts;
                    //m_down_sample_filter.setInputCloud( m_pts_aft_refind.makeShared() );
                    //m_down_sample_filter.filter( m_pts_aft_refind );
                }
            }

            void refine_mapping(std::map<int, pcl::PointCloud<pcl::PointXYZI>> &map_idx_pc,
                                MapOfPoses &pose3d_map_oir,
                                MapOfPoses &pose3d_map_opm)
            {
                assert(map_idx_pc.size() == pose3d_map_oir.size());
                assert(map_idx_pc.size() == pose3d_map_opm.size());

                std::vector<Eigen::Matrix<float, 3, 1>> pts_vec_ori, pts_vec_opm;
                for (auto it = pose3d_map_oir.begin(); it != pose3d_map_oir.end(); it++)
                {
                    int id = it->first;
                    Pose3d pose_ori = pose3d_map_oir.find(id)->second;
                    Pose3d pose_opm = pose3d_map_opm.find(id)->second;

                    pts_vec_ori = pcl_pts_to_eigen_pts<float, pcl::PointXYZI>(map_idx_pc.find(id)->second.makeShared());
                    pts_vec_opm = refine_pts(pts_vec_ori, pose_ori.q.toRotationMatrix(), pose_ori.p,
                                             pose_opm.q.toRotationMatrix(), pose_opm.p);
                    pcl::PointCloud<pcl::PointXYZI> pcl_pts = eigen_pt_to_pcl_pointcloud<pcl::PointXYZI>(pts_vec_opm);

                    //                  m_down_sample_filter.setInputCloud( pcl_pts.makeShared() );
                    //                  m_down_sample_filter.filter( pcl_pts );
                    m_pts_aft_refind += pcl_pts;
                    //m_down_sample_filter.setInputCloud( m_pts_aft_refind.makeShared() );
                    //m_down_sample_filter.filter( m_pts_aft_refind );
                    std::next(it, m_step_skip);
                }
            }
        };

        class PoseGraph3dErrorTerm
        {

        public:
            PoseGraph3dErrorTerm(const Pose3d &t_ab_measured,
                                 const Eigen::Matrix<double, 6, 6> &sqrt_information)
                : t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}

            template <typename T>
            bool operator()(const T *const p_a_ptr, const T *const q_a_ptr,
                            const T *const p_b_ptr, const T *const q_b_ptr,
                            T *residuals_ptr) const
            {
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
                Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

                Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
                Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

                // Compute the relative transformation between the two frames.
                Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
                Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

                // Represent the displacement between the two frames in the A frame.
                Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

                // Compute the error between the two orientation estimates.
                Eigen::Quaternion<T> delta_q =
                    t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();

                // Compute the residuals.
                // [ position         ]   [ delta_p          ]
                // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
                Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
                residuals.template block<3, 1>(0, 0) =
                    p_ab_estimated - t_ab_measured_.p.template cast<T>();
                residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

                // Scale the residuals by the measurement uncertainty.
                residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

                return true;
            }

            static ceres::CostFunction *Create(
                const Pose3d &t_ab_measured,
                const Eigen::Matrix<double, 6, 6> &sqrt_information)
            {
                return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(
                    new PoseGraph3dErrorTerm(t_ab_measured, sqrt_information));
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        private:
            // The measurement for the position of B relative to A in the A frame.
            const Pose3d t_ab_measured_;
            // The square root of the measurement information matrix.
            const Eigen::Matrix<double, 6, 6> sqrt_information_;
        };

        bool OutputPoses(const std::string &filename, const MapOfPoses &poses)
        {

            ofstream outfile(filename.c_str(), ios::trunc);

            if (!outfile)
            {
                LOG(ERROR) << "Error opening the file: " << filename;
                return false;
            }
            for (std::map<int, Pose3d, std::less<int>,
                          Eigen::aligned_allocator<std::pair<const int, Pose3d>>>::
                     const_iterator poses_iter = poses.begin();
                 poses_iter != poses.end(); ++poses_iter)
            {

                const std::map<int, Pose3d, std::less<int>,
                               Eigen::aligned_allocator<std::pair<const int, Pose3d>>>::
                    value_type &pair = *poses_iter;
                outfile << pair.first << " " << pair.second.p.transpose() << " "
                        << pair.second.q.x() << " " << pair.second.q.y() << " "
                        << pair.second.q.z() << " " << pair.second.q.w() << '\n';

                std::cout << pair.first << " " << pair.second.p.transpose() << " "
                          << pair.second.q.x() << " " << pair.second.q.y() << " "
                          << pair.second.q.z() << " " << pair.second.q.w() << '\n';
            }

            outfile.close();

            return true;
        }

        void BuildOptimizationProblem(const VectorOfConstraints &constraints,
                                      MapOfPoses *poses, ceres::Problem *problem)
        {

            if (constraints.empty())
            {
                std::cout << "No constraints, no problem to optimize." << std::endl;
                return;
            }

            ceres::LossFunction *loss_function = NULL;
            ceres::LocalParameterization *quaternion_local_parameterization =
                new ceres::EigenQuaternionParameterization;

            for (VectorOfConstraints::const_iterator constraints_iter =
                     constraints.begin();
                 constraints_iter != constraints.end(); ++constraints_iter)
            {
                const Constraint3d &constraint = *constraints_iter;

                MapOfPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);
                MapOfPoses::iterator pose_end_iter = poses->find(constraint.id_end);

                const Eigen::Matrix<double, 6, 6> sqrt_information =
                    constraint.information.llt().matrixL();
                // Ceres will take ownership of the pointer.
                ceres::CostFunction *cost_function =
                    PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

                problem->AddResidualBlock(cost_function, loss_function,
                                          pose_begin_iter->second.p.data(),
                                          pose_begin_iter->second.q.coeffs().data(),
                                          pose_end_iter->second.p.data(),
                                          pose_end_iter->second.q.coeffs().data());

                problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
                                             quaternion_local_parameterization);
                problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
                                             quaternion_local_parameterization);
            }

            // The pose graph optimization problem has six DOFs that are not fully
            // constrained. This is typically referred to as gauge freedom. You can apply
            // a rigid body transformation to all the nodes and the optimization problem
            // will still have the exact same cost. The Levenberg-Marquardt algorithm has
            // internal damping which mitigates this issue, but it is better to properly
            // constrain the gauge freedom. This can be done by setting one of the poses
            // as constant so the optimizer cannot change it.
            MapOfPoses::iterator pose_start_iter = poses->begin();
            CHECK(pose_start_iter != poses->end()) << "There are no poses.";
            problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
            problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
        }

        void pose_graph_optimization(MapOfPoses &poses,
                                     VectorOfConstraints &constraints)
        {
            ceres::Problem problem;
            BuildOptimizationProblem(constraints, &poses, &problem);

            ceres::Solver::Options options;
            options.max_num_iterations = 200;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            //std::cout << summary.FullReport() << '\n';
            std::cout << summary.BriefReport() << '\n';
            std::cout << summary.IsSolutionUsable() << std::endl;
        }

        LiPMatch();

        void loadTargetMap();

        ~LiPMatch();

        std::vector<Plane> mapPlanes;

        std::vector<Vehicle> mapVehicles;

        std::vector<Pole> mapPoles;

        SubgraphMatcher matcher;

        // SubgraphMatcher matcher2;

        std::vector<double> v_icp;

        MapOfPoses pose3d_map, pose3d_map_ori;
        VectorOfPose pose3d_vec;
        VectorOfConstraints constrain_vec;

        MapOfPoses optimized_pose3d_map;

        Mapping_refine map_rfn;

        std::map<int, pcl::PointCloud<pcl::PointXYZI>> map_id_pc;

        pcl::PointCloud<pcl::PointXYZI> refined_pt;

        pcl::PointCloud<pcl::PointXYZI> refined_pt_bef;

        pcl::PointCloud<pcl::PointXYZI> same_laserCloud;

        pcl::PointCloud<pcl::PointXYZI> same_laserCloud2;

        void run();

        bool LiPMatch_stop;

        bool LiPMatch_finished;

        bool stop_LiPMatch();

        std::vector<tools::m_keyframe> frameQueue;

        std::vector<std::shared_ptr<Maps_keyframe<float>>> keyframe_vec;

        Scene_alignment<float> scene_align;

        //原始点云坐标
        pcl::PointCloud<pcl::PointXYZI> laserCloudOri;

        pcl::PointCloud<pcl::PointXYZI> coeffSel;

        std::map<size_t, size_t> loop_closure_matchedid;

        pcl::PointCloud<pcl::PointXYZI> laserCloudOri_m1;
        pcl::PointCloud<pcl::PointXYZI> laserCloudOri_m2;

        pcl::PointCloud<pcl::PointXYZRGBA> laserCloudOri_m2_1;

        pcl::PointCloud<pcl::PointXYZI> laserCloudOri_mp1;

        pcl::PointCloud<pcl::PointXYZI> laserCloudOri_mp2;

        pcl::KdTreeFLANN<pcl::PointXYZI> m_kdtree_kfs;

        pcl::PointCloud<pcl::PointXYZI> all_kfs_pose;

        pcl::PointCloud<pcl::PointXYZI> mapToShow;

        bool isLocalize = false;

    private:
        void localize(tools::m_keyframe &c_keyframe, int keyFrameCount);

        void loopClosure(tools::m_keyframe &c_keyframe, int keyFrameCount);

        tools::TThreadHandle LiPMatch_hd;
    };

} // namespace LiPMatch_ns

#endif
