//#include <LiPMatch.h>
//#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//
//using namespace std;
//using namespace Eigen;
//using namespace LiPMatch_ns;
//
//pcl::PointCloud<pcl::PointXYZI>::Ptr map_structure_points(new pcl::PointCloud<pcl::PointXYZI>());
//pcl::PointCloud<pcl::PointXYZI>::Ptr map_vehicle_points(new pcl::PointCloud<pcl::PointXYZI>());
//pcl::PointCloud<pcl::PointXYZI>::Ptr map_cylinder_points(new pcl::PointCloud<pcl::PointXYZI>());
//
//std::shared_ptr<Maps_keyframe<float>> maps = std::make_shared<Maps_keyframe<float>>();
//
//
//
//
//LiPMatch::LiPMatch() : LiPMatch_stop(false), LiPMatch_finished(false)
//{
//    std::cout<<"loading structure points ..."<<std::endl;
//    pcl::io::loadPCDFile<pcl::PointXYZI>("/home/jjwen/software/catkin_rangenet_ws/src/map_structure_points.pcd", *map_structure_points);
//    std::cout<<"loading vehicle points ..."<<std::endl;
//    pcl::io::loadPCDFile<pcl::PointXYZI>("/home/jjwen/software/catkin_rangenet_ws/src/map_vehicle_points.pcd", *map_vehicle_points);
//    std::cout<<"loading cylinder points ..."<<std::endl;
//    pcl::io::loadPCDFile<pcl::PointXYZI>("/home/jjwen/software/catkin_rangenet_ws/src/map_cylinder_points.pcd", *map_cylinder_points);
//
//    vector<Vehicle> detectedLocalVehicles;
//    vector<Pole> detectedLocalPoles;
//    vector<Plane> detectedLocalPlanes;
//
//    pcl::PCA< pcl::PointXYZI > pca;
//    //聚类
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
//    std::vector<pcl::PointIndices> cluster_indices_vehicle;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_vehicle;
//    ec_vehicle.setClusterTolerance (0.202);
//    ec_vehicle.setMinClusterSize (85);
//    ec_vehicle.setMaxClusterSize (1500);
//    ec_vehicle.setSearchMethod (tree);
//    ec_vehicle.setInputCloud(map_vehicle_points);
//    ec_vehicle.extract (cluster_indices_vehicle);
//
//    int colorid = 0;
//    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_vehicle.begin (); it != cluster_indices_vehicle.end (); ++it)
//    {
//        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_vehicle(new pcl::PointCloud<pcl::PointXYZI>());
//        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//        {
//            map_vehicle_points->points[*pit].intensity = colorid;
//            laserCloudNgAftEe_vehicle->points.push_back (map_vehicle_points->points[*pit]);
//        }
//        pca.setInputCloud(laserCloudNgAftEe_vehicle);
//        Eigen::VectorXf eigenVal = pca.getEigenValues();
//
//        if (eigenVal[2] / eigenVal[0] < 0.005)
//            continue;
//
////        mapToShow += *laserCloudNgAftEe_vehicle;
//
//        colorid++;
//
////        std::cout<<eigenVal[0]<<" "<<eigenVal[1]<<" "<<eigenVal[2]<<std::endl;
//
//        Vehicle vc;
//        vc.VehiclePointCloudPtr = laserCloudNgAftEe_vehicle;
////        vc.keyFrameId = keyFrameCount;
//        vc.calcCenterAndElongation();
//        detectedLocalVehicles.push_back(vc);
//    }
//
//    observedVehicles.clear();
//    vVehicles.clear();
//
//    for (size_t i = 0; i < detectedLocalVehicles.size (); i++)
//    {
//        detectedLocalVehicles[i].id = vVehicles.size();
//        for(set<unsigned>::iterator it = observedVehicles.begin(); it != observedVehicles.end(); it++)
//        {
//            detectedLocalVehicles[i].neighborVehicles[*it] = 1;
//            vVehicles[*it].neighborVehicles[detectedLocalVehicles[i].id] = 1;
//        }
//        observedVehicles.insert(detectedLocalVehicles[i].id);
//        vVehicles.push_back(detectedLocalVehicles[i]);
//    }
//
//    std::cout<<"vVehicles.size() "<<vVehicles.size()<<std::endl;
//    maps->vVehicles = vVehicles;
//
//    //聚类
//    std::vector<pcl::PointIndices> cluster_indices_nature;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_nature;
//    ec_nature.setClusterTolerance (0.75);
//    ec_nature.setMinClusterSize (20);
//    ec_nature.setMaxClusterSize (750);
//    ec_nature.setSearchMethod (tree);
//    ec_nature.setInputCloud (map_cylinder_points);
//    ec_nature.extract (cluster_indices_nature);
//
//    colorid = 0;
//    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_nature.begin (); it != cluster_indices_nature.end (); ++it)
//    {
//        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_nature(new pcl::PointCloud<pcl::PointXYZI>());
//        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//        {
//            map_cylinder_points->points[*pit].intensity = colorid;
//            laserCloudNgAftEe_nature->points.push_back (map_cylinder_points->points[*pit]);
//        }
//        pca.setInputCloud(laserCloudNgAftEe_nature);
//        Eigen::VectorXf eigenVal = pca.getEigenValues();
//
//        if (eigenVal[1] / eigenVal[0] > 0.17)
//            continue;
//
////        mapToShow += *laserCloudNgAftEe_nature;
//        colorid++;
//
//        Pole vc;
//        vc.PolePointCloudPtr = laserCloudNgAftEe_nature;
////        vc.keyFrameId = keyFrameCount;
//        vc.calcCenterAndElongation();
//
//        detectedLocalPoles.push_back(vc);
//    }
//
//    observedPoles.clear();
//    vPoles.clear();
//
//    for (size_t i = 0; i < detectedLocalPoles.size (); i++)
//    {
//        detectedLocalPoles[i].id = vPoles.size();
//
//        // Update co-visibility graph
//        for(set<unsigned>::iterator it = observedPoles.begin(); it != observedPoles.end(); it++)
//        {
//            detectedLocalPoles[i].neighborPoles[*it] = 1;
//            vPoles[*it].neighborPoles[detectedLocalPoles[i].id] = 1;
//        }
//        observedPoles.insert(detectedLocalPoles[i].id);
//        vPoles.push_back(detectedLocalPoles[i]);
//    }
//
//    std::cout<<"vPoles.size() "<<vPoles.size()<<std::endl;
//    maps->vPoles = vPoles;
//
//
//    pcl::VoxelGrid<pcl::PointXYZI> gridMap;
//    gridMap.setLeafSize(0.8,0.8,0.8);
//    gridMap.setInputCloud(map_structure_points);
//    gridMap.filter(*map_structure_points);
//
//    colorid = 0;
//    //聚类
//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
//    ec.setClusterTolerance (0.85);
//    ec.setMinClusterSize (20);
//    ec.setMaxClusterSize (150000);
//    ec.setSearchMethod (tree);
//    ec.setInputCloud (map_structure_points);
//    ec.extract (cluster_indices);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe(new pcl::PointCloud<pcl::PointXYZI>());
//    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//    {
//        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//        {
//            map_structure_points->points[*pit].intensity = colorid;
//            laserCloudNgAftEe->points.push_back (map_structure_points->points[*pit]);
//        }
//        colorid++;
//    }
//
//
//    //区域增长法提取激光点云中的平面
//    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
//    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
//    normal_estimator.setSearchMethod (tree);
//    normal_estimator.setInputCloud (laserCloudNgAftEe);
//    normal_estimator.setKSearch (6);
//    normal_estimator.compute (*normals);
//
//    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
//    reg.setMinClusterSize (20);
//    reg.setMaxClusterSize (1000000);
//    reg.setSearchMethod (tree);
//    reg.setNumberOfNeighbours (15);
//    reg.setInputCloud (laserCloudNgAftEe);
//    reg.setInputNormals (normals);
//    reg.setSmoothnessThreshold (12.0 / 180.0 * M_PI);
//    reg.setCurvatureThreshold (9.0);
//
//    std::vector <pcl::PointIndices> clusters;
//    reg.extract (clusters);
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftReg(new pcl::PointCloud<pcl::PointXYZI>());
//    //创建一个模型参数对象，用于记录结果
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//    //inliers表示误差能容忍的点 记录的是点云的序号
//    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//    // 创建一个分割器
//    pcl::SACSegmentation<pcl::PointXYZI> seg;
//    // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
//    seg.setOptimizeCoefficients(true);
//    // Mandatory-设置目标几何形状areSameVehicle
//    seg.setModelType(pcl::SACMODEL_PLANE);
//    //分割方法：随机采样法
//    seg.setMethodType(pcl::SAC_RANSAC);
//    //设置误差容忍范围，也就是我说过的阈值
//    seg.setDistanceThreshold(0.1);
//
//    colorid = 0;
//    for (size_t i = 0 ; i < clusters.size() ; ++i)
//    {
//        Plane plane;
//        pcl::PointCloud<pcl::PointXYZI>::Ptr pointsOnPlane(new pcl::PointCloud<pcl::PointXYZI>());
//        pointsOnPlane->points.clear();
//
//        for (size_t j = 0 ; j < clusters[i].indices.size() ; ++j)
//        {
//            pcl::PointXYZI tmpPoint;
//            tmpPoint.x = laserCloudNgAftEe->points[clusters[i].indices[j]].x;
//            tmpPoint.y = laserCloudNgAftEe->points[clusters[i].indices[j]].y;
//            tmpPoint.z = laserCloudNgAftEe->points[clusters[i].indices[j]].z;
//            tmpPoint.intensity = i;
//            pointsOnPlane->points.push_back(tmpPoint);
//            laserCloudNgAftReg->points.push_back(tmpPoint);
//        }
//
//        //输入点云
//        seg.setInputCloud(pointsOnPlane);
//        //分割点云，获得平面和法向量
//        seg.segment(*inliers, *coefficients);
//
//        double planen[4];
//
//
//        planen[0] = coefficients->values[0];
//        planen[1] = coefficients->values[1];
//        planen[2] = coefficients->values[2];
//        planen[3] = -1.0 * coefficients->values[3];
//
////        std::cout<<"planen[0] "<<planen[0]<<" planen[1] "<<planen[1]<<" planen[2] "<<planen[2]<<" planen[3] "<<planen[3]<<std::endl;
////        std::cout<<planen[0]*planen[0]+planen[1]*planen[1]+planen[2]*planen[2]<<std::endl;
//
//        plane.v3normal = Vector3f(planen[0], planen[1], planen[2]);
//        plane.d = planen[3];
//        pcl::copyPointCloud(*pointsOnPlane, *plane.planePointCloudPtr);
//
//        float colors[3] = {0.1,0.5,0.85};
//
//        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudInlier(new pcl::PointCloud<pcl::PointXYZI>());
//
//        for (std::vector<int>::const_iterator pit = inliers->indices.begin (); pit != inliers->indices.end (); ++pit)
//        {
//            pointsOnPlane->points[*pit].intensity = colors[colorid % 3];
//            laserCloudInlier->points.push_back (pointsOnPlane->points[*pit]);
//        }
//
//        pcl::copyPointCloud(*laserCloudInlier, *plane.InplanePointCloudOriPtr);
//
//        colorid++;
//
//
//        plane.calcConvexHull(plane.planePointCloudPtr,planen);
//
//        plane.forcePtsLayOnPlane();
//
//        plane.computeMassCenterAndArea();
//        plane.calcElongationAndPpalDir();
//
//        plane.areaVoxels= plane.planePointCloudPtr->size() * 0.0025;
//
////        plane.keyFrameId = keyFrameCoun;
//
//        mapToShow += *plane.planePointCloudPtr;
//
//        detectedLocalPlanes.push_back(plane);
//    }
//
//    std::cout<<"detectedLocalPlanes.size() "<<detectedLocalPlanes.size ()<<std::endl;
//
//    observedPlanes.clear();
//    vPlanes.clear();
//    for (size_t i = 0; i < detectedLocalPlanes.size (); i++)
//    {
//        detectedLocalPlanes[i].id = vPlanes.size();
//        for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
//        {
//            detectedLocalPlanes[i].neighborPlanes[*it] = 1;
//            vPlanes[*it].neighborPlanes[detectedLocalPlanes[i].id] = 1;
//        }
//        observedPlanes.insert(detectedLocalPlanes[i].id);
//        vPlanes.push_back(detectedLocalPlanes[i]);
//    }
//
//    std::cout<<"vPlanes.size() "<<vPlanes.size()<<std::endl;
//
//    maps->vPlanes = vPlanes;
//
//    LiPMatch_hd = createThreadFromObjectMethod(this,&LiPMatch::run);
//
//    keyframe_vec.clear();
//
//    map_rfn.set_down_sample_resolution( 0.75 );
//
////    laserCloudOri_m1 += *laserCloudNgAftEe;
////    laserCloudOri_m1 += mapToShow;
//
//    laserCloudOri_m1 += *map_structure_points;
//
////    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterTargetMap;
////    downSizeFilterTargetMap.setLeafSize(0.50,0.50,0.50);
////    downSizeFilterTargetMap.setInputCloud(mapToShow.makeShared());
////    downSizeFilterTargetMap.filter(mapToShow);
//
//}
//
//
//bool LiPMatch::arePlanesNearby(Plane &plane1, Plane &plane2, const float distThreshold)
//{
//    float distThres2 = distThreshold * distThreshold;
//
//    // First we check distances between centroids and vertex to accelerate this check
//    if( (plane1.v3center - plane2.v3center).squaredNorm() < distThres2 )
//        return true;
//
//    for(unsigned i=1; i < plane1.polygonContourPtr->size(); i++)
//        if( (getVector3fromPointXYZ(plane1.polygonContourPtr->points[i]) - plane2.v3center).squaredNorm() < distThres2 )
//            return true;
//
//    for(unsigned j=1; j < plane2.polygonContourPtr->size(); j++)
//        if( (plane1.v3center - getVector3fromPointXYZ(plane2.polygonContourPtr->points[j]) ).squaredNorm() < distThres2 )
//            return true;
//
//    for(unsigned i=1; i < plane1.polygonContourPtr->size(); i++)
//        for(unsigned j=1; j < plane2.polygonContourPtr->size(); j++)
//            if( (diffPoints(plane1.polygonContourPtr->points[i], plane2.polygonContourPtr->points[j]) ).squaredNorm() < distThres2 )
//                return true;
//    return false;
//}
//
//void LiPMatch::detectPlanesCloud( m_keyframe &c_keyframe, int keyFrameCount)
//{
//    std::cout<<"new keyframe coming..."<<std::endl;
//    static pcl::VoxelGrid<pcl::PointXYZI> grid_1;
//    grid_1.setLeafSize(0.40,0.40,0.40);
//
//    grid_1.setInputCloud(c_keyframe.structurelaserCloud);
//    grid_1.filter (*c_keyframe.structurelaserCloud);
//
//    grid_1.setLeafSize(0.20,0.20,0.20);
//    grid_1.setInputCloud(c_keyframe.vehiclelaserCloud);
//    grid_1.filter (*c_keyframe.vehiclelaserCloud);
//
//    grid_1.setLeafSize(0.20,0.20,0.20);
//    grid_1.setInputCloud(c_keyframe.naturelaserCloud);
//    grid_1.filter (*c_keyframe.naturelaserCloud);
//
//    double time_behinSeg = pcl::getTime();
//
//    //////////////////////
//
////    std::cout<<"bef: "<<c_keyframe.vehiclelaserCloud->points.size()<<std::endl;
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
//    sor.setInputCloud(c_keyframe.vehiclelaserCloud);
////    sor.setRadiusSearch(0.35);
////    sor.setMinNeighborsInRadius(15);
//    sor.setMeanK(10);   //设置在进行统计时考虑查询点邻近点数
//    sor.setStddevMulThresh(0.1);
//    sor.setNegative(false);
//    sor.filter(*cloud_filter);
//    *c_keyframe.vehiclelaserCloud = *cloud_filter;
////    std::cout<<"aft: "<<c_keyframe.vehiclelaserCloud->points.size()<<std::endl;
//
//
//    int colorid = 0;
//    //聚类
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr treeVehicle (new pcl::search::KdTree<pcl::PointXYZI>);
//    treeVehicle->setInputCloud (c_keyframe.vehiclelaserCloud);
//    std::vector<pcl::PointIndices> cluster_indices_vehicle;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_vehicle;
//    ec_vehicle.setClusterTolerance (0.21);
////    ec.setMinClusterSize (100);
//    ec_vehicle.setMinClusterSize (150);
//    ec_vehicle.setMaxClusterSize (5000);
//    ec_vehicle.setSearchMethod (treeVehicle);
//    ec_vehicle.setInputCloud (c_keyframe.vehiclelaserCloud);
//    ec_vehicle.extract (cluster_indices_vehicle);
//    pcl::PCA< pcl::PointXYZI > pca;
//
//    vector<Vehicle> detectedLocalVehicles;
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr vehiclesPointCloud(new pcl::PointCloud<pcl::PointXYZI>);
//
//
//    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_vehicle.begin (); it != cluster_indices_vehicle.end (); ++it)
//    {
//        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_vehicle(new pcl::PointCloud<pcl::PointXYZI>());
//        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//        {
//            c_keyframe.vehiclelaserCloud->points[*pit].intensity = colorid;
//            laserCloudNgAftEe_vehicle->points.push_back (c_keyframe.vehiclelaserCloud->points[*pit]);
//        }
//        pca.setInputCloud(laserCloudNgAftEe_vehicle);
//        Eigen::VectorXf eigenVal = pca.getEigenValues();
//
////        if (eigenVal[2] / eigenVal[0] < 0.005)
////            continue;
//
//        if (eigenVal[2] / eigenVal[0] < 0.01)
//            continue;
//
//        if (eigenVal[1] / eigenVal[0] < 0.01)
//            continue;
//
//        *vehiclesPointCloud += *laserCloudNgAftEe_vehicle;
//
//        colorid++;
//
////        std::cout<<eigenVal[0]<<" "<<eigenVal[1]<<" "<<eigenVal[2]<<std::endl;
//
//        Vehicle vc;
//        vc.VehiclePointCloudPtr = laserCloudNgAftEe_vehicle;
//        vc.calcCenterAndElongation();
//
//        detectedLocalVehicles.push_back(vc);
//
//    }
//
//    observedVehicles.clear();
//    vVehicles.clear();
//
//    for (size_t i = 0; i < detectedLocalVehicles.size (); i++)
//    {
//        detectedLocalVehicles[i].id = vVehicles.size();
//
//        // Update co-visibility graph
//        for(set<unsigned>::iterator it = observedVehicles.begin(); it != observedVehicles.end(); it++)
//        {
//            detectedLocalVehicles[i].neighborVehicles[*it] = 1;
//            vVehicles[*it].neighborVehicles[detectedLocalVehicles[i].id] = 1;
//        }
//        observedVehicles.insert(detectedLocalVehicles[i].id);
//        vVehicles.push_back(detectedLocalVehicles[i]);
//    }
//
//
//    std::shared_ptr<Maps_keyframe<float>> mk = std::make_shared<Maps_keyframe<float>>();
//
//    std::cout<<"vVehicles.size() "<<vVehicles.size()<<std::endl;
//
//    mk->vVehicles = vVehicles;
//
//    //聚类
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr treeNature (new pcl::search::KdTree<pcl::PointXYZI>);
//    treeNature->setInputCloud (c_keyframe.naturelaserCloud);
//    std::vector<pcl::PointIndices> cluster_indices_nature;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_nature;
//    ec_nature.setClusterTolerance (0.21);
////    ec.setMinClusterSize (100);
//    ec_nature.setMinClusterSize (20);
//    ec_nature.setMaxClusterSize (500);
//    ec_nature.setSearchMethod (treeNature);
//    ec_nature.setInputCloud (c_keyframe.naturelaserCloud);
//    ec_nature.extract (cluster_indices_nature);
//    pcl::PCA< pcl::PointXYZI > pca1;
//
//    vector<Pole> detectedLocalPoles;
//
//    colorid = 0;
//    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_nature.begin (); it != cluster_indices_nature.end (); ++it)
//    {
//        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_nature(new pcl::PointCloud<pcl::PointXYZI>());
//        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//        {
//            c_keyframe.naturelaserCloud->points[*pit].intensity = colorid;
//            laserCloudNgAftEe_nature->points.push_back (c_keyframe.naturelaserCloud->points[*pit]);
//        }
//        pca1.setInputCloud(laserCloudNgAftEe_nature);
//        Eigen::VectorXf eigenVal = pca1.getEigenValues();
//
//        if (eigenVal[1] / eigenVal[0] > 0.135)
//            continue;
//
//        colorid++;
//
//        Pole vc;
//        vc.PolePointCloudPtr = laserCloudNgAftEe_nature;
//        vc.keyFrameId = keyFrameCount;
//        vc.calcCenterAndElongation();
//
//        detectedLocalPoles.push_back(vc);
//
////        std::cout<<eigenVal[0]<<" "<<eigenVal[1]<<" "<<eigenVal[2]<<std::endl;
//
//    }
//
//    observedPoles.clear();
//    vPoles.clear();
//
//    for (size_t i = 0; i < detectedLocalPoles.size (); i++)
//    {
//        detectedLocalPoles[i].id = vPoles.size();
//
//        // Update co-visibility graph
//        for(set<unsigned>::iterator it = observedPoles.begin(); it != observedPoles.end(); it++)
//        {
//            detectedLocalPoles[i].neighborPoles[*it] = 1;
//            vPoles[*it].neighborPoles[detectedLocalPoles[i].id] = 1;
//        }
//        observedPoles.insert(detectedLocalPoles[i].id);
//        vPoles.push_back(detectedLocalPoles[i]);
//    }
//
//    std::cout<<"vPoles.size() "<<vPoles.size()<<std::endl;
//
//    mk->vPoles = vPoles;
//
//
//    /////////////////////
//
//    //聚类
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
//    tree->setInputCloud (c_keyframe.structurelaserCloud);
//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
//    ec.setClusterTolerance (0.65);
////    ec.setMinClusterSize (100);
//    ec.setMinClusterSize (70);
//    ec.setMaxClusterSize (150000);
//    ec.setSearchMethod (tree);
//    ec.setInputCloud (c_keyframe.structurelaserCloud);
//    ec.extract (cluster_indices);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe(new pcl::PointCloud<pcl::PointXYZI>());
//    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//    {
//        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//        {
//            laserCloudNgAftEe->points.push_back (c_keyframe.structurelaserCloud->points[*pit]);
//        }
//    }
//
////    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe(new pcl::PointCloud<pcl::PointXYZI>());
////    laserCloudNgAftEe = c_keyframe.structurelaserCloud;
//
//    //区域增长法提取激光点云中的平面
//    pcl::search::Search<pcl::PointXYZI>::Ptr treeRg (new pcl::search::KdTree<pcl::PointXYZI>);
//    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
//    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
//    normal_estimator.setSearchMethod (treeRg);
//    normal_estimator.setInputCloud (laserCloudNgAftEe);
//    normal_estimator.setKSearch (10);
//    normal_estimator.compute (*normals);
//
//    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
//    reg.setMinClusterSize (70);
////    reg.setMinClusterSize (120);
//    reg.setMaxClusterSize (1000000);
//    reg.setSearchMethod (treeRg);
//    reg.setNumberOfNeighbours (20);
//    reg.setInputCloud (laserCloudNgAftEe);
//    reg.setInputNormals (normals);
//    reg.setSmoothnessThreshold (9.0 / 180.0 * M_PI);
//    reg.setCurvatureThreshold (7.0);
//
//    std::vector <pcl::PointIndices> clusters;
//    reg.extract (clusters);
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftReg(new pcl::PointCloud<pcl::PointXYZI>());
//    //创建一个模型参数对象，用于记录结果
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//    //inliers表示误差能容忍的点 记录的是点云的序号
//    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//    // 创建一个分割器
//    pcl::SACSegmentation<pcl::PointXYZI> seg;
//    // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
//    seg.setOptimizeCoefficients(true);
//    // Mandatory-设置目标几何形状areSameVehicle
//    seg.setModelType(pcl::SACMODEL_PLANE);
//    //分割方法：随机采样法
//    seg.setMethodType(pcl::SAC_RANSAC);
//    //设置误差容忍范围，也就是我说过的阈值
//    seg.setDistanceThreshold(0.075);
//
//    vector<Plane> detectedLocalPlanes;
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr curPlanePoints(new pcl::PointCloud<pcl::PointXYZI>());
//
//    for (size_t i = 0 ; i < clusters.size() ; ++i)
//    {
//        Plane plane;
//        pcl::PointCloud<pcl::PointXYZI>::Ptr pointsOnPlane(new pcl::PointCloud<pcl::PointXYZI>());
//        pointsOnPlane->points.clear();
//
//        for (size_t j = 0 ; j < clusters[i].indices.size() ; ++j)
//        {
//            pcl::PointXYZI tmpPoint;
//            tmpPoint.x = laserCloudNgAftEe->points[clusters[i].indices[j]].x;
//            tmpPoint.y = laserCloudNgAftEe->points[clusters[i].indices[j]].y;
//            tmpPoint.z = laserCloudNgAftEe->points[clusters[i].indices[j]].z;
//            tmpPoint.intensity = i;
//            pointsOnPlane->points.push_back(tmpPoint);
//            laserCloudNgAftReg->points.push_back(tmpPoint);
//        }
//
//        //输入点云
//        seg.setInputCloud(pointsOnPlane);
//        //分割点云，获得平面和法向量
//        seg.segment(*inliers, *coefficients);
//
//        double planen[4];
//
//        planen[0] = coefficients->values[0];
//        planen[1] = coefficients->values[1];
//        planen[2] = coefficients->values[2];
//        planen[3] = -1.0 * coefficients->values[3];
//
//        plane.v3normal = Vector3f(planen[0], planen[1], planen[2]);
//        plane.d = planen[3];
//        pcl::copyPointCloud(*pointsOnPlane, *plane.planePointCloudPtr);
//
//        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudInlier(new pcl::PointCloud<pcl::PointXYZI>());
//
//        for (std::vector<int>::const_iterator pit = inliers->indices.begin (); pit != inliers->indices.end (); ++pit)
//        {
//            laserCloudInlier->points.push_back (pointsOnPlane->points[*pit]);
//        }
//
//        pcl::copyPointCloud(*laserCloudInlier, *plane.InplanePointCloudOriPtr);
//
//
//        plane.calcConvexHull(plane.planePointCloudPtr,planen);
//
//        plane.forcePtsLayOnPlane();
//
//
//        plane.computeMassCenterAndArea();
//        plane.calcElongationAndPpalDir();
//
//        plane.areaVoxels= plane.planePointCloudPtr->size() * 0.0025;
//
//        *curPlanePoints += *plane.planePointCloudPtr;
//
//        detectedLocalPlanes.push_back(plane);
//    }
//
//    laserCloudOri_m2.points.clear();
//    laserCloudOri_m2 += *curPlanePoints;
//
//    // Merge detected planes with previous ones if they are the same
//    observedPlanes.clear();
//    vPlanes.clear();
//
//    for (size_t i = 0; i < detectedLocalPlanes.size (); i++)
//    {
//        detectedLocalPlanes[i].id = vPlanes.size();
//
//        // Update co-visibility graph
//        for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
//        {
//            detectedLocalPlanes[i].neighborPlanes[*it] = 1;
//            vPlanes[*it].neighborPlanes[detectedLocalPlanes[i].id] = 1;
//        }
//        observedPlanes.insert(detectedLocalPlanes[i].id);
//        vPlanes.push_back(detectedLocalPlanes[i]);
//    }
//
//    std::cout<<"vPlanes.size() ";
//    std::cout<<vPlanes.size()<<std::endl;
//
//    mk->vPlanes = vPlanes;
//
////    std::cout<<"mk->m_accumulated_g_pc.size() "<<mk->m_accumulated_g_pc.size()<<std::endl;
//
//    mk->m_accumulated_structure_pc = *c_keyframe.structurelaserCloud;
//    mk->m_accumulated_vehicle_pc = *c_keyframe.vehiclelaserCloud;
//
//
//    keyframe_vec.push_back( mk );
//
//    std::shared_ptr<Maps_keyframe<float>>& last_keyframe = keyframe_vec.back();
//
//    std::map<unsigned, unsigned> bestMatch; bestMatch.clear();
//
//    Subgraph currentSubgraph1(maps->vPlanes, 0);
//    Subgraph targetSubgraph1(last_keyframe->vPlanes, 0);
//
//    int unaycount;
//    std::map<unsigned, unsigned> resultingMatch = matcher.compareSubgraphs(currentSubgraph1, targetSubgraph1,
//                                                                                   unaycount);
//
//    if (matcher.allMatchGroups.size() < 1)
//        return;
//
//    std::cout << "matcher.allMatchGroups.size() " << matcher.allMatchGroups.size() << std::endl;
//
//    int allMatchedSize;
//
//    int greatMatchedSize = 0;
//    std::map<unsigned, unsigned> greatestMatch;
//    std::map<unsigned, unsigned> allgreatestMatch;
//
////    laserCloudOri_m2.points.clear();
////    laserCloudOri_m2.width = 0;
////    laserCloudOri_m2.height = 0;
//
//    for (auto itemmatch : matcher.allMatchGroups) {
//
//        bestMatch = itemmatch;
//
////        std::cout << "bestMatch.size() " << bestMatch.size() << std::endl;
//
//        if (bestMatch.size() < 1)
//            return;
//
//        std::vector <Eigen::Vector3f> kvc;
//        std::vector <Eigen::Vector3f> lvc;
//        std::vector <Eigen::Vector3f> kvn;
//        std::vector <Eigen::Vector3f> lvn;
//
//        for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++) {
//            kvc.push_back(maps->vPlanes[it->first].v3center);
//            lvc.push_back(last_keyframe->vPlanes[it->second].v3center);
//            kvn.push_back(maps->vPlanes[it->first].v3normal);
//            lvn.push_back(last_keyframe->vPlanes[it->second].v3normal);
//        }
//
//
//        //KITTI
//        std::map<unsigned, unsigned> bestMatchVehicle;
//        bestMatchVehicle.clear();
//
//        Subgraph currentSubgraph2(maps->vVehicles, 0);
//        Subgraph targetSubgraph2(last_keyframe->vVehicles, 0);
//
//        unaycount = 0;
//        std::map<unsigned, unsigned> resultingMatch1 = matcher.compareSubgraphsVehiclePlaneRef(currentSubgraph2,
//                                                                                               targetSubgraph2,
//                                                                                               unaycount, kvc,
//                                                                                               lvc, kvn, lvn);
//
//
//        std::vector <Eigen::Vector3f> vkvc;
//        std::vector <Eigen::Vector3f> vlvc;
//
//        for (map<unsigned, unsigned>::iterator it = resultingMatch1.begin(); it != resultingMatch1.end(); it++) {
//            vkvc.push_back(maps->vVehicles[it->first].v3center);
//            vlvc.push_back(last_keyframe->vVehicles[it->second].v3center);
//        }
//
//
//
//        bestMatchVehicle = resultingMatch1;
//
//        std::map<unsigned, unsigned> bestMatchPole;
//        bestMatchPole.clear();
//
//        Subgraph currentSubgraph3(maps->vPoles, 0);
//        Subgraph targetSubgraph3(last_keyframe->vPoles, 0);
//
//        matcher.v_kvc = vkvc;
//        matcher.v_lvc = vlvc;
//
//        unaycount = 0;
//        std::map<unsigned, unsigned> resultingMatch2 = matcher.compareSubgraphsPolePlaneRef(currentSubgraph3,
//                                                                                            targetSubgraph3, unaycount,
//                                                                                            kvc,
//                                                                                            lvc, kvn, lvn);
//
//        bestMatchPole = resultingMatch2;
//
////        std::cout << "bestMatchPole.size() " << bestMatchPole.size() << std::endl;
////        std::cout << "bestMatchVehicle.size() " << bestMatchVehicle.size() << std::endl;
//
////    laserCloudOri_m2.height = 1;
////    laserCloudOri_m2.width = vehiclesPointCloud->points.size();
////    laserCloudOri_m2 = *vehiclesPointCloud;
//
//
////    laserCloudOri_m2.points.clear();
////
////        if (bestMatch.size() > 0)
////            for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++) {
////                laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
////                laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
////            }
//
////    std::cout<<"matcher.allwinnerMatch.size() "<<matcher.allwinnerMatch.size()<<std::endl;
////    if (matcher.allwinnerMatch.size() > 0)
////        for (map<unsigned, unsigned>::iterator it = matcher.allwinnerMatch.begin(); it != matcher.allwinnerMatch.end(); it++)
////        {
////            laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
////
////            for (size_t k = 0 ; k < last_keyframe->vPlanes[it->second].planePointCloudPtr->points.size() ; ++k)
////            {
////                last_keyframe->vPlanes[it->second].planePointCloudPtr->points[k].z = last_keyframe->vPlanes[it->second].planePointCloudPtr->points[k].z + 10.0;
////            }
////            laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
////        }
//
////    pcl::copyPointCloud(*vehiclesPointCloud,laserCloudOri_m2);
//
////    laserCloudOri_m2 += *map_cylinder_points;
////    laserCloudOri_m2 = *map_vehicle_points;
//
////    laserCloudOri_m2 += *map_structure_points;
//
////    for (size_t k = 0 ; k < vehiclesPointCloud->points.size() ; ++k)
////    {
////        vehiclesPointCloud->points[k].z = vehiclesPointCloud->points[k].z + 10.0;
////    }
////
////    for (size_t k = 0 ; k < curPlanePoints->points.size() ; ++k)
////    {
////        curPlanePoints->points[k].z = curPlanePoints->points[k].z + 10.0;
////    }
////    laserCloudOri_m2 += *curPlanePoints;
//
//        //KITTI
//        if (bestMatch.size() < 2) {
//            return;
//        }
//
//        std::vector<int> firstINdexes;
//        for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++) {
//            firstINdexes.push_back(it->first);
//        }
//
//        pcl::PointCloud <pcl::PointXYZI> normPOints;
//        for (int it = 0; it < firstINdexes.size(); ++it) {
//            pcl::PointXYZI phe;
//            phe.x = maps->vPlanes[firstINdexes[it]].v3normal(0);
//            phe.y = maps->vPlanes[firstINdexes[it]].v3normal(1);
//            phe.z = maps->vPlanes[firstINdexes[it]].v3normal(2);
//            normPOints.points.push_back(phe);
//        }
//
//        //聚类
//        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree <pcl::PointXYZI>);
//        tree1->setInputCloud(normPOints.makeShared());
//        std::vector <pcl::PointIndices> cluster_indices2;
//        pcl::EuclideanClusterExtraction <pcl::PointXYZI> ec1;
//        ec1.setClusterTolerance(0.15);
//        ec1.setMinClusterSize(1);
//        ec1.setMaxClusterSize(100);
//        ec1.setSearchMethod(tree1);
//        ec1.setInputCloud(normPOints.makeShared());
//        ec1.extract(cluster_indices2);
//
////        std::cout << "cluster_indices2.size() " << cluster_indices2.size() << std::endl;
//
//        if (cluster_indices2.size() < 2) {
//            return;
//        }
//
////    if (bestMatch.size() < 3)
////        return;
//
//        allMatchedSize = bestMatch.size();
//
//        allMatchedSize += bestMatchVehicle.size();
//
//        allMatchedSize += bestMatchPole.size();
//
////    if (bestMatchVehicle.size() > 1)
////    {
////        allMatchedSize += bestMatchVehicle.size();
////    }
////    if (bestMatchPole.size() > 1)
////    {
////        allMatchedSize += bestMatchPole.size();
////    }
//
//
////    std::cout<<"comp time "<<aveTime/(pointSearchInd.size())<<std::endl;
//
////        std::cout << "allMatchedSize " << allMatchedSize << std::endl;
//
////        if (allMatchedSize > 5) {
////            for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
////        {
////            laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
//////            laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
////
//////            pcl::PointCloud<pcl::PointXYZI>::Ptr tempz1(new pcl::PointCloud<pcl::PointXYZI>());
//////            *tempz1 = *maps->vPlanes[it->first].planePointCloudPtr;
//////            for (size_t k = 0 ; k < tempz1->points.size() ; ++k)
//////            {
//////                tempz1->points[k].z = tempz1->points[k].z + 10.0;
//////            }
//////            laserCloudOri_m2 += *tempz1;
////
////            pcl::PointCloud<pcl::PointXYZI>::Ptr tempz(new pcl::PointCloud<pcl::PointXYZI>());
////            *tempz = *last_keyframe->vPlanes[it->second].planePointCloudPtr;
////            for (size_t k = 0 ; k < tempz->points.size() ; ++k)
////            {
////                tempz->points[k].z = tempz->points[k].z + 10.0;
////            }
////            laserCloudOri_m2 += *tempz;
////        }
////
////            for (auto itm : bestMatch)
////            {
////                allgreatestMatch.insert(itm);
////            }
////
////        }
//
////        std::cout<<allgreatestMatch.size()<<std::endl;
//        if (allMatchedSize > greatMatchedSize)
//        {
//            greatMatchedSize = allMatchedSize;
//            greatestMatch = bestMatch;
//
//        }
//
//    }
//
//
//
//
//    allMatchedSize = greatMatchedSize;
//    bestMatch = greatestMatch;
//
//
////    laserCloudOri_m2 += *map_structure_points;
//
//
//
////    if (bestMatch.size() < 1)
////        return;
////
////
////    std::vector<Eigen::Vector3f> kvc;
////    std::vector<Eigen::Vector3f> lvc;
////    std::vector<Eigen::Vector3f> kvn;
////    std::vector<Eigen::Vector3f> lvn;
////
////    for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
////    {
////        kvc.push_back(maps->vPlanes[it->first].v3center);
////        lvc.push_back(last_keyframe->vPlanes[it->second].v3center);
////        kvn.push_back(maps->vPlanes[it->first].v3normal);
////        lvn.push_back(last_keyframe->vPlanes[it->second].v3normal);
////    }
////
////
////    //KITTI
////    std::map<unsigned, unsigned> bestMatchVehicle; bestMatchVehicle.clear();
////
////    Subgraph currentSubgraph2(maps->vVehicles, 0);
////    Subgraph targetSubgraph2(last_keyframe->vVehicles, 0);
////
////    unaycount = 0;
////    std::map<unsigned, unsigned> resultingMatch1 = matcher.compareSubgraphsVehiclePlaneRef(currentSubgraph2, targetSubgraph2, unaycount, kvc,
////                                                                                                  lvc, kvn, lvn);
////
////    bestMatchVehicle = resultingMatch1;
////
////    std::map<unsigned, unsigned> bestMatchPole; bestMatchPole.clear();
////
////    Subgraph currentSubgraph3(maps->vPoles, 0);
////    Subgraph targetSubgraph3(last_keyframe->vPoles, 0);
////
////    unaycount = 0;
////    std::map<unsigned, unsigned> resultingMatch2 = matcher.compareSubgraphsPolePlaneRef(currentSubgraph3, targetSubgraph3, unaycount, kvc,
////                                                                                               lvc, kvn, lvn);
////
////    bestMatchPole = resultingMatch2;
////
////    std::cout<<"bestMatchPole.size() "<<bestMatchPole.size()<<std::endl;
////    std::cout<<"bestMatchVehicle.size() "<<bestMatchVehicle.size()<<std::endl;
////
//////    laserCloudOri_m2.height = 1;
//////    laserCloudOri_m2.width = vehiclesPointCloud->points.size();
//////    laserCloudOri_m2 = *vehiclesPointCloud;
////
////
////    laserCloudOri_m2.points.clear();
////
////    if (bestMatch.size() > 0)
////        for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
////        {
////            laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
////            laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
////        }
////
//////    std::cout<<"matcher.allwinnerMatch.size() "<<matcher.allwinnerMatch.size()<<std::endl;
//////    if (matcher.allwinnerMatch.size() > 0)
//////        for (map<unsigned, unsigned>::iterator it = matcher.allwinnerMatch.begin(); it != matcher.allwinnerMatch.end(); it++)
//////        {
//////            laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
//////
//////            for (size_t k = 0 ; k < last_keyframe->vPlanes[it->second].planePointCloudPtr->points.size() ; ++k)
//////            {
//////                last_keyframe->vPlanes[it->second].planePointCloudPtr->points[k].z = last_keyframe->vPlanes[it->second].planePointCloudPtr->points[k].z + 10.0;
//////            }
//////            laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
//////        }
////
//////    pcl::copyPointCloud(*vehiclesPointCloud,laserCloudOri_m2);
////
//////    laserCloudOri_m2 += *map_cylinder_points;
//////    laserCloudOri_m2 = *map_vehicle_points;
////
//////    laserCloudOri_m2 += *map_structure_points;
////
//////    for (size_t k = 0 ; k < vehiclesPointCloud->points.size() ; ++k)
//////    {
//////        vehiclesPointCloud->points[k].z = vehiclesPointCloud->points[k].z + 10.0;
//////    }
//////
//////    for (size_t k = 0 ; k < curPlanePoints->points.size() ; ++k)
//////    {
//////        curPlanePoints->points[k].z = curPlanePoints->points[k].z + 10.0;
//////    }
//////    laserCloudOri_m2 += *curPlanePoints;
////
////    //KITTI
////    if (bestMatch.size() < 2) {
////        return;
////    }
////
////    std::vector<int> firstINdexes;
////    for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++) {
////        firstINdexes.push_back(it->first);
////    }
////
////    pcl::PointCloud<pcl::PointXYZI> normPOints;
////    for (int it = 0; it < firstINdexes.size(); ++it)
////    {
////        pcl::PointXYZI phe;
////        phe.x = maps->vPlanes[firstINdexes[it]].v3normal(0);
////        phe.y = maps->vPlanes[firstINdexes[it]].v3normal(1);
////        phe.z = maps->vPlanes[firstINdexes[it]].v3normal(2);
////        normPOints.points.push_back(phe);
////    }
////
////    //聚类
////    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
////    tree1->setInputCloud(normPOints.makeShared());
////    std::vector<pcl::PointIndices> cluster_indices2;
////    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec1;
////    ec1.setClusterTolerance(0.15);
////    ec1.setMinClusterSize(1);
////    ec1.setMaxClusterSize(100);
////    ec1.setSearchMethod(tree1);
////    ec1.setInputCloud(normPOints.makeShared());
////    ec1.extract(cluster_indices2);
////
////    std::cout<<"cluster_indices2.size() "<<cluster_indices2.size()<<std::endl;
////
////    if (cluster_indices2.size() < 2) {
////        return;
////    }
////
//////    if (bestMatch.size() < 3)
//////        return;
////
////    int allMatchedSize = bestMatch.size();
////
////    allMatchedSize += bestMatchVehicle.size();
////
////    allMatchedSize += bestMatchPole.size();
////
//////    if (bestMatchVehicle.size() > 1)
//////    {
//////        allMatchedSize += bestMatchVehicle.size();
//////    }
//////    if (bestMatchPole.size() > 1)
//////    {
//////        allMatchedSize += bestMatchPole.size();
//////    }
////
////
//////    std::cout<<"comp time "<<aveTime/(pointSearchInd.size())<<std::endl;
////
//    std::cout<<"allMatchedSize "<<allMatchedSize<<std::endl;
//
//
////KITTI
//    if (allMatchedSize < 7) {
//        return;
//    }
//
//    std::cout<<"localization successful !"<<std::endl;
////    laserCloudOri_m2.points.clear();
//
////    if (bestMatch.size() > 0)
////        for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
////        {
////            laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
////            laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
////        }
//
//
//
//
////    if (allgreatestMatch.size() > 0)
////        for (map<unsigned, unsigned>::iterator it = allgreatestMatch.begin(); it != allgreatestMatch.end(); it++)
////        {
////            laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
////            laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
////        }
//
//
//
////    if (bestMatchVehicle.size() > 0)
////        for (map<unsigned, unsigned>::iterator it = bestMatchVehicle.begin(); it != bestMatchVehicle.end(); it++)
////        {
////            laserCloudOri_m2 += *maps->vVehicles[it->first].VehiclePointCloudPtr;
////            laserCloudOri_m2 += *last_keyframe->vVehicles[it->second].VehiclePointCloudPtr;
////        }
//
////    if (bestMatchPole.size() > 0)
////        for (map<unsigned, unsigned>::iterator it = bestMatchPole.begin(); it != bestMatchPole.end(); it++)
////        {
////            laserCloudOri_m2 += *maps->vPoles[it->first].PolePointCloudPtr;
////            laserCloudOri_m2 += *last_keyframe->vPoles[it->second].PolePointCloudPtr;
////        }
//
//}
//
//void LiPMatch::run()
//{
//    size_t numPrevKFs = 0;
//
//    while(!LiPMatch_stop)  // Stop loop if LiPMatch
//    {
//        if( numPrevKFs == frameQueue.size() )
//        {
//            sleep(10);
//        }
//        else
//        {
//            detectPlanesCloud( frameQueue[numPrevKFs], numPrevKFs);
//            ++numPrevKFs;
//        }
//    }
//    LiPMatch_finished = true;
//}
//
//
//
//bool LiPMatch::stop_LiPMatch()
//{
//    LiPMatch_stop = true;
//    while(!LiPMatch_finished)
//        sleep(1);
//
//    cout << "Waiting for LiPMatch thread to die.." << endl;
//
//    joinThread(LiPMatch_hd);
//    LiPMatch_hd.clear();
//
//    return true;
//}
//
//LiPMatch::~LiPMatch()
//{
//    cout << "\n\n\nLiPMatch destructor called -> Save color information to file\n";
//
//    stop_LiPMatch();
//
//    cout << " .. LiPMatch has died." << endl;
//}
//
//
//
//
//
//
//


#include <LiPMatch.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace Eigen;
using namespace LiPMatch_ns;

pcl::PointCloud<pcl::PointXYZI>::Ptr map_structure_points(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr map_vehicle_points(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr map_cylinder_points(new pcl::PointCloud<pcl::PointXYZI>());

std::shared_ptr<Maps_keyframe<float>> maps = std::make_shared<Maps_keyframe<float>>();




// LiPMatch::LiPMatch() : LiPMatch_stop(false), LiPMatch_finished(false)
// {
//     std::cout<<"loading structure points ..."<<std::endl;
//     pcl::io::loadPCDFile<pcl::PointXYZI>("/home/jjwen/software/catkin_rangenet_ws/src/map_structure_points.pcd", *map_structure_points);
//     std::cout<<"loading vehicle points ..."<<std::endl;
//     pcl::io::loadPCDFile<pcl::PointXYZI>("/home/jjwen/software/catkin_rangenet_ws/src/map_vehicle_points.pcd", *map_vehicle_points);
//     std::cout<<"loading cylinder points ..."<<std::endl;
//     pcl::io::loadPCDFile<pcl::PointXYZI>("/home/jjwen/software/catkin_rangenet_ws/src/map_cylinder_points.pcd", *map_cylinder_points);

//     vector<Vehicle> detectedLocalVehicles;
//     vector<Pole> detectedLocalPoles;
//     vector<Plane> detectedLocalPlanes;

//     pcl::PCA< pcl::PointXYZI > pca;
//     //聚类
//     pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
//     std::vector<pcl::PointIndices> cluster_indices_vehicle;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_vehicle;
//     ec_vehicle.setClusterTolerance (0.205);
//     ec_vehicle.setMinClusterSize (80);
//     ec_vehicle.setMaxClusterSize (7500);
//     ec_vehicle.setSearchMethod (tree);
//     ec_vehicle.setInputCloud(map_vehicle_points);
//     ec_vehicle.extract (cluster_indices_vehicle);

//     int colorid = 0;
//     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_vehicle.begin (); it != cluster_indices_vehicle.end (); ++it)
//     {
//         pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_vehicle(new pcl::PointCloud<pcl::PointXYZI>());
//         for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//         {
//             map_vehicle_points->points[*pit].intensity = colorid;
//             laserCloudNgAftEe_vehicle->points.push_back (map_vehicle_points->points[*pit]);
//         }
//         pca.setInputCloud(laserCloudNgAftEe_vehicle);
//         Eigen::VectorXf eigenVal = pca.getEigenValues();

//         if (eigenVal[2] / eigenVal[0] < 0.01)
//             continue;

//         if (eigenVal[1] / eigenVal[0] < 0.01)
//             continue;

//         mapToShow += *laserCloudNgAftEe_vehicle;

//         colorid++;

//         Vehicle vc;
//         vc.VehiclePointCloudPtr = laserCloudNgAftEe_vehicle;
//         vc.calcCenterAndElongation();
//         detectedLocalVehicles.push_back(vc);
//     }

//     observedVehicles.clear();
//     vVehicles.clear();

//     for (size_t i = 0; i < detectedLocalVehicles.size (); i++)
//     {
//         detectedLocalVehicles[i].id = vVehicles.size();
//         for(set<unsigned>::iterator it = observedVehicles.begin(); it != observedVehicles.end(); it++)
//         {
//             detectedLocalVehicles[i].neighborVehicles[*it] = 1;
//             vVehicles[*it].neighborVehicles[detectedLocalVehicles[i].id] = 1;
//         }
//         observedVehicles.insert(detectedLocalVehicles[i].id);
//         vVehicles.push_back(detectedLocalVehicles[i]);
//     }

//     std::cout<<"vVehicles.size() "<<vVehicles.size()<<std::endl;
//     maps->vVehicles = vVehicles;

//     //聚类
//     std::vector<pcl::PointIndices> cluster_indices_nature;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_nature;
//     ec_nature.setClusterTolerance (0.75);
//     ec_nature.setMinClusterSize (20);
//     ec_nature.setMaxClusterSize (750);
//     ec_nature.setSearchMethod (tree);
//     ec_nature.setInputCloud (map_cylinder_points);
//     ec_nature.extract (cluster_indices_nature);

//     colorid = 0;
//     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_nature.begin (); it != cluster_indices_nature.end (); ++it)
//     {
//         pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_nature(new pcl::PointCloud<pcl::PointXYZI>());
//         for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//         {
//             map_cylinder_points->points[*pit].intensity = colorid;
//             laserCloudNgAftEe_nature->points.push_back (map_cylinder_points->points[*pit]);
//         }
//         pca.setInputCloud(laserCloudNgAftEe_nature);
//         Eigen::VectorXf eigenVal = pca.getEigenValues();

//         if (eigenVal[1] / eigenVal[0] > 0.135)
//             continue;

//         mapToShow += *laserCloudNgAftEe_nature;
//         colorid++;

//         Pole vc;
//         vc.PolePointCloudPtr = laserCloudNgAftEe_nature;
//         vc.calcCenterAndElongation();
//         detectedLocalPoles.push_back(vc);
//     }

//     observedPoles.clear();
//     vPoles.clear();

//     for (size_t i = 0; i < detectedLocalPoles.size (); i++)
//     {
//         detectedLocalPoles[i].id = vPoles.size();

//         // Update co-visibility graph
//         for(set<unsigned>::iterator it = observedPoles.begin(); it != observedPoles.end(); it++)
//         {
//             detectedLocalPoles[i].neighborPoles[*it] = 1;
//             vPoles[*it].neighborPoles[detectedLocalPoles[i].id] = 1;
//         }
//         observedPoles.insert(detectedLocalPoles[i].id);
//         vPoles.push_back(detectedLocalPoles[i]);
//     }

//     std::cout<<"vPoles.size() "<<vPoles.size()<<std::endl;
//     maps->vPoles = vPoles;

// //    pcl::VoxelGrid<pcl::PointXYZI> gridMap;
// //    gridMap.setLeafSize(0.8,0.8,0.8);
// //    gridMap.setInputCloud(map_structure_points);
// //    gridMap.filter(*map_structure_points);

//     //聚类
//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
//     ec.setClusterTolerance (0.65);
//     ec.setMinClusterSize (70);
//     ec.setMaxClusterSize (150000);
//     ec.setSearchMethod (tree);
//     ec.setInputCloud (map_structure_points);
//     ec.extract (cluster_indices);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe(new pcl::PointCloud<pcl::PointXYZI>());
//     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//     {
//         for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//         {
//             laserCloudNgAftEe->points.push_back (map_structure_points->points[*pit]);
//         }
//     }

//     //区域增长法提取激光点云中的平面
//     pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
//     pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
//     normal_estimator.setSearchMethod (tree);
//     normal_estimator.setInputCloud (laserCloudNgAftEe);
//     normal_estimator.setKSearch (10);
//     normal_estimator.compute (*normals);

//     pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
//     reg.setMinClusterSize (70);
//     reg.setMaxClusterSize (1000000);
//     reg.setSearchMethod (tree);
//     reg.setNumberOfNeighbours (20);
//     reg.setInputCloud (laserCloudNgAftEe);
//     reg.setInputNormals (normals);
//     reg.setSmoothnessThreshold (9.0 / 180.0 * M_PI);
//     reg.setCurvatureThreshold (7.0);

//     std::vector <pcl::PointIndices> clusters;
//     reg.extract (clusters);

//     pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftReg(new pcl::PointCloud<pcl::PointXYZI>());
//     //创建一个模型参数对象，用于记录结果
//     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//     //inliers表示误差能容忍的点 记录的是点云的序号
//     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//     // 创建一个分割器
//     pcl::SACSegmentation<pcl::PointXYZI> seg;
//     // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
//     seg.setOptimizeCoefficients(true);
//     // Mandatory-设置目标几何形状areSameVehicle
//     seg.setModelType(pcl::SACMODEL_PLANE);
//     //分割方法：随机采样法
//     seg.setMethodType(pcl::SAC_RANSAC);
//     //设置误差容忍范围，也就是我说过的阈值
//     seg.setDistanceThreshold(0.075);

//     colorid = 0;
//     for (size_t i = 0 ; i < clusters.size() ; ++i)
//     {
//         Plane plane;
//         pcl::PointCloud<pcl::PointXYZI>::Ptr pointsOnPlane(new pcl::PointCloud<pcl::PointXYZI>());
//         pointsOnPlane->points.clear();

//         for (size_t j = 0 ; j < clusters[i].indices.size() ; ++j)
//         {
//             pcl::PointXYZI tmpPoint;
//             tmpPoint.x = laserCloudNgAftEe->points[clusters[i].indices[j]].x;
//             tmpPoint.y = laserCloudNgAftEe->points[clusters[i].indices[j]].y;
//             tmpPoint.z = laserCloudNgAftEe->points[clusters[i].indices[j]].z;
//             tmpPoint.intensity = i;
//             pointsOnPlane->points.push_back(tmpPoint);
//             laserCloudNgAftReg->points.push_back(tmpPoint);
//         }

//         //输入点云
//         seg.setInputCloud(pointsOnPlane);
//         //分割点云，获得平面和法向量
//         seg.segment(*inliers, *coefficients);

//         double planen[4];


//         planen[0] = coefficients->values[0];
//         planen[1] = coefficients->values[1];
//         planen[2] = coefficients->values[2];
//         planen[3] = -1.0 * coefficients->values[3];

// //        std::cout<<"planen[0] "<<planen[0]<<" planen[1] "<<planen[1]<<" planen[2] "<<planen[2]<<" planen[3] "<<planen[3]<<std::endl;
// //        std::cout<<planen[0]*planen[0]+planen[1]*planen[1]+planen[2]*planen[2]<<std::endl;

//         plane.v3normal = Vector3f(planen[0], planen[1], planen[2]);
//         plane.d = planen[3];
//         pcl::copyPointCloud(*pointsOnPlane, *plane.planePointCloudPtr);

//         float colors[3] = {0.1,0.5,0.85};

//         pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudInlier(new pcl::PointCloud<pcl::PointXYZI>());

//         for (std::vector<int>::const_iterator pit = inliers->indices.begin (); pit != inliers->indices.end (); ++pit)
//         {
//             pointsOnPlane->points[*pit].intensity = colorid;
//             laserCloudInlier->points.push_back (pointsOnPlane->points[*pit]);
//         }

//         pcl::copyPointCloud(*laserCloudInlier, *plane.InplanePointCloudOriPtr);

//         colorid++;


//         plane.calcConvexHull(plane.planePointCloudPtr,planen);

//         // plane.forcePtsLayOnPlane();

//         plane.computeMassCenterAndArea();
//         plane.calcElongationAndPpalDir();

//         plane.areaVoxels= plane.planePointCloudPtr->size() * 0.0025;

//         mapToShow += *plane.planePointCloudPtr;

//         detectedLocalPlanes.push_back(plane);
//     }

//     observedPlanes.clear();
//     vPlanes.clear();
//     for (size_t i = 0; i < detectedLocalPlanes.size (); i++)
//     {
//         detectedLocalPlanes[i].id = vPlanes.size();
//         for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
//         {
//             detectedLocalPlanes[i].neighborPlanes[*it] = 1;
//             vPlanes[*it].neighborPlanes[detectedLocalPlanes[i].id] = 1;
//         }
//         observedPlanes.insert(detectedLocalPlanes[i].id);
//         vPlanes.push_back(detectedLocalPlanes[i]);
//     }

//     std::cout<<"vPlanes.size() "<<vPlanes.size()<<std::endl;

//     maps->vPlanes = vPlanes;

//     LiPMatch_hd = createThreadFromObjectMethod(this,&LiPMatch::run);

//     keyframe_vec.clear();

//     map_rfn.set_down_sample_resolution( 0.75 );

// //    laserCloudOri_m1 += *laserCloudNgAftEe;
//     laserCloudOri_m1 += mapToShow;

// //    laserCloudOri_m1 += *map_structure_points;

//     pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterTargetMap;
//     downSizeFilterTargetMap.setLeafSize(0.50,0.50,0.50);
//     downSizeFilterTargetMap.setInputCloud(laserCloudOri_m1.makeShared());
//     downSizeFilterTargetMap.filter(laserCloudOri_m1);

// }




LiPMatch::LiPMatch() : LiPMatch_stop(false), LiPMatch_finished(false)
{
    std::cout<<"loading structure points ..."<<std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZI>("/home/jjwen/software/catkin_rangenet_ws/src/map_structure_points.pcd", *map_structure_points);
    std::cout<<"loading vehicle points ..."<<std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZI>("/home/jjwen/software/catkin_rangenet_ws/src/map_vehicle_points.pcd", *map_vehicle_points);
    std::cout<<"loading cylinder points ..."<<std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZI>("/home/jjwen/software/catkin_rangenet_ws/src/map_cylinder_points.pcd", *map_cylinder_points);

    vector<Vehicle> detectedLocalVehicles;
    vector<Pole> detectedLocalPoles;
    vector<Plane> detectedLocalPlanes;

    pcl::PCA< pcl::PointXYZI > pca;
    //聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> cluster_indices_vehicle;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_vehicle;
    ec_vehicle.setClusterTolerance (0.205);
    ec_vehicle.setMinClusterSize (80);
    ec_vehicle.setMaxClusterSize (7500);
    ec_vehicle.setSearchMethod (tree);
    ec_vehicle.setInputCloud(map_vehicle_points);
    ec_vehicle.extract (cluster_indices_vehicle);

    int colorid = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_vehicle.begin (); it != cluster_indices_vehicle.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_vehicle(new pcl::PointCloud<pcl::PointXYZI>());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            map_vehicle_points->points[*pit].intensity = colorid;
            laserCloudNgAftEe_vehicle->points.push_back (map_vehicle_points->points[*pit]);
        }
        pca.setInputCloud(laserCloudNgAftEe_vehicle);
        Eigen::VectorXf eigenVal = pca.getEigenValues();

        if (eigenVal[2] / eigenVal[0] < 0.01)
            continue;

        if (eigenVal[1] / eigenVal[0] < 0.01)
            continue;

        mapToShow += *laserCloudNgAftEe_vehicle;

        colorid++;

        Vehicle vc;
        vc.VehiclePointCloudPtr = laserCloudNgAftEe_vehicle;
        vc.calcCenterAndElongation();
        detectedLocalVehicles.push_back(vc);
    }

    observedVehicles.clear();
    vVehicles.clear();

    for (size_t i = 0; i < detectedLocalVehicles.size (); i++)
    {
        detectedLocalVehicles[i].id = vVehicles.size();
        for(set<unsigned>::iterator it = observedVehicles.begin(); it != observedVehicles.end(); it++)
        {
            detectedLocalVehicles[i].neighborVehicles[*it] = 1;
            vVehicles[*it].neighborVehicles[detectedLocalVehicles[i].id] = 1;
        }
        observedVehicles.insert(detectedLocalVehicles[i].id);
        vVehicles.push_back(detectedLocalVehicles[i]);
    }

    std::cout<<"vVehicles.size() "<<vVehicles.size()<<std::endl;
    maps->vVehicles = vVehicles;

    //聚类
    std::vector<pcl::PointIndices> cluster_indices_nature;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_nature;
    ec_nature.setClusterTolerance (0.21);
    ec_nature.setMinClusterSize (20);
    ec_nature.setMaxClusterSize (500);
    ec_nature.setSearchMethod (tree);
    ec_nature.setInputCloud (map_cylinder_points);
    ec_nature.extract (cluster_indices_nature);

    colorid = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_nature.begin (); it != cluster_indices_nature.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_nature(new pcl::PointCloud<pcl::PointXYZI>());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            map_cylinder_points->points[*pit].intensity = colorid;
            laserCloudNgAftEe_nature->points.push_back (map_cylinder_points->points[*pit]);
        }
        pca.setInputCloud(laserCloudNgAftEe_nature);
        Eigen::VectorXf eigenVal = pca.getEigenValues();

        if (eigenVal[1] / eigenVal[0] > 0.135)
            continue;

        mapToShow += *laserCloudNgAftEe_nature;
        colorid++;

        Pole vc;
        vc.PolePointCloudPtr = laserCloudNgAftEe_nature;
        vc.calcCenterAndElongation();
        detectedLocalPoles.push_back(vc);
    }

    observedPoles.clear();
    vPoles.clear();

    for (size_t i = 0; i < detectedLocalPoles.size (); i++)
    {
        detectedLocalPoles[i].id = vPoles.size();

        // Update co-visibility graph
        for(set<unsigned>::iterator it = observedPoles.begin(); it != observedPoles.end(); it++)
        {
            detectedLocalPoles[i].neighborPoles[*it] = 1;
            vPoles[*it].neighborPoles[detectedLocalPoles[i].id] = 1;
        }
        observedPoles.insert(detectedLocalPoles[i].id);
        vPoles.push_back(detectedLocalPoles[i]);
    }

    std::cout<<"vPoles.size() "<<vPoles.size()<<std::endl;
    maps->vPoles = vPoles;

//    pcl::VoxelGrid<pcl::PointXYZI> gridMap;
//    gridMap.setLeafSize(0.8,0.8,0.8);
//    gridMap.setInputCloud(map_structure_points);
//    gridMap.filter(*map_structure_points);

    //聚类
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.60);
    ec.setMinClusterSize (60);
    ec.setMaxClusterSize (150000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (map_structure_points);
    ec.extract (cluster_indices);
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe(new pcl::PointCloud<pcl::PointXYZI>());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            laserCloudNgAftEe->points.push_back (map_structure_points->points[*pit]);
        }
    }

    //区域增长法提取激光点云中的平面
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (laserCloudNgAftEe);
    normal_estimator.setKSearch (10);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize (60);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (20);
    reg.setInputCloud (laserCloudNgAftEe);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (9.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (7.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftReg(new pcl::PointCloud<pcl::PointXYZI>());
    //创建一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 创建一个分割器
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setOptimizeCoefficients(true);
    // Mandatory-设置目标几何形状areSameVehicle
    seg.setModelType(pcl::SACMODEL_PLANE);
    //分割方法：随机采样法
    seg.setMethodType(pcl::SAC_RANSAC);
    //设置误差容忍范围，也就是我说过的阈值
    seg.setDistanceThreshold(0.075);

    colorid = 0;
    for (size_t i = 0 ; i < clusters.size() ; ++i)
    {
        Plane plane;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointsOnPlane(new pcl::PointCloud<pcl::PointXYZI>());
        pointsOnPlane->points.clear();

        for (size_t j = 0 ; j < clusters[i].indices.size() ; ++j)
        {
            pcl::PointXYZI tmpPoint;
            tmpPoint.x = laserCloudNgAftEe->points[clusters[i].indices[j]].x;
            tmpPoint.y = laserCloudNgAftEe->points[clusters[i].indices[j]].y;
            tmpPoint.z = laserCloudNgAftEe->points[clusters[i].indices[j]].z;
            tmpPoint.intensity = i;
            pointsOnPlane->points.push_back(tmpPoint);
            laserCloudNgAftReg->points.push_back(tmpPoint);
        }

        //输入点云
        seg.setInputCloud(pointsOnPlane);
        //分割点云，获得平面和法向量
        seg.segment(*inliers, *coefficients);

        double planen[4];


        planen[0] = coefficients->values[0];
        planen[1] = coefficients->values[1];
        planen[2] = coefficients->values[2];
        planen[3] = -1.0 * coefficients->values[3];

//        std::cout<<"planen[0] "<<planen[0]<<" planen[1] "<<planen[1]<<" planen[2] "<<planen[2]<<" planen[3] "<<planen[3]<<std::endl;
//        std::cout<<planen[0]*planen[0]+planen[1]*planen[1]+planen[2]*planen[2]<<std::endl;

        plane.v3normal = Vector3f(planen[0], planen[1], planen[2]);
        plane.d = planen[3];
        pcl::copyPointCloud(*pointsOnPlane, *plane.planePointCloudPtr);

        float colors[3] = {0.1,0.5,0.85};

        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudInlier(new pcl::PointCloud<pcl::PointXYZI>());

        for (std::vector<int>::const_iterator pit = inliers->indices.begin (); pit != inliers->indices.end (); ++pit)
        {
            pointsOnPlane->points[*pit].intensity = colorid;
            laserCloudInlier->points.push_back (pointsOnPlane->points[*pit]);
        }

        pcl::copyPointCloud(*laserCloudInlier, *plane.InplanePointCloudOriPtr);

        colorid++;


        plane.calcConvexHull(plane.planePointCloudPtr,planen);

        // plane.forcePtsLayOnPlane();

        plane.computeMassCenterAndArea();
        plane.calcElongationAndPpalDir();

        plane.areaVoxels= plane.planePointCloudPtr->size() * 0.0025;

        mapToShow += *plane.planePointCloudPtr;

        detectedLocalPlanes.push_back(plane);
    }

    observedPlanes.clear();
    vPlanes.clear();
    for (size_t i = 0; i < detectedLocalPlanes.size (); i++)
    {
        detectedLocalPlanes[i].id = vPlanes.size();
        for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
        {
            detectedLocalPlanes[i].neighborPlanes[*it] = 1;
            vPlanes[*it].neighborPlanes[detectedLocalPlanes[i].id] = 1;
        }
        observedPlanes.insert(detectedLocalPlanes[i].id);
        vPlanes.push_back(detectedLocalPlanes[i]);
    }

    std::cout<<"vPlanes.size() "<<vPlanes.size()<<std::endl;

    maps->vPlanes = vPlanes;

    LiPMatch_hd = createThreadFromObjectMethod(this,&LiPMatch::run);

    keyframe_vec.clear();

    map_rfn.set_down_sample_resolution( 0.75 );

//    laserCloudOri_m1 += *laserCloudNgAftEe;
    laserCloudOri_m1 += mapToShow;

//    laserCloudOri_m1 += *map_structure_points;

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterTargetMap;
    downSizeFilterTargetMap.setLeafSize(0.50,0.50,0.50);
    downSizeFilterTargetMap.setInputCloud(laserCloudOri_m1.makeShared());
    downSizeFilterTargetMap.filter(laserCloudOri_m1);

}


//自定义排序函数  
static bool sortFun(const std::map<unsigned, unsigned> &p1, const std::map<unsigned, unsigned> &p2)
{
    return p1.size() > p2.size(); //升序排列  
}




void LiPMatch::detectPlanesCloud( m_keyframe &c_keyframe, int keyFrameCount)
{
    std::cout<<"new keyframe coming..."<<std::endl;
    static pcl::VoxelGrid<pcl::PointXYZI> grid_1;
    grid_1.setLeafSize(0.40,0.40,0.40);

    grid_1.setInputCloud(c_keyframe.structurelaserCloud);
    grid_1.filter (*c_keyframe.structurelaserCloud);

    grid_1.setLeafSize(0.20,0.20,0.20);
    grid_1.setInputCloud(c_keyframe.vehiclelaserCloud);
    grid_1.filter (*c_keyframe.vehiclelaserCloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(c_keyframe.vehiclelaserCloud);
    sor.setMeanK(10);   //设置在进行统计时考虑查询点邻近点数
    sor.setStddevMulThresh(0.5);
    sor.setNegative(false);
    sor.filter(*c_keyframe.vehiclelaserCloud);

    grid_1.setLeafSize(0.20,0.20,0.20);
    grid_1.setInputCloud(c_keyframe.naturelaserCloud);
    grid_1.filter (*c_keyframe.naturelaserCloud);

    double time_behinSeg = pcl::getTime();


    int colorid = 0;
    //聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr treeVehicle (new pcl::search::KdTree<pcl::PointXYZI>);
    treeVehicle->setInputCloud (c_keyframe.vehiclelaserCloud);
    std::vector<pcl::PointIndices> cluster_indices_vehicle;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_vehicle;
    ec_vehicle.setClusterTolerance (0.205);
//    ec.setMinClusterSize (100);
    ec_vehicle.setMinClusterSize (75);
    ec_vehicle.setMaxClusterSize (7500);
    ec_vehicle.setSearchMethod (treeVehicle);
    ec_vehicle.setInputCloud (c_keyframe.vehiclelaserCloud);
    ec_vehicle.extract (cluster_indices_vehicle);
    pcl::PCA< pcl::PointXYZI > pca;

    vector<Vehicle> detectedLocalVehicles;

    pcl::PointCloud<pcl::PointXYZI>::Ptr vehiclesPointCloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_vehicle.begin (); it != cluster_indices_vehicle.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_vehicle(new pcl::PointCloud<pcl::PointXYZI>());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            c_keyframe.vehiclelaserCloud->points[*pit].intensity = colorid;
            laserCloudNgAftEe_vehicle->points.push_back (c_keyframe.vehiclelaserCloud->points[*pit]);
        }
        pca.setInputCloud(laserCloudNgAftEe_vehicle);
        Eigen::VectorXf eigenVal = pca.getEigenValues();

//        if (eigenVal[2] / eigenVal[0] < 0.005)
//            continue;

        if (eigenVal[2] / eigenVal[0] < 0.01)
            continue;

        if (eigenVal[1] / eigenVal[0] < 0.01)
            continue;

        *vehiclesPointCloud += *laserCloudNgAftEe_vehicle;

        colorid++;

//        std::cout<<eigenVal[0]<<" "<<eigenVal[1]<<" "<<eigenVal[2]<<std::endl;

        Vehicle vc;
        vc.VehiclePointCloudPtr = laserCloudNgAftEe_vehicle;
        vc.calcCenterAndElongation();

        detectedLocalVehicles.push_back(vc);

    }

    // laserCloudOri_m2 += *vehiclesPointCloud;


    observedVehicles.clear();
    vVehicles.clear();

    for (size_t i = 0; i < detectedLocalVehicles.size (); i++)
    {
        detectedLocalVehicles[i].id = vVehicles.size();

        // Update co-visibility graph
        for(set<unsigned>::iterator it = observedVehicles.begin(); it != observedVehicles.end(); it++)
        {
            detectedLocalVehicles[i].neighborVehicles[*it] = 1;
            vVehicles[*it].neighborVehicles[detectedLocalVehicles[i].id] = 1;
        }
        observedVehicles.insert(detectedLocalVehicles[i].id);
        vVehicles.push_back(detectedLocalVehicles[i]);
    }


    std::shared_ptr<Maps_keyframe<float>> mk = std::make_shared<Maps_keyframe<float>>();

    std::cout<<"vVehicles.size() "<<vVehicles.size()<<std::endl;

    mk->vVehicles = vVehicles;

    //聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr treeNature (new pcl::search::KdTree<pcl::PointXYZI>);
    treeNature->setInputCloud (c_keyframe.naturelaserCloud);
    std::vector<pcl::PointIndices> cluster_indices_nature;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_nature;
    ec_nature.setClusterTolerance (0.21);
//    ec.setMinClusterSize (100);
    ec_nature.setMinClusterSize (20);
    ec_nature.setMaxClusterSize (500);
    ec_nature.setSearchMethod (treeNature);
    ec_nature.setInputCloud (c_keyframe.naturelaserCloud);
    ec_nature.extract (cluster_indices_nature);
    pcl::PCA< pcl::PointXYZI > pca1;

    vector<Pole> detectedLocalPoles;

    colorid = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_nature.begin (); it != cluster_indices_nature.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_nature(new pcl::PointCloud<pcl::PointXYZI>());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            c_keyframe.naturelaserCloud->points[*pit].intensity = colorid;
            laserCloudNgAftEe_nature->points.push_back (c_keyframe.naturelaserCloud->points[*pit]);
        }
        pca1.setInputCloud(laserCloudNgAftEe_nature);
        Eigen::VectorXf eigenVal = pca1.getEigenValues();

        if (eigenVal[1] / eigenVal[0] > 0.135)
            continue;

        colorid++;

        Pole vc;
        vc.PolePointCloudPtr = laserCloudNgAftEe_nature;
        vc.keyFrameId = keyFrameCount;
        vc.calcCenterAndElongation();

        detectedLocalPoles.push_back(vc);

    }

    observedPoles.clear();
    vPoles.clear();

    for (size_t i = 0; i < detectedLocalPoles.size (); i++)
    {
        detectedLocalPoles[i].id = vPoles.size();

        // Update co-visibility graph
        for(set<unsigned>::iterator it = observedPoles.begin(); it != observedPoles.end(); it++)
        {
            detectedLocalPoles[i].neighborPoles[*it] = 1;
            vPoles[*it].neighborPoles[detectedLocalPoles[i].id] = 1;
        }
        observedPoles.insert(detectedLocalPoles[i].id);
        vPoles.push_back(detectedLocalPoles[i]);
    }

    std::cout<<"vPoles.size() "<<vPoles.size()<<std::endl;

    mk->vPoles = vPoles;


    /////////////////////

    //聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (c_keyframe.structurelaserCloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.65);
//    ec.setMinClusterSize (100);
    ec.setMinClusterSize (70);
    ec.setMaxClusterSize (150000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (c_keyframe.structurelaserCloud);
    ec.extract (cluster_indices);
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe(new pcl::PointCloud<pcl::PointXYZI>());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            laserCloudNgAftEe->points.push_back (c_keyframe.structurelaserCloud->points[*pit]);
        }
    }

//    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe(new pcl::PointCloud<pcl::PointXYZI>());
//    laserCloudNgAftEe = c_keyframe.structurelaserCloud;

    //区域增长法提取激光点云中的平面
    pcl::search::Search<pcl::PointXYZI>::Ptr treeRg (new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (treeRg);
    normal_estimator.setInputCloud (laserCloudNgAftEe);
    normal_estimator.setKSearch (10);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize (70);
//    reg.setMinClusterSize (120);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (treeRg);
    reg.setNumberOfNeighbours (20);
    reg.setInputCloud (laserCloudNgAftEe);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (9.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (7.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftReg(new pcl::PointCloud<pcl::PointXYZI>());
    //创建一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 创建一个分割器
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setOptimizeCoefficients(true);
    // Mandatory-设置目标几何形状areSameVehicle
    seg.setModelType(pcl::SACMODEL_PLANE);
    //分割方法：随机采样法
    seg.setMethodType(pcl::SAC_RANSAC);
    //设置误差容忍范围，也就是我说过的阈值
    seg.setDistanceThreshold(0.075);

    vector<Plane> detectedLocalPlanes;

    pcl::PointCloud<pcl::PointXYZI>::Ptr curPlanePoints(new pcl::PointCloud<pcl::PointXYZI>());

    for (size_t i = 0 ; i < clusters.size() ; ++i)
    {
        Plane plane;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointsOnPlane(new pcl::PointCloud<pcl::PointXYZI>());
        pointsOnPlane->points.clear();

        for (size_t j = 0 ; j < clusters[i].indices.size() ; ++j)
        {
            pcl::PointXYZI tmpPoint;
            tmpPoint.x = laserCloudNgAftEe->points[clusters[i].indices[j]].x;
            tmpPoint.y = laserCloudNgAftEe->points[clusters[i].indices[j]].y;
            tmpPoint.z = laserCloudNgAftEe->points[clusters[i].indices[j]].z;
            tmpPoint.intensity = i;
            pointsOnPlane->points.push_back(tmpPoint);
            laserCloudNgAftReg->points.push_back(tmpPoint);
        }

        //输入点云
        seg.setInputCloud(pointsOnPlane);
        //分割点云，获得平面和法向量
        seg.segment(*inliers, *coefficients);

        double planen[4];

        planen[0] = coefficients->values[0];
        planen[1] = coefficients->values[1];
        planen[2] = coefficients->values[2];
        planen[3] = -1.0 * coefficients->values[3];

        plane.v3normal = Vector3f(planen[0], planen[1], planen[2]);
        plane.d = planen[3];
        pcl::copyPointCloud(*pointsOnPlane, *plane.planePointCloudPtr);

        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudInlier(new pcl::PointCloud<pcl::PointXYZI>());

        for (std::vector<int>::const_iterator pit = inliers->indices.begin (); pit != inliers->indices.end (); ++pit)
        {
            laserCloudInlier->points.push_back (pointsOnPlane->points[*pit]);
        }

        pcl::copyPointCloud(*laserCloudInlier, *plane.InplanePointCloudOriPtr);


        plane.calcConvexHull(plane.planePointCloudPtr,planen);

        // plane.forcePtsLayOnPlane();


        plane.computeMassCenterAndArea();
        plane.calcElongationAndPpalDir();

        plane.areaVoxels= plane.planePointCloudPtr->size() * 0.0025;

        *curPlanePoints += *plane.planePointCloudPtr;

        detectedLocalPlanes.push_back(plane);
    }

//    laserCloudOri_m2.points.clear();
//    laserCloudOri_m2 += *curPlanePoints;

    // Merge detected planes with previous ones if they are the same
    observedPlanes.clear();
    vPlanes.clear();

    for (size_t i = 0; i < detectedLocalPlanes.size (); i++)
    {
        detectedLocalPlanes[i].id = vPlanes.size();

        // Update co-visibility graph
        for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
        {
            detectedLocalPlanes[i].neighborPlanes[*it] = 1;
            vPlanes[*it].neighborPlanes[detectedLocalPlanes[i].id] = 1;
        }
        observedPlanes.insert(detectedLocalPlanes[i].id);
        vPlanes.push_back(detectedLocalPlanes[i]);
    }

    std::cout<<"vPlanes.size() ";
    std::cout<<vPlanes.size()<<std::endl;

    mk->vPlanes = vPlanes;

//    std::cout<<"mk->m_accumulated_g_pc.size() "<<mk->m_accumulated_g_pc.size()<<std::endl;

    mk->m_accumulated_structure_pc = *c_keyframe.structurelaserCloud;
    mk->m_accumulated_vehicle_pc = *c_keyframe.vehiclelaserCloud;


    keyframe_vec.push_back( mk );

    std::shared_ptr<Maps_keyframe<float>>& last_keyframe = keyframe_vec.back();

    std::map<unsigned, unsigned> bestMatch; bestMatch.clear();

    Subgraph currentSubgraph1(maps->vPlanes, 0);
    Subgraph targetSubgraph1(last_keyframe->vPlanes, 0);

    int unaycount;
    std::map<unsigned, unsigned> resultingMatch = matcher.compareSubgraphs(currentSubgraph1, targetSubgraph1,
                                                                           unaycount);

    if (matcher.allMatchGroups.size() < 1)
        return;

    std::cout << "matcher.allMatchGroups.size() " << matcher.allMatchGroups.size() << std::endl;

    int allMatchedSize;

    int greatMatchedSize = 0;
    std::map<unsigned, unsigned> greatestMatch;
    std::map<unsigned, unsigned> allgreatestMatch;

//    laserCloudOri_m2.points.clear();
//    laserCloudOri_m2.width = 0;
//    laserCloudOri_m2.height = 0;
    
    //KITTI
    std::map<unsigned, unsigned> bestMatchVehicle;
    bestMatchVehicle.clear();

    std::map<unsigned, unsigned> bestMatchPole;
    bestMatchPole.clear();

    std::vector<std::map<unsigned, unsigned> > AllSamebestMatch; AllSamebestMatch.clear();
    std::vector<int> AllSamebestMatchSize; AllSamebestMatchSize.clear();

    /******************/

    // std::cout<<"vVehicles.size() "<<vVehicles.size()<<std::endl;

    // if ()


    laserCloudOri_m2.points.clear();
    laserCloudOri_m2.width = 0;
    laserCloudOri_m2.height = 0;

    if (last_keyframe->vVehicles.size() > 6)
    {
        Subgraph currentSubgraphv(maps->vVehicles, 0);
        Subgraph targetSubgraphv(last_keyframe->vVehicles, 0);

        std::cout<<"begin compareSubgraphsVehicleWithoutPlaneRef"<<std::endl;
        int tunaycount;
        std::map<unsigned, unsigned> resultingMatchtvt = matcher.compareSubgraphsVehicleWithoutPlaneRef(currentSubgraphv, targetSubgraphv, tunaycount);
        std::cout<<"resultingMatchtvt.size() "<<resultingMatchtvt.size()<<std::endl;

        if (resultingMatchtvt.size() >= 7)
        {
            for (map<unsigned, unsigned>::iterator it = resultingMatchtvt.begin(); it != resultingMatchtvt.end(); it++)
            {
                laserCloudOri_m2 += *maps->vVehicles[it->first].VehiclePointCloudPtr;
                laserCloudOri_m2 += *last_keyframe->vVehicles[it->second].VehiclePointCloudPtr;
            }

            std::cout<<"localization successful !"<<std::endl;
            return;
        }
    }


    if (last_keyframe->vPoles.size() > 6)
    {
        Subgraph currentSubgraphcy(maps->vPoles, 0);
        Subgraph targetSubgraphcy(last_keyframe->vPoles, 0);
        
        std::cout<<"begin compareSubgraphsPoleWithoutPlaneRef"<<std::endl;
        int tunaycountc;
        std::map<unsigned, unsigned> resultingMatchtvtc = matcher.compareSubgraphsPoleWithoutPlaneRef(currentSubgraphcy, targetSubgraphcy, tunaycountc);
        std::cout<<"resultingMatchtvtc.size() "<<resultingMatchtvtc.size()<<std::endl;
        
        if (resultingMatchtvtc.size() >= 7)
        {
            for (map<unsigned, unsigned>::iterator it = resultingMatchtvtc.begin(); it != resultingMatchtvtc.end(); it++)
            {
                laserCloudOri_m2 += *maps->vPoles[it->first].PolePointCloudPtr;
                laserCloudOri_m2 += *last_keyframe->vPoles[it->second].PolePointCloudPtr;
            }
            std::cout<<"localization successful !"<<std::endl;
            return;
        }
    }

    /******************/
    // for (size_t i = 0 ; i < matcher.allMatchGroups.size() ; ++i)
    // {
    //     std::cout<<matcher.allMatchGroups[i].size()<<std::endl;
    // }

    // std::cout<<"======================"<<std::endl;
	
    sort(matcher.allMatchGroups.begin(), matcher.allMatchGroups.end(), sortFun);
    
    // for (size_t i = 0 ; i < matcher.allMatchGroups.size() ; ++i)
    // {
    //     std::cout<<matcher.allMatchGroups[i].size()<<std::endl;
    // }

    int maxSize = matcher.allMatchGroups[0].size();

    for (size_t i = 0 ; i < matcher.allMatchGroups.size() ; ++i)
    {
        if (matcher.allMatchGroups[i].size() < (maxSize-1) )
            break;

        bestMatch = matcher.allMatchGroups[i];

        if (bestMatch.size() < 3)
            continue;

        bestMatchVehicle.clear();
        bestMatchPole.clear();

        if (last_keyframe->vVehicles.size() > 0)
        {
            std::vector <Eigen::Vector3f> kvc;
            std::vector <Eigen::Vector3f> lvc;
            std::vector <Eigen::Vector3f> kvn;
            std::vector <Eigen::Vector3f> lvn;
            
            for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++) {
                kvc.push_back(maps->vPlanes[it->first].v3center);
                lvc.push_back(last_keyframe->vPlanes[it->second].v3center);
                kvn.push_back(maps->vPlanes[it->first].v3normal);
                lvn.push_back(last_keyframe->vPlanes[it->second].v3normal);
            }
            
            Subgraph currentSubgraph2(maps->vVehicles, 0);
            Subgraph targetSubgraph2(last_keyframe->vVehicles, 0);
            
            unaycount = 0;
            std::map<unsigned, unsigned> resultingMatch1 = matcher.compareSubgraphsVehiclePlaneRef(currentSubgraph2,
                                                                                                   targetSubgraph2,
                                                                                                   unaycount, kvc,
                                                                                                   lvc, kvn, lvn);
                                                                                                   
            std::vector <Eigen::Vector3f> vkvc;
            std::vector <Eigen::Vector3f> vlvc;

            for (map<unsigned, unsigned>::iterator it = resultingMatch1.begin(); it != resultingMatch1.end(); it++) {
                vkvc.push_back(maps->vVehicles[it->first].v3center);
                vlvc.push_back(last_keyframe->vVehicles[it->second].v3center);}
                
            bestMatchVehicle = resultingMatch1;

            if (last_keyframe->vPoles.size() > 0)
            {
                Subgraph currentSubgraph3(maps->vPoles, 0);
                Subgraph targetSubgraph3(last_keyframe->vPoles, 0);

                matcher.v_kvc = vkvc;
                matcher.v_lvc = vlvc;

                unaycount = 0;
                std::map<unsigned, unsigned> resultingMatch2 = matcher.compareSubgraphsPolePlaneRef(currentSubgraph3,
                                                                                            targetSubgraph3, unaycount,
                                                                                            kvc,
                                                                                            lvc, kvn, lvn);
                bestMatchPole = resultingMatch2;
            }
        }


//        std::cout << "bestMatchPole.size() " << bestMatchPole.size() << std::endl;
//        std::cout << "bestMatchVehicle.size() " << bestMatchVehicle.size() << std::endl;

//    laserCloudOri_m2.height = 1;
//    laserCloudOri_m2.width = vehiclesPointCloud->points.size();
//    laserCloudOri_m2 = *vehiclesPointCloud;


//    laserCloudOri_m2.points.clear();
//
//        if (bestMatch.size() > 0)
//            for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++) {
//                laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
//                laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
//            }

//    std::cout<<"matcher.allwinnerMatch.size() "<<matcher.allwinnerMatch.size()<<std::endl;
//    if (matcher.allwinnerMatch.size() > 0)
//        for (map<unsigned, unsigned>::iterator it = matcher.allwinnerMatch.begin(); it != matcher.allwinnerMatch.end(); it++)
//        {
//            laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
//
//            for (size_t k = 0 ; k < last_keyframe->vPlanes[it->second].planePointCloudPtr->points.size() ; ++k)
//            {
//                last_keyframe->vPlanes[it->second].planePointCloudPtr->points[k].z = last_keyframe->vPlanes[it->second].planePointCloudPtr->points[k].z + 10.0;
//            }
//            laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
//        }

//    pcl::copyPointCloud(*vehiclesPointCloud,laserCloudOri_m2);

//    laserCloudOri_m2 += *map_cylinder_points;
//    laserCloudOri_m2 = *map_vehicle_points;

//    laserCloudOri_m2 += *map_structure_points;

//    for (size_t k = 0 ; k < vehiclesPointCloud->points.size() ; ++k)
//    {
//        vehiclesPointCloud->points[k].z = vehiclesPointCloud->points[k].z + 10.0;
//    }
//
//    for (size_t k = 0 ; k < curPlanePoints->points.size() ; ++k)
//    {
//        curPlanePoints->points[k].z = curPlanePoints->points[k].z + 10.0;
//    }
//    laserCloudOri_m2 += *curPlanePoints;


//         std::vector<int> firstINdexes;
//         for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++) {
//             firstINdexes.push_back(it->first);
//         }

//         pcl::PointCloud <pcl::PointXYZI> normPOints;
//         for (int it = 0; it < firstINdexes.size(); ++it) {
//             pcl::PointXYZI phe;
//             phe.x = maps->vPlanes[firstINdexes[it]].v3normal(0);
//             phe.y = maps->vPlanes[firstINdexes[it]].v3normal(1);
//             phe.z = maps->vPlanes[firstINdexes[it]].v3normal(2);
//             normPOints.points.push_back(phe);
//         }

//         //聚类
//         pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree <pcl::PointXYZI>);
//         tree1->setInputCloud(normPOints.makeShared());
//         std::vector <pcl::PointIndices> cluster_indices2;
//         pcl::EuclideanClusterExtraction <pcl::PointXYZI> ec1;
//         ec1.setClusterTolerance(0.15);
//         ec1.setMinClusterSize(1);
//         ec1.setMaxClusterSize(100);
//         ec1.setSearchMethod(tree1);
//         ec1.setInputCloud(normPOints.makeShared());
//         ec1.extract(cluster_indices2);

// //        std::cout << "cluster_indices2.size() " << cluster_indices2.size() << std::endl;

//         if (cluster_indices2.size() < 2) {
//             continue;
//         }

//    if (bestMatch.size() < 3)
//        return;

        allMatchedSize = bestMatch.size();

        allMatchedSize += bestMatchVehicle.size();

        allMatchedSize += bestMatchPole.size();

//    if (bestMatchVehicle.size() > 1)
//    {
//        allMatchedSize += bestMatchVehicle.size();
//    }
//    if (bestMatchPole.size() > 1)
//    {
//        allMatchedSize += bestMatchPole.size();
//    }


//    std::cout<<"comp time "<<aveTime/(pointSearchInd.size())<<std::endl;

    //    std::cout << "allMatchedSize " << allMatchedSize << std::endl;

    //    std::cout << "matcher.allMatchGroups_height[i] " << matcher.allMatchGroups_height[i] << std::endl;
    //    std::cout << "matcher.allMatchGroups_normal[i] " << matcher.allMatchGroups_normal[i] << std::endl;
    //    std::cout << "matcher.allMatchGroups_dist_centers[i] " << matcher.allMatchGroups_dist_centers[i] << std::endl;

    //    if (allMatchedSize > 5) {

    //        for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
    //        {
    //            laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
               
    //            pcl::PointCloud<pcl::PointXYZI>::Ptr tempz(new pcl::PointCloud<pcl::PointXYZI>());
    //            *tempz = *last_keyframe->vPlanes[it->second].planePointCloudPtr;
    //            for (size_t k = 0 ; k < tempz->points.size() ; ++k)
    //            {
    //                tempz->points[k].z = tempz->points[k].z + 10.0;
    //            }
    //            laserCloudOri_m2 += *tempz;
    //        }

    //     //    std::cout << "matcher.allMatchGroups_height[i] " << matcher.allMatchGroups_height[i] << std::endl;
    //     //    std::cout << "matcher.allMatchGroups_normal[i] " << matcher.allMatchGroups_normal[i] << std::endl;
    //     //    std::cout << "matcher.allMatchGroups_dist_centers[i] " << matcher.allMatchGroups_dist_centers[i] << std::endl;

    //     //    for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
    //     //    {
    //     //        std::cout<<it->first<<"      "<<it->second<<std::endl;
    //     //    }

    //         for (auto itm : bestMatch)
    //         {
    //             allgreatestMatch.insert(itm);
    //         }

    //    }

        // std::cout<<greatMatchedSize<<std::endl;
        if (allMatchedSize >= greatMatchedSize)
        {
            greatMatchedSize = allMatchedSize;
            greatestMatch = bestMatch;

        }

        AllSamebestMatch.push_back(bestMatch);
        AllSamebestMatchSize.push_back(allMatchedSize);

    }

    allMatchedSize = greatMatchedSize;
    bestMatch = greatestMatch;

    // std::cout<<"allMatchedSize "<<allMatchedSize<<std::endl;
    // std::cout<<"bestMatch.size() "<<bestMatch.size()<<std::endl;

    // bestMatch.clear();
    // for (size_t i = 0 ; i < AllSamebestMatchSize.size() ; ++i)
    // {
    //     if (AllSamebestMatchSize[i] == greatMatchedSize)
    //     {
    //         for (auto item : AllSamebestMatch[i])
    //         {
    //             bestMatch.insert(item);
    //         }
    //     }
    // }

    // allMatchedSize = greatMatchedSize;

    std::cout<<"allMatchedSize "<<allMatchedSize<<std::endl;
    std::cout<<"bestMatch.size() "<<bestMatch.size()<<std::endl;





//    laserCloudOri_m2 += *map_structure_points;



//    if (bestMatch.size() < 1)
//        return;
//
//
//    std::vector<Eigen::Vector3f> kvc;
//    std::vector<Eigen::Vector3f> lvc;
//    std::vector<Eigen::Vector3f> kvn;
//    std::vector<Eigen::Vector3f> lvn;
//
//    for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
//    {
//        kvc.push_back(maps->vPlanes[it->first].v3center);
//        lvc.push_back(last_keyframe->vPlanes[it->second].v3center);
//        kvn.push_back(maps->vPlanes[it->first].v3normal);
//        lvn.push_back(last_keyframe->vPlanes[it->second].v3normal);
//    }
//
//
//    //KITTI
//    std::map<unsigned, unsigned> bestMatchVehicle; bestMatchVehicle.clear();
//
//    Subgraph currentSubgraph2(maps->vVehicles, 0);
//    Subgraph targetSubgraph2(last_keyframe->vVehicles, 0);
//
//    unaycount = 0;
//    std::map<unsigned, unsigned> resultingMatch1 = matcher.compareSubgraphsVehiclePlaneRef(currentSubgraph2, targetSubgraph2, unaycount, kvc,
//                                                                                                  lvc, kvn, lvn);
//
//    bestMatchVehicle = resultingMatch1;
//
//    std::map<unsigned, unsigned> bestMatchPole; bestMatchPole.clear();
//
//    Subgraph currentSubgraph3(maps->vPoles, 0);
//    Subgraph targetSubgraph3(last_keyframe->vPoles, 0);
//
//    unaycount = 0;
//    std::map<unsigned, unsigned> resultingMatch2 = matcher.compareSubgraphsPolePlaneRef(currentSubgraph3, targetSubgraph3, unaycount, kvc,
//                                                                                               lvc, kvn, lvn);
//
//    bestMatchPole = resultingMatch2;
//
//    std::cout<<"bestMatchPole.size() "<<bestMatchPole.size()<<std::endl;
//    std::cout<<"bestMatchVehicle.size() "<<bestMatchVehicle.size()<<std::endl;
//
////    laserCloudOri_m2.height = 1;
////    laserCloudOri_m2.width = vehiclesPointCloud->points.size();
////    laserCloudOri_m2 = *vehiclesPointCloud;
//
//


//    laserCloudOri_m2.points.clear();

//     if (bestMatch.size() > 0)
//         for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
//         {
//             laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
//             laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
//         }

////    std::cout<<"matcher.allwinnerMatch.size() "<<matcher.allwinnerMatch.size()<<std::endl;
////    if (matcher.allwinnerMatch.size() > 0)
////        for (map<unsigned, unsigned>::iterator it = matcher.allwinnerMatch.begin(); it != matcher.allwinnerMatch.end(); it++)
////        {
////            laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
////
////            for (size_t k = 0 ; k < last_keyframe->vPlanes[it->second].planePointCloudPtr->points.size() ; ++k)
////            {
////                last_keyframe->vPlanes[it->second].planePointCloudPtr->points[k].z = last_keyframe->vPlanes[it->second].planePointCloudPtr->points[k].z + 10.0;
////            }
////            laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
////        }
//
////    pcl::copyPointCloud(*vehiclesPointCloud,laserCloudOri_m2);
//
////    laserCloudOri_m2 += *map_cylinder_points;
////    laserCloudOri_m2 = *map_vehicle_points;
//
////    laserCloudOri_m2 += *map_structure_points;
//
////    for (size_t k = 0 ; k < vehiclesPointCloud->points.size() ; ++k)
////    {
////        vehiclesPointCloud->points[k].z = vehiclesPointCloud->points[k].z + 10.0;
////    }
////
////    for (size_t k = 0 ; k < curPlanePoints->points.size() ; ++k)
////    {
////        curPlanePoints->points[k].z = curPlanePoints->points[k].z + 10.0;
////    }
////    laserCloudOri_m2 += *curPlanePoints;
//
//    //KITTI
//    if (bestMatch.size() < 2) {
//        return;
//    }
//
//    std::vector<int> firstINdexes;
//    for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++) {
//        firstINdexes.push_back(it->first);
//    }
//
//    pcl::PointCloud<pcl::PointXYZI> normPOints;
//    for (int it = 0; it < firstINdexes.size(); ++it)
//    {
//        pcl::PointXYZI phe;
//        phe.x = maps->vPlanes[firstINdexes[it]].v3normal(0);
//        phe.y = maps->vPlanes[firstINdexes[it]].v3normal(1);
//        phe.z = maps->vPlanes[firstINdexes[it]].v3normal(2);
//        normPOints.points.push_back(phe);
//    }
//
//    //聚类
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
//    tree1->setInputCloud(normPOints.makeShared());
//    std::vector<pcl::PointIndices> cluster_indices2;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec1;
//    ec1.setClusterTolerance(0.15);
//    ec1.setMinClusterSize(1);
//    ec1.setMaxClusterSize(100);
//    ec1.setSearchMethod(tree1);
//    ec1.setInputCloud(normPOints.makeShared());
//    ec1.extract(cluster_indices2);
//
//    std::cout<<"cluster_indices2.size() "<<cluster_indices2.size()<<std::endl;
//
//    if (cluster_indices2.size() < 2) {
//        return;
//    }
//
////    if (bestMatch.size() < 3)
////        return;
//
//    int allMatchedSize = bestMatch.size();
//
//    allMatchedSize += bestMatchVehicle.size();
//
//    allMatchedSize += bestMatchPole.size();
//
////    if (bestMatchVehicle.size() > 1)
////    {
////        allMatchedSize += bestMatchVehicle.size();
////    }
////    if (bestMatchPole.size() > 1)
////    {
////        allMatchedSize += bestMatchPole.size();
////    }
//
//
////    std::cout<<"comp time "<<aveTime/(pointSearchInd.size())<<std::endl;
//
    // std::cout<<"allMatchedSize "<<allMatchedSize<<std::endl;


    //    laserCloudOri_m2.points.clear();

//    if (bestMatch.size() > 0)
//        for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
//        {
//            laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
//            laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
//        }


//KITTI
    if (allMatchedSize < 7) {
        return;
    }

    std::cout<<"localization successful !"<<std::endl;

//    laserCloudOri_m2.points.clear();
   if (bestMatch.size() > 0)
       for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
       {
           laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
           laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
       }




//    if (allgreatestMatch.size() > 0)
//        for (map<unsigned, unsigned>::iterator it = allgreatestMatch.begin(); it != allgreatestMatch.end(); it++)
//        {
//            laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
//            laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
//        }



//    if (bestMatchVehicle.size() > 0)
//        for (map<unsigned, unsigned>::iterator it = bestMatchVehicle.begin(); it != bestMatchVehicle.end(); it++)
//        {
//            laserCloudOri_m2 += *maps->vVehicles[it->first].VehiclePointCloudPtr;
//            laserCloudOri_m2 += *last_keyframe->vVehicles[it->second].VehiclePointCloudPtr;
//        }

//    if (bestMatchPole.size() > 0)
//        for (map<unsigned, unsigned>::iterator it = bestMatchPole.begin(); it != bestMatchPole.end(); it++)
//        {
//            laserCloudOri_m2 += *maps->vPoles[it->first].PolePointCloudPtr;
//            laserCloudOri_m2 += *last_keyframe->vPoles[it->second].PolePointCloudPtr;
//        }

}




void LiPMatch::genGlobalMap()
{}



void LiPMatch::run()
{
    size_t numPrevKFs = 0;

    while(!LiPMatch_stop)  // Stop loop if LiPMatch
    {
        if( numPrevKFs == frameQueue.size() )
        {
            sleep(10);
        }
        else
        {
            detectPlanesCloud( frameQueue[numPrevKFs], numPrevKFs);
            ++numPrevKFs;
        }
    }
    LiPMatch_finished = true;
}



bool LiPMatch::stop_LiPMatch()
{
    LiPMatch_stop = true;
    while(!LiPMatch_finished)
        sleep(1);

    cout << "Waiting for LiPMatch thread to die.." << endl;

    joinThread(LiPMatch_hd);
    LiPMatch_hd.clear();

    return true;
}

LiPMatch::~LiPMatch()
{
    cout << "\n\n\nLiPMatch destructor called -> Save color information to file\n";

    stop_LiPMatch();

    cout << " .. LiPMatch has died." << endl;
}







