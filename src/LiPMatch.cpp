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





LiPMatch::LiPMatch() : LiPMatch_stop(false), LiPMatch_finished(false)
{
    std::cout<<"loading structure points ..."<<std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZI>("/home/jjwen/software/catkin_rangenet_ws/src/map_structure_points00.pcd", *map_structure_points);
    std::cout<<"loading vehicle points ..."<<std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZI>("/home/jjwen/software/catkin_rangenet_ws/src/map_vehicle_points00.pcd", *map_vehicle_points);
    std::cout<<"loading cylinder points ..."<<std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZI>("/home/jjwen/software/catkin_rangenet_ws/src/map_cylinder_points00.pcd", *map_cylinder_points);

    vector<Vehicle> detectedLocalVehicles;
    vector<Pole> detectedLocalPoles;
    vector<Plane> detectedLocalPlanes;

    pcl::PCA< pcl::PointXYZI > pca;
    //聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> cluster_indices_vehicle;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_vehicle;
    ec_vehicle.setClusterTolerance (0.205);
    ec_vehicle.setMinClusterSize (75);
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

    std::cout<<"vVehicles.size() "<<detectedLocalVehicles.size()<<std::endl;
    maps->vVehicles = detectedLocalVehicles;

    //聚类
    std::vector<pcl::PointIndices> cluster_indices_nature;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_nature;
    ec_nature.setClusterTolerance (0.21);
    ec_nature.setMinClusterSize (15);
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

    std::cout<<"vPoles.size() "<<detectedLocalPoles.size()<<std::endl;
    maps->vPoles = detectedLocalPoles;

//    pcl::VoxelGrid<pcl::PointXYZI> gridMap;
//    gridMap.setLeafSize(0.8,0.8,0.8);
//    gridMap.setInputCloud(map_structure_points);
//    gridMap.filter(*map_structure_points);

    //聚类
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.60);
    ec.setMinClusterSize (55);
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
    reg.setMinClusterSize (55);
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

        colorid++;

        plane.calcConvexHull(plane.planePointCloudPtr,planen);

        plane.computeMassCenterAndArea();
        plane.calcElongationAndPpalDir();

        plane.areaVoxels= plane.planePointCloudPtr->size() * 0.0025;

        mapToShow += *plane.planePointCloudPtr;

        detectedLocalPlanes.push_back(plane);
    }

    std::cout<<"vPlanes.size() "<<detectedLocalPlanes.size()<<std::endl;

    maps->vPlanes = detectedLocalPlanes;

    LiPMatch_hd = tools::createThreadFromObjectMethod(this,&LiPMatch::run);

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




void LiPMatch::detectPlanesCloud( tools::m_keyframe &c_keyframe, int keyFrameCount)
{
    std::cout<<"new keyframe coming... "<<keyFrameCount<<std::endl;
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


    // static pcl::VoxelGrid<pcl::PointXYZI> grid_1;
    // grid_1.setLeafSize(0.20,0.20,0.20);

    // grid_1.setInputCloud(c_keyframe.structurelaserCloud);
    // grid_1.filter (*c_keyframe.structurelaserCloud);

    // grid_1.setLeafSize(0.10,0.10,0.10);
    // grid_1.setInputCloud(c_keyframe.vehiclelaserCloud);
    // grid_1.filter (*c_keyframe.vehiclelaserCloud);

    // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    // sor.setInputCloud(c_keyframe.vehiclelaserCloud);
    // sor.setMeanK(10);   //设置在进行统计时考虑查询点邻近点数
    // sor.setStddevMulThresh(0.5);
    // sor.setNegative(false);
    // sor.filter(*c_keyframe.vehiclelaserCloud);

    // grid_1.setLeafSize(0.10,0.10,0.10);
    // grid_1.setInputCloud(c_keyframe.naturelaserCloud);
    // grid_1.filter (*c_keyframe.naturelaserCloud);


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
        
        Vehicle vc;
        vc.VehiclePointCloudPtr = laserCloudNgAftEe_vehicle;
        vc.calcCenterAndElongation();

        detectedLocalVehicles.push_back(vc);

    }

    // laserCloudOri_m2 += *vehiclesPointCloud;

    std::shared_ptr<Maps_keyframe<float>> mk = std::make_shared<Maps_keyframe<float>>();

    std::cout<<"vVehicles.size() "<<detectedLocalVehicles.size()<<std::endl;

    mk->vVehicles = detectedLocalVehicles;

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

    std::cout<<"vPoles.size() "<<detectedLocalPoles.size()<<std::endl;

    mk->vPoles = detectedLocalPoles;


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

        plane.calcConvexHull(plane.planePointCloudPtr,planen);

        plane.computeMassCenterAndArea();
        plane.calcElongationAndPpalDir();

        plane.areaVoxels= plane.planePointCloudPtr->size() * 0.0025;

        *curPlanePoints += *plane.planePointCloudPtr;

        detectedLocalPlanes.push_back(plane);
    }

//    laserCloudOri_m2.points.clear();
//    laserCloudOri_m2 += *curPlanePoints;


    std::cout<<"vPlanes.size() ";
    std::cout<<detectedLocalPlanes.size()<<std::endl;

    mk->vPlanes = detectedLocalPlanes;

//    std::cout<<"mk->m_accumulated_g_pc.size() "<<mk->m_accumulated_g_pc.size()<<std::endl;

    // mk->m_accumulated_structure_pc = *c_keyframe.structurelaserCloud;
    // mk->m_accumulated_vehicle_pc = *c_keyframe.vehiclelaserCloud;


    keyframe_vec.push_back( mk );

    std::shared_ptr<Maps_keyframe<float>>& last_keyframe = keyframe_vec.back();

    laserCloudOri_m2.points.clear();
    laserCloudOri_m2.width = 0;
    laserCloudOri_m2.height = 0;

    std::map<unsigned, unsigned> resultingMatchtvt;  resultingMatchtvt.clear();

    if (last_keyframe->vVehicles.size() > 6)
    {
        Subgraph currentSubgraphv(maps->vVehicles);
        Subgraph targetSubgraphv(last_keyframe->vVehicles);

        std::cout<<"begin compareSubgraphsVehicleWithoutPlaneRef"<<std::endl;
        int tunaycount;
        resultingMatchtvt = matcher.compareSubgraphsVehicleWithoutPlaneRef(currentSubgraphv, targetSubgraphv, tunaycount);
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


    std::map<unsigned, unsigned> resultingMatchtvtc; resultingMatchtvtc.clear();
    if (last_keyframe->vPoles.size() > 6)
    {
        Subgraph currentSubgraphcy(maps->vPoles);
        Subgraph targetSubgraphcy(last_keyframe->vPoles);
        
        std::cout<<"begin compareSubgraphsPoleWithoutPlaneRef"<<std::endl;
        int tunaycountc;
        resultingMatchtvtc = matcher.compareSubgraphsPoleWithoutPlaneRef(currentSubgraphcy, targetSubgraphcy, tunaycountc);
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


    // //////////////////////
    // int sumSizeH = resultingMatchtvt.size();
    
    // std::vector <Eigen::Vector3f> vkvc1;
    // std::vector <Eigen::Vector3f> vlvc1;

    // for (map<unsigned, unsigned>::iterator it = resultingMatchtvt.begin(); it != resultingMatchtvt.end(); it++) 
    // {
    //     vkvc1.push_back(maps->vVehicles[it->first].v3center);
    //     vlvc1.push_back(last_keyframe->vVehicles[it->second].v3center);
    // }

    // if (vkvc1.size() > 0 && resultingMatchtvtc.size() > 0)
    // {
    //     for (size_t i = 0 ; i < vkvc1.size() ; ++i)
    //     {
    //         bool foundp = true;
    //         for (map<unsigned, unsigned>::iterator it = resultingMatchtvtc.begin(); it != resultingMatchtvtc.end(); it++) 
    //         {
    //             auto pole1 = maps->vPoles[it->first];
    //             auto pole2 = last_keyframe->vPoles[it->second];

    //             float disk = sqrt((pole1.v3center - vkvc1[i]).dot(pole1.v3center - vkvc1[i]));
    //             float disl = sqrt((pole2.v3center - vlvc1[i]).dot(pole2.v3center - vlvc1[i]));
    //             float radio = fabs(disk) / fabs(disl);
    //             if (radio > 1.2 || radio < 1/1.2)
    //             {
    //                 foundp = false;
    //                 break;
    //             }
    //         }
    //         if (foundp)
    //           sumSizeH++;
    //     }
    // }

    // std::cout<<"sumSizeH  "<<sumSizeH<<std::endl;
    // ///////////////



    std::map<unsigned, unsigned> bestMatch; bestMatch.clear();


    Subgraph currentSubgraph1(maps->vPlanes);
    Subgraph targetSubgraph1(last_keyframe->vPlanes);

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

    std::vector<std::map<unsigned, unsigned> > AllbestPlaneMatch;    AllbestPlaneMatch.clear();
    std::vector<std::map<unsigned, unsigned> > AllbestVehicleMatch;  AllbestVehicleMatch.clear();
    std::vector<std::map<unsigned, unsigned> > AllbestCylinderMatch; AllbestCylinderMatch.clear();

    std::vector<int> AllSamebestMatchSize; AllSamebestMatchSize.clear();

	
    sort(matcher.allMatchGroups.begin(), matcher.allMatchGroups.end(), sortFun);
    

    int maxSize = matcher.allMatchGroups[0].size();


    for (size_t i = 0 ; i < matcher.allMatchGroups.size() ; ++i)
    {
        if (matcher.allMatchGroups[i].size() < (maxSize-1) )
            break;

        bestMatch = matcher.allMatchGroups[i];

        if (bestMatch.size() < 2)
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
            
            Subgraph currentSubgraph2(maps->vVehicles);
            Subgraph targetSubgraph2(last_keyframe->vVehicles);
            
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
                Subgraph currentSubgraph3(maps->vPoles);
                Subgraph targetSubgraph3(last_keyframe->vPoles);

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
        
        // std::cout << "bestMatch.size() " << bestMatch.size() << std::endl;
        // std::cout << "bestMatchPole.size() " << bestMatchPole.size() << std::endl;
        // std::cout << "bestMatchVehicle.size() " << bestMatchVehicle.size() << std::endl;

        // std::cout<<"bestMatch.size() "<<bestMatch.size()<<std::endl;
        // if (bestMatch.size() > 0)
        //     for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++) {
        //        laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
        //        laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
        //    }



        allMatchedSize = bestMatch.size();

        allMatchedSize += bestMatchVehicle.size();

        allMatchedSize += bestMatchPole.size();


        // std::cout<<greatMatchedSize<<std::endl;
        if (allMatchedSize >= greatMatchedSize)
        {
            greatMatchedSize = allMatchedSize;
            greatestMatch = bestMatch;

        }

        AllSamebestMatchSize.push_back(allMatchedSize);
        AllbestPlaneMatch.push_back(bestMatch);
        AllbestVehicleMatch.push_back(bestMatchVehicle);
        AllbestCylinderMatch.push_back(bestMatchPole);
        
        // std::cout<<"allMatchedSize "<<allMatchedSize<<std::endl;
    }


    int sameSIZE = 0;
    for (size_t i = 0 ; i < AllSamebestMatchSize.size() ; ++i)
    // for (int i = int(AllSamebestMatchSize.size()) - 1 ; i >= 0 ; --i)
    {
        // if ((AllSamebestMatchSize[i] == greatMatchedSize) || (AllSamebestMatchSize[i] == greatMatchedSize-1))
        if (AllSamebestMatchSize[i] == greatMatchedSize)
        {
            sameSIZE++;
            // laserCloudOri_m2.points.clear();
            // laserCloudOri_m2.width = 0;
            // laserCloudOri_m2.height = 0;
            
            if (AllbestPlaneMatch[i].size() > 0)
            for (map<unsigned, unsigned>::iterator it = AllbestPlaneMatch[i].begin(); it != AllbestPlaneMatch[i].end(); it++) {
                laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
                laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
            }
            
            if (AllbestVehicleMatch[i].size() > 0)
            for (map<unsigned, unsigned>::iterator it = AllbestVehicleMatch[i].begin(); it != AllbestVehicleMatch[i].end(); it++) {
                laserCloudOri_m2 += *maps->vVehicles[it->first].VehiclePointCloudPtr;
                laserCloudOri_m2 += *last_keyframe->vVehicles[it->second].VehiclePointCloudPtr;
            }
            
            if (AllbestCylinderMatch[i].size() > 0)
            for (map<unsigned, unsigned>::iterator it = AllbestCylinderMatch[i].begin(); it != AllbestCylinderMatch[i].end(); it++) {
                laserCloudOri_m2 += *maps->vPoles[it->first].PolePointCloudPtr;
                laserCloudOri_m2 += *last_keyframe->vPoles[it->second].PolePointCloudPtr;
            }
        }
    }

    std::cout<<"sameSIZE "<<sameSIZE<<std::endl;


    allMatchedSize = greatMatchedSize;
    bestMatch = greatestMatch;

    // std::cout<<"allMatchedSize "<<allMatchedSize<<std::endl;
    // std::cout<<"bestMatch.size() "<<bestMatch.size()<<std::endl;


    std::cout<<"allMatchedSize "<<allMatchedSize<<std::endl;
    std::cout<<"bestMatch.size() "<<bestMatch.size()<<std::endl;



// //KITTI
//     if (allMatchedSize < 7) {
//         return;
//     }

    // std::cout<<"localization successful !"<<std::endl;

// //    laserCloudOri_m2.points.clear();
//    if (bestMatch.size() > 0)
//        for (map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
//        {
//            laserCloudOri_m2 += *maps->vPlanes[it->first].planePointCloudPtr;
//            laserCloudOri_m2 += *last_keyframe->vPlanes[it->second].planePointCloudPtr;
//        }




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

    tools::joinThread(LiPMatch_hd);
    LiPMatch_hd.clear();

    return true;
}

LiPMatch::~LiPMatch()
{
    cout << "\n\n\nLiPMatch destructor called -> Save color information to file\n";

    stop_LiPMatch();

    cout << " .. LiPMatch has died." << endl;
}







