#include <makeTargetmapcore.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace Eigen;
using namespace Makemap_ns;


Makemap::Makemap() : Makemap_stop(false), Makemap_finished(false)
{
    Makemap_hd = tools::createThreadFromObjectMethod(this,&Makemap::run);
}


void Makemap::detectPlanesCloud( tools::m_keyframe &c_keyframe, int keyFrameCount)
{
    static pcl::VoxelGrid<pcl::PointXYZI> grid_1;
    grid_1.setLeafSize(0.40,0.40,0.40);

    grid_1.setInputCloud(c_keyframe.structurelaserCloud);
    grid_1.filter (*c_keyframe.structurelaserCloud);

    grid_1.setLeafSize(0.20,0.20,0.20);
    grid_1.setInputCloud(c_keyframe.vehiclelaserCloud);
    grid_1.filter (*c_keyframe.vehiclelaserCloud);
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(c_keyframe.vehiclelaserCloud);
    sor.setMeanK(10);  
    sor.setStddevMulThresh(0.5);
    sor.setNegative(false);
    sor.filter(*c_keyframe.vehiclelaserCloud);

    grid_1.setLeafSize(0.2,0.2,0.2);
    grid_1.setInputCloud(c_keyframe.naturelaserCloud);
    grid_1.filter (*c_keyframe.naturelaserCloud);

    laserCloudOri_mp1.points.clear();
    laserCloudOri_mp1.width = 0;
    laserCloudOri_mp1.height = 0;


    int colorid = 0;
    //聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr treeVehicle (new pcl::search::KdTree<pcl::PointXYZI>);
    treeVehicle->setInputCloud (c_keyframe.vehiclelaserCloud);
    std::vector<pcl::PointIndices> cluster_indices_vehicle;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_vehicle;
    ec_vehicle.setClusterTolerance (0.205);
//    ec.setMinClusterSize (100);
    ec_vehicle.setMinClusterSize (75);
    ec_vehicle.setMaxClusterSize (15000);
    ec_vehicle.setSearchMethod (treeVehicle);
    ec_vehicle.setInputCloud (c_keyframe.vehiclelaserCloud);
    ec_vehicle.extract (cluster_indices_vehicle);
    pcl::PCA< pcl::PointXYZI > pca;

    pcl::PointCloud<pcl::PointXYZI>::Ptr curCylinderPoints(new pcl::PointCloud<pcl::PointXYZI>());

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

        if (eigenVal[2] / eigenVal[0] < 0.01)
            continue;

        if (eigenVal[1] / eigenVal[0] < 0.01)
            continue;
            
        allVehiclesBef += *laserCloudNgAftEe_vehicle;

        *curCylinderPoints += *laserCloudNgAftEe_vehicle;

        colorid++;
    }

    laserCloudOri_mp1 += *curCylinderPoints;


    //聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr treeNature (new pcl::search::KdTree<pcl::PointXYZI>);
    treeNature->setInputCloud (c_keyframe.naturelaserCloud);
    std::vector<pcl::PointIndices> cluster_indices_nature;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_nature;
    ec_nature.setClusterTolerance (0.21);
    ec_nature.setMinClusterSize (20);
    ec_nature.setMaxClusterSize (1500);
    ec_nature.setSearchMethod (treeNature);
    ec_nature.setInputCloud (c_keyframe.naturelaserCloud);
    ec_nature.extract (cluster_indices_nature);
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_natures(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PCA< pcl::PointXYZI > pca1;

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

        *laserCloudNgAftEe_natures += *laserCloudNgAftEe_nature;

        allCylindersBef += *laserCloudNgAftEe_nature;

        colorid++;
    }

   laserCloudOri_mp1 += *laserCloudNgAftEe_natures;





    //聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (c_keyframe.structurelaserCloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.65);
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
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (treeRg);
    reg.setNumberOfNeighbours (20);
    reg.setInputCloud (laserCloudNgAftEe);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (9.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (7.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

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

    pcl::PointCloud<pcl::PointXYZI>::Ptr curPlanePoints(new pcl::PointCloud<pcl::PointXYZI>());

    for (size_t i = 0 ; i < clusters.size() ; ++i)
    {
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
        }

        //输入点云
        seg.setInputCloud(pointsOnPlane);
        //分割点云，获得平面和法向量
        seg.segment(*inliers, *coefficients);

        allPlanesBef += *pointsOnPlane;

        *curPlanePoints += *pointsOnPlane;

    }

    laserCloudOri_mp1 += *curPlanePoints;



}



void Makemap::genPlaneMap()
{
    std::cout<<"merge planes ..."<<std::endl;
    
    //聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (allPlanesBef.makeShared());
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.60);
    ec.setMinClusterSize (60);
    // ec.setMinClusterSize (180);
    ec.setMaxClusterSize (150000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (allPlanesBef.makeShared());
    ec.extract (cluster_indices);
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe(new pcl::PointCloud<pcl::PointXYZI>());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            laserCloudNgAftEe->points.push_back (allPlanesBef.points[*pit]);
        }
    }

    //区域增长法提取激光点云中的平面
    pcl::search::Search<pcl::PointXYZI>::Ptr treeRg (new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (treeRg);
    normal_estimator.setInputCloud (laserCloudNgAftEe);
    normal_estimator.setKSearch (10);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
   // reg.setMinClusterSize (80);
    reg.setMinClusterSize (60);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (treeRg);
    reg.setNumberOfNeighbours (20);
    reg.setInputCloud (laserCloudNgAftEe);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (9.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (7.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

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

    for (size_t i = 0 ; i < clusters.size() ; ++i)
    {
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
        }

        //输入点云
        seg.setInputCloud(pointsOnPlane);
        //分割点云，获得平面和法向量
        seg.segment(*inliers, *coefficients); 

        laserCloudOri_m2 += *pointsOnPlane;
    }


   std::cout<<"saving plane points ......."<<std::endl;
   std::cout<<"the number of extracted planes: "<<clusters.size()<<std::endl;
   pcl::io::savePCDFileASCII ("/home/jjwen/software/catkin_rangenet_ws/src/map_structure_points00.pcd", allPlanesBef);

   std::cout<<"done"<<std::endl;
}

void Makemap::genCylinderMap()
{
    std::cout<<"merge cylinders ..."<<std::endl;

    pcl::VoxelGrid<pcl::PointXYZI> gridMap;
    gridMap.setLeafSize(0.2,0.2,0.2);
    gridMap.setInputCloud(allCylindersBef.makeShared());
    gridMap.filter(allCylindersBef);

    //聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr treeNature (new pcl::search::KdTree<pcl::PointXYZI>);
    treeNature->setInputCloud (allCylindersBef.makeShared());
    std::vector<pcl::PointIndices> cluster_indices_nature;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_nature;
    ec_nature.setClusterTolerance (0.21);
//    ec.setMinClusterSize (100);
    ec_nature.setMinClusterSize (20);
    ec_nature.setMaxClusterSize (500);
    ec_nature.setSearchMethod (treeNature);
    ec_nature.setInputCloud (allCylindersBef.makeShared());
    ec_nature.extract (cluster_indices_nature);
    pcl::PCA< pcl::PointXYZI > pca1;

    int colorid = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_nature.begin (); it != cluster_indices_nature.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_nature(new pcl::PointCloud<pcl::PointXYZI>());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            allCylindersBef.points[*pit].intensity = colorid;
            laserCloudNgAftEe_nature->points.push_back (allCylindersBef.points[*pit]);
        }
        pca1.setInputCloud(laserCloudNgAftEe_nature);
        Eigen::VectorXf eigenVal = pca1.getEigenValues();

        if (eigenVal[1] / eigenVal[0] > 0.135)
            continue;

        colorid++;

        laserCloudOri_m2 += *laserCloudNgAftEe_nature;

    }


    std::cout<<"saving cylinder points ......."<<std::endl;
    std::cout<<"the number of extracted cylinders: "<<colorid<<std::endl;
    pcl::io::savePCDFileASCII ("/home/jjwen/software/catkin_rangenet_ws/src/map_cylinder_points00.pcd", allCylindersBef);
    std::cout<<"done"<<std::endl;
}

void Makemap::genVehicleMap()
{
   std::cout<<"merge vehicles ..."<<std::endl;

   pcl::VoxelGrid<pcl::PointXYZI> gridMap;
   gridMap.setLeafSize(0.20,0.20,0.20);
   gridMap.setInputCloud(allVehiclesBef.makeShared());
   gridMap.filter(allVehiclesBef);

   pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
   sor.setInputCloud(allVehiclesBef.makeShared());
   sor.setMeanK(10);   //设置在进行统计时考虑查询点邻近点数
   sor.setStddevMulThresh(0.5);
   sor.setNegative(false);
   sor.filter(allVehiclesBef);

   //聚类
   pcl::search::KdTree<pcl::PointXYZI>::Ptr treeVehicle (new pcl::search::KdTree<pcl::PointXYZI>);
   treeVehicle->setInputCloud (allVehiclesBef.makeShared());
   std::vector<pcl::PointIndices> cluster_indices_vehicle;
   pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_vehicle;
   ec_vehicle.setClusterTolerance (0.205);
//    ec.setMinClusterSize (100);
   ec_vehicle.setMinClusterSize (85);
   ec_vehicle.setMaxClusterSize (10000);
   ec_vehicle.setSearchMethod (treeVehicle);
   ec_vehicle.setInputCloud (allVehiclesBef.makeShared());
   ec_vehicle.extract (cluster_indices_vehicle);
   pcl::PCA< pcl::PointXYZI > pca;

   int colorid = 0;

   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_vehicle.begin (); it != cluster_indices_vehicle.end (); ++it)
   {
       pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudNgAftEe_vehicle(new pcl::PointCloud<pcl::PointXYZI>());
       for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
       {
           allVehiclesBef.points[*pit].intensity = colorid;
           laserCloudNgAftEe_vehicle->points.push_back (allVehiclesBef.points[*pit]);
       }
       pca.setInputCloud(laserCloudNgAftEe_vehicle);
       Eigen::VectorXf eigenVal = pca.getEigenValues();

       if (eigenVal[2] / eigenVal[0] < 0.01)
           continue;

       if (eigenVal[1] / eigenVal[0] < 0.01)
           continue;

       colorid++;

       laserCloudOri_m2 += *laserCloudNgAftEe_vehicle;
   }


   std::cout<<"saving vehicle points ......."<<std::endl;
   std::cout<<"the number of extracted vehicles: "<<colorid<<std::endl;
   pcl::io::savePCDFileASCII ("/home/jjwen/software/catkin_rangenet_ws/src/map_vehicle_points00.pcd", allVehiclesBef);

   std::cout<<"done"<<std::endl;
}





void Makemap::run()
{
    size_t numPrevKFs = 0;

    while(!Makemap_stop)  // Stop loop if Makemap
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
    Makemap_finished = true;
}



bool Makemap::stop_Makemap()
{
    Makemap_stop = true;
    while(!Makemap_finished)
        sleep(1);

    cout << "Waiting for Makemap thread to die.." << endl;

    tools::joinThread(Makemap_hd);
    Makemap_hd.clear();

    return true;
}

Makemap::~Makemap()
{
    cout << "\n\n\nMakemap destructor called -> Save color information to file\n";

    stop_Makemap();

    cout << " .. Makemap has died." << endl;
}
