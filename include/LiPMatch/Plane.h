#ifndef __PBMAP_PLANE_H
#define __PBMAP_PLANE_H

#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <set>
#include <map>
#include <Plane.h>
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <cassert>
#include "shapes/convexplane.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define SMALL_NUM  0.00000001 // anything that avoids division overflow

namespace LiPMatch_ns {

    struct Segment
    {
        Segment(pcl::PointXYZI p0, pcl::PointXYZI p1) :
                P0(p0), P1(p1)
        {};

        pcl::PointXYZI P0, P1;
    };


  class Plane
  {
    // This must be added to any CSerializable derived class:

   public:

      Plane() : elongation(1.0), polygonContourPtr(new pcl::PointCloud<pcl::PointXYZI>),
                planePointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>), InplanePointCloudOriPtr(new pcl::PointCloud<pcl::PointXYZI>)
                {
          matched = false;
                }

      void calcConvexHull(pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud, double plane[4])
      {
          ConvexPlane cplane(plane, pointCloud);
          std::vector<std::vector<double>> convexps = cplane.rPlanes();

          polygonContourPtr->points.clear();
          pcl::PointXYZI center; center.x = 0; center.y = 0; center.z = 0;
          for (size_t i = 0 ; i < convexps.size() ; ++i)
          {
              pcl::PointXYZI tmpPoint;
              tmpPoint.x = convexps[i][0];
              tmpPoint.y = convexps[i][1];
              tmpPoint.z = convexps[i][2];
              center.x += tmpPoint.x;
              center.y += tmpPoint.y;
              center.z += tmpPoint.z;
              polygonContourPtr->points.push_back(tmpPoint);
          }
          center.x /= convexps.size();
          center.y /= convexps.size();
          center.z /= convexps.size();
          v3center[0] = center.x;
          v3center[1] = center.y;
          v3center[2] = center.z;
      }

      void computeMassCenterAndArea()
      {
          int k0, k1, k2;

          // Find axis with largest normal component and project onto perpendicular plane
          k0 = (fabs (v3normal[0] ) > fabs (v3normal[1])) ? 0  : 1;
          k0 = (fabs (v3normal[k0]) > fabs (v3normal[2])) ? k0 : 2;
          k1 = (k0 + 1) % 3;
          k2 = (k0 + 2) % 3;

          // cos(theta), where theta is the angle between the polygon and the projected plane
          float ct = fabs ( v3normal[k0] );
          float AreaX2 = 0.0;
//          Eigen::Vector3f massCenter = Eigen::Vector3f::Zero();
          float p_i[3], p_j[3];

          for (unsigned int i = 0; i < polygonContourPtr->points.size (); i++)
          {
              p_i[0] = polygonContourPtr->points[i].x; p_i[1] = polygonContourPtr->points[i].y; p_i[2] = polygonContourPtr->points[i].z;
              int j = (i + 1) % polygonContourPtr->points.size ();
              p_j[0] = polygonContourPtr->points[j].x; p_j[1] = polygonContourPtr->points[j].y; p_j[2] = polygonContourPtr->points[j].z;
              double cross_segment = p_i[k1] * p_j[k2] - p_i[k2] * p_j[k1];

              AreaX2 += cross_segment;
//              massCenter[k1] += (p_i[k1] + p_j[k1]) * cross_segment;
//              massCenter[k2] += (p_i[k2] + p_j[k2]) * cross_segment;
          }
          areaHull = fabs (AreaX2) / (2 * ct);

//          massCenter[k1] /= (3*AreaX2);
//          massCenter[k2] /= (3*AreaX2);
//          massCenter[k0] = (v3normal.dot(v3center) - v3normal[k1]*massCenter[k1] - v3normal[k2]*massCenter[k2]) / v3normal[k0];

//          v3center = massCenter;
//          d = -(v3normal. dot (v3center));
      }

      void calcElongationAndPpalDir()
    {
        pcl::PCA< pcl::PointXYZ > pca;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPlanePointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*planePointCloudPtr,*tmpPlanePointCloudPtr);
        //*tmpPlanePointCloudPtr = *planePointCloudPtr;
    
        pca.setInputCloud(tmpPlanePointCloudPtr);
        Eigen::VectorXf eigenVal = pca.getEigenValues();

        eigenval = eigenVal;

//  if( eigenVal[0] > 2 * eigenVal[1] )
        {
            elongation = sqrt(eigenVal[0] / eigenVal[1]);

        }
    }

//      void forcePtsLayOnPlane()
//      {
//          // The plane equation has the form Ax + By + Cz + D = 0, where the vector N=(A,B,C) is the normal and the constant D can be calculated as D = -N*(PlanePoint) = -N*PlaneCenter
//          const double D = -(v3normal.dot(v3center));
//          for(unsigned i = 0; i < planePointCloudPtr->size(); i++)
//          {
//              double dist = v3normal[0]*planePointCloudPtr->points[i].x + v3normal[1]*planePointCloudPtr->points[i].y + v3normal[2]*planePointCloudPtr->points[i].z + D;
//              planePointCloudPtr->points[i].x -= v3normal[0] * dist;
//              planePointCloudPtr->points[i].y -= v3normal[1] * dist;
//              planePointCloudPtr->points[i].z -= v3normal[2] * dist;
//          }
//          // Do the same with the points defining the convex hull
//          for(unsigned i = 0; i < polygonContourPtr->size(); i++)
//          {
//              double dist = v3normal[0]*polygonContourPtr->points[i].x + v3normal[1]*polygonContourPtr->points[i].y + v3normal[2]*polygonContourPtr->points[i].z + D;
//              polygonContourPtr->points[i].x -= v3normal[0] * dist;
//              polygonContourPtr->points[i].y -= v3normal[1] * dist;
//              polygonContourPtr->points[i].z -= v3normal[2] * dist;
//          }
//      }



        void forcePtsLayOnPlane()
        {
            // The plane equation has the form Ax + By + Cz + D = 0, where the vector N=(A,B,C) is the normal and the constant D can be calculated as D = -N*(PlanePoint) = -N*PlaneCenter
            const double D = -(v3normal.dot(v3center));
            pcl::PointXYZI center; center.x = 0; center.y = 0; center.z = 0;
            for(unsigned i = 0; i < planePointCloudPtr->size(); i++)
            {
                double dist = v3normal[0]*planePointCloudPtr->points[i].x + v3normal[1]*planePointCloudPtr->points[i].y + v3normal[2]*planePointCloudPtr->points[i].z + D;
                planePointCloudPtr->points[i].x -= v3normal[0] * dist;
                planePointCloudPtr->points[i].y -= v3normal[1] * dist;
                planePointCloudPtr->points[i].z -= v3normal[2] * dist;

                center.x += planePointCloudPtr->points[i].x;
                center.y += planePointCloudPtr->points[i].y;
                center.z += planePointCloudPtr->points[i].z;
            }
            center.x /= planePointCloudPtr->size();
            center.y /= planePointCloudPtr->size();
            center.z /= planePointCloudPtr->size();
            v3center[0] = center.x;
            v3center[1] = center.y;
            v3center[2] = center.z;


            // Do the same with the points defining the convex hull
            for(unsigned i = 0; i < polygonContourPtr->size(); i++)
            {
                double dist = v3normal[0]*polygonContourPtr->points[i].x + v3normal[1]*polygonContourPtr->points[i].y + v3normal[2]*polygonContourPtr->points[i].z + D;
                polygonContourPtr->points[i].x -= v3normal[0] * dist;
                polygonContourPtr->points[i].y -= v3normal[1] * dist;
                polygonContourPtr->points[i].z -= v3normal[2] * dist;
            }
        }






      bool isPlaneNearby(Plane &plane_nearby, const float distThreshold)
    {
        float distThres2 = distThreshold * distThreshold;

        // First we check distances between centroids and vertex to accelerate this check
        if( (v3center - plane_nearby.v3center).squaredNorm() < distThres2 )
            return true;

        for(unsigned i=1; i < polygonContourPtr->size(); i++)
            if( (getVector3fromPointXYZ(polygonContourPtr->points[i]) - plane_nearby.v3center).squaredNorm() < distThres2 )
                return true;

        for(unsigned j=1; j < plane_nearby.polygonContourPtr->size(); j++)
            if( (v3center - getVector3fromPointXYZ(plane_nearby.polygonContourPtr->points[j]) ).squaredNorm() < distThres2 )
                return true;

        for(unsigned i=1; i < polygonContourPtr->size(); i++)
            for(unsigned j=1; j < plane_nearby.polygonContourPtr->size(); j++)
                if( (diffPoints(polygonContourPtr->points[i], plane_nearby.polygonContourPtr->points[j]) ).squaredNorm() < distThres2 )
                    return true;

        // a) Between an edge and a vertex
        // b) Between two edges (imagine two polygons on perpendicular planes)
        // a) & b)
        for(unsigned i=1; i < polygonContourPtr->size(); i++)
            for(unsigned j=1; j < plane_nearby.polygonContourPtr->size(); j++)
                if(dist3D_Segment_to_Segment2(Segment(polygonContourPtr->points[i],polygonContourPtr->points[i-1]), Segment(plane_nearby.polygonContourPtr->points[j],plane_nearby.polygonContourPtr->points[j-1])) < distThres2)
                    return true;

        return false;
    }

      template<class pointPCL>
      Eigen::Vector3f getVector3fromPointXYZ(pointPCL &pt)
      {
          return Eigen::Vector3f(pt.x,pt.y,pt.z);
      }

      template <class POINT>
      inline Eigen::Vector3f diffPoints(const POINT &P1, const POINT &P2)
      {
          Eigen::Vector3f diff;
          diff[0] = P1.x - P2.x;
          diff[1] = P1.y - P2.y;
          diff[2] = P1.z - P2.z;
          return diff;
      }

      float dist3D_Segment_to_Segment2( Segment S1, Segment S2)
        {
            Eigen::Vector3f   u = diffPoints(S1.P1, S1.P0);
            Eigen::Vector3f   v = diffPoints(S2.P1, S2.P0);
            Eigen::Vector3f   w = diffPoints(S1.P0, S2.P0);
            float    a = u.dot(u);        // always >= 0
            float    b = u.dot(v);
            float    c = v.dot(v);        // always >= 0
            float    d = u.dot(w);
            float    e = v.dot(w);
            float    D = a*c - b*b;       // always >= 0
            float    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
            float    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

            // compute the line parameters of the two closest points
            if (D < SMALL_NUM) { // the lines are almost parallel
                sN = 0.0;        // force using point P0 on segment S1
                sD = 1.0;        // to prevent possible division by 0.0 later
                tN = e;
                tD = c;
            }
            else {                // get the closest points on the infinite lines
                sN = (b*e - c*d);
                tN = (a*e - b*d);
                if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
                    sN = 0.0;
                    tN = e;
                    tD = c;
                }
                else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
                    sN = sD;
                    tN = e + b;
                    tD = c;
                }
            }

            if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
                tN = 0.0;
                // recompute sc for this edge
                if (-d < 0.0)
                    sN = 0.0;
                else if (-d > a)
                    sN = sD;
                else {
                    sN = -d;
                    sD = a;
                }
            }
            else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
                tN = tD;
                // recompute sc for this edge
                if ((-d + b) < 0.0)
                    sN = 0;
                else if ((-d + b) > a)
                    sN = sD;
                else {
                    sN = (-d + b);
                    sD = a;
                }
            }
            // finally do the division to get sc and tc
            sc = (fabs(sN) < SMALL_NUM ? 0.0 : sN / sD);
            tc = (fabs(tN) < SMALL_NUM ? 0.0 : tN / tD);

            // get the difference of the two closest points
            Eigen::Vector3f dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)

            return dP.squaredNorm();   // return the closest distance
        }





        /**!
     *  Parameters to allow the plane-based representation of the map by a graph
    */
    unsigned id;
    unsigned keyFrameId;
    std::map<unsigned,unsigned> neighborPlanes;


    /**!
     *  Geometric description
    */
    Eigen::Vector3f v3center;
    Eigen::Vector3f v3normal;
    float d;
    float elongation; // This is the reatio between the lengths of the plane in the two principal directions
    float areaVoxels;
    float areaHull;

    bool matched;

    Eigen::Vector3f eigenval;


    pcl::PointCloud<pcl::PointXYZI>::Ptr polygonContourPtr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr planePointCloudPtr;

    pcl::PointCloud<pcl::PointXYZI>::Ptr InplanePointCloudOriPtr;



  };

 } // End of namespaces


#endif
