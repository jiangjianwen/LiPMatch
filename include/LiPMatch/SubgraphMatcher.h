#ifndef __SUBGRAPHMATCHER_H
#define __SUBGRAPHMATCHER_H



#include <Subgraph.h>
#include <Plane.h>
#include <Pole.h>
#include <Vehicle.h>

namespace LiPMatch_ns {
  
  class SubgraphMatcher
  {
    public:
    
    SubgraphMatcher();
    
    bool evalUnaryConstraintsPlane(Plane &plane1, Plane &plane2);

    bool evalUnaryConstraintsVehicle(Vehicle &vehicle1, Vehicle &vehicle2);

    bool evalUnaryConstraintsVehicleWithoutP(Vehicle &vehicle1, Vehicle &vehicle2);
    
    bool evalUnaryConstraintsPole(Pole &pole1, Pole &pole2);

    bool evalBinaryConstraints(Plane &plane1, Plane &plane2, Plane &planeA, Plane &planeB);

    bool evalBinaryConstraintsVehicle(Vehicle &Ref, Vehicle &neigRef, Vehicle &Check, Vehicle &neigCheck);

    bool evalBinaryConstraintsPole(Pole &Ref, Pole &neigRef, Pole &Check, Pole &neigCheck);


    void exploreSubgraphTreeR(std::set<unsigned> &evalRef, std::set<unsigned> &evalCheck, std::map<unsigned, unsigned> &matched);

    void exploreSubgraphTreeRVehicle(std::set<unsigned> &sourceVehicles, std::set<unsigned> &targetVehicles, std::map<unsigned, unsigned> &matched);


    void exploreSubgraphTreeRVehicleWithoutP(std::set<unsigned> &sourceVehicles, std::set<unsigned> &targetVehicles, std::map<unsigned, unsigned> &matched);


    bool evalUnaryConstraintsPoleWithoutP(Pole &pole1, Pole &pole2);

    void exploreSubgraphTreeRPole(std::set<unsigned> &sourcePoles, std::set<unsigned> &targetPoles, std::map<unsigned, unsigned> &matched);


    void exploreSubgraphTreeRPoleWithoutP(std::set<unsigned> &sourcePoles, std::set<unsigned> &targetPoles, std::map<unsigned, unsigned> &matched);



    std::map<unsigned,unsigned> compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget, int& unaryCount);


    std::map<unsigned,unsigned> compareSubgraphsVehiclePlaneRef(Subgraph &subgraphSource, Subgraph &subgraphTarget, int& unaryCount,
                                                                std::vector<Eigen::Vector3f>& kvc, std::vector<Eigen::Vector3f>& lvc,
                                                                std::vector<Eigen::Vector3f>& kvn, std::vector<Eigen::Vector3f>& lvn);

    std::map<unsigned,unsigned> compareSubgraphsVehicleWithoutPlaneRef(Subgraph &subgraphSource, Subgraph &subgraphTarget, int& unaryCount);

    std::map<unsigned,unsigned> compareSubgraphsPolePlaneRef(Subgraph &subgraphSource, Subgraph &subgraphTarget, int& unaryCount,
                                                             std::vector<Eigen::Vector3f>& kvc, std::vector<Eigen::Vector3f>& lvc,
                                                             std::vector<Eigen::Vector3f>& kvn, std::vector<Eigen::Vector3f>& lvn);


    std::map<unsigned,unsigned> compareSubgraphsPoleWithoutPlaneRef(Subgraph &subgraphSource, Subgraph &subgraphTarget, int& unaryCount);


    Subgraph *subgraphSrc;

    Subgraph *subgraphTrg;
    

    float height_threshold;
    float angle_threshold;
    float dist_threshold;
    float area_threshold;
    float elongation_threshold;
    float elongation_threshold2;
    float dist_thresholdvpv;
    float dist_thresholdvp;
    float dist_thresholdp;
    float dist_thresholdv;
    
    std::vector<std::vector<int8_t> > hashUnaryConstraints;

    std::map<unsigned, unsigned> allwinnerMatch;

    std::vector<std::map<unsigned, unsigned> > allMatchGroups;
    
    std::vector<int> allMatchGroupsSize;
    
    std::vector<float> allMatchGroups_height;
        std::vector<float> allMatchGroups_normal;
        std::vector<float> allMatchGroups_dist_centers;

        std::vector<Eigen::Vector3f> v_kvc;
        std::vector<Eigen::Vector3f> v_lvc;

        std::vector<float> vdif_height;
        std::vector<float> vdif_height2;
        std::vector<float> vdif_normal;
        std::vector<float> vrel_dist_centers;

        std::vector<float> vdif_v_2_v;

        std::vector<float> vah;
        std::vector<float> vea;

        float wdif_height;
        float wdif_height2;
        float wdif_normal;
        float wrel_dist_centers;

        float wal;
        float wea;

        bool withoutplanevehilceok = false;

        bool withoutplanecylinderok = false;
 
  private:

    std::map<unsigned, unsigned> winnerMatch;



    float winnerMatchArea;

    std::map<unsigned, unsigned> winnerMatchVehicle;

    std::map<unsigned, unsigned> winnerMatchPole;


    std::vector<Eigen::Vector3f> c_kvc;
    std::vector<Eigen::Vector3f> c_lvc;
    std::vector<Eigen::Vector3f> c_kvn;
    std::vector<Eigen::Vector3f> c_lvn;






  };

 } // End of namespaces

#endif
