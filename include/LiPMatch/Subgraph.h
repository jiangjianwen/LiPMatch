#ifndef __SUBGRAPH_H
#define __SUBGRAPH_H


#include <Plane.h>
#include <Vehicle.h>
#include <Pole.h>


namespace LiPMatch_ns {
    class Subgraph
            {
            public:

    Subgraph(std::vector<Plane> &vPlanesI, const unsigned &refPlaneId)
    {
        vPlanes = vPlanesI;
        subgraphPlanesIdx.insert(refPlaneId);

        // Add neighbors co-visible neighbors
        for(std::map<unsigned,unsigned>::iterator it = vPlanes[refPlaneId].neighborPlanes.begin(); it != vPlanes[refPlaneId].neighborPlanes.end(); it++)
            subgraphPlanesIdx.insert(it->first);
    };

    Subgraph(std::vector<Vehicle> &vPlanesI, const unsigned &refPlaneId)
    {
        vVehicles = vPlanesI;
        subgraphVehiclesIdx.insert(refPlaneId);

        // Add neighbors co-visible neighbors
        for(std::map<unsigned,unsigned>::iterator it = vVehicles[refPlaneId].neighborVehicles.begin(); it != vVehicles[refPlaneId].neighborVehicles.end(); it++)
            subgraphVehiclesIdx.insert(it->first);
    };

    Subgraph(std::vector<Pole> &vPlanesI, const unsigned &refPlaneId)
    {
        vPoles = vPlanesI;
        subgraphPolesIdx.insert(refPlaneId);

        // Add neighbors co-visible neighbors
        for(std::map<unsigned,unsigned>::iterator it = vPoles[refPlaneId].neighborPoles.begin(); it != vPoles[refPlaneId].neighborPoles.end(); it++)
            subgraphPolesIdx.insert(it->first);
    };


    std::vector<Plane> vPlanes;

    std::set<unsigned> subgraphPlanesIdx;

    std::vector<Vehicle> vVehicles;

    std::set<unsigned> subgraphVehiclesIdx;

    std::vector<Pole> vPoles;

    std::set<unsigned> subgraphPolesIdx;

    };

 } // End of namespaces

#endif
