#ifndef __SUBGRAPH_H
#define __SUBGRAPH_H


#include <Plane.h>
#include <Vehicle.h>
#include <Pole.h>


namespace LiPMatch_ns {
    class Subgraph
            {
            public:

    Subgraph(std::vector<Plane> &vPlanesI)
    {
        vPlanes = vPlanesI;

        for(size_t i = 0 ; i < vPlanes.size(); i++)
        {
            subgraphPlanesIdx.insert(i);
        }

    };

    Subgraph(std::vector<Vehicle> &vPlanesI)
    {
        vVehicles = vPlanesI;

        for(size_t i = 0 ; i < vVehicles.size(); i++)
        {
            subgraphVehiclesIdx.insert(i);
        }
    };

    Subgraph(std::vector<Pole> &vPlanesI)
    {
        vPoles = vPlanesI;
        for(size_t i = 0 ; i < vPoles.size(); i++)
        {
            subgraphPolesIdx.insert(i);
        }
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
