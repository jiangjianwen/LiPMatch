#include <SubgraphMatcher.h>


using namespace std;
using namespace LiPMatch_ns;


SubgraphMatcher::SubgraphMatcher()
{}



bool SubgraphMatcher::evalUnaryConstraintsPlane(Plane &plane1, Plane &plane2) {

    double rel_areas = plane1.areaHull / plane2.areaHull;
    double rel_elong = plane1.elongation / plane2.elongation;

//    float area_threshold = 2.0;
    float area_threshold_inv = 1.0/area_threshold;

    if (rel_areas < area_threshold_inv || rel_areas > area_threshold) {
        return false;
    }

//    float elongation_threshold = 2.0;
    float elongation_threshold_inv = 1.0/elongation_threshold;

    float elongation_threshold_inv2 = 1.0/elongation_threshold2;


    if (rel_elong < elongation_threshold_inv || rel_elong > elongation_threshold) {
        return false;
    }


    return true;
}





bool SubgraphMatcher::evalUnaryConstraintsVehicleWithoutP(Vehicle &vehicle1, Vehicle &vehicle2) {

    float elongation_thresholdh = 1.1;
    float elongation_threshold_inv = 1.0/elongation_thresholdh;

    double rel_elong = vehicle1.elongation / vehicle2.elongation;

    if (rel_elong < elongation_threshold_inv || rel_elong > elongation_thresholdh) {
        return false;
    }

    return true;
}






bool SubgraphMatcher::evalUnaryConstraintsVehicle(Vehicle &vehicle1, Vehicle &vehicle2) {

    float dist_threshold_inv = 1/dist_thresholdvpv;

    for (size_t i = 0 ; i < c_kvc.size() ; ++i)
    {
        float disk = c_kvn[i].dot(vehicle1.v3center - c_kvc[i]);
        float disl = c_lvn[i].dot(vehicle2.v3center - c_lvc[i]);

        float radio = fabs(disk) / fabs(disl);

        if (radio > dist_thresholdvpv || radio < dist_threshold_inv)
            return false;
    }

    return true;
}

bool SubgraphMatcher::evalUnaryConstraintsPoleWithoutP(Pole &pole1, Pole &pole2) {

    float elongation_thresholdh = 1.2;
    float elongation_threshold_inv = 1.0/elongation_thresholdh;

    double rel_elong = pole1.elongation / pole2.elongation;

    if (rel_elong < elongation_threshold_inv || rel_elong > elongation_thresholdh) {
        return false;
    }

    return true;
}


bool SubgraphMatcher::evalUnaryConstraintsPole(Pole &pole1, Pole &pole2) {

    float dist_threshold_inv = 1/dist_thresholdvp;

    for (size_t i = 0 ; i < c_kvc.size() ; ++i)
    {
        float disk = c_kvn[i].dot(pole1.v3center - c_kvc[i]);
        float disl = c_lvn[i].dot(pole2.v3center - c_lvc[i]);

        float radio = fabs(disk) / fabs(disl);

        if (radio > dist_thresholdvp || radio < dist_threshold_inv)
            return false;
    }

    if (v_kvc.size() > 0)
    {
    for (size_t i = 0 ; i < v_kvc.size() ; ++i)
    {
        float disk = sqrt((pole1.v3center - v_kvc[i]).dot(pole1.v3center - v_kvc[i]));
        float disl = sqrt((pole2.v3center - v_lvc[i]).dot(pole2.v3center - v_lvc[i]));

        float radio = fabs(disk) / fabs(disl);

        if (radio > dist_thresholdvp || radio < dist_threshold_inv)
            return false;

//        if (fabs(disk - disl) > 1.00)
//            return false;
    }
    }



    return true;
}

//bool SubgraphMatcher::evalBinaryConstraints(Plane &Ref, Plane &neigRef, Plane &Check, Plane &neigCheck)
//{
//    double dif_height = fabs(Ref.v3normal.dot(neigRef.v3center - Ref.v3center) - Check.v3normal.dot(neigCheck.v3center - Check.v3center));
//    float height_threshold = 2.5;
//
//    if(dif_height > height_threshold)
//    {
//        return false;
//    }
//
//    double dif_height2 = fabs(neigRef.v3normal.dot(Ref.v3center - neigRef.v3center) - neigCheck.v3normal.dot(Check.v3center - neigCheck.v3center));
//    if(dif_height2 > height_threshold)
//    {
//        return false;
//    }
//
//    // Normal
//    float angle_threshold = 8.5;
////    float angle_threshold = 12.5;
//
//
//    double dif_normal = fabs(RAD2DEG( acos( Ref.v3normal.dot(neigRef.v3normal)) - acos( Check.v3normal.dot(neigCheck.v3normal)) ) );
//    if( dif_normal > angle_threshold )
//    {
//        return false;
//    }
//
//    float dist_threshold = 2.0;
//    float dist_threshold_inv = 1/dist_threshold;
//
//    // Relative distance
//    double rel_dist_centers = sqrt( (Ref.v3center - neigRef.v3center).dot(Ref.v3center - neigRef.v3center) / ((Check.v3center - neigCheck.v3center).dot(Check.v3center - neigCheck.v3center)) );
//    {
//        if( rel_dist_centers < dist_threshold_inv || rel_dist_centers > dist_threshold )
//        {
//            return false;
//        }
//    }
//
//    return true;
//}



bool SubgraphMatcher::evalBinaryConstraints(Plane &Ref, Plane &neigRef, Plane &Check, Plane &neigCheck)
{
    double dif_height = (Ref.v3normal.dot(neigRef.v3center - Ref.v3center)) / (Check.v3normal.dot(neigCheck.v3center - Check.v3center));
    float height_threshold_inv = 1/height_threshold;

    if(dif_height > height_threshold || dif_height < height_threshold_inv)
    {
        return false;
    }

    if (dif_height < 1.0)
        dif_height = 1.0/dif_height;

    double dif_height2 = (neigRef.v3normal.dot(Ref.v3center - neigRef.v3center)) / (neigCheck.v3normal.dot(Check.v3center - neigCheck.v3center));
    if(dif_height2 > height_threshold || dif_height2 < height_threshold_inv)
    {
        return false;
    }
    
    if (dif_height2 < 1.0)
        dif_height2 = 1.0/dif_height2;



    float angle_threshold_inv = 1/angle_threshold;


    double dif_normal = RAD2DEG(acos( Ref.v3normal.dot(neigRef.v3normal))) / RAD2DEG(acos( Check.v3normal.dot(neigCheck.v3normal))) ;
    if( dif_normal > angle_threshold || dif_normal < angle_threshold_inv )
    {
        return false;
    }

    if (dif_normal < 1.0)
        dif_normal = 1.0/dif_normal;


    float dist_threshold_inv = 1/dist_threshold;

    // Relative distance
    double rel_dist_centers = sqrt( (Ref.v3center - neigRef.v3center).dot(Ref.v3center - neigRef.v3center) / ((Check.v3center - neigCheck.v3center).dot(Check.v3center - neigCheck.v3center)) );
    {
        if( rel_dist_centers < dist_threshold_inv || rel_dist_centers > dist_threshold )
        {
            return false;
        }
    }

    if (rel_dist_centers < 1.0)
        rel_dist_centers = 1.0/rel_dist_centers;


    vdif_height.push_back(dif_height);
    vdif_height.push_back(dif_height2);
    // vdif_height2.push_back(dif_height2);
    vrel_dist_centers.push_back(rel_dist_centers);
    vdif_normal.push_back(dif_normal);

    float al;
    if (Ref.areaHull > Check.areaHull)
        al = Ref.areaHull / Check.areaHull;
    else
        al = Check.areaHull / Ref.areaHull;

    float al2;
    if (neigRef.areaHull > neigCheck.areaHull)
        al2 = neigRef.areaHull / neigCheck.areaHull;
    else
        al2 = neigCheck.areaHull / neigRef.areaHull;

    vah.push_back(al);
    vah.push_back(al2);


    float ea;
    if (Ref.elongation > Check.elongation)
        ea = Ref.elongation / Check.elongation;
    else
        ea = Check.elongation / Ref.elongation;

    float ea2;
    if (neigRef.elongation > neigCheck.elongation)
        ea2 = neigRef.elongation / neigCheck.elongation;
    else
        ea2 = neigCheck.elongation / neigRef.elongation;

    vea.push_back(ea);
    vea.push_back(ea2);


    return true;
}







//bool SubgraphMatcher::evalBinaryConstraints(Plane &Ref, Plane &neigRef, Plane &Check, Plane &neigCheck)
//{
//    double dif_height = (Ref.v3normal.dot(neigRef.v3center - Ref.v3center)) / (Check.v3normal.dot(neigCheck.v3center - Check.v3center));
//    float height_threshold_inv = 1/height_threshold;
//
//
//    if(dif_height > height_threshold || dif_height < height_threshold_inv)
//    {
//        return false;
//    }
//
//    double dif_height2 = (neigRef.v3normal.dot(Ref.v3center - neigRef.v3center)) / (neigCheck.v3normal.dot(Check.v3center - neigCheck.v3center));
//    if(dif_height2 > height_threshold || dif_height2 < height_threshold_inv)
//    {
//        return false;
//    }
//
//
//
//
////    double dif_height = fabs(Ref.v3normal.dot(neigRef.v3center - Ref.v3center) - Check.v3normal.dot(neigCheck.v3center - Check.v3center));
//////    float height_threshold = 1.95;
////
////    if(dif_height > height_threshold)
////    {
////        return false;
////    }
////
////    double dif_height2 = fabs(neigRef.v3normal.dot(Ref.v3center - neigRef.v3center) - neigCheck.v3normal.dot(Check.v3center - neigCheck.v3center));
////    if(dif_height2 > height_threshold)
////    {
////        return false;
////    }
//
//    // Normal
////    float angle_threshold = 3.4;
////    float angle_threshold = 12.5;
//
//    float angle_threshold_inv = 1/angle_threshold;
//
//
//    double dif_normal = RAD2DEG(acos( Ref.v3normal.dot(neigRef.v3normal))) / RAD2DEG(acos( Check.v3normal.dot(neigCheck.v3normal))) ;
//    if( dif_normal > angle_threshold || dif_normal < angle_threshold_inv )
//    {
//        return false;
//    }
//
//
//
//
//
////    double dif_normal = fabs(RAD2DEG( acos( Ref.v3normal.dot(neigRef.v3normal)) - acos( Check.v3normal.dot(neigCheck.v3normal)) ) );
////    if( dif_normal > angle_threshold )
////    {
////        return false;
////    }
//
////    float dist_threshold = 2.0;
//    float dist_threshold_inv = 1/dist_threshold;
//
//    // Relative distance
//    double rel_dist_centers = sqrt( (Ref.v3center - neigRef.v3center).dot(Ref.v3center - neigRef.v3center) / ((Check.v3center - neigCheck.v3center).dot(Check.v3center - neigCheck.v3center)) );
//    {
//        if( rel_dist_centers < dist_threshold_inv || rel_dist_centers > dist_threshold )
//        {
//            return false;
//        }
//    }
//
//
//    vdif_height.push_back(dif_height);
//    vdif_height2.push_back(dif_height2);
//    vdif_normal.push_back(dif_normal);
//    vrel_dist_centers.push_back(rel_dist_centers);
//
//    float al;
//    if (Ref.areaHull > Check.areaHull)
//        al = Ref.areaHull / Check.areaHull;
//    else
//        al = Check.areaHull / Ref.areaHull;
//
//    float al2;
//    if (neigRef.areaHull > neigCheck.areaHull)
//        al2 = neigRef.areaHull / neigCheck.areaHull;
//    else
//        al2 = neigCheck.areaHull / neigRef.areaHull;
//
//    vah.push_back(al);
//    vah.push_back(al2);
//
//    float ea;
//    if (Ref.elongation > Check.elongation)
//        ea = Ref.elongation / Check.elongation;
//    else
//        ea = Check.elongation / Ref.elongation;
//
//    float ea2;
//    if (neigRef.elongation > neigCheck.elongation)
//        ea2 = neigRef.elongation / neigCheck.elongation;
//    else
//        ea2 = neigCheck.elongation / neigRef.elongation;
//
//    vea.push_back(ea);
//    vea.push_back(ea2);
//
//
//    return true;
//}




bool SubgraphMatcher::evalBinaryConstraintsVehicle(Vehicle &Ref, Vehicle &neigRef, Vehicle &Check, Vehicle &neigCheck)
{
    float dist_threshold_inv = 1/dist_thresholdv;

    // Relative distance
    double rel_dist_centers = sqrt( (Ref.v3center - neigRef.v3center).dot(Ref.v3center - neigRef.v3center) / ((Check.v3center - neigCheck.v3center).dot(Check.v3center - neigCheck.v3center)) );

    if( rel_dist_centers < dist_threshold_inv || rel_dist_centers > dist_thresholdv )
    {
        return false;
    }

    if (rel_dist_centers < 1.0)
       rel_dist_centers = 1.0 / rel_dist_centers;

    vdif_v_2_v.push_back(rel_dist_centers);

//    vdif_normal.push_back(rel_dist_centers);
//
//    float res = 0.0;
//    int hs = 0;
//    for (auto item : vdif_normal)
//    {
//        if (item > 1.0) {
//            res += item;
//            hs++;
//        }
//    }
//
//    res = res / hs;
//
//    std::cout<<"res "<<res<<std::endl;


    return true;
}

bool SubgraphMatcher::evalBinaryConstraintsPole(Pole &Ref, Pole &neigRef, Pole &Check, Pole &neigCheck)
{
    float dist_thresholdp_inv = 1/dist_thresholdp;

    // Relative distance
    double rel_dist_centers = sqrt( (Ref.v3center - neigRef.v3center).dot(Ref.v3center - neigRef.v3center) / ((Check.v3center - neigCheck.v3center).dot(Check.v3center - neigCheck.v3center)) );

    if( rel_dist_centers < dist_thresholdp_inv || rel_dist_centers > dist_thresholdp )
    {
        return false;
    }

//    vdif_normal.push_back(rel_dist_centers);
//
//    float res = 0.0;
//    int hs = 0;
//    for (auto item : vdif_normal)
//    {
//        if (item > 1.0) {
//            res += item;
//            hs++;
//        }
//    }
//
//    res = res / hs;
//
//    std::cout<<"res "<<res<<std::endl;

    return true;
}







void SubgraphMatcher::exploreSubgraphTreeR(set<unsigned> &sourcePlanes, set<unsigned> &targetPlanes, map<unsigned, unsigned> &matched)
{

    unsigned requiredMatches = static_cast<unsigned>(winnerMatch.size());

    while(!sourcePlanes.empty())
    {
        set<unsigned>::iterator it1 = sourcePlanes.begin();
        if( (matched.size() + min(sourcePlanes.size(),targetPlanes.size())) <= requiredMatches )
        {
//            cout << "End branch recursive search. Too short " << matched.size() << " prev winner " << winnerMatch.size() << endl;
            return;
        }

        for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
        {
            if( hashUnaryConstraints[*it1][*it2] != 1 )
                continue;
            bool binaryFail = false;
            for(map<unsigned, unsigned>::iterator it_matched = matched.begin(); it_matched != matched.end(); it_matched++)
                if( !evalBinaryConstraints(subgraphSrc->vPlanes[*it1], subgraphSrc->vPlanes[it_matched->first], subgraphTrg->vPlanes[*it2], subgraphTrg->vPlanes[it_matched->second]) )
                {
                    binaryFail = true;
                    break;
                }
            if(binaryFail)
                continue;

        

            // If this point is reached, the planes it1 and it2 are candidates to be the same
            set<unsigned> nextSrcPlanes = sourcePlanes;

//            std::cout<<"nextSrcPlanes.size() "<<nextSrcPlanes.size()<<std::endl;
            nextSrcPlanes.erase(*it1);
//            std::cout<<"nextSrcPlanes.size() "<<nextSrcPlanes.size()<<std::endl;

            set<unsigned> nextTrgPlanes = targetPlanes;
            nextTrgPlanes.erase(*it2);
            map<unsigned, unsigned> nextMatched = matched;
            nextMatched[*it1] = *it2;

//            std::cout<<*it1<<" "<<*it2<<std::endl;
//            hashUnaryConstraints[*it1][*it2] = 0;

            exploreSubgraphTreeR(nextSrcPlanes, nextTrgPlanes, nextMatched);


//            //new add
//            hashUnaryConstraints[*it1][*it2] = 0;
//            map<unsigned, unsigned> nextMatched = matched;
//            nextMatched[*it1] = *it2;
//
//            exploreSubgraphTreeR(sourcePlanes, targetPlanes, nextMatched);
        }
        sourcePlanes.erase(it1);
    } // End while

    if (matched.size() > 1) {
//        std::cout << "matched.size() " << matched.size() << std::endl;

        allMatchGroups.push_back(matched);
        allMatchGroupsSize.push_back(matched.size());

        float aveh = 0.0;
        for (auto vh : vdif_height)
        {
            aveh += vh;
        }
        aveh = aveh / vdif_height.size();
        allMatchGroups_height.push_back(aveh);

        // aveh = 0.0;
        // for (auto vh : vdif_normal)
        // {
        //     aveh += vh;
        // }
        // aveh = aveh / vdif_normal.size();
        // allMatchGroups_normal.push_back(aveh);

        // aveh = 0.0;
        // for (auto vh : vrel_dist_centers)
        // {
        //     aveh += vh;
        // }
        // aveh = aveh / vrel_dist_centers.size();
        // allMatchGroups_dist_centers.push_back(aveh);

        aveh = 0.0;
        for (auto vh : vah)
        {
            aveh += vh;
        }
        aveh = aveh / vah.size();
        allMatchGroups_dist_centers.push_back(aveh);

        aveh = 0.0;
        for (auto vh : vea)
        {
            aveh += vh;
        }
        aveh = aveh / vea.size();
        allMatchGroups_normal.push_back(aveh);

        vah.clear();
        vea.clear();
        
        vdif_height.clear();
        vrel_dist_centers.clear();
        vdif_normal.clear();

//        for (auto itm : matched)
//        {
//            allwinnerMatch.insert(itm);
//        }
//
//
//        float area1 = 0.0;
//        for(map<unsigned, unsigned>::iterator it_matched = matched.begin(); it_matched != matched.end(); it_matched++) {
//            area1 += subgraphSrc->vPlanes[it_matched->first].areaHull;
//        }
//        std::cout<<area1<<std::endl;
    }

    float area1 = 0.0;
    for(map<unsigned, unsigned>::iterator it_matched = matched.begin(); it_matched != matched.end(); it_matched++) {
        area1 += subgraphSrc->vPlanes[it_matched->first].areaHull;
    }

    float com = area1 * matched.size();


    if(com >= winnerMatchArea)
    {
        winnerMatchArea = com;
        winnerMatch = matched;
    }

}






// void SubgraphMatcher::exploreSubgraphTreeR(set<unsigned> &sourcePlanes, set<unsigned> &targetPlanes, map<unsigned, unsigned> &matched)
// {

//     unsigned requiredMatches = static_cast<unsigned>(winnerMatch.size());

//     //受初值影响很大！！！！！！！！！改进点！！！！！！！！！！！
//     //while(!sourcePlanes.empty())
//     for (set<unsigned>::iterator it1 = sourcePlanes.begin() ; it1 != sourcePlanes.end() ; ++it1)
//     {
//         if( (matched.size() + min(sourcePlanes.size(),targetPlanes.size())) <= requiredMatches )
//         {
// //            cout << "End branch recursive search. Too short " << matched.size() << " prev winner " << winnerMatch.size() << endl;
//             return;
//         }

//         for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
//         {
//             if( hashUnaryConstraints[*it1][*it2] != 1 )
//                 continue;
//             bool binaryFail = false;
//             for(map<unsigned, unsigned>::iterator it_matched = matched.begin(); it_matched != matched.end(); it_matched++)
//                 if( !evalBinaryConstraints(subgraphSrc->vPlanes[*it1], subgraphSrc->vPlanes[it_matched->first], subgraphTrg->vPlanes[*it2], subgraphTrg->vPlanes[it_matched->second]) )
//                 {
//                     binaryFail = true;
//                     break;
//                 }
//             if(binaryFail)
//                 continue;

        

//             // If this point is reached, the planes it1 and it2 are candidates to be the same
//             set<unsigned> nextSrcPlanes = sourcePlanes;

// //            std::cout<<"nextSrcPlanes.size() "<<nextSrcPlanes.size()<<std::endl;
//             nextSrcPlanes.erase(*it1);
// //            std::cout<<"nextSrcPlanes.size() "<<nextSrcPlanes.size()<<std::endl;

//             set<unsigned> nextTrgPlanes = targetPlanes;
//             nextTrgPlanes.erase(*it2);
//             map<unsigned, unsigned> nextMatched = matched;
//             nextMatched[*it1] = *it2;

// //            std::cout<<*it1<<" "<<*it2<<std::endl;
// //            hashUnaryConstraints[*it1][*it2] = 0;

//             exploreSubgraphTreeR(nextSrcPlanes, nextTrgPlanes, nextMatched);


// //            //new add
// //            hashUnaryConstraints[*it1][*it2] = 0;
// //            map<unsigned, unsigned> nextMatched = matched;
// //            nextMatched[*it1] = *it2;
// //
// //            exploreSubgraphTreeR(sourcePlanes, targetPlanes, nextMatched);
//         }
//     } // End while

//     if (matched.size() > 2) {
// //        std::cout << "matched.size() " << matched.size() << std::endl;

//         allMatchGroups.push_back(matched);

// //        for (auto itm : matched)
// //        {
// //            allwinnerMatch.insert(itm);
// //        }
// //
// //
// //        float area1 = 0.0;
// //        for(map<unsigned, unsigned>::iterator it_matched = matched.begin(); it_matched != matched.end(); it_matched++) {
// //            area1 += subgraphSrc->vPlanes[it_matched->first].areaHull;
// //        }
// //        std::cout<<area1<<std::endl;
//     }

//     float area1 = 0.0;
//     for(map<unsigned, unsigned>::iterator it_matched = matched.begin(); it_matched != matched.end(); it_matched++) {
//         area1 += subgraphSrc->vPlanes[it_matched->first].areaHull;
//     }

//     float com = area1 * matched.size();


//     if(com >= winnerMatchArea){
// //        if (area1 > winnerMatchArea)
//         {

// //            std::cout<<"area1 "<<area1<<std::endl;
// //            std::cout<<"matched.size() "<<matched.size()<<std::endl;


//             winnerMatchArea = com;
//             winnerMatch = matched;

//         }
//     }

// }



void SubgraphMatcher::exploreSubgraphTreeRVehicle(set<unsigned> &sourceVehicles, set<unsigned> &targetVehicles, map<unsigned, unsigned> &matched)
{

    unsigned requiredMatches = static_cast<unsigned>(winnerMatchVehicle.size());

    while(!sourceVehicles.empty())
    {
        set<unsigned>::iterator it1 = sourceVehicles.begin();
        if( (matched.size() + min(sourceVehicles.size(),targetVehicles.size())) <= requiredMatches )
        {
            return;
        }

        for(set<unsigned>::iterator it2 = targetVehicles.begin(); it2 != targetVehicles.end(); it2++)
        {
            if( hashUnaryConstraints[*it1][*it2] != 1 )
                continue;
            bool binaryFail = false;
            for(map<unsigned, unsigned>::iterator it_matched = matched.begin(); it_matched != matched.end(); it_matched++)
                if( !evalBinaryConstraintsVehicle(subgraphSrc->vVehicles[*it1], subgraphSrc->vVehicles[it_matched->first], subgraphTrg->vVehicles[*it2], subgraphTrg->vVehicles[it_matched->second]) )
                {
                    binaryFail = true;
                    break;
                }
            if(binaryFail)
                continue;
            // If this point is reached, the planes it1 and it2 are candidates to be the same
            set<unsigned> nextSrcPlanes = sourceVehicles;
            nextSrcPlanes.erase(*it1);
            set<unsigned> nextTrgPlanes = targetVehicles;
            nextTrgPlanes.erase(*it2);
            map<unsigned, unsigned> nextMatched = matched;
            nextMatched[*it1] = *it2;

            exploreSubgraphTreeRVehicle(nextSrcPlanes, nextTrgPlanes, nextMatched);
        }
        sourceVehicles.erase(it1);
    } // End while

    if (matched.size() > 1)
    {
    }

    if(matched.size() > winnerMatchVehicle.size()){
//        float areaMatched = calcAreaMatched(matched);
//        cout << "End branch recursive search. matched " << matched.size() << " A " << areaMatched << " prev winner " << endl;
        winnerMatchVehicle = matched;
    }
}




void SubgraphMatcher::exploreSubgraphTreeRVehicleWithoutP(set<unsigned> &sourceVehicles, set<unsigned> &targetVehicles, map<unsigned, unsigned> &matched)
{
    if(matched.size() > winnerMatchVehicle.size()){
        winnerMatchVehicle = matched;
    }

    if (matched.size() > 7 || withoutplanevehilceok)
    {
        withoutplanevehilceok = true;
        return;
    }

    unsigned requiredMatches = static_cast<unsigned>(winnerMatchVehicle.size());

    while(!sourceVehicles.empty())
    {
        if (withoutplanevehilceok)
            return;
        set<unsigned>::iterator it1 = sourceVehicles.begin();
        if( (matched.size() + min(sourceVehicles.size(),targetVehicles.size())) <= requiredMatches )
        {
            return;
        }

        for(set<unsigned>::iterator it2 = targetVehicles.begin(); it2 != targetVehicles.end(); it2++)
        {
            if( hashUnaryConstraints[*it1][*it2] != 1 )
                continue;
            bool binaryFail = false;
            for(map<unsigned, unsigned>::iterator it_matched = matched.begin(); it_matched != matched.end(); it_matched++)
                if( !evalBinaryConstraintsVehicle(subgraphSrc->vVehicles[*it1], subgraphSrc->vVehicles[it_matched->first], subgraphTrg->vVehicles[*it2], subgraphTrg->vVehicles[it_matched->second]) )
                {
                    binaryFail = true;
                    break;
                }
            if(binaryFail)
                continue;
            // If this point is reached, the planes it1 and it2 are candidates to be the same
            set<unsigned> nextSrcPlanes = sourceVehicles;
            nextSrcPlanes.erase(*it1);
            set<unsigned> nextTrgPlanes = targetVehicles;
            nextTrgPlanes.erase(*it2);
            map<unsigned, unsigned> nextMatched = matched;
            nextMatched[*it1] = *it2;

            exploreSubgraphTreeRVehicleWithoutP(nextSrcPlanes, nextTrgPlanes, nextMatched);

            if (withoutplanevehilceok)
               return;

        }
        sourceVehicles.erase(it1);
    } // End while

    if(matched.size() > winnerMatchVehicle.size()){
        winnerMatchVehicle = matched;
    }

    if (matched.size() > 7)
    {
        withoutplanevehilceok = true;
        return;
    }
}






void SubgraphMatcher::exploreSubgraphTreeRPole(set<unsigned> &sourcePoles, set<unsigned> &targetPoles, map<unsigned, unsigned> &matched)
{
    unsigned requiredMatches = static_cast<unsigned>(winnerMatchPole.size());
    while(!sourcePoles.empty())
    {
        set<unsigned>::iterator it1 = sourcePoles.begin();
        if( (matched.size() + min(sourcePoles.size(),targetPoles.size())) <= requiredMatches )
        {
            return;
        }

        for(set<unsigned>::iterator it2 = targetPoles.begin(); it2 != targetPoles.end(); it2++)
        {
            if( hashUnaryConstraints[*it1][*it2] != 1 )
                continue;
            bool binaryFail = false;
            for(map<unsigned, unsigned>::iterator it_matched = matched.begin(); it_matched != matched.end(); it_matched++)
                if( !evalBinaryConstraintsPole(subgraphSrc->vPoles[*it1], subgraphSrc->vPoles[it_matched->first], subgraphTrg->vPoles[*it2], subgraphTrg->vPoles[it_matched->second]) )
                {
                    binaryFail = true;
                    break;
                }
            if(binaryFail)
                continue;
            // If this point is reached, the planes it1 and it2 are candidates to be the same
            set<unsigned> nextSrcPlanes = sourcePoles;
            nextSrcPlanes.erase(*it1);
            set<unsigned> nextTrgPlanes = targetPoles;
            nextTrgPlanes.erase(*it2);
            map<unsigned, unsigned> nextMatched = matched;
            nextMatched[*it1] = *it2;

            exploreSubgraphTreeRPole(nextSrcPlanes, nextTrgPlanes, nextMatched);

        }
        sourcePoles.erase(it1);
    } // End while

    if(matched.size() > winnerMatchPole.size()){
        winnerMatchPole = matched;
    }

}



void SubgraphMatcher::exploreSubgraphTreeRPoleWithoutP(set<unsigned> &sourcePoles, set<unsigned> &targetPoles, map<unsigned, unsigned> &matched)
{
    if(matched.size() > winnerMatchPole.size()){
        winnerMatchPole = matched;
    }

    if (matched.size() > 7 || withoutplanecylinderok)
    {
        withoutplanecylinderok = true;
        return;
    }

    unsigned requiredMatches = static_cast<unsigned>(winnerMatchPole.size());

    while(!sourcePoles.empty())
    {
        if (withoutplanecylinderok)
            return;
        set<unsigned>::iterator it1 = sourcePoles.begin();
        if( (matched.size() + min(sourcePoles.size(),targetPoles.size())) <= requiredMatches )
        {
            return;
        }

        for(set<unsigned>::iterator it2 = targetPoles.begin(); it2 != targetPoles.end(); it2++)
        {
            if( hashUnaryConstraints[*it1][*it2] != 1 )
                continue;
            bool binaryFail = false;
            for(map<unsigned, unsigned>::iterator it_matched = matched.begin(); it_matched != matched.end(); it_matched++)
                if( !evalBinaryConstraintsPole(subgraphSrc->vPoles[*it1], subgraphSrc->vPoles[it_matched->first], subgraphTrg->vPoles[*it2], subgraphTrg->vPoles[it_matched->second]) )
                {
                    binaryFail = true;
                    break;
                }
            if(binaryFail)
                continue;
            // If this point is reached, the planes it1 and it2 are candidates to be the same
            set<unsigned> nextSrcPlanes = sourcePoles;
            nextSrcPlanes.erase(*it1);
            set<unsigned> nextTrgPlanes = targetPoles;
            nextTrgPlanes.erase(*it2);
            map<unsigned, unsigned> nextMatched = matched;
            nextMatched[*it1] = *it2;

            exploreSubgraphTreeRPoleWithoutP(nextSrcPlanes, nextTrgPlanes, nextMatched);

            if (withoutplanecylinderok)
               return;
        }
        sourcePoles.erase(it1);
    } // End while

    if(matched.size() > winnerMatchPole.size()){
        winnerMatchPole = matched;
    }

    if (matched.size() > 7)
    {
        withoutplanecylinderok = true;
        return;
    }
}



std::map<unsigned,unsigned> SubgraphMatcher::compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget, int& unaryCount)
{
    subgraphSrc = &subgraphSource;
    subgraphTrg = &subgraphTarget;
    map<unsigned, unsigned> matched;

    winnerMatch.clear();
    allwinnerMatch.clear();
    allMatchGroups.clear();

    winnerMatchArea = 0.0;

    std::set<unsigned> sourcePlanes = subgraphSrc->subgraphPlanesIdx;
    std::set<unsigned> targetPlanes = subgraphTrg->subgraphPlanesIdx;

    // Fill Hash table of unary constraints
    hashUnaryConstraints = std::vector<std::vector<int8_t> >(subgraphSrc->vPlanes.size(), std::vector<int8_t>(subgraphTrg->vPlanes.size()) );

//    bool mValid = false;

    for(set<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++) {
        for (set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++) {

            hashUnaryConstraints[*it1][*it2] = (evalUnaryConstraintsPlane(subgraphSrc->vPlanes[*it1],
                                                                          subgraphTrg->vPlanes[*it2]) ? 1 : 0);
//            if (hashUnaryConstraints[*it1][*it2])
//                mValid = true;
        }
//        for (set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
//            std::cout<<int(hashUnaryConstraints[*it1][*it2])<<" ";
//        std::cout<<std::endl;
    }

    int unaryMatchedsize = 0;

    for(set<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++) {
        for (set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++) {
            if (int(hashUnaryConstraints[*it1][*it2]) == 1)
                unaryMatchedsize++;
        }
    }

//    std::cout<<"unaryMatchedsize "<<unaryMatchedsize<<std::endl;

    unaryCount = unaryMatchedsize;
//
//    std::cout<<"unaryMatchedsize "<<unaryMatchedsize<<std::endl;

    vdif_height.clear();
    vdif_height2.clear();
    vdif_normal.clear();
    vrel_dist_centers.clear();

    vah.clear();
    vea.clear();

    exploreSubgraphTreeR(sourcePlanes, targetPlanes, matched);

    float res = 0.0;
    int hs = 0;
    for (auto item : vdif_height)
    {
        if (item > 1.0) {
            res += item;
            hs++;
        }
    }

    wdif_height = res / hs;

    res = 0.0;
    hs = 0;
    for (auto item : vdif_height2)
    {
        if (item > 1.0) {
            res += item;
            hs++;
        }
    }

    wdif_height2 = res / hs;

    res = 0.0;
    hs = 0;
    for (auto item : vrel_dist_centers)
    {
        if (item > 1.0) {
            res += item;
            hs++;
        }
    }

    wrel_dist_centers = res / hs;

    res = 0.0;
    hs = 0;
    for (auto item : vah)
    {
        if (item > 1.0) {
            res += item;
            hs++;
        }
    }

    wal = res / hs;

    res = 0.0;
    hs = 0;
    for (auto item : vea)
    {
        if (item > 1.0) {
            res += item;
            hs++;
        }
    }

    wea = res / hs;


//    std::cout<<"winnerMatch.size() "<<winnerMatch.size()<<std::endl;
//    std::cout<<"winnerMatchArea "<<winnerMatchArea<<std::endl;


//    std::set<unsigned> sourcePlanes1 = subgraphSrc->subgraphPlanesIdx;
//    std::set<unsigned> targetPlanes1 = subgraphTrg->subgraphPlanesIdx;
//
//    unaryMatchedsize = 0;
//
//    std::cout<<"sourcePlanes.size() "<<sourcePlanes1.size()<<std::endl;
//    for(set<unsigned>::iterator it1 = sourcePlanes1.begin(); it1 != sourcePlanes1.end(); it1++) {
//        for (set<unsigned>::iterator it2 = targetPlanes1.begin(); it2 != targetPlanes1.end(); it2++) {
//            if (int(hashUnaryConstraints[*it1][*it2]) == 1)
//                unaryMatchedsize++;
//        }
//    }
//    std::cout<<"unaryMatchedsize "<<unaryMatchedsize<<std::endl;
//
//
//
//    winnerMatch.clear();
//    map<unsigned, unsigned> matched1;
//
//    exploreSubgraphTreeR(sourcePlanes1, targetPlanes1, matched1);
//
//    std::cout<<"winnerMatch.size() "<<winnerMatch.size()<<std::endl;
//
//
//    std::set<unsigned> sourcePlanes2 = subgraphSrc->subgraphPlanesIdx;
//    std::set<unsigned> targetPlanes2 = subgraphTrg->subgraphPlanesIdx;
//
//    unaryMatchedsize = 0;
//
//    for(set<unsigned>::iterator it1 = sourcePlanes2.begin(); it1 != sourcePlanes2.end(); it1++) {
//        for (set<unsigned>::iterator it2 = targetPlanes2.begin(); it2 != targetPlanes2.end(); it2++) {
//            if (int(hashUnaryConstraints[*it1][*it2]) == 1)
//                unaryMatchedsize++;
//        }
//    }
//    std::cout<<"unaryMatchedsize "<<unaryMatchedsize<<std::endl;
//
//
//    winnerMatch.clear();
//    map<unsigned, unsigned> matched2;
//
//    exploreSubgraphTreeR(sourcePlanes2, targetPlanes2, matched2);
//
//    std::cout<<"winnerMatch.size() "<<winnerMatch.size()<<std::endl;
//
//    unaryMatchedsize = 0;
//
//    for(set<unsigned>::iterator it1 = sourcePlanes2.begin(); it1 != sourcePlanes2.end(); it1++) {
//        for (set<unsigned>::iterator it2 = targetPlanes2.begin(); it2 != targetPlanes2.end(); it2++) {
//            if (int(hashUnaryConstraints[*it1][*it2]) == 1)
//                unaryMatchedsize++;
//        }
//    }
//    std::cout<<"unaryMatchedsize "<<unaryMatchedsize<<std::endl;


    return winnerMatch;
}


std::map<unsigned,unsigned> SubgraphMatcher::compareSubgraphsVehiclePlaneRef(Subgraph &subgraphSource, Subgraph &subgraphTarget, int& unaryCount,
                                                                             std::vector<Eigen::Vector3f>& kvc, std::vector<Eigen::Vector3f>& lvc,
                                                                             std::vector<Eigen::Vector3f>& kvn, std::vector<Eigen::Vector3f>& lvn)
{
    c_kvc = kvc;
    c_lvc = lvc;
    c_kvn = kvn;
    c_lvn = lvn;

    subgraphSrc = &subgraphSource;
    subgraphTrg = &subgraphTarget;
    map<unsigned, unsigned> matched;

    winnerMatchVehicle.clear();

    std::set<unsigned> sourceVehicles = subgraphSrc->subgraphVehiclesIdx;
    std::set<unsigned> targetVehicles = subgraphTrg->subgraphVehiclesIdx;

    hashUnaryConstraints = std::vector<std::vector<int8_t> >(subgraphSrc->vVehicles.size(), std::vector<int8_t>(subgraphTrg->vVehicles.size()) );

    for(set<unsigned>::iterator it1 = sourceVehicles.begin(); it1 != sourceVehicles.end(); it1++) {
        for (set<unsigned>::iterator it2 = targetVehicles.begin(); it2 != targetVehicles.end(); it2++) {

            hashUnaryConstraints[*it1][*it2] = (evalUnaryConstraintsVehicle(subgraphSrc->vVehicles[*it1],
                                                                            subgraphTrg->vVehicles[*it2]) ? 1 : 0);
        }
//        for (set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
//            std::cout<<int(hashUnaryConstraints[*it1][*it2])<<" ";
//        std::cout<<std::endl;
    }

    int unaryMatchedsize = 0;

    for(set<unsigned>::iterator it1 = sourceVehicles.begin(); it1 != sourceVehicles.end(); it1++) {
        for (set<unsigned>::iterator it2 = targetVehicles.begin(); it2 != targetVehicles.end(); it2++) {
            if (int(hashUnaryConstraints[*it1][*it2]) == 1)
                unaryMatchedsize++;
        }
    }

    unaryCount = unaryMatchedsize;
//
//    std::cout<<"unaryMatchedsize "<<unaryMatchedsize<<std::endl;

    exploreSubgraphTreeRVehicle(sourceVehicles, targetVehicles, matched);
    return winnerMatchVehicle;
}




std::map<unsigned,unsigned> SubgraphMatcher::compareSubgraphsVehicleWithoutPlaneRef(Subgraph &subgraphSource, Subgraph &subgraphTarget, int& unaryCount)
{
    subgraphSrc = &subgraphSource;
    subgraphTrg = &subgraphTarget;
    map<unsigned, unsigned> matched;

    winnerMatchVehicle.clear();

    std::set<unsigned> sourceVehicles = subgraphSrc->subgraphVehiclesIdx;
    std::set<unsigned> targetVehicles = subgraphTrg->subgraphVehiclesIdx;

    hashUnaryConstraints = std::vector<std::vector<int8_t> >(subgraphSrc->vVehicles.size(), std::vector<int8_t>(subgraphTrg->vVehicles.size()) );

    for(set<unsigned>::iterator it1 = sourceVehicles.begin(); it1 != sourceVehicles.end(); it1++) {
        for (set<unsigned>::iterator it2 = targetVehicles.begin(); it2 != targetVehicles.end(); it2++) {

            // hashUnaryConstraints[*it1][*it2] = 1;

            hashUnaryConstraints[*it1][*it2] = (evalUnaryConstraintsVehicleWithoutP(subgraphSrc->vVehicles[*it1],
                                                                            subgraphTrg->vVehicles[*it2]) ? 1 : 0);
        }
//        for (set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
//            std::cout<<int(hashUnaryConstraints[*it1][*it2])<<" ";
//        std::cout<<std::endl;
    }

    int unaryMatchedsize = 0;

    // for(set<unsigned>::iterator it1 = sourceVehicles.begin(); it1 != sourceVehicles.end(); it1++) {
    //     for (set<unsigned>::iterator it2 = targetVehicles.begin(); it2 != targetVehicles.end(); it2++) {
    //         if (int(hashUnaryConstraints[*it1][*it2]) == 1)
    //             unaryMatchedsize++;
    //     }
    // }

    unaryCount = unaryMatchedsize;
//
//    std::cout<<"unaryMatchedsize "<<unaryMatchedsize<<std::endl;

    dist_thresholdv = 1.05;

    withoutplanevehilceok = false;

    exploreSubgraphTreeRVehicleWithoutP(sourceVehicles, targetVehicles, matched);

    // dist_thresholdv = 1.30;
    dist_thresholdv = 1.07;

    return winnerMatchVehicle;
}






std::map<unsigned,unsigned> SubgraphMatcher::compareSubgraphsPolePlaneRef(Subgraph &subgraphSource, Subgraph &subgraphTarget, int& unaryCount,
                                                                             std::vector<Eigen::Vector3f>& kvc, std::vector<Eigen::Vector3f>& lvc,
                                                                             std::vector<Eigen::Vector3f>& kvn, std::vector<Eigen::Vector3f>& lvn)
{
    c_kvc = kvc;
    c_lvc = lvc;
    c_kvn = kvn;
    c_lvn = lvn;



    subgraphSrc = &subgraphSource;
    subgraphTrg = &subgraphTarget;
    map<unsigned, unsigned> matched;

    winnerMatchPole.clear();

    std::set<unsigned> sourcePoles = subgraphSrc->subgraphPolesIdx;
    std::set<unsigned> targetPoles = subgraphTrg->subgraphPolesIdx;

    hashUnaryConstraints = std::vector<std::vector<int8_t> >(subgraphSrc->vPoles.size(), std::vector<int8_t>(subgraphTrg->vPoles.size()) );

    for(set<unsigned>::iterator it1 = sourcePoles.begin(); it1 != sourcePoles.end(); it1++) {
        for (set<unsigned>::iterator it2 = targetPoles.begin(); it2 != targetPoles.end(); it2++) {

            hashUnaryConstraints[*it1][*it2] = (evalUnaryConstraintsPole(subgraphSrc->vPoles[*it1],
                                                                            subgraphTrg->vPoles[*it2]) ? 1 : 0);
        }
//        for (set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
//            std::cout<<int(hashUnaryConstraints[*it1][*it2])<<" ";
//        std::cout<<std::endl;
    }

    int unaryMatchedsize = 0;

    for(set<unsigned>::iterator it1 = sourcePoles.begin(); it1 != sourcePoles.end(); it1++) {
        for (set<unsigned>::iterator it2 = targetPoles.begin(); it2 != targetPoles.end(); it2++) {
            if (int(hashUnaryConstraints[*it1][*it2]) == 1)
                unaryMatchedsize++;
        }
    }

    unaryCount = unaryMatchedsize;
//
//    std::cout<<"unaryMatchedsize "<<unaryMatchedsize<<std::endl;

    exploreSubgraphTreeRPole(sourcePoles, targetPoles, matched);
    return winnerMatchPole;
}


std::map<unsigned,unsigned> SubgraphMatcher::compareSubgraphsPoleWithoutPlaneRef(Subgraph &subgraphSource, Subgraph &subgraphTarget, int& unaryCount)
{
    subgraphSrc = &subgraphSource;
    subgraphTrg = &subgraphTarget;
    map<unsigned, unsigned> matched;

    winnerMatchPole.clear();

    std::set<unsigned> sourcePoles = subgraphSrc->subgraphPolesIdx;
    std::set<unsigned> targetPoles = subgraphTrg->subgraphPolesIdx;

    hashUnaryConstraints = std::vector<std::vector<int8_t> >(subgraphSrc->vPoles.size(), std::vector<int8_t>(subgraphTrg->vPoles.size()) );

    for(set<unsigned>::iterator it1 = sourcePoles.begin(); it1 != sourcePoles.end(); it1++) {
        for (set<unsigned>::iterator it2 = targetPoles.begin(); it2 != targetPoles.end(); it2++) {

            // hashUnaryConstraints[*it1][*it2] = 1;

            hashUnaryConstraints[*it1][*it2] = (evalUnaryConstraintsPoleWithoutP(subgraphSrc->vPoles[*it1],
                                                                            subgraphTrg->vPoles[*it2]) ? 1 : 0);
        }
//        for (set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
//            std::cout<<int(hashUnaryConstraints[*it1][*it2])<<" ";
//        std::cout<<std::endl;
    }

    int unaryMatchedsize = 0;

    // for(set<unsigned>::iterator it1 = sourcePoles.begin(); it1 != sourcePoles.end(); it1++) {
    //     for (set<unsigned>::iterator it2 = targetPoles.begin(); it2 != targetPoles.end(); it2++) {
    //         if (int(hashUnaryConstraints[*it1][*it2]) == 1)
    //             unaryMatchedsize++;
    //     }
    // }

    unaryCount = unaryMatchedsize;
//
//    std::cout<<"unaryMatchedsize "<<unaryMatchedsize<<std::endl;


    dist_thresholdp = 1.02;

    withoutplanecylinderok = false;

    exploreSubgraphTreeRPoleWithoutP(sourcePoles, targetPoles, matched);

    withoutplanecylinderok = false;


    dist_thresholdp = 1.03;


    return winnerMatchPole;
}

