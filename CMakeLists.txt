cmake_minimum_required(VERSION 2.8.3)
project(LiPMatch)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
  rangenet_lib
)


find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)

include_directories(/home/jjwen/software/slam6d-code/include)


include_directories(
		include
		include/LiPMatch
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${rangenet_lib_INCLUDE_DIRS}
	${TENSORRT_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs visualization_msgs
  rangenet_lib
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)




SET(LIBLiPMatch_SRC src/LiPMatch.cpp
		            src/SubgraphMatcher.cpp)

ADD_LIBRARY(LiPMatch SHARED ${LIBLiPMatch_SRC})
target_link_libraries(LiPMatch ${PCL_LIBRARIES} ${CERES_LIBRARIES}
		/home/jjwen/software/slam6d-code/obj/libshape.a)





add_executable(ascanRegistration src/scanRegistration.cpp
		                         src/RangenetAPI.cpp)
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${OpenCV_LIBS})

add_executable(alaserOdometry src/laserOdometry.cpp)
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp
		                     src/RangenetAPI.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
		${OpenCV_LIBS}
		LiPMatch
		)

#add_executable(loopclosureFast src/loopclosureFast.cpp)
#target_link_libraries(loopclosureFast ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS} LiPMatch)


add_executable(makeTargetMap src/makeTargetMap.cpp
                             src/makeTargetmapcore.cpp)
target_link_libraries(makeTargetMap ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})


add_executable(localizationFast src/localizationFast.cpp)
target_link_libraries(localizationFast ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS} LiPMatch)


message(STATUS "Without OpenMP")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=1 -DOPENMP_NUM_THREADS=1")


add_executable(kittiHelper src/kittiHelper.cpp
		                   src/RangenetAPI.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})




