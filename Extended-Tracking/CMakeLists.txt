cmake_minimum_required(VERSION 2.8)
project(Extended_tracking)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()


#======================ARToolkit configuration======================================
#set(CMAKE_C_FLAGS "-O3 -fPIC -march=core2 -DHAVE_NFT=1 -I/usr/include/x86_64-linux-gnu -pthread -I/usr/include/gstreamer-0.10 -I/usr/
#include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/libxml2")
#set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -fPIC -march=core2 -DHAVE_NFT=1 -I/usr/include/x86_64-linux-gnu -pthread -I/usr/include/
#gstreamer-0.10 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/libxml2")




#======================ORB_SLAM configuration=======================================
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native -std=c++11 )




#=====================Extended-tracking configuration=========================================
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -fPIC -DHAVE_NFT=1 -I/usr/include/x86_64-linux-gnu -pthread -I/usr/include/gstreamer-0.10 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/libxml2")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -std=c++11 -fPIC -DHAVE_NFT=1 -I/usr/include/x86_64-linux-gnu -pthread -I/usr/include/gstreamer-0.10 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/libxml2")





#======================build ORB-SLAM2 libs========================================

add_definitions(-DCOMPILEDWITHC11)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/examples/extented-tracking
${PROJECT_SOURCE_DIR}/include/linux-x86_64
)

link_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/lib/linux-x86_64
${PROJECT_SOURCE_DIR}/lib
/usr/lib/x86_64-linux-gnu
/usr/lib64
)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(ORB_SLAM2 SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
)

target_link_libraries(ORB_SLAM2
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

#===========================================================================




#================build Extended-tracking==================================

set(SRC_DIR ${PROJECT_SOURCE_DIR}/examples/extented-tracking)
set(SRC_LIST ${SRC_DIR}/extended-tracking ${SRC_DIR}/ARMarkerNFT.cc ${SRC_DIR}/trackingSub.cc)
set(LIBS KPM AR2 ARUtil ARgsub_lite ARvideo AR ARICP AR glut GLU GL X11 m pthread jpeg gstreamer-0.10 gobject-2.0
gmodule-2.0 pthread gthread-2.0 glib-2.0 xml2 dc1394 raw1394 ORB_SLAM2 pangolin opencv_core opencv_imgcodecs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(extended-tracking
${SRC_LIST}
)
target_link_libraries(extended-tracking ${LIBS})

#=========================================================================
