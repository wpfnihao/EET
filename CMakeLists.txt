cmake_minimum_required(VERSION 2.4.6)
PROJECT(endeffector_tracking)

SET(CMAKE_BUILD_TYPE Debug)

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# VISP
FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
	INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include)

ADD_LIBRARY(kltFbTracker src/kltFbTracker.cpp)
ADD_LIBRARY(mbtEdgeTracker src/mbtEdgeTracker.cpp)
ADD_LIBRARY(condensPose src/condensPose.cpp)
ADD_LIBRARY(mbtEdgeKltTracker src/mbtEdgeKltTracker.cpp)
ADD_LIBRARY(mbtKltTracker src/mbtKltTracker.cpp)
ADD_EXECUTABLE(main src/main.cpp)
TARGET_LINK_LIBRARIES(main ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(main kltFbTracker)
TARGET_LINK_LIBRARIES(main mbtEdgeTracker)
TARGET_LINK_LIBRARIES(main condensPose)
TARGET_LINK_LIBRARIES(main mbtEdgeKltTracker)
TARGET_LINK_LIBRARIES(main mbtKltTracker)
