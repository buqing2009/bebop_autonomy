cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME bebopDriver)
project(${PROJECT_NAME})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries




set(BEBOPDRIVER_SOURCE_DIR
	src)
	
set(BEBOPDRIVER_INCLUDE_DIR
	src/include
	)

set(BEBOPDRIVER_SOURCE_FILES
	src/source/Bebop.cpp
	)
	
set(BEBOPDRIVER_HEADER_FILES
	src/include/Bebop.h
	)


find_package(catkin REQUIRED
	COMPONENTS  roscpp std_msgs sensor_msgs cv_bridge image_transport droneMsgs)

#OpenCV
find_package(OpenCV REQUIRED)


catkin_package(
        INCLUDE_DIRS ${BEBOPDRIVER_INCLUDE_DIR}
        LIBRARIES bebopDriver
	DEPENDS OpenCV
        CATKIN_DEPENDS  roscpp std_msgs sensor_msgs cv_bridge droneMsgs image_transport 
  )



include_directories(${BEBOPDRIVER_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})







add_library(bebopDriver ${BEBOPDRIVER_SOURCE_FILES} ${BEBOPDRIVER_HEADER_FILES})
add_dependencies(bebopDriver ${catkin_EXPORTED_TARGETS})
target_link_libraries(bebopDriver ${OpenCV_LIBS})
target_link_libraries(bebopDriver ${catkin_LIBRARIES})




