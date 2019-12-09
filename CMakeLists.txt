cmake_minimum_required (VERSION 3.5.1)

# support for Libnabo  
SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
# Fallback to cmake_modules
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
# Possibly map additional variables to the EIGEN3_ prefix.
else()
	message("Eigen found")
	set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


find_program(_OpenRAVE_CONFIG_EXECUTABLE NAMES ${_OpenRAVE_CONFIG_NAME} DOC "openrave executable")
  if( _OpenRAVE_CONFIG_EXECUTABLE )
    get_filename_component(_OpenRAVE_PATH "${_OpenRAVE_CONFIG_EXECUTABLE}" PATH) # bin
    get_filename_component(_OpenRAVE_PATH "${_OpenRAVE_PATH}" PATH)
    set(_OpenRAVE_PATHS ${_OpenRAVE_PATHS} ${_OpenRAVE_PATH})
  endif( _OpenRAVE_CONFIG_EXECUTABLE )
find_package(OpenRAVE NO_MODULE PATHS ${_OpenRAVE_PATHS})


message("OpenRave Path")
message(STATUS "${_OpenRAVE_PATHS}")
message(STATUS "${_OpenRAVE_CONFIG_EXECUTABLE}")

# include_directories(${_OpenRAVE_PATHS}/include/openrave-0.9/)
include_directories(/usr/local/include/)
include_directories(/usr/local/include/openrave-0.9/)
# include_directories(/usr/local/include/openrave-0.9)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/data)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/collada_urdf/collada_urdf/include/)
include_directories(${OpenRAVE_INCLUDE_DIRS})

# ROS
include_directories(/opt/ros/kinetic/include)
include_directories(/opt/ros/kinetic/include/kdl)
include_directories(/opt/ros/kinetic/include/urdf)
link_directories(/opt/ros/kinetic/lib)


link_directories(/usr/local/lib)
link_directories(/usr/local/lib/openrave0.9-plugins/)

set(SOURCES_main src/test.cpp)
set(SOURCES_verify src/verify_ik.cpp)
set(SOURCES_smZ src/test_smZ.cpp)
set(SOURCES_convert src/gen_analytical_ik.cpp)

set(DEP_LIB urdf
			${PROJECT_SOURCE_DIR}/collada_urdf/collada_urdf/build/devel/lib/libcollada_urdf.so
			/usr/lib/x86_64-linux-gnu/libboost_iostreams.a
			/usr/lib/x86_64-linux-gnu/libboost_system.a
			/usr/lib/x86_64-linux-gnu/libboost_filesystem.a
			/usr/lib/x86_64-linux-gnu/libutil.a
			/usr/local/lib/openrave0.9-plugins/libbasecontrollers.so
			/usr/local/lib/openrave0.9-plugins/libbaserobots.so
			/usr/local/lib/openrave0.9-plugins/libbasesamplers.so
			/usr/local/lib/openrave0.9-plugins/libbasesensors.so
			/usr/local/lib/openrave0.9-plugins/libconfigurationcache.so
			/usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_0_9/openravepy_configurationcache.so
			/usr/local/lib/openrave0.9-plugins/libdualmanipulation.so
			/usr/local/lib/openrave0.9-plugins/libgrasper.so
			/usr/local/lib/openrave0.9-plugins/libikfastsolvers.so
			/usr/local/lib/openrave0.9-plugins/liblogging.so
			/usr/local/lib/openrave0.9-plugins/libpqprave.so
			/usr/local/lib/openrave0.9-plugins/librmanipulation.so
			/usr/local/lib/openrave0.9-plugins/librplanners.so
			/usr/local/lib/openrave0.9-plugins/libtextserver.so
			/usr/lib/x86_64-linux-gnu/libboost_thread.a
			/usr/local/lib/openrave0.9-plugins/libfclrave.so)


add_executable (test_ikfast ${SOURCES_main})
add_executable (gen_analytical_ik ${SOURCES_convert})
add_executable (verify_ik ${SOURCES_verify})
add_executable (test_smZ ${SOURCES_smZ})

target_link_libraries(test_ikfast ${DEP_LIB})
target_link_libraries(gen_analytical_ik ${DEP_LIB})
target_link_libraries(verify_ik ${DEP_LIB})
target_link_libraries(test_smZ ${DEP_LIB})

