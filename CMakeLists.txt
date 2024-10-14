cmake_minimum_required(VERSION 3.5)
project(wolf_controller_core)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 nor C++14 support. Please use a different C++ compiler.")
endif()

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    catkin_package(INCLUDE_DIRS include LIBRARIES wpg controller_core CATKIN_DEPENDS wolf_wbid)
elseif(${ament_cmake_FOUND})
    ament_export_include_directories(include)
    ament_export_libraries(wpg controller_core)
    ament_package()
endif()

# Eigen
find_package(Eigen3 REQUIRED)
# WoLF
find_package(wolf_wbid REQUIRED)
find_package(wolf_estimation REQUIRED)
find_package(wolf_controller_utils REQUIRED)
# RT gui
find_package(rt_gui QUIET)
# RT logger
find_package(rt_logger QUIET)

if(${rt_gui_FOUND})
    message(STATUS "Found rt_gui, creating interface for wolf_controller.")
    add_definitions(-DRT_GUI)
else()
    message(STATUS "rt_gui not found.")
endif()

if(${rt_logger_FOUND})
    message(STATUS "Found rt_logger, creating loggers for wolf_controller.")
    add_definitions(-DRT_LOGGER)
else()
    message(STATUS "rt_logger not found.")
endif()

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${wolf_wbid_INCLUDE_DIRS}
    ${wolf_estimation_INCLUDE_DIRS}
    ${wolf_controller_utils_INCLUDE_DIRS}
    ${rt_gui_INCLUDE_DIRS}
    ${rt_logger_INCLUDE_DIRS}
)

add_library(wpg SHARED
    src/wpg/com_planner.cpp
    src/wpg/foot_state_machine.cpp
    src/wpg/foot_trajectory_ellipse.cpp
    src/wpg/foot_trajectory_interface.cpp
    src/wpg/footholds_planner.cpp
    src/wpg/gait_generator.cpp)
target_link_libraries(wpg ${wolf_wbid_LIBRARIES} ${wolf_estimation_LIBRARIES} ${wolf_controller_utils_LIBRARIES} ${rt_gui_LIBRARIES} ${rt_logger_LIBRARIES})

add_library(controller_core SHARED
    src/controller_core.cpp
    src/state_machine.cpp
    src/state_estimator.cpp
    src/impedance.cpp
    src/terrain_estimator.cpp
    src/force_estimator.cpp)
target_link_libraries(controller_core wpg ${wolf_wbid_LIBRARIES} ${wolf_estimation_LIBRARIES} ${wolf_controller_utils_LIBRARIES} ${rt_gui_LIBRARIES} ${rt_logger_LIBRARIES})


install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

install(
  TARGETS wpg controller_core
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)
