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

# Eigen
find_package(Eigen3 REQUIRED)
# ADVR Packages
find_package(OpenSoT REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(ModelInterfaceRBDL REQUIRED)
# WoLF
find_package(wolf_wbid REQUIRED)
find_package(wolf_estimation REQUIRED)
find_package(wolf_controller_utils REQUIRED)
# RT gui
find_package(rt_gui QUIET)

if(${rt_gui_FOUND})
    message(STATUS "Found rt_gui, creating interface for wolf_controller.")
    add_definitions(-DRT_GUI)
else()
    message(STATUS "rt_gui not found.")
endif()

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS wolf_msgs wolf_controller_utils
#)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenSoT_INCLUDE_DIRS}
    ${XBotInterface_INCLUDE_DIRS}
    ${ModelInterfaceRBDL_INCLUDE_DIRS}
    ${wolf_wbid_INCLUDE_DIRS}
    ${wolf_estimation_INCLUDE_DIRS}
    ${wolf_controller_utils_INCLUDE_DIRS}
    ${rt_gui_INCLUDE_DIRS}
    ${ocs2_msgs_INCLUDE_DIRS}
)

#add_library(ros_wrappers src/ros_wrappers/cartesian.cpp src/ros_wrappers/com.cpp src/ros_wrappers/momentum.cpp
#    src/ros_wrappers/postural.cpp src/ros_wrappers/wrench.cpp src/ros_wrappers/controller.cpp)
#target_link_libraries(ros_wrappers ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES}
#    ${XBotInterface_LIBRARIES} ${rt_gui_LIBRARIES})

add_library(wpg
    src/wpg/com_planner.cpp
    src/wpg/foot_state_machine.cpp
    src/wpg/foot_trajectory_ellipse.cpp
    src/wpg/foot_trajectory_interface.cpp
    src/wpg/footholds_planner.cpp
    src/wpg/gait_generator.cpp)
target_link_libraries(wpg ${wolf_wbid_LIBRARIES} ${wolf_estimation_LIBRARIES} ${rt_gui_LIBRARIES})

add_library(wolf_controller_core
    src/controller_core.cpp
    src/state_machine.cpp
    src/state_estimator.cpp
    src/impedance.cpp
    src/terrain_estimator.cpp
    src/force_estimator.cpp)
target_link_libraries(wolf_controller_core wpg ${wolf_wbid_LIBRARIES} ${wolf_estimation_LIBRARIES} ${wolf_controller_utils_LIBRARIES} ${rt_gui_LIBRARIES})

#add_executable(plot_node src/plot_node.cpp)
#target_link_libraries(plot_node ${catkin_LIBRARIES})

#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(TARGETS wpg wolf_controller_core
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(FILES controller_plugin.xml
#DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  PATTERN ".svn" EXCLUDE)
#
#install(DIRECTORY params/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
#  PATTERN ".svn" EXCLUDE)
#
#install(DIRECTORY resources/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
#  PATTERN ".svn" EXCLUDE)

#install(DIRECTORY rviz/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
#  PATTERN ".svn" EXCLUDE)
#
#install(DIRECTORY plotjuggler/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plotjuggler
#  PATTERN ".svn" EXCLUDE)
#
#catkin_install_python(PROGRAMS scripts/keyboard_node
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#if (CATKIN_ENABLE_TESTING)
#  find_package(catkin REQUIRED COMPONENTS roscpp rostest urdf controller_manager hardware_interface wolf_hardware_interface std_msgs)
#  find_package(OpenSoT REQUIRED)
#  include_directories(include ${catkin_INCLUDE_DIRS})
#
#  add_library(robot_dummy test/robot_dummy.cpp)
#  target_link_libraries(robot_dummy ${catkin_LIBRARIES} ${PROJECT_NAME})
#
#  add_executable(test_state_estimation test/test_state_estimation.cpp)
#  target_link_libraries(test_state_estimation ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenSoT_LIBRARIES})
#
#  #add_executable(test_id_problem test/test_id_problem.cpp)
#  #target_link_libraries(test_id_problem ${catkin_LIBRARIES} ${PROJECT_NAME} gtest)
#
#  #add_executable(test_controller test/test_controller.cpp )
#  #target_link_libraries(test_controller ${catkin_LIBRARIES} ${PROJECT_NAME} gtest robot_dummy)
#
#  add_executable(test_trajectory test/test_trajectory.cpp)
#  target_link_libraries(test_trajectory ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenSoT_LIBRARIES} gtest)
#
#  add_rostest_gtest(test_controller
#    test/test_controller.test
#    test/test_controller.cpp)
#  target_link_libraries(test_controller ${catkin_LIBRARIES} ${PROJECT_NAME} gtest robot_dummy)
#
#  add_rostest_gtest(test_id_problem
#    test/test_id_problem.test
#    test/test_id_problem.cpp)
#  target_link_libraries(test_id_problem ${catkin_LIBRARIES} ${PROJECT_NAME} gtest)
#endif()
