cmake_minimum_required(VERSION 3.1)
project(dc_pmppi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    nav_msgs
    geometry_msgs
    std_msgs
    sensor_msgs
    tf
    ackermann_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp nav_msgs geometry_msgs std_msgs sensor_msgs tf
)

# 内嵌 xtensor 第三方库路径
set(XTENSOR_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/xtensor/include
    ${CMAKE_CURRENT_SOURCE_DIR}/xsimd/include
    ${CMAKE_CURRENT_SOURCE_DIR}/xtl/include
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
    message(STATUS "dc_pmppi: OpenMP acceleration enabled")
else()
    message(WARNING "dc_pmppi: OpenMP unavailable; using serial fallback")
endif()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${XTENSOR_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/dc_pmppi_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME}_node OpenMP::OpenMP_CXX)
endif()

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}_disturbance_estimator_test
        test/disturbance_estimator_test.cpp)
    if(TARGET ${PROJECT_NAME}_disturbance_estimator_test)
        target_include_directories(${PROJECT_NAME}_disturbance_estimator_test PRIVATE
            include
            ${XTENSOR_INCLUDE_DIRS})
        if(OpenMP_CXX_FOUND)
            target_link_libraries(
                ${PROJECT_NAME}_disturbance_estimator_test OpenMP::OpenMP_CXX)
        endif()
    endif()
endif()

# 安装 Python 脚本为可执行
catkin_install_python(PROGRAMS
    scripts/JZJ_path.py
    scripts/twist_to_ackermann.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装配置文件
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# 安装 launch 文件
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 安装 RViz 配置
install(DIRECTORY scripts/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
