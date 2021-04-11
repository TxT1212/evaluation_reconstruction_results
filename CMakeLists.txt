cmake_minimum_required(VERSION 3.0.0)
project(evaluation VERSION 0.1.0)

include(CTest)
enable_testing()

# OPENCV
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# PCL
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARIES_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Eigen
# include_directories("/usr/include/eigen3")
find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR})


add_executable(evaluation main.cpp)
add_executable(depth2cloud depth2cloud.cc)
add_executable(findBoard findBoard.cc)
add_executable(poseEvaluation poseEvaluation.cpp)

target_link_libraries(evaluation
  ${OpenCV_LIBRARIES}
  ${PCL_LIBS}
  ${PCL_LIBRARIES}
)
target_link_libraries(depth2cloud
  ${OpenCV_LIBRARIES}
  ${PCL_LIBS}
  ${PCL_LIBRARIES}
)
target_link_libraries(poseEvaluation
  ${OpenCV_LIBRARIES}
)
target_link_libraries(findBoard
  ${OpenCV_LIBRARIES}
)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
