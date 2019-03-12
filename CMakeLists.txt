cmake_minimum_required(VERSION 3.5)

project(occupancy_grid_mapping)

# include_directories(include)

# find_package(PythonLibs 3.6)


find_package(OpenCV 3.0.0 REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    # message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

    INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

    add_executable(occupancy_grid_mapping main.cpp)
    target_link_libraries(occupancy_grid_mapping ${OpenCV_LIBS})

else()
    message("OpenCV not found, so we won't build the webcam_face_pose_ex example.")
endif()



