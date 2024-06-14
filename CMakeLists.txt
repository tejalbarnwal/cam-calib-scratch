cmake_minimum_required(VERSION 3.10)

project(cam_calib_scratch)

## reference link: https://cmake.org/cmake/help/latest/policy/CMP0146.html
# cmake_policy(SET CMP0146 NEW)
# enable_language(CUDA)
# set(CMAKE_CUDA_ARCHITECTURES 86)
### use the below FindCUDA module for cmake version before 3.10
# find_package(CUDA REQUIRED)
# if (CUDA_FOUND)
#     message("CUDA found")
# else()
#     message("CUDA not found")
# endif()
# include_directories(${CUDA_INCLUDE_DIRS}) and target_link_libraries(${CUDA_LIBRARIES})


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

foreach(dep math;random;gui;maps)
	# if not building from inside MRPT source tree, find it as a cmake
	# imported project:
	if (NOT TARGET mrpt::${dep})
		find_package(mrpt-${dep} REQUIRED)
        if (mrpt-${dep}_FOUND)
            message(${dep}"dep found")
        else()
            message("no found")
        endif()
	endif()
endforeach()

include_directories(
    include
    ${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)


add_executable(calib_main src/calibration_main.cpp src/calib/compute_init_param.cpp )
target_link_libraries(calib_main ${EIGEN_LIBRARIES} ${OpenCV_LIBRARIES} )

add_executable(eg_mrpt_opt src/reference_opt_egs/eg_mrpt_opt.cpp)
foreach(dep math;random;gui;maps)
	target_link_libraries(eg_mrpt_opt mrpt::${dep})
endforeach()

add_executable(eigen_lm2 src/reference_opt_egs/eigen_lm2.cpp)
target_link_libraries(eigen_lm2 ${EIGEN_LIBRARIES})