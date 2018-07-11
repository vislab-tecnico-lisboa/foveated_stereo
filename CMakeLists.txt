cmake_minimum_required(VERSION 2.8.3)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB CPP_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/stereo_sensors/*.cpp)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
   cpp
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_30,code=sm_30 -Xcompiler -fPIC)
#SET(LIB_TYPE STATIC) #set the lib type
#CUDA_ADD_LIBRARY(TestLib ${LIB_TYPE} cpp/test.cu cpp/test.h)

#cuda_compile(teste_o cpp/test.cu)
#cuda_compile(DRIVER_O driver.cu OPTIONS -G)

#cuda_add_executable(teste cpp/test.cu)

## Declare a cpp library
add_library(foveated_stereo_lib
  ${CPP_FILES}
)

#target_link_libraries(foveated_stereo_lib TestLib ${CUDA_LIBRARIES})

