cmake_minimum_required(VERSION 3.10)
project(darknet_vendor VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

####################
# Build options
####################

option(DARKNET_CUDA "Build darknet with CUDA support" Off)
option(DARKNET_OPENCV "Build darknet with OpenCV support" On)

####################
# Download Darknet
####################

if(CMAKE_VERSION VERSION_LESS "3.11")
  include("cmake/FetchContent.cmake")
else()
  include(FetchContent)
endif()

# Same version as Debian Buster
set(darknet_git_tag "61c9d02ec461e30d55762ec7669d6a1d3c356fb2")

FetchContent_Declare(darknet-download
  GIT_REPOSITORY
    "https://github.com/AlexeyAB/darknet.git"
#    "https://github.com/pjreddie/darknet.git"
#  GIT_TAG
#    ${darknet_git_tag}
)

FetchContent_GetProperties(darknet-download)
if(NOT darknet-download_POPULATED)
  FetchContent_Populate(darknet-download)
endif()

set(darknet_src_dir "${darknet-download_SOURCE_DIR}")

####################
# Classify files
####################
set(darknet_public_headers
  "${CMAKE_CURRENT_BINARY_DIR}/darknet-src/include/darknet.h"
)

set(darknet_cuda_sources
  "${darknet_src_dir}/src/activation_kernels.cu"
  "${darknet_src_dir}/src/avgpool_layer_kernels.cu"
  "${darknet_src_dir}/src/blas_kernels.cu"
  "${darknet_src_dir}/src/col2im_kernels.cu"
  "${darknet_src_dir}/src/convolutional_kernels.cu"
  "${darknet_src_dir}/src/crop_layer_kernels.cu"
  "${darknet_src_dir}/src/deconvolutional_kernels.cu"
  "${darknet_src_dir}/src/dropout_layer_kernels.cu"
  "${darknet_src_dir}/src/im2col_kernels.cu"
  "${darknet_src_dir}/src/maxpool_layer_kernels.cu"
  "${darknet_src_dir}/src/network_kernels.cu" # ALEXEYAB
)

set(darknet_exec_c_sources
  "${darknet_src_dir}/src/art.c"
  "${darknet_src_dir}/src/captcha.c"
  "${darknet_src_dir}/src/cifar.c"
  "${darknet_src_dir}/src/classifier.c"
  "${darknet_src_dir}/src/coco.c"
  "${darknet_src_dir}/src/darknet.c"
  "${darknet_src_dir}/src/detector.c"
  "${darknet_src_dir}/src/go.c"
#  "${darknet_src_dir}/src/instance-segmenter.c" # ALEXEYAB
#  "${darknet_src_dir}/src/lsd.c" # ALEXEYAB
#  "${darknet_src_dir}/src/nightmare.c"
  "${darknet_src_dir}/src/super.c"
  "${darknet_src_dir}/src/rnn.c"
#  "${darknet_src_dir}/src/regressor.c" # ALEXEYAB
#  "${darknet_src_dir}/src/segmenter.c" # ALEXEYAB
  "${darknet_src_dir}/src/tag.c"
  "${darknet_src_dir}/src/yolo.c"
)

set(darknet_lib_cpp_sources
  "${darknet_src_dir}/src/image_opencv.cpp"
  "${darknet_src_dir}/src/http_stream" # ALEXEYAB
  "${darknet_src_dir}/src/yolo_console_dll.cpp"
  "${darknet_src_dir}/src/yolo_v2_class.cpp"
)


set(darknet_lib_c_sources
  "${darknet_src_dir}/src/activation_layer.c"
  "${darknet_src_dir}/src/activations.c"
  "${darknet_src_dir}/src/avgpool_layer.c"
  "${darknet_src_dir}/src/batchnorm_layer.c"
  "${darknet_src_dir}/src/blas.c"
  "${darknet_src_dir}/src/box.c"
  "${darknet_src_dir}/src/col2im.c"
  "${darknet_src_dir}/src/compare.c"
  "${darknet_src_dir}/src/connected_layer.c"
  "${darknet_src_dir}/src/conv_lstm_layer.c"
  "${darknet_src_dir}/src/convolutional_layer.c"
  "${darknet_src_dir}/src/cost_layer.c"
  "${darknet_src_dir}/src/cpu_gemm.c"
  "${darknet_src_dir}/src/crnn_layer.c"
  "${darknet_src_dir}/src/crop_layer.c"
  "${darknet_src_dir}/src/dark_cuda.c"
  "${darknet_src_dir}/src/data.c"
  "${darknet_src_dir}/src/deconvolutional_layer.c"
  "${darknet_src_dir}/src/demo.c"
  "${darknet_src_dir}/src/detection_layer.c"
  "${darknet_src_dir}/src/detector.c"
  "${darknet_src_dir}/src/dice.c"
  "${darknet_src_dir}/src/dropout_layer.c"
  "${darknet_src_dir}/src/gaussian_yolo_layer.c"
  "${darknet_src_dir}/src/gemm.c"
  "${darknet_src_dir}/src/getopt.c"
  "${darknet_src_dir}/src/gettimeofday.c"
  "${darknet_src_dir}/src/gru_layer.c"
  "${darknet_src_dir}/src/im2col.c"
  "${darknet_src_dir}/src/image.c"
  "${darknet_src_dir}/src/layer.c"
  "${darknet_src_dir}/src/list.c"
  "${darknet_src_dir}/src/local_layer.c"
  "${darknet_src_dir}/src/lstm_layer.c"
  "${darknet_src_dir}/src/matrix.c"
  "${darknet_src_dir}/src/maxpool_layer.c"
  "${darknet_src_dir}/src/network.c"
  "${darknet_src_dir}/src/nightmare.c"
  "${darknet_src_dir}/src/normalization_layer.c"
  "${darknet_src_dir}/src/option_list.c"
  "${darknet_src_dir}/src/parser.c"
  "${darknet_src_dir}/src/region_layer.c"
  "${darknet_src_dir}/src/reorg_layer.c"
  "${darknet_src_dir}/src/reorg_old_layer.c"
  "${darknet_src_dir}/src/rnn_layer.c"
  "${darknet_src_dir}/src/rnn_vid.c"
  "${darknet_src_dir}/src/route_layer.c"
  "${darknet_src_dir}/src/sam_layer.c"
  "${darknet_src_dir}/src/scale_channels_layer.c"
  "${darknet_src_dir}/src/shortcut_layer.c"
  "${darknet_src_dir}/src/softmax_layer.c"
  "${darknet_src_dir}/src/swag.c"
  "${darknet_src_dir}/src/tree.c"
  "${darknet_src_dir}/src/upsample_layer.c"
  "${darknet_src_dir}/src/utils.c"
  "${darknet_src_dir}/src/voxel.c"
  "${darknet_src_dir}/src/writing.c"
  "${darknet_src_dir}/src/yolo_layer.c"
)


####################
# Darknet Library
####################
# set(THREADS_PREFER_PTHREAD_FLAG True)
find_package(Threads REQUIRED)

if(DARKNET_CUDA)
  enable_language(CUDA)
  find_package(CUDA REQUIRED)

  # Flags taken from https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
  if(NOT DEFINED DARKNET_VENDOR_CUDA_FLAGS)
    if(CUDA_VERSION VERSION_GREATER 10)
      set(DARKNET_VENDOR_CUDA_FLAGS
        "-arch=sm_50"
        "-gencode=arch=compute_50,code=sm_50"
        "-gencode=arch=compute_52,code=sm_52"
        "-gencode=arch=compute_60,code=sm_60"
        "-gencode=arch=compute_61,code=sm_61"
        "-gencode=arch=compute_70,code=sm_70"
        "-gencode=arch=compute_75,code=sm_75"
        "-gencode=arch=compute_75,code=compute_75"
      )
    elseif(CUDA_VERSION VERSION_GREATER 9)
      set(DARKNET_VENDOR_CUDA_FLAGS
        "-arch=sm_50"
        "-gencode=arch=compute_50,code=sm_50"
        "-gencode=arch=compute_52,code=sm_52"
        "-gencode=arch=compute_60,code=sm_60"
        "-gencode=arch=compute_61,code=sm_61"
        "-gencode=arch=compute_70,code=sm_70"
        "-gencode=arch=compute_70,code=compute_70"
      )
    endif()
  endif()

  add_library(darknet SHARED ${darknet_lib_c_sources} ${darknet_lib_cpp_sources} ${darknet_cuda_sources})
  target_compile_options(darknet PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:${DARKNET_VENDOR_CUDA_FLAGS}>
  )
  target_compile_definitions(darknet PUBLIC GPU=1)
  target_link_libraries(darknet PUBLIC
    cuda cudart cublas curand
  )
else()  # not using CUDA
  add_library(darknet SHARED ${darknet_lib_c_sources} ${darknet_lib_cpp_sources})
endif()

set_target_properties(darknet PROPERTIES LINKER_LANGUAGE C)
target_include_directories(darknet PUBLIC
  $<BUILD_INTERFACE:${darknet_src_dir}/include>
  $<BUILD_INTERFACE:${darknet_src_dir}/src>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${darknet_src_dir}/3rdparty/stb/include> # ALEXEYAB
)

if(DARKNET_OPENCV)
  find_package(OpenCV REQUIRED)
  target_include_directories(darknet PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(darknet PUBLIC ${OpenCV_LIBRARIES})
  target_compile_definitions(darknet PUBLIC OPENCV=1)
endif()

# TODO(sloretz) pthread replacement on Windows?
target_link_libraries(darknet PUBLIC Threads::Threads)
if(UNIX)
  # Need to link C math library on some platforms
  target_link_libraries(darknet PUBLIC m)
endif()

####################
# Darknet_vendor target
####################

# Add interface library for darknet_vendor headers
add_library(darknet_vendor INTERFACE)
target_include_directories(darknet_vendor INTERFACE
  $<BUILD_INTERFACE:include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(darknet_vendor INTERFACE darknet)

configure_file("include/darknet_vendor/version.h.in" "version.h" @ONLY)

####################
# Darknet Executable
####################
add_executable(darknet_exec ${darknet_exec_c_sources})
target_link_libraries(darknet_exec PUBLIC darknet)
set_target_properties(darknet_exec PROPERTIES OUTPUT_NAME "darknet")

####################
# ProjectConfig.cmake
####################

set(INCLUDE_INSTALL_DIR "include")
set(SHARE_CMAKE_INSTALL_DIR "share/darknet_vendor/cmake")
set(EXEC_INSTALL_DIR "bin")
include(CMakePackageConfigHelpers)
configure_package_config_file(
  "cmake/darknet_vendor-config.cmake.in"
  "darknet_vendor-config.cmake"
  INSTALL_DESTINATION "${SHARE_CMAKE_INSTALL_DIR}"
  PATH_VARS "INCLUDE_INSTALL_DIR" "SHARE_CMAKE_INSTALL_DIR" "EXEC_INSTALL_DIR"
)

write_basic_package_version_file(
  "darknet_vendor-version.cmake"
  COMPATIBILITY SameMajorVersion)

####################
# Installation
####################

# Install darknet public headers
install(
  DIRECTORY
    "${darknet_src_dir}/include/"
  DESTINATION
    "include"
)

# Install darknet_vendor public headers
install(
  DIRECTORY
    "include/"
  DESTINATION
    "include"
  FILES_MATCHING PATTERN
    "*.h"
)
install(
  FILES
  "${CMAKE_CURRENT_BINARY_DIR}/version.h"
  DESTINATION
    "include/darknet_vendor"
)

# Install darknet library
install(
  TARGETS darknet
  EXPORT "darknet_vendor-targets"
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin"
)

# Install darknet_vendor interface library
install(
  TARGETS darknet_vendor
  EXPORT "darknet_vendor-targets"
)

# Install darknet demo executable
install(TARGETS darknet_exec DESTINATION "${EXEC_INSTALL_DIR}")

# Install config files and helper scripts
install(
  DIRECTORY
    "${darknet_src_dir}/cfg"
  DESTINATION
    "share/darknet"
)
install(
  DIRECTORY
    "${darknet_src_dir}/data"
  DESTINATION
    "share/darknet"
)
install(
  DIRECTORY
    "${darknet_src_dir}/scripts"
  DESTINATION
    "share/darknet"
)
#[[ ALEXEYAB
install(
  DIRECTORY
    "${darknet_src_dir}/python"
  DESTINATION
    "share/darknet"
)
#]]
# Add an entry to the ament index
install(
  FILES
    resource/darknet_vendor
  DESTINATION
    "share/ament_index/resource_index/packages/"
)

# Install license files
install(
  FILES
    "${darknet_src_dir}/LICENSE"
#    "${darknet_src_dir}/LICENSE.fuck" # ALEXEYAB
#    "${darknet_src_dir}/LICENSE.gen"
#    "${darknet_src_dir}/LICENSE.gpl"
#    "${darknet_src_dir}/LICENSE.meta"
#    "${darknet_src_dir}/LICENSE.mit"
#    "${darknet_src_dir}/LICENSE.v1"
  DESTINATION
    "share/darknet"
)

# Install project-config.cmake and project-version.cmake
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/darknet_vendor-config.cmake"
  DESTINATION
    share/darknet_vendor/cmake
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/darknet_vendor-version.cmake"
  DESTINATION
    share/darknet_vendor/cmake
)

# install exported targets
install(
  EXPORT
    "darknet_vendor-targets"
  NAMESPACE
    "darknet_vendor::"
  DESTINATION
    "share/darknet_vendor/cmake"
)

