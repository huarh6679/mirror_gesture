cmake_minimum_required(VERSION 3.0.2)
project(mirror_pose_test)

set(CMAKE_BUILD_TYPE DEBUG)
# set(CMAKE_BUILD_TYPE RELEASE)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

# if(CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wno-deprecated-declarations -O3 -funroll-loops -g")
# endif()


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(OpenCV REQUIRED)

###########
## Build ##
###########


message(${CMAKE_HOST_SYSTEM_NAME})
message(${CMAKE_HOST_SYSTEM_PROCESSOR})
message(${CMAKE_SYSTEM_NAME})
message(${CMAKE_SYSTEM_PROCESSOR})
message(${CMAKE_SYSTEM_VERSION})
if(ANDROID)
  # message(${CMAKE_EXPORT_COMPILE_COMMANDS})
  # message(${CMAKE_ANDROID_STL_TYPE})
  # message(${ANDROID_PLATFORM})
  # message(${ANDROID_ABI})
  # message(${CMAKE_ANDROID_ARCH_ABI})
  # message(${ANDROID_NDK})
  # message(${CMAKE_ANDROID_NDK})
  # message(${CMAKE_TOOLCHAIN_FILE})
endif()

if(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Linux")
  message("this is Linux")
endif()

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  message("this is x86_64 cpu")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  message("this is aarch64 cpu")
endif()

# VITURE_XR_Glasses_SDK
if(ANDROID)
  SET(viture_Dir ${CMAKE_CURRENT_SOURCE_DIR}/libs/VITURE_XR_Glasses/VITURE_XR_Glasses_SDK_for_Android/release)
  include_directories(${viture_Dir}/include)
  link_directories(${viture_Dir}/${ANDROID_ABI})
  SET(CARINA_VIO_LIBRARY ${viture_Dir}/${ANDROID_ABI}/libcarina_vio.so)
  SET(CLOUD_PROTOCOL_LIBRARY ${viture_Dir}/${ANDROID_ABI}/libcloud_protocol.so)
  SET(GLASSES_LIBRARY ${viture_Dir}/${ANDROID_ABI}/libglasses.so)
else()
  SET(viture_Dir ${CMAKE_CURRENT_SOURCE_DIR}/libs/VITURE_XR_Glasses/VITURE_XR_Glasses_SDK_for_Linux_x86_64/release)
  include_directories(${viture_Dir}/include)
  link_directories(${viture_Dir}/${CMAKE_SYSTEM_PROCESSOR})
  SET(CARINA_VIO_LIBRARY ${viture_Dir}/${CMAKE_SYSTEM_PROCESSOR}/libcarina_vio.so)
  SET(GLASSES_LIBRARY ${viture_Dir}/${CMAKE_SYSTEM_PROCESSOR}/libglasses.so)
endif()

link_directories(
  libs
)

add_executable(virtual_capture
  virtual_capture.cpp
  )
target_link_libraries(virtual_capture PRIVATE
  
  ${COM_LIBS}
  ${COMMON_LIBRARY}
  ${JSONCPP_LIBRARY}
  ${GLASSES_LIBRARY}
  ${CARINA_VIO_LIBRARY}
  ${CLOUD_PROTOCOL_LIBRARY}
  hidapi-hidraw
  hidapi-libusb
)

install(TARGETS virtual_capture
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  # RUNTIME DESTINATION bin
)



add_executable(virtual_forcalib
  virtual_forcalib.cpp
  )
target_link_libraries(virtual_forcalib PRIVATE
  
  ${COM_LIBS}
  ${COMMON_LIBRARY}
  ${JSONCPP_LIBRARY}
  ${GLASSES_LIBRARY}
  ${CARINA_VIO_LIBRARY}
  ${CLOUD_PROTOCOL_LIBRARY}
  hidapi-hidraw
  hidapi-libusb
)

install(TARGETS virtual_forcalib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  # RUNTIME DESTINATION bin
)