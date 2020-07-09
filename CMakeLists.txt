# 设置工程名为SensorFusion
project(SensorFusion)

# 设置所需要的CMake的最低版本号
cmake_minimum_required (VERSION 2.8)

# 使用C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
find_package(roscpp REQUIRED)#测试
# 设置使用DEBUG模式还是RELEASE模式
#set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_BUILD_TYPE RELEASE)

include_directories(src
${roscpp_INCLUDE_DIRS})  #测试

# 找到src目录及其子目录下的所有c文件和h文件
file(GLOB_RECURSE PROJECT_SOURCES "src/*.cpp" "src/*.c" "src/*.cc")
file(GLOB_RECURSE PROJECT_HEADERS "src/*.hpp" "src/*.h" "src/*.hh")

# 设置二进制文件的生成地址，即bin目录，编译后会有名为SensorFusion或SensorFusion_d的二进制文件生成在bin目录下
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 根据RELEASE或DEBUG模式，生成不同名的可执行文件，用以区分
#if( ${CMAKE_BUILD_TYPE} MATCHES "RELEASE" )
add_executable(SensorFusion ${PROJECT_SOURCES} ${PROJECT_HEADERS})
#else()
#    add_executable(SensorFusion_d ${PROJECT_SOURCES} ${PROJECT_HEADERS})
#endif()
target_link_libraries(SensorFusion ${roscpp_LIBRARIES})#测试
