cmake_minimum_required(VERSION 3.14)
project(foxglove_websocket CXX)
#list(APPEND CMAKE_MODULE_PATH /usr/lib/cmake/)
#set(nlohmann_json_DIR /usr/lib/cmake/ )
#find_package(nlohmann_json REQUIRED)
#find_package(websocketpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)


add_definitions(-DASIO_STANDALONE)
find_package(Boost COMPONENTS system  REQUIRED )
include_directories(/usr/include/boost)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../websocketpp)
#include_directories(single_include)
add_library(${PROJECT_NAME} SHARED src/parameter.cpp src/serialization.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

install(TARGETS ${PROJECT_NAME})
install (DIRECTORY ${CMAKE_SOURCE_DIR}/include/
        DESTINATION include)
install(FILES ${CMAKE_SOURCE_DIR}/LICENSE DESTINATION ${CMAKE_INSTALL_PREFIX}/licenses/)
