cmake_minimum_required(VERSION 2.8)
set(PROJECTNAME make-it-roll)
project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH})

find_package(OpenCV)

include(iCubOptions)
include(iCubHelpers)

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
)
add_executable(${PROJECTNAME} ${source} ${header})

target_link_libraries(${PROJECTNAME} icubmod ${YARP_LIBRARIES})
