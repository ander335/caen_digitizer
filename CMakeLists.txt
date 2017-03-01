cmake_minimum_required(VERSION 3.0)
project(caen_digitizer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES CaenDigitizer.cpp Helpers.cpp)

add_library(caen_digitizer SHARED ${SOURCE_FILES})

include_directories(
        /usr/include/python2.7
        ${Boost_INCLUDE_DIR}
        /usr/include/jsoncpp
        /usr/local/include/mongo
)

target_link_libraries(caen_digitizer -lCAENComm -lCAENDigitizer )
