# mavlink-server/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(mavlink-server VERSION 0.1.0
        DESCRIPTION "mavlink-server library"
        LANGUAGES CXX)

add_library(${PROJECT_NAME} STATIC)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

find_package (Threads)

target_sources(${PROJECT_NAME}
    PUBLIC
        mavlink-server.h
    PRIVATE
        mavlink-server.cpp
)

target_link_libraries(${PROJECT_NAME} 
        PUBLIC
           Threads::Threads
           mavlink-interface
           tcp_server_client
        )


target_include_directories(${PROJECT_NAME} PUBLIC ./)