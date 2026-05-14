cmake_minimum_required(VERSION 3.10)
project(autonomous_driving)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_WITH_PROTOBUF "Build with Protobuf support for communication" ON)
option(BUILD_WITH_FOXGLOVE "Build with Foxglove Studio visualization module" ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(YAML-CPP REQUIRED)

set(PROTOBUF_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/proto_output)
file(MAKE_DIRECTORY ${PROTOBUF_OUTPUT_DIR})

if(BUILD_WITH_PROTOBUF)
    find_package(Protobuf REQUIRED)
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/proto
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

if(BUILD_WITH_PROTOBUF)
    include_directories(
        ${PROTOBUF_OUTPUT_DIR}
        ${PROTOBUF_INCLUDE_DIRS}
    )
    add_definitions(-DBUILD_WITH_PROTOBUF)
endif()

file(GLOB_RECURSE SRC_FILES 
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)
list(REMOVE_ITEM SRC_FILES ${PROJECT_SOURCE_DIR}/src/main.cpp)

set(PROTO_SRCS "")
set(PROTO_HDRS "")

if(BUILD_WITH_PROTOBUF)
    protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS 
        ${PROJECT_SOURCE_DIR}/proto/communication.proto
    )
    
    set(PROTOBUF_GENERATED_CPP ${PROTOBUF_OUTPUT_DIR}/communication.pb.cc)
    set(PROTOBUF_GENERATED_H ${PROTOBUF_OUTPUT_DIR}/communication.pb.h)
    
    add_custom_command(
        OUTPUT ${PROTOBUF_GENERATED_CPP} ${PROTOBUF_GENERATED_H}
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --cpp_out=${PROTOBUF_OUTPUT_DIR} --proto_path=${PROJECT_SOURCE_DIR}/proto ${PROJECT_SOURCE_DIR}/proto/communication.proto
        MAIN_DEPENDENCY ${PROJECT_SOURCE_DIR}/proto/communication.proto
        COMMENT "Generating Protobuf files to ${PROTOBUF_OUTPUT_DIR}"
    )
    
    add_custom_target(generate_protobuf ALL
        DEPENDS ${PROTOBUF_GENERATED_CPP} ${PROTOBUF_GENERATED_H}
    )
endif()

add_executable(autonomous_driving 
    src/main.cpp
    ${SRC_FILES}
)

if(BUILD_WITH_PROTOBUF)
    add_dependencies(autonomous_driving generate_protobuf)
    target_sources(autonomous_driving PRIVATE 
        ${PROTOBUF_GENERATED_CPP}
        ${PROTOBUF_GENERATED_H}
    )
    target_link_libraries(autonomous_driving
        ${PROTOBUF_LIBRARIES}
    )
endif()

target_link_libraries(autonomous_driving
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    pthread
)

install(TARGETS autonomous_driving 
    RUNTIME DESTINATION bin
)

if(BUILD_WITH_PROTOBUF)
    install(FILES ${PROTOBUF_GENERATED_H} DESTINATION include/proto)
endif()

file(GLOB TEST_FILES ${PROJECT_SOURCE_DIR}/test/*.cpp)
file(GLOB TEST_PLANNING_FILES ${PROJECT_SOURCE_DIR}/test/planning/*.cpp)

list(APPEND TEST_FILES ${TEST_PLANNING_FILES})

foreach(TEST_FILE ${TEST_FILES})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_FILE} ${SRC_FILES})
    
    if(BUILD_WITH_PROTOBUF)
        add_dependencies(${TEST_NAME} generate_protobuf)
        target_sources(${TEST_NAME} PRIVATE 
            ${PROTOBUF_GENERATED_CPP}
            ${PROTOBUF_GENERATED_H}
        )
        target_link_libraries(${TEST_NAME}
            ${PROTOBUF_LIBRARIES}
        )
    endif()
    
    target_link_libraries(${TEST_NAME}
        ${EIGEN3_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        pthread
    )
    
    set_target_properties(${TEST_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/test
    )
endforeach()