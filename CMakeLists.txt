cmake_minimum_required(VERSION 3.5)
project(hummanArm VERSION 3.30.3 LANGUAGES CXX)

# Set the C++ compiler for Windows
if(WIN32)
    set(CMAKE_CXX_COMPILER "C:/msys64/ucrt64/bin/gcc.exe")
endif()

# Include directories
include_directories(include)


set(INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

# Collect all header files (including Eigen)
file(GLOB HEADER_FILES 
    ${INCLUDE_DIRS}/hummanArm/*.hpp
    ${INCLUDE_DIRS}/Eigen/*)

# Collect all source files
file(GLOB SOURCE_FILES 
    ${SRC_DIR}/*.cpp)

# Add the library with collected files
add_library(hummanArm ${HEADER_FILES} ${SOURCE_FILES})

# Link libraries (if needed, replace stdc++ with the proper library on your platform)
target_link_libraries(hummanArm)

# Require C++17
target_compile_features(hummanArm PUBLIC cxx_std_17)

# Install targets and headers
install(TARGETS hummanArm
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/hummanArm DESTINATION include)

# Enable testing
enable_testing()
file(GLOB TEST_SOURCES
    tests/*.cpp)

# Loop over all collected test source files and create an executable for each
foreach(TEST_FILE ${TEST_SOURCES})
    # Extract test name from file path (just the file name without extension)
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    # Create an executable for each test file
    add_executable(${TEST_NAME} ${TEST_FILE})

    # Link the test executable to the hummanArm library and other necessary libraries
    target_link_libraries(${TEST_NAME} hummanArm stdc++)

    # Register the test
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()  