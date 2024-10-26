cmake_minimum_required(VERSION 3.5)
project(hummanArm VERSION 3.30.3 LANGUAGES CXX)

# Set the C++ compiler for Windows
if(WIN32)
    set(CMAKE_CXX_COMPILER "C:/msys64/ucrt64/bin/gcc.exe")
endif()

# Include directories
include_directories(include)

# Define the library hummanArm
add_library(hummanArm
    include/hummanArm/hello_world.hpp
    include/hummanArm/math.hpp
    include/hummanArm/data.hpp
    include/hummanArm/debug.hpp
    src/hello_world.cpp
    src/math.cpp
    src/data.cpp
)

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

# Add executable tests
add_executable(hello_world_test tests/hello_world_test.cpp)
add_executable(forward_test tests/forward_test.cpp)
add_executable(forward_test2 tests/forward_test2.cpp)
add_executable(inverse_test tests/inverse_test.cpp)
add_executable(list_inverse_test tests/list_inverse_test.cpp)
add_executable(generate_inverse_test tests/generate_inverse_test.cpp)
add_executable(forward_str tests/forward_str.cpp)

# Link tests to the hummanArm library and other necessary libraries
target_link_libraries(hello_world_test hummanArm stdc++)
target_link_libraries(forward_test hummanArm stdc++)
target_link_libraries(forward_test2 hummanArm stdc++)
target_link_libraries(inverse_test hummanArm stdc++)
target_link_libraries(list_inverse_test hummanArm stdc++)
target_link_libraries(generate_inverse_test hummanArm stdc++)
target_link_libraries(forward_str hummanArm stdc++)

# Register tests
add_test(NAME hello_world_test COMMAND hello_world_test)
add_test(NAME forward_test COMMAND forward_test)
add_test(NAME forward_test2 COMMAND forward_test2)
add_test(NAME inverse_test COMMAND inverse_test)
add_test(NAME generate_inverse_test COMMAND generate_inverse_test)
add_test(NAME forward_str COMMAND forward_str)
