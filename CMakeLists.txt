cmake_minimum_required(VERSION 3.5)
project(hummanArm VERSION 3.30.3 LANGUAGES CXX)
set(CMAKE_CXX_COMPILER "C:/msys64/ucrt64/bin/gcc.exe")


include_directories(include)

add_library(hummanArm

    include/hummanArm/hello_world.hpp
    include/hummanArm/math.hpp
    include/hummanArm/data.hpp
    include/hummanArm/debug.hpp




    src/hello_world.cpp
    src/math.cpp
    src/data.cpp
)


target_link_libraries(hummanArm)
target_compile_features(hummanArm PUBLIC cxx_std_17)


if(WIN32)
    install(TARGETS hummanArm
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
    )
    install(DIRECTORY include/hummanArm DESTINATION include)
else()
    install(TARGETS hummanArm
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
    )
    install(DIRECTORY include/hummanArm DESTINATION include)
endif()


enable_testing()

add_executable(hello_world_test tests/hello_world_test.cpp)
add_executable(prod_test tests/prod_test.cpp)
add_executable(forward_test tests/forward_test.cpp)
add_executable(forward_test2 tests/forward_test2.cpp)
add_executable(createDataBase tests/createDataBase.cpp)
add_executable(RobotRead tests/RobotRead.cpp)
add_executable(inverse_test tests/inverse_test.cpp)
add_executable(list_inverse_test tests/list_inverse_test.cpp)
add_executable(generate_inverse_test tests/generate_inverse_test.cpp)








target_link_libraries(hello_world_test hummanArm stdc++)
target_link_libraries(prod_test hummanArm stdc++)
target_link_libraries(forward_test hummanArm stdc++)
target_link_libraries(forward_test2 hummanArm stdc++)
target_link_libraries(createDataBase hummanArm stdc++)
target_link_libraries(RobotRead hummanArm stdc++)
target_link_libraries(inverse_test hummanArm stdc++)
target_link_libraries(list_inverse_test hummanArm stdc++)
target_link_libraries(generate_inverse_test hummanArm stdc++)








add_test(NAME hello_world_test COMMAND hello_world_test)
add_test(NAME prod_test COMMAND prod_test)
add_test(NAME forward_test COMMAND forward_test)
add_test(NAME forward_test2 COMMAND forward_test2)
add_test(NAME createDataBase COMMAND createDataBase)
add_test(NAME RobotRead COMMAND RobotRead)
add_test(NAME inverse_test COMMAND inverse_test)
add_test(NAME generate_inverse_test COMMAND generate_inverse_test)







