# Tests CMakeLists.txt

# Download doctest
if(NOT EXISTS ${CMAKE_SOURCE_DIR}/tests/doctest.h)
    message(STATUS "Downloading doctest...")
    file(DOWNLOAD 
         "https://raw.githubusercontent.com/doctest/doctest/master/doctest/doctest.h" 
         "${CMAKE_SOURCE_DIR}/tests/doctest.h"
         SHOW_PROGRESS)
endif()

# Test executable
set(TEST_SOURCES
    test_main.cpp
    test_element.cpp
    test_core_elements.cpp
    test_asset_elements.cpp
    test_body_elements.cpp
    test_integration.cpp
    test_urdf_conversion.cpp
    test_actuator_sensor.cpp
    test_temporary_files.cpp
    test_name_utilities.cpp
)

add_executable(tests ${TEST_SOURCES})
target_include_directories(tests PRIVATE .)
target_link_libraries(tests PRIVATE mjcf)
target_compile_features(tests PRIVATE cxx_std_17)

# Add test target
enable_testing()
add_test(NAME mjcf_tests COMMAND tests)