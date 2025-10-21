# Examples CMakeLists.txt

# Get all cpp files in the examples directory
file(GLOB EXAMPLE_SOURCES "*.cpp")

# Create an executable for each example file
foreach(EXAMPLE_SOURCE ${EXAMPLE_SOURCES})
    # Get the filename without extension to use as executable name
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_SOURCE} NAME_WE)
    
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_SOURCE})
    target_link_libraries(${EXAMPLE_NAME} mjcf)
    target_compile_features(${EXAMPLE_NAME} PRIVATE cxx_std_17)
    # set executable output directory
    set_target_properties(${EXAMPLE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
endforeach()
