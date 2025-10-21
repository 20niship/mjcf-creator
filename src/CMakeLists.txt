set(MJCF_SOURCES
    mjcf/element.cpp
    mjcf/enums.cpp
    mjcf/core_elements.cpp
    mjcf/asset_elements.cpp
    mjcf/body_elements.cpp
    mjcf/actuator_elements.cpp
    mjcf/sensor_elements.cpp
    mjcf/urdf_converter.cpp

    # externals 
    ext/tinyxml2.cpp
    # ext/loguru/loguru.cpp
)

add_library(mjcf STATIC ${MJCF_SOURCES})
target_include_directories(mjcf PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/ext
)
target_compile_features(mjcf PUBLIC cxx_std_17)
