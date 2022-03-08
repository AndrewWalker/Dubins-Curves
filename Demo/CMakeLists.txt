set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(
    DubinsPathDemo
)

target_sources(
    DubinsPathDemo 
    PRIVATE
        Demo.cpp
)

target_link_libraries(
    DubinsPathDemo
    PUBLIC
        DubinsCurve
)

install(
    TARGETS DubinsPathDemo
    EXPORT DubinsPathDemo-config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    EXPORT DubinsPathDemo-config
    NAMESPACE DubinsPathDemo::
    DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake
)

