add_library(
    DubinsCurve
)

target_sources(
    DubinsCurve
    PRIVATE
        DubinsCurve.cpp
)

target_include_directories(
    DubinsCurve 
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(
    TARGETS DubinsCurve
    EXPORT DubinsCurveCore-config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/DubinsCurve/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/DubinsCurve
)

install(
    EXPORT DubinsCurveCore-config
    NAMESPACE DubinsCurveCore::
    DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake
)
