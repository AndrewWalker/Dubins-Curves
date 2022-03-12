add_library(
    DubinsPath
)

target_sources(
    DubinsPath
    PRIVATE
        DubinsPath.cpp
)

target_include_directories(
    DubinsPath 
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(
    TARGETS DubinsPath
    EXPORT DubinsPathCore-config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/DubinsPath/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/DubinsPath
)

install(
    EXPORT DubinsPathCore-config
    NAMESPACE DubinsPathCore::
    DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake
)
