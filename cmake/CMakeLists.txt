cmake_minimum_required(VERSION 3.5.0)
project(AirSim)

include(CMakePackageConfigHelpers)
set(ConfigPackageLocation lib/cmake/AirSim)

if (NIX)
  find_package(rpclib REQUIRED)
elseif (UNIX)
  add_subdirectory("rpclib_wrapper")
endif ()

add_subdirectory("AirLib")
add_subdirectory("MavLinkCom")
add_subdirectory("AirLibUnitTests")
add_subdirectory("HelloDrone")
add_subdirectory("HelloSpawnedDrones")
add_subdirectory("HelloCar")
add_subdirectory("DroneShell")
add_subdirectory("DroneServer")

install(
  TARGETS AirLib MavLinkCom 
  EXPORT AirSimTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  FILE_SET HEADERS
)

install(
  EXPORT AirSimTargets
  FILE
    AirSimTargets.cmake
  NAMESPACE
    AirSim::
  DESTINATION
    ${ConfigPackageLocation}
)

# TODO: Add some sort of versioning to AirSim?
#write_basic_package_version_file(
#  "${CMAKE_CURRENT_BINARY_DIR}/ClimbingStats/ClimbingStatsConfigVersion.cmake"
#  VERSION ${Upstream_VERSION}
#  COMPATIBILITY AnyNewerVersion
#)


configure_file(AirSimConfig.cmake
  "${CMAKE_CURRENT_SOURCE_DIR}/AirSim/AirSimConfig.cmake"
  COPYONLY
)



install(
  FILES
    AirSimConfig.cmake
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
