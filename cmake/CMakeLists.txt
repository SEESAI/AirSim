cmake_minimum_required(VERSION 3.5.0)
project(AirSim)


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


