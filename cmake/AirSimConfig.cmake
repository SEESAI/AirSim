include(CMakeFindDependencyMacro)
find_dependency(Threads)
find_dependency(rpclib)
include("${CMAKE_CURRENT_LIST_DIR}/AirSimTargets.cmake")
