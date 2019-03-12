cmake_minimum_required(VERSION 3.5.1)
project(libcarla-microbenchmark)

if (CMAKE_BUILD_TYPE STREQUAL "Client")
  set(carla_config client)
elseif (CMAKE_BUILD_TYPE STREQUAL "Server")
  set(carla_config server)
endif ()

link_directories(
    ${RPCLIB_LIB_PATH}
    ${GBENCHMARK_LIB_PATH})

file(GLOB libcarla_benchmark_sources
    "${libcarla_source_path}/microbenchmark/*.cpp"
    "${libcarla_source_path}/microbenchmark/*.h")

# Create targets for debug and release in the same build type.
foreach(target libcarla_benchmark_${carla_config}_release)
    add_executable(${target} ${libcarla_benchmark_sources})

    target_include_directories(${target} PRIVATE
        "${BOOST_INCLUDE_PATH}"
        "${RPCLIB_INCLUDE_PATH}"
        "${GBENCHMARK_INCLUDE_PATH}")

    if (WIN32)
        target_link_libraries(${target} "rpc.lib")
        target_link_libraries(${target} "benchmark_main.lib")
        target_link_libraries(${target} "benchmark.lib")
    else()
        target_link_libraries(${target} "-lrpc")
        target_link_libraries(${target} "-lbenchmark_main")
        target_link_libraries(${target} "-lbenchmark")
    endif()

    install(TARGETS ${target} DESTINATION test)
endforeach(target)

# Specific options for release.
set_target_properties(libcarla_benchmark_${carla_config}_release PROPERTIES COMPILE_FLAGS ${CMAKE_CXX_FLAGS_RELEASE})
target_link_libraries(libcarla_benchmark_${carla_config}_release "carla_${carla_config}")
