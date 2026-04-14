cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(boating_gz)

find_package(ament_cmake REQUIRED)

# Gazebo version, use env variable or ROS 2 official pairing for LTS
# https://gazebosim.org/docs/latest/ros_installation/
if(DEFINED ENV{GZ_VERSION})
  set(GZ_VERSION "$ENV{GZ_VERSION}")
elseif(DEFINED ENV{IGN_VERSION})
  set(GZ_VERSION "$ENV{IGN_VERSION}")
elseif("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "lyrical")
  set(GZ_VERSION "jetty")
elseif("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "jazzy")
  set(GZ_VERSION "harmonic")
else()
  set(GZ_VERSION "fortress")
endif()

message(STATUS "Compiling against Gazebo ${GZ_VERSION}")
set(GZ_TARGET_PREFIX gz)

if("${GZ_VERSION}" STRGREATER_EQUAL "jetty")
  set(GZ_MSGS msgs)
  set(GZ_TRANSPORT transport)
  set(GZ_PLUGIN plugin)
  set(GZ_COMMON common)
  set(GZ_SIM sim)
  set(GZ_MATH math)
  set(GZ_RDR rendering)
  set(GZ_SENSORS sensors)
  set(GZ_UTILS utils)
  set(GZ_FUEL fuel_tools)

elseif("${GZ_VERSION}" STREQUAL "harmonic")
  set(GZ_MSGS msgs10)
  set(GZ_TRANSPORT transport13)
  set(GZ_PLUGIN plugin2)
  set(GZ_COMMON common5)
  set(GZ_SIM sim8)
  set(GZ_MATH math7)
  set(GZ_RDR rendering8)
  set(GZ_SENSORS sensors8)
  set(GZ_UTILS utils2)
  set(GZ_FUEL fuel_tools9)

elseif("${GZ_VERSION}" STREQUAL "garden")
  set(GZ_MSGS msgs9)
  set(GZ_TRANSPORT transport12)
  set(GZ_PLUGIN plugin2)
  set(GZ_COMMON common5)
  set(GZ_SIM sim7)
  set(GZ_MATH math7)
  set(GZ_RDR rendering7)
  set(GZ_SENSORS sensors7)
  set(GZ_UTILS utils2)
  set(GZ_FUEL fuel_tools8)

else()
  set(GZ_TARGET_PREFIX ignition)
  set(GZ_MSGS msgs8)
  set(GZ_TRANSPORT transport11)
  set(GZ_PLUGIN plugin1)
  set(GZ_COMMON common4)
  set(GZ_SIM gazebo6)
  set(GZ_MATH math6)
  set(GZ_RDR rendering6)
  set(GZ_SENSORS sensors6)
  set(GZ_UTILS utils1)
  set(GZ_FUEL fuel_tools7)
  add_definitions(-DWITH_IGNITION_FORTRESS)
  add_definitions(-DGZ_UTILS_UNIQUE_IMPL_PTR=IGN_UTILS_UNIQUE_IMPL_PTR)
  add_definitions(-DGZ_PROFILE=IGN_PROFILE)
  add_definitions(-DGZ_ADD_PLUGIN=IGNITION_ADD_PLUGIN)
  add_definitions(-DGZ_ADD_PLUGIN_ALIAS=IGNITION_ADD_PLUGIN_ALIAS)
  add_definitions(-Dgzerr=ignerr)
  add_definitions(-Dgzwarn=ignwarn)
  add_definitions(-Dgzmsg=ignmsg)
  add_definitions(-Dgzdbg=igndbg)
endif()

add_definitions(-DGZ_NS=${GZ_TARGET_PREFIX})
find_package(${GZ_TARGET_PREFIX}-${GZ_MSGS} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_TRANSPORT} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_PLUGIN} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_SIM} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_COMMON} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_MATH} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_RDR} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_SENSORS} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_UTILS} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_FUEL} REQUIRED)

find_package(Eigen3 REQUIRED)

# Waves core plugin
add_library(boating-Waves SHARED
  src/Wavefield.cc
)
target_link_libraries(boating-Waves PUBLIC
  ${GZ_TARGET_PREFIX}-${GZ_COMMON}::core
  ${GZ_TARGET_PREFIX}-${GZ_SIM}::core
  ${GZ_TARGET_PREFIX}-${GZ_MATH}
  Eigen3::Eigen
)
install(
  TARGETS boating-Waves
  DESTINATION lib)


# other plugins
list(APPEND BOATING_GZ_SRC
  Hull
  WaveVisual
  Anemometer
  LiftDrag
  PublisherPlugin
)

foreach(PLUGIN ${BOATING_GZ_SRC})
  add_library(boating-${PLUGIN} SHARED src/${PLUGIN}.cc)
  target_link_libraries(boating-${PLUGIN} PUBLIC
    ${GZ_TARGET_PREFIX}-${GZ_SIM}::core
    ${GZ_TARGET_PREFIX}-${GZ_PLUGIN}::core
    ${GZ_TARGET_PREFIX}-${GZ_RDR}::core
    ${GZ_TARGET_PREFIX}-${GZ_SENSORS}::core
    ${GZ_TARGET_PREFIX}-${GZ_UTILS}::core
    ${GZ_TARGET_PREFIX}-${GZ_TRANSPORT}::core
    boating-Waves
    Eigen3::Eigen
  )
  list(APPEND BOATING_GZ_PLUGINS boating-${PLUGIN})
endforeach()

install(TARGETS ${BOATING_GZ_PLUGINS} DESTINATION lib)


#============================================================================
# Resources
install(DIRECTORY
  models
  worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
