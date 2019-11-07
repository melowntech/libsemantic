# bump version here
set(semantic_VERSION 1.0)

define_module(LIBRARY semantic=${semantic_VERSION}
  DEPENDS
  BuildSystem>=1.4 C++>=14
  shtools
  geo jsoncpp
  geometry>=1.12
  Boost_IOSTREAMS
  )

set(semantic_SOURCES
  world.hpp world.cpp
  roof.hpp roof.cpp

  io.hpp io.cpp

  mesh.hpp
  mesh/mesh.cpp mesh/mesh.incl.hpp
  mesh/building.hpp
  mesh/roof.hpp
  mesh/roof/circular.cpp
  mesh/roof/rectangular.cpp
  mesh/tree.hpp mesh/tree.cpp

  ogr.hpp
  ogr/ogr.cpp ogr/ogr.incl.hpp
  ogr/building.hpp
  ogr/roof.hpp
  ogr/roof/circular.cpp
  ogr/roof/rectangular.cpp
  ogr/tree.hpp ogr/tree.cpp

  featurelayers.hpp featurelayers.cpp

  gpkg.hpp gpkg.cpp
  )

add_library(semantic STATIC ${semantic_SOURCES})
buildsys_library(semantic)

target_link_libraries(semantic ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(semantic ${MODULE_DEFINITIONS})
