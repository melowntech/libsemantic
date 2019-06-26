# bump version here
set(semantic_VERSION 0.0)

define_module(LIBRARY semantic=${semantic_VERSION}
  DEPENDS
  geo jsoncpp
  )

set(semantic_SOURCES
  world.hpp
  io.hpp io.cpp

  mesh.hpp
  mesh/mesh.cpp mesh/mesh.incl.hpp
  mesh/roof.hpp
  mesh/roof/circular.cpp
  mesh/roof/rectangular.cpp

  ogr.hpp
  ogr/ogr.cpp ogr/ogr.incl.hpp
  ogr/roof.hpp
  ogr/roof/circular.cpp
  ogr/roof/rectangular.cpp
  )

add_library(semantic STATIC ${semantic_SOURCES})
buildsys_library(semantic)

target_link_libraries(semantic ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(semantic ${MODULE_DEFINITIONS})
