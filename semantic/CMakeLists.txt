# bump version here
set(semantic_VERSION 0.0)

define_module(LIBRARY semantic=${semantic_VERSION}
  DEPENDS
  geo jsoncpp
  )

set(semantic_SOURCES
  world.hpp
  io.hpp io.cpp
  mesh.hpp mesh.cpp
  )

add_library(semantic STATIC ${semantic_SOURCES})
buildsys_library(semantic)

target_link_libraries(semantic ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(semantic ${MODULE_DEFINITIONS})
