# used indirectly by module service (from libservice repo)

define_module(BINARY semantic2obj
  DEPENDS semantic service utility
  )

set(semantic2obj_SOURCES
  semantic2obj.cpp
  )

add_executable(semantic2obj ${semantic2obj_SOURCES})
target_link_libraries(semantic2obj ${MODULE_LIBRARIES})
buildsys_binary(semantic2obj)

set(semantic2semantic_SOURCES
  semantic2semantic.cpp
  )

add_executable(semantic2semantic ${semantic2semantic_SOURCES})
target_link_libraries(semantic2semantic ${MODULE_LIBRARIES})
buildsys_binary(semantic2semantic)

set(semantic2gpkg_SOURCES
  semantic2gpkg.cpp
  )

add_executable(semantic2gpkg ${semantic2gpkg_SOURCES})
target_link_libraries(semantic2gpkg ${MODULE_LIBRARIES})
buildsys_binary(semantic2gpkg)
