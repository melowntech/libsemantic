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
